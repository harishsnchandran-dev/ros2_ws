#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import random
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion
from scipy.spatial.transform import Rotation as R

# Coordinates
PICK_POS = [0.6, 0.0, 0.2] 
PLACE_POS = [0.0, 0.6, 0.2]
TRANSIT_POS = [0.42, 0.42, 0.4]
OFFSET_Z = 0.2

# MODIFIED ORIENTATION: Rotated Wrist 1 & 2 by 90 degrees
TARGET_RPY = [1.5708, 1.5708, 0.0] 

class PickAndPlace90(Node):
    def __init__(self):
        super().__init__('pnp_90')
        self.declare_parameter('speed_multiplier', 1.0)
        self._arm_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        self.current_joints = None
        self.last_valid_q = None
        
        self.get_logger().info('Connecting to controllers...')
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        while not self._ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        self.get_logger().info('90-Degree P&P Node Ready (Static Run).')

    def joint_cb(self, msg):
        self.current_joints = msg

    def solve_ik(self, pos, relax_collisions=True):
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = 'ur5_arm'
        ik_req.pose_stamped.header.frame_id = 'base_link'
        ik_req.pose_stamped.pose.position.x, ik_req.pose_stamped.pose.position.y, ik_req.pose_stamped.pose.position.z = pos
        q_quat = R.from_euler('xyz', TARGET_RPY).as_quat()
        ik_req.pose_stamped.pose.orientation = Quaternion(x=q_quat[0], y=q_quat[1], z=q_quat[2], w=q_quat[3])
        ik_req.avoid_collisions = not relax_collisions
        
        state = RobotState()
        if self.last_valid_q:
            state.joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            state.joint_state.position = list(self.last_valid_q[:6])
        elif self.current_joints:
            state.joint_state = self.current_joints
        
        ik_req.robot_state = state
        req.ik_request = ik_req
        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.error_code.val == 1:
            self.last_valid_q = res.solution.joint_state.position
            return res.solution.joint_state.position
        return None

    def move_arm(self, q, name, base_duration=3.0):
        multiplier = self.get_parameter('speed_multiplier').get_parameter_value().double_value
        effective_duration = max(0.5, base_duration / multiplier)
        
        self.get_logger().info(f'Moving to {name.upper()} ({multiplier}x speed)')
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        pt = JointTrajectoryPoint()
        pt.positions = list(q[:6]) 
        pt.time_from_start = Duration(sec=int(effective_duration), nanosec=int((effective_duration % 1) * 1e9))
        goal.trajectory.points = [pt]
        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle and handle.accepted:
            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)

    def move_gripper(self, pos, name):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [pos, pos]
        pt.time_from_start = Duration(sec=1, nanosec=0)
        goal.trajectory.points = [pt]
        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle and handle.accepted:
            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)

    def execute_point(self, pos, label):
        while self.current_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        q = self.solve_ik(pos, relax_collisions=False)
        if not q: q = self.solve_ik(pos, relax_collisions=True)
        if q:
            self.move_arm(q, label)
            return True
        return False

    def run(self):
        self.last_valid_q = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self.move_arm(self.last_valid_q, 'home')
        
        self.move_gripper(0.04, 'open')
        pick_app = [PICK_POS[0], PICK_POS[1], PICK_POS[2] + OFFSET_Z]
        if not self.execute_point(pick_app, 'pick_approach'): return
        if not self.execute_point(PICK_POS, 'pick_goal'): return
        self.move_gripper(0.0, 'close')
        time.sleep(1.0)
        if not self.execute_point(pick_app, 'pick_away'): return
        
        if not self.execute_point(TRANSIT_POS, '90deg_transit'): return
        
        place_app = [PLACE_POS[0], PLACE_POS[1], PLACE_POS[2] + OFFSET_Z]
        if not self.execute_point(place_app, 'place_approach'): return
        if not self.execute_point(PLACE_POS, 'place_goal'): return
        self.move_gripper(0.04, 'open')
        time.sleep(1.0)
        if not self.execute_point(place_app, 'place_away'): return
        
        self.last_valid_q = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self.move_arm(self.last_valid_q, 'home (final)')

def main():
    rclpy.init()
    node = PickAndPlace90()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
