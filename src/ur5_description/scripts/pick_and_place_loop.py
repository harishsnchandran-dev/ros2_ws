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
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import String
PICK_POS = [0.6, 0.0, 0.2] 
PLACE_POS = [0.0, 0.6, 0.2]
TRANSIT_POS = [0.42, 0.42, 0.4]
OFFSET_Z = 0.2

# MODIFIED ORIENTATION: Rotated Wrist 1 & 2 by 90 degrees (1.5708 rad)
# Standard downward was [0, 3.14, 0]
# Adding 90 deg offsets to create the user's desired approach angle
TARGET_RPY = [1.5708, 1.5708, 0.0] 

CUBE_URDF = '<?xml version="1.0" ?><robot name="cube"><link name="link"><visual><geometry><box size="0.04 0.04 0.04"/></geometry><material name="Red"><color rgba="1.0 0.0 0.0 1.0"/></material></visual><collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision><inertial><mass value="0.1"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial></link></robot>'

class PickAndPlaceLoop(Node):
    def __init__(self):
        super().__init__('pnp_loop')
        
        # ── Declare Dynamic Speed Parameter ──────────────────────────────────
        self.declare_parameter('speed_multiplier', 1.0)
        
        self._arm_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._spawn_client = self.create_client(SpawnEntity, '/world/empty/create')
        self._delete_client = self.create_client(DeleteEntity, '/world/empty/remove')
        self._joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self._status_pub = self.create_publisher(String, '/safety_pnp/status', 10)
        
        self.current_joints = None
        self.last_valid_q = None
        
        self.get_logger().info('Connecting to controllers...')
        self._arm_client.wait_for_server()
        self.get_logger().info('Arm server ready.')
        self._gripper_client.wait_for_server()
        self.get_logger().info('Gripper server ready.')
        self._spawn_client.wait_for_service()
        self.get_logger().info('Spawn service ready.')
        self._delete_client.wait_for_service()
        self.get_logger().info('Delete service ready.')
        self.get_logger().info('Looping P&P Node Ready (Dynamic Speed + Wrist Rotations).')

    def joint_cb(self, msg):
        self.current_joints = msg

    def reset_cube(self):
        del_req = DeleteEntity.Request()
        del_req.entity.name = 'cube'
        self._delete_client.call_async(del_req)
        time.sleep(1.0)
        spawn_req = SpawnEntity.Request()
        spawn_req.entity_factory.name = 'cube'
        spawn_req.entity_factory.sdf = CUBE_URDF
        spawn_req.entity_factory.pose.position.x = 0.6
        spawn_req.entity_factory.pose.position.y = 0.0
        spawn_req.entity_factory.pose.position.z = 1.27
        self._spawn_client.call_async(spawn_req)
        time.sleep(1.0)

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
        # ── Dynamic Velocity Scaling ─────────────────────────────────────────
        multiplier = self.get_parameter('speed_multiplier').get_parameter_value().double_value
        effective_duration = max(0.5, base_duration / multiplier)
        
        self.get_logger().info(f'Moving to {name.upper()} (Speed: {multiplier}x, Duration: {effective_duration:.2f}s)')
        
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
        if not q:
            for i in range(5):
                jiggle = [pos[0]+random.uniform(-0.01,0.01), pos[1]+random.uniform(-0.01,0.01), pos[2]+random.uniform(-0.01,0.01)]
                q = self.solve_ik(jiggle, relax_collisions=True)
                if q: break
        if q:
            self.move_arm(q, label)
            return True
        return False

    def _publish_status(self, msg_text):
        msg = String()
        msg.data = msg_text
        self._status_pub.publish(msg)

    def run(self):
        loop_count = 1
        while rclpy.ok():
            self.get_logger().info(f'--- Cycle #{loop_count} ---')
            self._publish_status(f'Cycle #{loop_count} - Initializing')
            self.last_valid_q = [0.0, -1.57, 0.0, -1.57, 3.14159, 0.0]
            self.move_arm(self.last_valid_q, 'home')
            
            self._publish_status(f'Cycle #{loop_count} - Picking')
            self.move_gripper(0.04, 'open')
            pick_app = [PICK_POS[0], PICK_POS[1], PICK_POS[2] + OFFSET_Z]
            if not self.execute_point(pick_app, 'pick_approach'): continue
            if not self.execute_point(PICK_POS, 'pick_goal'): continue
            self.move_gripper(0.0, 'close')
            time.sleep(1.0)
            if not self.execute_point(pick_app, 'pick_away'): continue
            
            if not self.execute_point(TRANSIT_POS, 'transit'): continue
            
            self._publish_status(f'Cycle #{loop_count} - Placing')
            place_app = [PLACE_POS[0], PLACE_POS[1], PLACE_POS[2] + OFFSET_Z]
            if not self.execute_point(place_app, 'place_approach'): continue
            if not self.execute_point(PLACE_POS, 'place_goal'): continue
            self.move_gripper(0.04, 'open')
            time.sleep(1.0)
            if not self.execute_point(place_app, 'place_away'): continue
            
            self._publish_status(f'Cycle #{loop_count} - Resetting Cube')
            self.reset_cube()
            loop_count += 1

def main():
    rclpy.init()
    node = PickAndPlaceLoop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
