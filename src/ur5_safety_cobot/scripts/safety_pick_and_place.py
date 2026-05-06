#!/usr/bin/env python3
"""
safety_pick_and_place.py — UR5 looped pick-and-place with dynamic safety.

Identical operation to the original pick_and_place_loop.py but with
safety-aware speed control:
  - Reads `speed_multiplier` param every motion (set by safety_zone_manager)
  - speed_multiplier == 0.0 → PAUSE (spin-wait until human moves away)
  - speed_multiplier  < 1.0 → slower trajectories
  - speed_multiplier == 1.0 → normal operation

Uses URDF/meshes from ur5_description package.
"""
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
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R

# Coordinates
PICK_POS = [0.6, 0.0, 0.2]
PLACE_POS = [0.0, 0.6, 0.2]
TRANSIT_POS = [0.42, 0.42, 0.4]
OFFSET_Z = 0.2
TARGET_RPY = [1.5708, 1.5708, 0.0]

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

CUBE_URDF = (
    '<?xml version="1.0" ?><robot name="cube"><link name="link">'
    '<visual><geometry><box size="0.04 0.04 0.04"/></geometry>'
    '<material name="Red"><color rgba="1.0 0.0 0.0 1.0"/></material></visual>'
    '<collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision>'
    '<inertial><mass value="0.1"/>'
    '<inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>'
    '</inertial></link></robot>')


class SafetyPickAndPlace(Node):
    def __init__(self):
        super().__init__('safety_pnp_node')

        # Dynamic speed parameter (set by safety_zone_manager)
        self.declare_parameter('speed_multiplier', 1.0)

        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._spawn_client = self.create_client(SpawnEntity, '/world/empty/create')
        self._delete_client = self.create_client(DeleteEntity, '/world/empty/remove')
        self._joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        # Status publisher
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
        self.get_logger().info('Safety Pick-and-Place Node READY.')

    def _joint_cb(self, msg):
        self.current_joints = msg

    def _get_speed(self):
        return self.get_parameter('speed_multiplier').get_parameter_value().double_value

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _wait_if_stopped(self):
        """Spin-wait while speed_multiplier == 0.0 (STOP zone)."""
        while rclpy.ok():
            speed = self._get_speed()
            if speed > 0.0:
                return speed
            self._publish_status('PAUSED — human too close')
            self.get_logger().warn('PAUSED — waiting for human to move away...',
                                   throttle_duration_sec=3.0)
            rclpy.spin_once(self, timeout_sec=0.2)

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

    def solve_ik(self, pos, relax=True):
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = 'ur5_arm'
        ik_req.pose_stamped.header.frame_id = 'base_link'
        ik_req.pose_stamped.pose.position.x = pos[0]
        ik_req.pose_stamped.pose.position.y = pos[1]
        ik_req.pose_stamped.pose.position.z = pos[2]
        q = R.from_euler('xyz', TARGET_RPY).as_quat()
        ik_req.pose_stamped.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3])
        ik_req.avoid_collisions = not relax

        state = RobotState()
        if self.last_valid_q:
            state.joint_state.name = ARM_JOINTS
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
        # Check safety before every motion
        speed = self._wait_if_stopped()
        effective = max(0.5, base_duration / speed)

        self._publish_status(f'MOVING {name.upper()} (speed={speed:.2f}x)')
        self.get_logger().info(
            f'Moving to {name.upper()} (Speed: {speed}x, Duration: {effective:.2f}s)')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = list(q[:6])
        pt.time_from_start = Duration(
            sec=int(effective),
            nanosec=int((effective % 1) * 1e9))
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
        q = self.solve_ik(pos, relax=False)
        if not q:
            q = self.solve_ik(pos, relax=True)
        if not q:
            for _ in range(5):
                jiggle = [p + random.uniform(-0.01, 0.01) for p in pos]
                q = self.solve_ik(jiggle, relax=True)
                if q:
                    break
        if q:
            self.move_arm(q, label)
            return True
        return False

    def run(self):
        """Infinite pick-and-place loop. Never exits unless Ctrl+C / shutdown."""
        cycle = 1
        while rclpy.ok():
            self.get_logger().info(f'══════ Cycle #{cycle} ══════')
            self._publish_status(f'CYCLE {cycle} — starting')

            # ── 0. Reset cube at start of every cycle ─────────────────────
            self.reset_cube()

            # ── 1. Go HOME ────────────────────────────────────────────────
            self.last_valid_q = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            self.move_arm(self.last_valid_q, 'home')

            # ── 2. PICK sequence ──────────────────────────────────────────
            self.move_gripper(0.04, 'open')
            pick_app = [PICK_POS[0], PICK_POS[1], PICK_POS[2] + OFFSET_Z]
            if not self.execute_point(pick_app, 'pick_approach'):
                self.get_logger().warn(f'Cycle #{cycle}: IK fail at pick_approach — retrying cycle')
                continue
            if not self.execute_point(PICK_POS, 'pick_goal'):
                self.get_logger().warn(f'Cycle #{cycle}: IK fail at pick_goal — retrying cycle')
                continue
            self.move_gripper(0.0, 'close')
            time.sleep(1.0)
            if not self.execute_point(pick_app, 'pick_away'):
                self.get_logger().warn(f'Cycle #{cycle}: IK fail at pick_away — retrying cycle')
                continue

            # ── 3. TRANSIT ────────────────────────────────────────────────
            if not self.execute_point(TRANSIT_POS, 'transit'):
                self.get_logger().warn(f'Cycle #{cycle}: IK fail at transit — retrying cycle')
                continue

            # ── 4. PLACE sequence ─────────────────────────────────────────
            place_app = [PLACE_POS[0], PLACE_POS[1], PLACE_POS[2] + OFFSET_Z]
            if not self.execute_point(place_app, 'place_approach'):
                self.get_logger().warn(f'Cycle #{cycle}: IK fail at place_approach — retrying cycle')
                continue
            if not self.execute_point(PLACE_POS, 'place_goal'):
                self.get_logger().warn(f'Cycle #{cycle}: IK fail at place_goal — retrying cycle')
                continue
            self.move_gripper(0.04, 'open')
            time.sleep(1.0)
            if not self.execute_point(place_app, 'place_away'):
                self.get_logger().warn(f'Cycle #{cycle}: IK fail at place_away — retrying cycle')
                continue

            # ── 5. Cycle complete ─────────────────────────────────────────
            self._publish_status(f'CYCLE {cycle} — complete')
            self.get_logger().info(f'Cycle #{cycle} DONE ✓')
            cycle += 1


def main():
    rclpy.init()
    node = SafetyPickAndPlace()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
