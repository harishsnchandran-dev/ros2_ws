#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class GaitController(Node):
    def __init__(self):
        super().__init__('gait_controller')

        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        self.vx = 0.4  # Slower default for stability
        self.wz = 0.0
        self.phase = 0.0

        # Physical Properties (Matching quadbot.xacro)
        self.l_coxa = 0.05
        self.l_femur = 0.10
        self.l_tibia = 0.15

        # Gait parameters - optimized for stability
        self.step_height = 0.03  # Lower lift
        self.step_length = 0.08  # Shorter stride
        self.nominal_z = -0.18   # Safe vertical depth
        self.nominal_r = 0.14    # Wider stance (crucial!)

        self.create_timer(0.04, self.tick) # 25 Hz
        self.get_logger().info('GaitController ready with IK Stance Stabilization!')

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def solve_ik(self, r, z):
        # 2-Link IK for Femur/Tibia
        # r = radius from coxa joint, z = depth (negative)
        d_sq = r**2 + z**2
        d = math.sqrt(d_sq)
        
        # Law of Cosines
        l1, l2 = self.l_femur, self.l_tibia
        try:
            cos_beta = (l1**2 + l2**2 - d_sq) / (2 * l1 * l2)
            beta = math.acos(max(-1.0, min(1.0, cos_beta)))
            
            cos_alpha = (l1**2 + d_sq - l2**2) / (2 * l1 * d)
            alpha = math.acos(max(-1.0, min(1.0, cos_alpha)))
            
            theta_femur = math.atan2(r, -z) - alpha
            theta_tibia = math.pi - beta
        except (ZeroDivisionError, ValueError):
            return 0.0, 0.0
            
        return theta_femur, theta_tibia

    def tick(self):
        speed = max(abs(self.vx), abs(self.wz))

        # Advance phase (time-based)
        if speed > 0.05:
            self.phase = (self.phase + speed * 0.2) % (2 * math.pi)
        else:
            # Return to neutral if stopped
            self.phase = 0.0

        # Tripod groups
        # Group A: RF, LM, RB. Group B: LF, RM, LB.
        def get_leg_angles(leg_idx, phase_offset):
            p = (self.phase + phase_offset) % (2 * math.pi)
            
            # Cartesian path (Leg Frame: X is world forward, Y is world sideways, Z is world vertical)
            # Lift is a semi-circle in X-Z plane during phase [0, pi]
            # Stance is a straight line back during phase [pi, 2pi]
            
            # Horizontal movement
            stride_x = math.cos(p) * self.step_length * 0.5 * self.vx
            stride_y = math.cos(p) * self.step_length * 0.5 * self.wz
            
            # Vertical lift
            if p < math.pi: # Swing phase
                dz = math.sin(p) * self.step_height
                dx = -math.cos(p) * self.step_length * 0.5 * self.vx
            else: # Stance phase
                dz = 0.0
                dx = (p - 1.5 * math.pi) / math.pi * self.step_length * self.vx
                # Simplification: just use the cosine for smooth back-and-forth
                dx = -stride_x
            
            # Target (r, z) in coxa-femur-tibia plane
            # We treat coxa yaw separately
            target_r = self.nominal_r + dx
            target_z = self.nominal_z + dz
            
            # IK
            femur, tibia = self.solve_ik(target_r, target_z)
            
            # Coxa yaw
            coxa = stride_y  # Rough approximation for turning
            
            return [coxa, femur, tibia]

        # Mapping (matches controllers.yaml order: rf, rm, rb, lf, lm, lb)
        def mirrored(angles):
            # Mirror the coxa yaw for the left side
            return [-angles[0], angles[1], angles[2]]

        # Order: rf, rm, rb, lf, lm, lb
        leg_rf = get_leg_angles(0, 0.0)
        leg_rm = get_leg_angles(1, math.pi)
        leg_rb = get_leg_angles(2, 0.0)
        leg_lf = mirrored(get_leg_angles(3, math.pi))
        leg_lm = mirrored(get_leg_angles(4, 0.0))
        leg_lb = mirrored(get_leg_angles(5, math.pi))

        msg = Float64MultiArray()
        msg.data = [float(v) for v in (leg_rf + leg_rm + leg_rb + leg_lf + leg_lm + leg_lb)]
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GaitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
