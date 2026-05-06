#!/usr/bin/env python3
"""
QUADBOT Autonomous Controller
Walks forward immediately. Steers away from obstacles using LIDAR.
Goal: traverse the arena from one end to the other.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.scan = None
        self.turn_dir = 1.0
        self.state = 'FORWARD'
        self.state_t = 0.0

        # Publish at 10 Hz immediately
        self.create_timer(0.1, self.tick)
        self.get_logger().info('AutonomousController started — walking forward!')

    def scan_callback(self, msg):
        self.scan = msg

    def sector_min(self, a_min_deg, a_max_deg):
        if self.scan is None:
            return float('inf')
        a0 = math.radians(a_min_deg)
        a1 = math.radians(a_max_deg)
        vals = []
        for i, r in enumerate(self.scan.ranges):
            a = self.scan.angle_min + i * self.scan.angle_increment
            if a0 <= a <= a1 and self.scan.range_min < r < self.scan.range_max:
                vals.append(r)
        return min(vals) if vals else float('inf')

    def tick(self):
        self.state_t += 0.1

        twist = Twist()

        front = self.sector_min(-30, 30)
        left  = self.sector_min(30, 90)
        right = self.sector_min(-90, -30)

        if self.state == 'FORWARD':
            if front > 0.8:
                twist.linear.x = 0.5
                # Gentle correction
                if left < 0.5:
                    twist.angular.z = -0.4
                elif right < 0.5:
                    twist.angular.z = 0.4
            else:
                self.turn_dir = 1.0 if left > right else -1.0
                self.state = 'TURN'
                self.state_t = 0.0
                self.get_logger().info(f'Obstacle! Turning {"left" if self.turn_dir > 0 else "right"}')

        elif self.state == 'TURN':
            twist.angular.z = 0.8 * self.turn_dir
            if front > 1.0 or self.state_t > 5.0:
                self.state = 'FORWARD'
                self.state_t = 0.0
                self.get_logger().info('Path clear, moving forward')

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
