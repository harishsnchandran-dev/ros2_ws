#!/usr/bin/env python3
"""
safety_dashboard.py — Terminal-based real-time safety dashboard.

Subscribes to:
  /safety_zone/status   — zone + distance + speed
  /safety_pnp/status    — PnP operational state
  /human_distance       — raw distance (cm)

Prints a live ASCII dashboard to the terminal.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import os


class SafetyDashboard(Node):
    def __init__(self):
        super().__init__('safety_dashboard')

        self.zone_status = 'WAITING...'
        self.pnp_status = 'WAITING...'
        self.raw_distance = 0.0

        self.create_subscription(String, '/safety_zone/status', self._zone_cb, 10)
        self.create_subscription(String, '/safety_pnp/status', self._pnp_cb, 10)
        self.create_subscription(Float32, '/human_distance', self._dist_cb, 10)
        self.create_timer(0.5, self._render)

        self.get_logger().info('Safety Dashboard started.')

    def _zone_cb(self, msg):
        self.zone_status = msg.data

    def _pnp_cb(self, msg):
        self.pnp_status = msg.data

    def _dist_cb(self, msg):
        self.raw_distance = msg.data

    def _render(self):
        # Determine zone for coloring
        zone = 'UNKNOWN'
        if 'NORMAL' in self.zone_status:
            zone = 'NORMAL'
        elif 'REDUCED' in self.zone_status:
            zone = 'REDUCED'
        elif 'STOP' in self.zone_status:
            zone = 'STOP'

        # ANSI colours
        GREEN = '\033[92m'
        YELLOW = '\033[93m'
        RED = '\033[91m'
        BOLD = '\033[1m'
        RESET = '\033[0m'
        CYAN = '\033[96m'

        colour = {
            'NORMAL': GREEN, 'REDUCED': YELLOW,
            'STOP': RED, 'UNKNOWN': RESET}[zone]

        bar_len = 40
        max_dist = 300.0
        fill = int(min(self.raw_distance / max_dist, 1.0) * bar_len)
        bar = '█' * fill + '░' * (bar_len - fill)

        os.system('clear' if os.name == 'posix' else 'cls')
        print(f"""
{BOLD}{CYAN}╔══════════════════════════════════════════════════════════╗
║          UR5  SAFETY  COBOT  DASHBOARD                 ║
╠══════════════════════════════════════════════════════════╣{RESET}
{BOLD}  Safety Zone:{RESET}   {colour}{BOLD}{zone}{RESET}
{BOLD}  Distance:{RESET}      {self.raw_distance:>7.1f} cm   [{bar}]
{BOLD}  Zone Detail:{RESET}   {self.zone_status}
{BOLD}  Robot State:{RESET}   {self.pnp_status}
{BOLD}{CYAN}╠══════════════════════════════════════════════════════════╣
║  {GREEN}■ NORMAL (>200cm){RESET}  {YELLOW}■ REDUCED (100-200cm){RESET}  {RED}■ STOP (<100cm){RESET} {CYAN}║
╚══════════════════════════════════════════════════════════╝{RESET}
""")


def main():
    rclpy.init()
    node = SafetyDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
