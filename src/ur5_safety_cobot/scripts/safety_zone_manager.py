#!/usr/bin/env python3
"""
safety_zone_manager.py — Three-zone safety controller for UR5 cobot.

Subscribes to /human_distance (Float32, cm) and classifies into:
  NORMAL  (> 200 cm) → speed = 1.0
  REDUCED (100-200)  → speed = 0.3
  STOP    (< 100 cm) → speed = 0.0

Sets speed_multiplier param on PnP node via SetParameters service.
Fail-safe: STOP if no distance reading for > timeout seconds.
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import time

NORMAL = 'NORMAL'
REDUCED = 'REDUCED'
STOP = 'STOP'


class SafetyZoneManager(Node):
    def __init__(self):
        super().__init__('safety_zone_manager')

        # Declare parameters
        self.declare_parameter('stop_distance_cm', 100.0)
        self.declare_parameter('reduced_distance_cm', 200.0)
        self.declare_parameter('normal_speed', 1.0)
        self.declare_parameter('reduced_speed', 0.3)
        self.declare_parameter('stop_speed', 0.0)
        self.declare_parameter('distance_timeout_sec', 2.0)
        self.declare_parameter('zone_hysteresis_cm', 10.0)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('distance_topic', '/human_distance')
        self.declare_parameter('speed_param_target', '/safety_pnp_node')

        self.stop_dist = self.get_parameter('stop_distance_cm').value
        self.reduced_dist = self.get_parameter('reduced_distance_cm').value
        self.normal_speed = self.get_parameter('normal_speed').value
        self.reduced_speed = self.get_parameter('reduced_speed').value
        self.stop_speed = self.get_parameter('stop_speed').value
        self.timeout = self.get_parameter('distance_timeout_sec').value
        self.hysteresis = self.get_parameter('zone_hysteresis_cm').value
        self.update_rate = self.get_parameter('update_rate_hz').value
        dist_topic = self.get_parameter('distance_topic').value
        self.param_target = self.get_parameter('speed_param_target').value

        self.current_zone = NORMAL
        self.last_distance_cm = float('inf')
        self.last_distance_time = time.time()
        self.human_detected = False

        self._dist_sub = self.create_subscription(Float32, dist_topic, self._dist_cb, 10)
        self._zone_pub = self.create_publisher(String, '/safety_zone/status', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/safety_zone/markers', 10)
        self._param_client = self.create_client(
            SetParameters, f'{self.param_target}/set_parameters')

        self._timer = self.create_timer(1.0 / self.update_rate, self._evaluate)
        self.get_logger().info(
            f'SafetyZoneManager | STOP<{self.stop_dist}cm | '
            f'REDUCED<{self.reduced_dist}cm | NORMAL>={self.reduced_dist}cm')

    def _dist_cb(self, msg):
        # Filter out invalid readings (0 or near-zero = detector noise)
        if msg.data <= 10.0:
            self.get_logger().debug(
                f'Ignoring invalid distance: {msg.data:.1f} cm (< 10 cm minimum)')
            return
        self.last_distance_cm = msg.data
        self.last_distance_time = time.time()
        self.human_detected = True

    def _evaluate(self):
        elapsed = time.time() - self.last_distance_time

        if self.human_detected and elapsed > self.timeout:
            new_zone = STOP
            self.get_logger().warn(f'Timeout ({elapsed:.1f}s) → STOP (fail-safe)')
        elif not self.human_detected:
            new_zone = NORMAL
        else:
            d = self.last_distance_cm
            h = self.hysteresis
            if self.current_zone == NORMAL:
                if d < self.stop_dist:
                    new_zone = STOP
                elif d < self.reduced_dist:
                    new_zone = REDUCED
                else:
                    new_zone = NORMAL
            elif self.current_zone == REDUCED:
                if d < self.stop_dist - h:
                    new_zone = STOP
                elif d > self.reduced_dist + h:
                    new_zone = NORMAL
                else:
                    new_zone = REDUCED
            elif self.current_zone == STOP:
                if d > self.reduced_dist + h:
                    new_zone = NORMAL
                elif d > self.stop_dist + h:
                    new_zone = REDUCED
                else:
                    new_zone = STOP
            else:
                new_zone = STOP

        if new_zone != self.current_zone:
            self.get_logger().info(
                f'Zone: {self.current_zone} → {new_zone} '
                f'(dist={self.last_distance_cm:.1f}cm)')
            self.current_zone = new_zone

        speed_map = {NORMAL: self.normal_speed, REDUCED: self.reduced_speed, STOP: self.stop_speed}
        speed = speed_map[self.current_zone]
        self._set_speed(speed)

        status = String()
        status.data = f'{self.current_zone}|dist={self.last_distance_cm:.1f}cm|speed={speed:.2f}x'
        self._zone_pub.publish(status)
        self._publish_markers()

    def _set_speed(self, speed):
        if not self._param_client.service_is_ready():
            return
        req = SetParameters.Request()
        p = Parameter()
        p.name = 'speed_multiplier'
        p.value = ParameterValue()
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = speed
        req.parameters = [p]
        self._param_client.call_async(req)

    def _publish_markers(self):
        ma = MarkerArray()
        zones = [
            ('stop', self.stop_dist, (1.0, 0.0, 0.0, 0.15)),
            ('reduced', self.reduced_dist, (1.0, 0.65, 0.0, 0.10)),
            ('normal', 300.0, (0.0, 1.0, 0.0, 0.05)),
        ]
        for i, (name, rad, rgba) in enumerate(zones):
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'safety_zones'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.z = -0.5
            m.scale.x = rad / 100.0 * 2
            m.scale.y = rad / 100.0 * 2
            m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.lifetime = Duration(sec=0, nanosec=500_000_000)
            if (name == 'stop' and self.current_zone == STOP) or \
               (name == 'reduced' and self.current_zone == REDUCED) or \
               (name == 'normal' and self.current_zone == NORMAL):
                m.color.a = min(m.color.a * 4, 0.8)
            ma.markers.append(m)
        self._marker_pub.publish(ma)


def main():
    rclpy.init()
    node = SafetyZoneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
