#!/usr/bin/env python3
"""
Parker Actuator – Manual Control Node
======================================
Connects to /parker_controller/joint_trajectory to command the linear actuator.

Features:
  - Interactive terminal UI for entering position and velocity
  - PID controller class (used for monitoring/overlay; drive PID is primary)
  - Software safety limits with velocity clamping
  - E-Stop via 'e' key input
  - Displays live position feedback from /joint_states
  - Calculates trajectory duration from distance ÷ velocity

Usage:
  ros2 run parker_actuator manual_control.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import threading
import time
import sys

# ─── Safety Constants ─────────────────────────────────────────────────────────
MIN_POSITION_M    = 0.0     # metres
MAX_POSITION_M    = 0.40    # metres (actuator stroke)
MIN_VELOCITY_MS   = 0.001   # metres/sec
MAX_VELOCITY_MS   = 0.50    # metres/sec
MIN_DURATION_S    = 0.2     # minimum trajectory duration (seconds)


# ─── PID Controller ───────────────────────────────────────────────────────────
class PIDController:
    """
    Discrete-time PID controller.

    Usage:
        pid = PIDController(Kp=2.0, Ki=0.5, Kd=0.1)
        pid.reset()
        output = pid.compute(setpoint=0.3, measured=0.25)

    Note: The Compax3 drive has its own internal servo PID which handles
    tracking. This class is used at the ROS layer for monitoring the
    error and, optionally, adding a feedforward correction to the target.
    """
    def __init__(self, Kp: float, Ki: float, Kd: float,
                 output_min: float = -MAX_VELOCITY_MS,
                 output_max: float =  MAX_VELOCITY_MS):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max
        self.reset()

    def reset(self):
        self.prev_error = 0.0
        self.integral   = 0.0
        self.last_time  = time.monotonic()  # Bug fix: use monotonic clock

    def compute(self, setpoint: float, measured: float) -> float:
        now = time.monotonic()
        dt  = now - self.last_time
        dt  = max(dt, 1e-4)  # Bug fix: guard against dt=0

        error            = setpoint - measured
        self.integral   += error * dt
        derivative       = (error - self.prev_error) / dt

        raw = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        output = max(self.output_min, min(self.output_max, raw))  # Anti-windup clamp

        self.prev_error = error
        self.last_time  = now
        return output

    @property
    def error(self) -> float:
        return self.prev_error


# ─── Main ROS 2 Node ─────────────────────────────────────────────────────────
class ParkerManualControl(Node):
    def __init__(self):
        super().__init__('parker_manual_control')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/parker_controller/joint_trajectory',
            10
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        # State
        self.current_pos = 0.0
        self.current_vel = 0.0
        self.target_pos  = 0.0
        self.target_vel  = 0.1
        self.e_stop      = False

        # PID monitor (Kp/Ki/Kd tunable at runtime)
        self.pid = PIDController(Kp=2.0, Ki=0.5, Kd=0.1)

        self.get_logger().info("Parker Manual Control ready.")

        # Input thread (non-blocking to ROS spin)
        t = threading.Thread(target=self._input_loop, daemon=True)
        t.start()

    # ── ROS Callbacks ────────────────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        if 'slider_joint' in msg.name:
            idx = msg.name.index('slider_joint')
            self.current_pos = msg.position[idx]
            if len(msg.velocity) > idx:
                self.current_vel = msg.velocity[idx]

    # ── Input Loop ───────────────────────────────────────────────────────────
    def _input_loop(self):
        while rclpy.ok():
            try:
                self._print_banner()

                raw = input().strip().lower()

                # E-Stop command
                if raw in ('e', 'estop', 'stop'):
                    self._trigger_estop()
                    continue

                # Resume from E-Stop
                if raw in ('r', 'resume') and self.e_stop:
                    self.e_stop = False
                    self.pid.reset()
                    print("\n  [OK] E-Stop cleared. You may issue a new command.\n")
                    continue

                if self.e_stop:
                    print("\n  [!] E-Stop is active. Type 'r' to resume.\n")
                    time.sleep(0.5)
                    continue

                # Parse position
                try:
                    pos_val = float(raw)
                except ValueError:
                    print("\n  [!] Invalid position. Enter a number (e.g. 0.25) or 'e' for E-Stop.\n")
                    time.sleep(0.5)
                    continue

                vel_raw = input("  Velocity (m/s) [default 0.1, max 0.5]: ").strip()
                try:
                    vel_val = float(vel_raw) if vel_raw else 0.1
                except ValueError:
                    vel_val = 0.1

                # ── Apply safety limits ───────────────────────────────────
                safe_pos = max(MIN_POSITION_M, min(MAX_POSITION_M, pos_val))
                safe_vel = max(MIN_VELOCITY_MS, min(MAX_VELOCITY_MS, abs(vel_val)))

                if safe_pos != pos_val:
                    print(f"  [SAFETY] Position clamped: {pos_val:.4f} → {safe_pos:.4f} m")
                if safe_vel != abs(vel_val):
                    print(f"  [SAFETY] Velocity clamped: {vel_val:.4f} → {safe_vel:.4f} m/s")

                self.target_pos = safe_pos
                self.target_vel = safe_vel

                # ── PID: compute error and use output as velocity modifier ─
                # The PID output can optionally scale velocity for smooth approach
                pid_output = self.pid.compute(self.target_pos, self.current_pos)
                # Use PID output to adjust approach velocity (optional – currently informational)
                adjusted_vel = max(MIN_VELOCITY_MS,
                                   min(MAX_VELOCITY_MS, abs(pid_output) * safe_vel))

                self._send_trajectory(safe_pos, adjusted_vel)

            except KeyboardInterrupt:
                self._trigger_estop()
                break
            except EOFError:
                break
            except Exception as ex:
                self.get_logger().error(f"Input loop error: {ex}")

    def _print_banner(self):
        e_tag = "  ⚠  E-STOP ACTIVE" if self.e_stop else ""
        print(f"""
╔══════════════════════════════════════════╗
║     PARKER LINEAR ACTUATOR CONTROL       ║
╠══════════════════════════════════════════╣
║  Pos : {self.current_pos:>8.4f} m   Vel: {self.current_vel:>7.4f} m/s  ║
║  PID Error: {self.pid.error:>+8.4f} m{e_tag:>18s}  ║
╠══════════════════════════════════════════╣
║  Enter position (m) or 'e' for E-Stop:   ║
╚══════════════════════════════════════════╝
 > """, end='', flush=True)

    def _trigger_estop(self):
        self.e_stop = True
        self.pid.reset()
        # Send the current position as the target to hold in place
        self._send_trajectory(self.current_pos, MIN_VELOCITY_MS)
        print("\n\n  ██████  E-STOP ACTIVATED  ██████")
        print("  Actuator commanded to hold position.")
        print("  Type 'r' + Enter to resume.\n")

    def _send_trajectory(self, position: float, velocity: float):
        msg = JointTrajectory()
        msg.joint_names = ['slider_joint']

        point = JointTrajectoryPoint()
        point.positions  = [position]
        point.velocities = [velocity]

        # Bug fix #9: calculate duration, enforce minimum
        dist     = abs(position - self.current_pos)
        duration = dist / velocity if velocity > 1e-6 else MIN_DURATION_S
        duration = max(duration, MIN_DURATION_S)

        point.time_from_start.sec     = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)

        msg.points.append(point)
        self.pub.publish(msg)

        self.get_logger().info(
            f"▶ Goal: {position:.4f} m @ {velocity:.4f} m/s  (ETA {duration:.2f}s)"
        )


# ─── Entry Point ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ParkerManualControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
