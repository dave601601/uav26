#!/usr/bin/env python3
"""Keyboard teleop for fc_sim.

Publishes fc_sim_msgs/Setpoint on /fc/setpoint at 100 Hz. The user
drives the drone via WASD-style keys; altitude is held by an inline
PD against /odom_truth.

Keys:
  w / s  : pitch forward / back   (drone moves +x / -x in FLU)
  a / d  : roll left / right      (drone moves +y / -y in FLU)
  q / e  : yaw left / right
  r / f  : target altitude +/- 0.5 m
  space  : level + zero rates (auto-hover at current altitude)
  x      : arm / disarm toggle
  z, Ctrl-C : quit

Each key press asserts its setpoint for COMMAND_HOLD_S; the setpoint
then auto-decays to zero so the drone settles if you stop typing.
Altitude is held independently by a PD loop on /odom_truth.

Usage:
    Terminal A:  ros2 launch fc_sim teleop.launch.py
    Terminal B:  ros2 run fc_sim teleop_pub.py
"""
from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from fc_sim_msgs.msg import Setpoint


PITCH_STEP = 0.10        # rad
ROLL_STEP = 0.10         # rad
YAWRATE_STEP = 0.30      # rad/s
ALT_STEP = 0.5           # m, per R/F press
COMMAND_HOLD_S = 0.30    # how long a key's setpoint stays asserted
TICK_HZ = 100.0


class Teleop(Node):
    def __init__(self) -> None:
        super().__init__("teleop_pub")

        self.declare_parameter("target_altitude", 2.0)
        self.declare_parameter("hover_thrust_norm", 0.500)
        self.declare_parameter("kp_alt", 0.04)
        self.declare_parameter("kd_alt", 0.10)
        self.declare_parameter("thrust_min", 0.40)
        self.declare_parameter("thrust_max", 0.75)

        self._target_alt = float(self.get_parameter("target_altitude").value)
        self._hover = float(self.get_parameter("hover_thrust_norm").value)
        self._kp = float(self.get_parameter("kp_alt").value)
        self._kd = float(self.get_parameter("kd_alt").value)
        self._tmin = float(self.get_parameter("thrust_min").value)
        self._tmax = float(self.get_parameter("thrust_max").value)

        self._z = 0.0
        self._vz = 0.0
        self._have_odom = False

        self._roll_sp = 0.0
        self._pitch_sp = 0.0
        self._yawrate_sp = 0.0
        self._arm = True
        self._last_cmd_ns: int | None = None
        self._log_counter = 0

        self._sub = self.create_subscription(
            Odometry, "/odom_truth", self._on_odom, 10
        )
        self._pub = self.create_publisher(Setpoint, "/fc/setpoint", 10)
        self._timer = self.create_timer(1.0 / TICK_HZ, self._tick)

        # Put stdin in cbreak mode so single keystrokes arrive without
        # waiting for Enter, while leaving signals (Ctrl-C) intact.
        self._fd = sys.stdin.fileno()
        self._term_saved = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

        self.get_logger().info(
            "teleop_pub ready. WASD=pitch/roll, QE=yaw, RF=altitude, "
            "space=hover-level, x=arm toggle, z or Ctrl-C to quit. "
            f"target_alt={self._target_alt:.2f} m, arm={self._arm}"
        )

    def restore_term(self) -> None:
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._term_saved)

    def _on_odom(self, msg: Odometry) -> None:
        self._z = msg.pose.pose.position.z
        self._vz = msg.twist.twist.linear.z
        self._have_odom = True

    def _read_key(self) -> str | None:
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def _apply_key(self, ch: str) -> None:
        if ch == "w":
            self._pitch_sp = +PITCH_STEP
        elif ch == "s":
            self._pitch_sp = -PITCH_STEP
        elif ch == "a":
            self._roll_sp = -ROLL_STEP
        elif ch == "d":
            self._roll_sp = +ROLL_STEP
        elif ch == "q":
            self._yawrate_sp = -YAWRATE_STEP
        elif ch == "e":
            self._yawrate_sp = +YAWRATE_STEP
        elif ch == "r":
            self._target_alt += ALT_STEP
            self.get_logger().info(f"target_alt -> {self._target_alt:.2f} m")
            return
        elif ch == "f":
            self._target_alt = max(0.5, self._target_alt - ALT_STEP)
            self.get_logger().info(f"target_alt -> {self._target_alt:.2f} m")
            return
        elif ch == " ":
            self._roll_sp = 0.0
            self._pitch_sp = 0.0
            self._yawrate_sp = 0.0
            self.get_logger().info("level + zero rates")
            return
        elif ch == "x":
            self._arm = not self._arm
            self.get_logger().info(f"arm -> {self._arm}")
            return
        elif ch == "z":
            self.get_logger().info("quit")
            rclpy.shutdown()
            return
        else:
            return
        self._last_cmd_ns = self.get_clock().now().nanoseconds

    def _tick(self) -> None:
        ch = self._read_key()
        if ch is not None:
            self._apply_key(ch)

        if self._last_cmd_ns is not None:
            age_s = (self.get_clock().now().nanoseconds - self._last_cmd_ns) * 1e-9
            if age_s > COMMAND_HOLD_S:
                self._roll_sp = 0.0
                self._pitch_sp = 0.0
                self._yawrate_sp = 0.0

        if self._have_odom:
            err_z = self._target_alt - self._z
            thrust = self._hover + self._kp * err_z - self._kd * self._vz
            thrust = max(self._tmin, min(self._tmax, thrust))
        else:
            thrust = self._hover

        sp = Setpoint()
        sp.mode = Setpoint.MODE_ATTITHR
        sp.arm = self._arm
        sp.roll_sp = self._roll_sp
        sp.pitch_sp = self._pitch_sp
        sp.yawrate_sp = self._yawrate_sp
        sp.vz_sp = 0.0
        sp.thrust_norm = float(thrust)
        self._pub.publish(sp)

        self._log_counter += 1
        if self._log_counter >= int(TICK_HZ):  # 1 Hz
            self._log_counter = 0
            self.get_logger().info(
                f"z={self._z:+.2f} target={self._target_alt:.2f} "
                f"sp=(r={self._roll_sp:+.2f},p={self._pitch_sp:+.2f},"
                f"yawr={self._yawrate_sp:+.2f}) thr={thrust:.3f} "
                f"arm={int(self._arm)}"
            )


def main() -> None:
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_term()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
