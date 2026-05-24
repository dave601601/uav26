#!/usr/bin/env python3
"""Closed-loop hover publisher for the open-loop sim demo.

Subscribes /odom_truth (ground-truth z), publishes fc_sim_msgs/Setpoint
at 100 Hz with a P-controlled thrust_norm so the drone holds a target
altitude. roll/pitch/yawrate stay at zero — this is a vertical-only
hold, not a position hold. Horizontal drift will accumulate; for
horizontal hold use line_tracer.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from fc_sim_msgs.msg import Setpoint


class HoverPub(Node):
    def __init__(self) -> None:
        super().__init__("hover_pub")

        self.declare_parameter("target_altitude", 2.0)
        self.declare_parameter("hover_thrust_norm", 0.500)
        self.declare_parameter("kp_alt", 0.04)
        self.declare_parameter("publish_hz", 100.0)
        self.declare_parameter("thrust_min", 0.45)
        self.declare_parameter("thrust_max", 0.60)

        self._target_alt = float(self.get_parameter("target_altitude").value)
        self._hover = float(self.get_parameter("hover_thrust_norm").value)
        self._kp = float(self.get_parameter("kp_alt").value)
        self._thrust_min = float(self.get_parameter("thrust_min").value)
        self._thrust_max = float(self.get_parameter("thrust_max").value)

        self._z: float = 0.0
        self._have_odom = False

        self._sub = self.create_subscription(
            Odometry, "/odom_truth", self._on_odom, 10
        )
        self._pub = self.create_publisher(Setpoint, "/fc/setpoint", 10)

        hz = float(self.get_parameter("publish_hz").value)
        self._timer = self.create_timer(1.0 / hz, self._on_tick)

        self.get_logger().info(
            f"hover_pub: target_alt={self._target_alt:.2f} m "
            f"hover_thrust={self._hover:.3f} kp_alt={self._kp:.3f}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._z = float(msg.pose.pose.position.z)
        self._have_odom = True

    def _on_tick(self) -> None:
        if not self._have_odom:
            return

        err = self._target_alt - self._z
        thrust = self._hover + self._kp * err
        thrust = max(self._thrust_min, min(self._thrust_max, thrust))

        sp = Setpoint()
        sp.mode = Setpoint.MODE_ATTITHR
        sp.arm = True
        sp.roll_sp = 0.0
        sp.pitch_sp = 0.0
        sp.yawrate_sp = 0.0
        sp.vz_sp = 0.0
        sp.thrust_norm = float(thrust)
        self._pub.publish(sp)


def main() -> None:
    rclpy.init()
    node = HoverPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
