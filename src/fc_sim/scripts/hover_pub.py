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

        # Closed-loop altitude hold: PD on z with vz feedback.
        # Plant model: d_thrust_force / d_thrust_norm ~ 35.3 N (per
        # unit; 900 g/motor 2212-920KV 4S train), m = 1.182 kg, so the
        # loop's natural freq is omega_n = sqrt(35.3 * kp / m).
        # Critical-damping kd ~ 2*sqrt(kp). Norms below are the old
        # 600 g-train values rescaled by 0.667 so the commanded forces
        # are unchanged.
        self.declare_parameter("target_altitude", 2.0)
        self.declare_parameter("hover_thrust_norm", 0.333)
        self.declare_parameter("kp_alt", 0.02)
        self.declare_parameter("kd_alt", 0.067)
        self.declare_parameter("publish_hz", 100.0)
        self.declare_parameter("thrust_min", 0.27)
        self.declare_parameter("thrust_max", 0.50)
        # Takeoff burst: when the drone is below this altitude AND
        # almost stationary, command a stronger thrust to break static
        # ground contact friction. Once airborne, normal PD takes over.
        self.declare_parameter("takeoff_thrust_norm", 0.57)
        self.declare_parameter("takeoff_z_threshold", 0.30)

        self._target_alt = float(self.get_parameter("target_altitude").value)
        self._hover = float(self.get_parameter("hover_thrust_norm").value)
        self._kp = float(self.get_parameter("kp_alt").value)
        self._kd = float(self.get_parameter("kd_alt").value)
        self._thrust_min = float(self.get_parameter("thrust_min").value)
        self._thrust_max = float(self.get_parameter("thrust_max").value)
        self._takeoff_thrust = float(self.get_parameter("takeoff_thrust_norm").value)
        self._takeoff_z = float(self.get_parameter("takeoff_z_threshold").value)

        self._z: float = 0.0
        self._vz: float = 0.0
        self._have_odom = False
        self._log_counter = 0

        self._sub = self.create_subscription(
            Odometry, "/odom_truth", self._on_odom, 10
        )
        self._pub = self.create_publisher(Setpoint, "/fc/setpoint", 10)

        hz = float(self.get_parameter("publish_hz").value)
        self._timer = self.create_timer(1.0 / hz, self._on_tick)

        self.get_logger().info(
            f"hover_pub: target_alt={self._target_alt:.2f} m "
            f"hover_thrust={self._hover:.3f} kp={self._kp:.3f} kd={self._kd:.3f}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._z = float(msg.pose.pose.position.z)
        # Odometry's twist is body-frame; for a level drone body z = world z.
        # Once the drone tilts this is an approximation, but the hover demo
        # commands roll/pitch=0 so the approximation holds.
        self._vz = float(msg.twist.twist.linear.z)
        self._have_odom = True

    def _on_tick(self) -> None:
        if not self._have_odom:
            return

        err_z = self._target_alt - self._z

        # Takeoff burst breaks ground friction when the drone is sitting
        # on the floor. Plain PD with kp=0.03 at err=2 only gives
        # thrust=0.56 which gz physics holds against the ground; the
        # burst gives the rotors a chance to lift before the closed
        # loop pulls thrust back down.
        if self._z < self._takeoff_z and abs(self._vz) < 0.2 and err_z > 0.5:
            thrust = self._takeoff_thrust
        else:
            # PD: positive err -> boost thrust; positive vz -> brake.
            thrust = self._hover + self._kp * err_z - self._kd * self._vz

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

        self._log_counter += 1
        if self._log_counter >= 100:                # ~1 Hz logging at 100 Hz
            self._log_counter = 0
            self.get_logger().info(
                f"z={self._z:+.2f} vz={self._vz:+.2f} "
                f"err={err_z:+.2f} thrust={thrust:.3f}"
            )


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
