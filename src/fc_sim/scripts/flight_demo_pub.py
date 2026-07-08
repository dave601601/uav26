#!/usr/bin/env python3
"""Scripted forward/back/roll/yaw demo on top of altitude hold.

Reuses the same PD altitude-hold loop as hover_pub.py and overlays a
time-scripted sequence of attitude / yawrate commands so each axis is
exercised in isolation. Run after the sim is up.

Sequence (default ~5 s per phase, configurable via `phase_duration`):
  0. HOLD            — pure hover, drone reaches target_altitude.
  1. PITCH_FORWARD   — pitch_sp = +pitch_amp, drone slides +X (FLU).
  2. HOLD            — release pitch, drone decelerates.
  3. PITCH_BACK      — pitch_sp = -pitch_amp, drone slides -X.
  4. HOLD
  5. ROLL_RIGHT      — roll_sp = +roll_amp, drone slides -Y (FLU = right).
  6. HOLD
  7. ROLL_LEFT       — roll_sp = -roll_amp, drone slides +Y.
  8. HOLD
  9. YAW_RIGHT       — yawrate_sp = +yaw_amp, drone yaws right.
 10. YAW_LEFT        — yawrate_sp = -yaw_amp, drone yaws left.
 11. HOLD            — final hover.

Altitude hold runs continuously; the demo prints current phase + state
at 1 Hz.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from fc_sim_msgs.msg import Setpoint


@dataclass
class Phase:
    name: str
    duration_s: float
    roll_sp: float = 0.0
    pitch_sp: float = 0.0
    yawrate_sp: float = 0.0


def _build_sequence(phase_dur: float, pitch_amp: float,
                    roll_amp: float, yaw_amp: float) -> List[Phase]:
    return [
        Phase("HOLD",          phase_dur),
        Phase("PITCH_FORWARD", phase_dur, pitch_sp=+pitch_amp),
        Phase("HOLD",          phase_dur),
        Phase("PITCH_BACK",    phase_dur, pitch_sp=-pitch_amp),
        Phase("HOLD",          phase_dur),
        Phase("ROLL_RIGHT",    phase_dur, roll_sp=+roll_amp),
        Phase("HOLD",          phase_dur),
        Phase("ROLL_LEFT",     phase_dur, roll_sp=-roll_amp),
        Phase("HOLD",          phase_dur),
        Phase("YAW_RIGHT",     phase_dur, yawrate_sp=+yaw_amp),
        Phase("YAW_LEFT",      phase_dur, yawrate_sp=-yaw_amp),
        Phase("HOLD",          phase_dur),
    ]


class FlightDemo(Node):
    def __init__(self) -> None:
        super().__init__("flight_demo_pub")

        # Altitude hold (same plant constants as hover_pub.py).
        self.declare_parameter("target_altitude", 2.0)
        self.declare_parameter("hover_thrust_norm", 0.333)
        self.declare_parameter("kp_alt", 0.02)
        self.declare_parameter("kd_alt", 0.067)
        self.declare_parameter("thrust_min", 0.27)
        self.declare_parameter("thrust_max", 0.40)

        # Sequence shape.
        self.declare_parameter("phase_duration", 5.0)
        self.declare_parameter("pitch_amp", 0.05)
        self.declare_parameter("roll_amp",  0.05)
        self.declare_parameter("yaw_amp",   0.3)
        self.declare_parameter("publish_hz", 100.0)

        self._target_alt = float(self.get_parameter("target_altitude").value)
        self._hover = float(self.get_parameter("hover_thrust_norm").value)
        self._kp = float(self.get_parameter("kp_alt").value)
        self._kd = float(self.get_parameter("kd_alt").value)
        self._thrust_min = float(self.get_parameter("thrust_min").value)
        self._thrust_max = float(self.get_parameter("thrust_max").value)

        self._sequence = _build_sequence(
            phase_dur=float(self.get_parameter("phase_duration").value),
            pitch_amp=float(self.get_parameter("pitch_amp").value),
            roll_amp=float(self.get_parameter("roll_amp").value),
            yaw_amp=float(self.get_parameter("yaw_amp").value),
        )

        self._z: float = 0.0
        self._vz: float = 0.0
        self._x: float = 0.0
        self._y: float = 0.0
        self._qw: float = 1.0
        self._qx: float = 0.0
        self._qy: float = 0.0
        self._qz: float = 0.0
        self._have_odom = False

        self._phase_idx = 0
        self._phase_start = None
        self._log_counter = 0

        self._sub = self.create_subscription(
            Odometry, "/odom_truth", self._on_odom, 10
        )
        self._pub = self.create_publisher(Setpoint, "/fc/setpoint", 10)

        hz = float(self.get_parameter("publish_hz").value)
        self._timer = self.create_timer(1.0 / hz, self._on_tick)

        total = sum(p.duration_s for p in self._sequence)
        self.get_logger().info(
            f"flight_demo: target_alt={self._target_alt:.2f} m, "
            f"{len(self._sequence)} phases, ~{total:.0f} s total"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._x = float(msg.pose.pose.position.x)
        self._y = float(msg.pose.pose.position.y)
        self._z = float(msg.pose.pose.position.z)
        self._vz = float(msg.twist.twist.linear.z)
        # Track orientation so the log can surface flip-on-spawn issues.
        self._qw = float(msg.pose.pose.orientation.w)
        self._qx = float(msg.pose.pose.orientation.x)
        self._qy = float(msg.pose.pose.orientation.y)
        self._qz = float(msg.pose.pose.orientation.z)
        self._have_odom = True

    def _current_phase(self) -> Phase:
        return self._sequence[self._phase_idx]

    def _advance_phase_if_due(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._phase_start is None:
            self._phase_start = now
            return
        elapsed = now - self._phase_start
        if elapsed < self._current_phase().duration_s:
            return

        # Phase-1 (the initial HOLD) doubles as takeoff: stay here until
        # the drone is at target altitude AND vz is small. Otherwise the
        # next PITCH/ROLL command crashes the drone mid-climb.
        if self._phase_idx == 0:
            at_alt = abs(self._target_alt - self._z) < 0.2 and abs(self._vz) < 0.2
            if not at_alt:
                return  # stay in HOLD, recheck next tick

        if self._phase_idx + 1 < len(self._sequence):
            self._phase_idx += 1
            self._phase_start = now
            self.get_logger().info(
                f">> phase {self._phase_idx + 1}/{len(self._sequence)}: "
                f"{self._current_phase().name}"
            )
        # else: stay in the final HOLD phase

    def _on_tick(self) -> None:
        if not self._have_odom:
            return

        self._advance_phase_if_due()
        ph = self._current_phase()

        # Altitude PD.
        err_z = self._target_alt - self._z
        thrust = self._hover + self._kp * err_z - self._kd * self._vz
        thrust = max(self._thrust_min, min(self._thrust_max, thrust))

        sp = Setpoint()
        sp.mode = Setpoint.MODE_ATTITHR
        sp.arm = True
        sp.roll_sp = ph.roll_sp
        sp.pitch_sp = ph.pitch_sp
        sp.yawrate_sp = ph.yawrate_sp
        sp.vz_sp = 0.0
        sp.thrust_norm = float(thrust)
        self._pub.publish(sp)

        self._log_counter += 1
        if self._log_counter >= 100:                # ~1 Hz
            self._log_counter = 0
            self.get_logger().info(
                f"[{ph.name:<13}] xy=({self._x:+.2f},{self._y:+.2f}) "
                f"z={self._z:+.2f} vz={self._vz:+.2f} "
                f"q=({self._qw:+.2f},{self._qx:+.2f},{self._qy:+.2f},{self._qz:+.2f}) "
                f"thr={thrust:.3f} "
                f"sp=({ph.roll_sp:+.2f},{ph.pitch_sp:+.2f},{ph.yawrate_sp:+.2f})"
            )


def main() -> None:
    rclpy.init()
    node = FlightDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
