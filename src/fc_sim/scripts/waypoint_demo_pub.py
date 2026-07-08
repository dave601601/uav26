#!/usr/bin/env python3
"""Waypoint follower for fc_sim.

Reads /odom_truth, runs a position PD that turns (x, y, z) error +
(vx, vy, vz) damping into (pitch, roll, thrust_norm) setpoints, and
publishes them as fc_sim_msgs/Setpoint at 100 Hz.

A waypoint list (`pattern` parameter) is walked from the first to the
last entry: the drone advances to the next waypoint once it is within
`reach_threshold` of the current one AND its translational speed has
dropped below `stable_speed`, or once `wp_timeout` seconds elapse on
the current waypoint (so a missed waypoint doesn't hang the demo).

Empirical sim sign conventions (confirmed in fc_sim progress notes):
    +pitch_sp -> drone slides +X
    +roll_sp  -> drone slides -Y

Controller is cascaded P-on-position -> P-on-velocity -> attitude:
    v_target_x = clamp(kp_pos * err_x, -v_max, +v_max)
    v_target_y = clamp(kp_pos * err_y, -v_max, +v_max)
    pitch_sp   = clamp(+kv * (v_target_x - vx), -max_tilt, +max_tilt)
    roll_sp    = clamp(-kv * (v_target_y - vy), -max_tilt, +max_tilt)
A pure position PD that hit max_tilt saturated forward acceleration
without enough brake authority and the drone routinely overshot by
10x.  The position-to-velocity outer loop caps the cruise speed so
the inner velocity loop has the headroom to actually stop.

Patterns:
    hover        : single waypoint at spawn altitude=2 m.
    forward_back : spawn -> +3 m in X -> back to spawn.
    box          : 3 m square around spawn, returns home.
    altitude     : climb to 3 m, descend to 1.5 m, return to 2 m.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from fc_sim_msgs.msg import Setpoint


SPAWN_X = 2.0   # competition.sdf spawn pose
SPAWN_Y = 4.0


@dataclass
class WP:
    x: float
    y: float
    z: float


PATTERNS: dict[str, list[WP]] = {
    "hover": [
        WP(SPAWN_X, SPAWN_Y, 2.0),
    ],
    "forward_back": [
        WP(SPAWN_X,       SPAWN_Y, 2.0),
        WP(SPAWN_X + 3.0, SPAWN_Y, 2.0),
        WP(SPAWN_X,       SPAWN_Y, 2.0),
    ],
    "box": [
        WP(SPAWN_X,       SPAWN_Y,       2.0),
        WP(SPAWN_X + 3.0, SPAWN_Y,       2.0),
        WP(SPAWN_X + 3.0, SPAWN_Y + 3.0, 2.0),
        WP(SPAWN_X,       SPAWN_Y + 3.0, 2.0),
        WP(SPAWN_X,       SPAWN_Y,       2.0),
    ],
    "altitude": [
        WP(SPAWN_X, SPAWN_Y, 2.0),
        WP(SPAWN_X, SPAWN_Y, 3.0),
        WP(SPAWN_X, SPAWN_Y, 1.5),
        WP(SPAWN_X, SPAWN_Y, 2.0),
    ],
}


class WaypointDemo(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_demo_pub")

        self.declare_parameter("pattern", "forward_back")
        self.declare_parameter("kp_pos", 0.70)       # outer P gain, 1/s
        self.declare_parameter("v_max", 0.50)        # cruise speed cap, m/s
        self.declare_parameter("kv", 0.50)           # inner P gain on velocity, s/m
        self.declare_parameter("max_tilt", 0.12)     # attitude cmd cap, rad
        self.declare_parameter("reach_threshold", 0.4)
        self.declare_parameter("stable_speed", 0.3)
        self.declare_parameter("wp_timeout", 25.0)
        self.declare_parameter("hover_thrust_norm", 0.333)
        self.declare_parameter("kp_alt", 0.067)
        self.declare_parameter("kd_alt", 0.167)
        self.declare_parameter("thrust_min", 0.27)
        self.declare_parameter("thrust_max", 0.47)

        pattern = str(self.get_parameter("pattern").value)
        if pattern not in PATTERNS:
            self.get_logger().error(
                f"unknown pattern '{pattern}'. options: {list(PATTERNS)}"
            )
            raise SystemExit(1)
        self._waypoints = PATTERNS[pattern]
        self._wp_idx = 0
        self._wp_start_s: float | None = None

        self._kp_pos = float(self.get_parameter("kp_pos").value)
        self._v_max = float(self.get_parameter("v_max").value)
        self._kv = float(self.get_parameter("kv").value)
        self._max_tilt = float(self.get_parameter("max_tilt").value)
        self._reach = float(self.get_parameter("reach_threshold").value)
        self._stable_v = float(self.get_parameter("stable_speed").value)
        self._wp_timeout = float(self.get_parameter("wp_timeout").value)
        self._hover = float(self.get_parameter("hover_thrust_norm").value)
        self._kp_alt = float(self.get_parameter("kp_alt").value)
        self._kd_alt = float(self.get_parameter("kd_alt").value)
        self._tmin = float(self.get_parameter("thrust_min").value)
        self._tmax = float(self.get_parameter("thrust_max").value)

        self._x = self._y = self._z = 0.0
        self._vx = self._vy = self._vz = 0.0
        self._have_odom = False
        self._log_counter = 0

        self._sub = self.create_subscription(
            Odometry, "/odom_truth", self._on_odom, 10
        )
        self._pub = self.create_publisher(Setpoint, "/fc/setpoint", 10)
        self._timer = self.create_timer(0.01, self._tick)

        self.get_logger().info(
            f"waypoint_demo: pattern={pattern}, {len(self._waypoints)} waypoints, "
            f"kp_pos={self._kp_pos} v_max={self._v_max} kv={self._kv} "
            f"max_tilt={math.degrees(self._max_tilt):.1f} deg, "
            f"reach={self._reach} m, stable_v={self._stable_v} m/s, "
            f"timeout={self._wp_timeout} s"
        )
        for i, wp in enumerate(self._waypoints):
            self.get_logger().info(
                f"  WP {i}: ({wp.x:+.2f}, {wp.y:+.2f}, {wp.z:+.2f})"
            )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self._x, self._y, self._z = p.x, p.y, p.z
        self._vx, self._vy, self._vz = v.x, v.y, v.z
        self._have_odom = True

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _current_wp(self) -> WP:
        return self._waypoints[self._wp_idx]

    def _maybe_advance(self) -> None:
        if self._wp_start_s is None:
            return
        if self._wp_idx >= len(self._waypoints) - 1:
            return                              # park at last waypoint
        wp = self._current_wp()
        dx, dy, dz = wp.x - self._x, wp.y - self._y, wp.z - self._z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        speed = math.sqrt(self._vx ** 2 + self._vy ** 2 + self._vz ** 2)
        elapsed = self._now_s() - self._wp_start_s

        reached = dist < self._reach and speed < self._stable_v
        timed_out = elapsed > self._wp_timeout
        if not (reached or timed_out):
            return

        tag = "reached" if reached else "TIMEOUT"
        self.get_logger().info(
            f"WP {self._wp_idx} {tag} at "
            f"({self._x:+.2f},{self._y:+.2f},{self._z:+.2f}) "
            f"dist={dist:.2f} speed={speed:.2f} after {elapsed:.1f}s"
        )
        self._wp_idx += 1
        self._wp_start_s = self._now_s()
        nw = self._current_wp()
        self.get_logger().info(
            f">> WP {self._wp_idx}: ({nw.x:+.2f},{nw.y:+.2f},{nw.z:+.2f})"
        )

    def _tick(self) -> None:
        if not self._have_odom:
            return
        if self._wp_start_s is None:
            self._wp_start_s = self._now_s()
            wp0 = self._current_wp()
            self.get_logger().info(
                f">> WP 0: ({wp0.x:+.2f},{wp0.y:+.2f},{wp0.z:+.2f})"
            )

        self._maybe_advance()
        wp = self._current_wp()

        err_x = wp.x - self._x
        err_y = wp.y - self._y
        err_z = wp.z - self._z

        # Outer P: position error -> commanded velocity (saturated to v_max).
        v_tgt_x = max(-self._v_max, min(self._v_max, self._kp_pos * err_x))
        v_tgt_y = max(-self._v_max, min(self._v_max, self._kp_pos * err_y))

        # Inner P: velocity error -> attitude. +pitch_sp drives +X (FLU),
        # +roll_sp drives -Y, so y sign is flipped.
        pitch_sp = +self._kv * (v_tgt_x - self._vx)
        roll_sp = -self._kv * (v_tgt_y - self._vy)
        pitch_sp = max(-self._max_tilt, min(self._max_tilt, pitch_sp))
        roll_sp = max(-self._max_tilt, min(self._max_tilt, roll_sp))

        thrust = self._hover + self._kp_alt * err_z - self._kd_alt * self._vz
        thrust = max(self._tmin, min(self._tmax, thrust))

        sp = Setpoint()
        sp.mode = Setpoint.MODE_ATTITHR
        sp.arm = True
        sp.roll_sp = roll_sp
        sp.pitch_sp = pitch_sp
        sp.yawrate_sp = 0.0
        sp.vz_sp = 0.0
        sp.thrust_norm = float(thrust)
        self._pub.publish(sp)

        self._log_counter += 1
        if self._log_counter >= 100:    # 1 Hz
            self._log_counter = 0
            dist = math.sqrt(err_x ** 2 + err_y ** 2 + err_z ** 2)
            self.get_logger().info(
                f"WP{self._wp_idx} tgt=({wp.x:+.2f},{wp.y:+.2f},{wp.z:+.2f}) "
                f"pos=({self._x:+.2f},{self._y:+.2f},{self._z:+.2f}) "
                f"v=({self._vx:+.2f},{self._vy:+.2f},{self._vz:+.2f}) "
                f"dist={dist:.2f} "
                f"sp=(r={roll_sp:+.2f},p={pitch_sp:+.2f}) thr={thrust:.3f}"
            )


def main() -> None:
    rclpy.init()
    node = WaypointDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
