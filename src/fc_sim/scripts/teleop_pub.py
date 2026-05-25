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

import queue
import sys
import termios
import threading
import tty

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from fc_sim_msgs.msg import Setpoint


def echo(*args) -> None:
    """Force-flushed plain print, bypasses rclpy logger (which can route to
    stderr or get buffered by ros2 run's pipe). The teleop terminal only
    has one job — show key feedback immediately."""
    print(*args, flush=True)


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
        # waiting for Enter. A background daemon thread does the blocking
        # sys.stdin.read(1) and pushes chars into a queue, decoupling the
        # main rclpy spin thread from terminal I/O entirely.
        self._fd = sys.stdin.fileno()
        self._term_saved = None
        self._stdin_ok = False
        self._key_q: queue.Queue[str] = queue.Queue()
        if sys.stdin.isatty():
            try:
                self._term_saved = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
                self._stdin_ok = True
                self._reader = threading.Thread(
                    target=self._reader_loop, daemon=True
                )
                self._reader.start()
            except (termios.error, OSError) as e:
                echo(f"[teleop] cbreak setup failed: {e}")
        else:
            echo(
                "[teleop] stdin is NOT a TTY -- keyboard input will not work. "
                "Drop into the container with `docker compose run uav-aruco bash` "
                "first, then run `ros2 run fc_sim teleop_pub.py` inside the shell."
            )

        echo(
            f"[teleop] ready. stdin_ok={self._stdin_ok}. "
            "WASD=pitch/roll, QE=yaw, RF=altitude, "
            "space=level+hover, x=arm toggle, z or Ctrl-C to quit. "
            f"target_alt={self._target_alt:.2f} m, arm={self._arm}."
        )
        if self._stdin_ok:
            echo("[teleop] press any key now -- you should see a recv line.")

    def restore_term(self) -> None:
        if self._term_saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._term_saved)

    def _on_odom(self, msg: Odometry) -> None:
        self._z = msg.pose.pose.position.z
        self._vz = msg.twist.twist.linear.z
        self._have_odom = True

    def _reader_loop(self) -> None:
        """Blocking sys.stdin.read in a daemon thread. Each byte is echoed
        and queued for the main tick to consume. Exits when EOF (stdin
        closed) or on any exception."""
        try:
            while True:
                ch = sys.stdin.read(1)
                if not ch:
                    echo("[teleop] stdin EOF -- exiting reader")
                    return
                echo(f"[teleop] recv {ch!r}")
                self._key_q.put(ch)
        except Exception as e:
            echo(f"[teleop] reader thread died: {e}")

    def _read_key(self) -> str | None:
        try:
            return self._key_q.get_nowait()
        except queue.Empty:
            return None

    def _log_event(self, label: str) -> None:
        """Single-line event echo shown on every recognized key press, the
        SPACE/X/Z action keys, and auto-decay. Always reflects the current
        full setpoint so the user sees what the drone is being told to do.
        Uses raw print so it's visible even when rclpy logger output is
        routed somewhere else."""
        echo(
            f"[teleop] {label:<14s} | sp r={self._roll_sp:+.2f} "
            f"p={self._pitch_sp:+.2f} yawr={self._yawrate_sp:+.2f} | "
            f"alt_tgt={self._target_alt:.2f} arm={int(self._arm)}"
        )

    def _apply_key(self, ch: str) -> None:
        if ch == "w":
            self._pitch_sp = +PITCH_STEP
            self._log_event("W pitch+")
        elif ch == "s":
            self._pitch_sp = -PITCH_STEP
            self._log_event("S pitch-")
        elif ch == "a":
            self._roll_sp = -ROLL_STEP
            self._log_event("A roll-")
        elif ch == "d":
            self._roll_sp = +ROLL_STEP
            self._log_event("D roll+")
        elif ch == "q":
            self._yawrate_sp = -YAWRATE_STEP
            self._log_event("Q yaw-")
        elif ch == "e":
            self._yawrate_sp = +YAWRATE_STEP
            self._log_event("E yaw+")
        elif ch == "r":
            self._target_alt += ALT_STEP
            self._log_event("R alt+")
            return
        elif ch == "f":
            self._target_alt = max(0.5, self._target_alt - ALT_STEP)
            self._log_event("F alt-")
            return
        elif ch == " ":
            self._roll_sp = 0.0
            self._pitch_sp = 0.0
            self._yawrate_sp = 0.0
            self._log_event("SPACE level")
            return
        elif ch == "x":
            self._arm = not self._arm
            self._log_event("X arm toggle")
            return
        elif ch == "z" or ch == "\x03":   # z or Ctrl-C as a raw char
            self._log_event("Z quit")
            raise SystemExit(0)
        else:
            echo(f"[teleop] unknown key: {ch!r}")
            return
        self._last_cmd_ns = self.get_clock().now().nanoseconds

    def _tick(self) -> None:
        ch = self._read_key()
        if ch is not None:
            self._apply_key(ch)

        if self._last_cmd_ns is not None:
            age_s = (self.get_clock().now().nanoseconds - self._last_cmd_ns) * 1e-9
            if age_s > COMMAND_HOLD_S:
                had_setpoint = (self._roll_sp != 0.0
                                or self._pitch_sp != 0.0
                                or self._yawrate_sp != 0.0)
                self._roll_sp = 0.0
                self._pitch_sp = 0.0
                self._yawrate_sp = 0.0
                self._last_cmd_ns = None
                if had_setpoint:
                    self._log_event("auto-decay")

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
            echo(
                f"[teleop] status        | z={self._z:+.2f} "
                f"target={self._target_alt:.2f} "
                f"sp=(r={self._roll_sp:+.2f},p={self._pitch_sp:+.2f},"
                f"yawr={self._yawrate_sp:+.2f}) thr={thrust:.3f} "
                f"arm={int(self._arm)}"
            )


def main() -> None:
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.restore_term()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
