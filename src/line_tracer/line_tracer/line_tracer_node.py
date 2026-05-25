"""line_tracer_node — perception + dead-reckoning + cmd_vel publisher.

Pulls camera (color, aligned-depth, info) from `/camera/camera/...`, runs
`perception.process_image`, converts pixel errors → body-frame metric
offsets using the depth-estimated altitude + camera intrinsics, then a
P-controller in `dead_reckoning` produces a body Twist on `/cmd_vel`.

Topics
------
Subscribed
  /camera/camera/color/image_raw                 sensor_msgs/Image
  /camera/camera/aligned_depth_to_color/image_raw sensor_msgs/Image
  /camera/camera/color/camera_info               sensor_msgs/CameraInfo
  /line_tracer/pixel_error                       geometry_msgs/Vector3
      External override of perception's (du, dv, psi_err). Used while
      perception is being tuned and as a unit-test handle.

Published
  /cmd_vel                          geometry_msgs/Twist  (body FLU)
  /odom_dr                          nav_msgs/Odometry    (world ENU)
  /waypoints/aruco                  visualization_msgs/MarkerArray
  /line_tracer/debug_image          sensor_msgs/Image    (BGR8 overlay)

Services
  /line_tracer/set_state            line_tracer_msgs/SetState

When the drone's flight controller is replaced (real STM32 instead of the
Gazebo fake FC), the only change here is what subscribes to `/cmd_vel`.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

try:
    from fc_sim_msgs.msg import Setpoint
except ImportError:                       # pragma: no cover
    Setpoint = None                       # type: ignore[assignment]
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

try:
    from line_tracer_msgs.srv import SetState
except ImportError:                       # pragma: no cover
    SetState = None                       # type: ignore[assignment]

from .dead_reckoning import (
    DeadReckoning,
    Gains,
    SetpointGains,
    State,
    body_vel_to_atti_thr,
    wrap_angle,
    world_to_body,
)
from .geom import CameraIntrinsics
from .grid import Grid
from .perception import PerceptionConfig, PerceptionResult, process_image, draw_debug_overlay
from .state_machine import MissionContext, StateMachine, StateName


# QoS for sensor streams: best-effort + small queue, matches realsense_camera defaults.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def _depth_to_meters(arr: np.ndarray) -> np.ndarray:
    """RealSense (16UC1, mm) and Gazebo sim (32FC1, m) co-existence."""
    if arr.dtype == np.uint16:
        return arr.astype(np.float32) * 1e-3
    return arr.astype(np.float32, copy=False)


def _central_median_depth(depth_m: np.ndarray, half_size: int = 8) -> Optional[float]:
    """Median of the central (2*half_size)x(2*half_size) window, ignoring
    non-positive / non-finite samples. Returns None if the window is empty."""
    h, w = depth_m.shape[:2]
    cy, cx = h // 2, w // 2
    patch = depth_m[
        max(0, cy - half_size): cy + half_size,
        max(0, cx - half_size): cx + half_size,
    ]
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


class LineTracerNode(Node):
    def __init__(self) -> None:
        super().__init__("line_tracer_node")

        # --- parameters (declared with defaults, overridable via params.yaml) -
        self.declare_parameter("target_altitude", 2.0)
        self.declare_parameter("kp_xy", 0.8)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("kp_z", 0.6)
        self.declare_parameter("max_vxy", 1.0)
        self.declare_parameter("max_wz", 1.0)
        self.declare_parameter("dr_dt", 0.05)            # 20 Hz integrator
        self.declare_parameter("external_override_ttl", 0.5)
        self.declare_parameter("publish_debug_image", True)
        # FC setpoint shaping. hover_thrust_norm sits a hair above the sim's
        # empirical hover point (~0.50) so the altitude controller defaults
        # to "slightly climbing" and the P+D fight altitude error harder.
        # thrust_min/_max clamp to the same band flight_demo uses (proven
        # stable with the firmware's 0.80 atti gain): the unclamped P term
        # at start-up alt_err=2 m would command full thrust 0.97 and the
        # high-gain attitude loop slams into limits before settling.
        self.declare_parameter("hover_thrust_norm", 0.52)
        self.declare_parameter("kp_alt_thrust", 0.25)        # thrust_norm per metre
        self.declare_parameter("kd_alt_thrust", 0.30)        # thrust_norm per (m/s vz)
        self.declare_parameter("thrust_min", 0.42)
        self.declare_parameter("thrust_max", 0.70)
        self.declare_parameter("max_atti_setpoint_rad", 0.15)   # ~8.6°
        # Takeoff burst: open-loop thrust to break ground contact when
        # the drone is sitting on the floor. Mirrors hover_pub.py.
        self.declare_parameter("takeoff_z_threshold", 0.30)
        self.declare_parameter("takeoff_thrust_norm", 0.85)
        # /odom_truth sanity gates: DartSim occasionally spits garbage
        # contact frames (|z| in the millions, |vz| in the thousands).
        # If those frames are accepted, the kd_alt_thrust term blows up
        # and thrust oscillates between thrust_min and thrust_max — the
        # primary cascade behind the 2026-05-25 ground-stick failure.
        self.declare_parameter("odom_truth_max_alt", 50.0)
        self.declare_parameter("odom_truth_max_vz", 30.0)
        # In sim the proposal's LIDAR-based Z estimator is not implemented;
        # /odom_truth substitutes for the lidar measurement. Set false on
        # real hardware so the depth-camera median path is used instead.
        self.declare_parameter("use_odom_truth_altitude", True)
        # perception
        self.declare_parameter("canny_low", 60)
        self.declare_parameter("canny_high", 180)
        self.declare_parameter("hough_threshold", 60)
        self.declare_parameter("hough_min_line_length", 40)
        self.declare_parameter("hough_max_line_gap", 20)
        self.declare_parameter("marker_size", 0.5)
        # depth fallback altitude when no depth has arrived yet (TAKEOFF init)
        self.declare_parameter("default_altitude", 0.0)
        self.declare_parameter("altitude_median_window", 5)
        # Mission FSM grid / context knobs.
        self.declare_parameter("grid_width", 30.0)
        self.declare_parameter("grid_depth", 20.0)
        self.declare_parameter("grid_cell", 4.0)
        self.declare_parameter("mission_max_records", 4)
        self.declare_parameter("waypoint_hover_seconds", 1.5)
        self.declare_parameter("waypoint_arrival_dist", 0.6)
        self.declare_parameter("return_arrival_dist", 0.4)
        self.declare_parameter("takeoff_alt_threshold", 1.8)
        self.declare_parameter("snap_max_err", 2.0)

        target_alt = float(self.get_parameter("target_altitude").value)
        self._gains = Gains(
            kp_xy=float(self.get_parameter("kp_xy").value),
            kp_yaw=float(self.get_parameter("kp_yaw").value),
            kp_z=float(self.get_parameter("kp_z").value),
            max_vxy=float(self.get_parameter("max_vxy").value),
            max_wz=float(self.get_parameter("max_wz").value),
            target_altitude=target_alt,
        )
        self._dr = DeadReckoning(self._gains, State())
        grid = Grid.from_extents(
            width=float(self.get_parameter("grid_width").value),
            depth=float(self.get_parameter("grid_depth").value),
            cell=float(self.get_parameter("grid_cell").value),
        )
        mission_ctx = MissionContext(
            grid=grid,
            max_records=int(self.get_parameter("mission_max_records").value),
            takeoff_alt_threshold=float(
                self.get_parameter("takeoff_alt_threshold").value
            ),
            waypoint_hover_seconds=float(
                self.get_parameter("waypoint_hover_seconds").value
            ),
            waypoint_arrival_dist=float(
                self.get_parameter("waypoint_arrival_dist").value
            ),
            return_arrival_dist=float(
                self.get_parameter("return_arrival_dist").value
            ),
            snap_max_err=float(self.get_parameter("snap_max_err").value),
        )
        self._fsm = StateMachine(
            initial=StateName.TAKEOFF,
            target_altitude=target_alt,
            context=mission_ctx,
        )

        self._perception_cfg = PerceptionConfig(
            canny_low=int(self.get_parameter("canny_low").value),
            canny_high=int(self.get_parameter("canny_high").value),
            hough_threshold=int(self.get_parameter("hough_threshold").value),
            hough_min_line_length=int(self.get_parameter("hough_min_line_length").value),
            hough_max_line_gap=int(self.get_parameter("hough_max_line_gap").value),
        )

        self._bridge = CvBridge()

        # --- latched state ---------------------------------------------------
        self._intrinsics: Optional[CameraIntrinsics] = None
        self._altitude_m: Optional[float] = (
            float(self.get_parameter("default_altitude").value) or None
        )
        # Rolling median window for altitude — kills single-frame depth
        # outliers (occasional NaN clusters from the sim depth camera).
        win = max(1, int(self.get_parameter("altitude_median_window").value))
        self._altitude_window: Deque[float] = deque(maxlen=win)
        self._latest_perception: Optional[PerceptionResult] = None
        self._external_pixel_error: Optional[Vector3] = None
        self._external_pixel_error_t: Optional[float] = None
        self._fsm_state_prev: Optional[StateName] = None

        # FC setpoint shaping constants (cached from params)
        self._hover_thrust_norm = float(self.get_parameter("hover_thrust_norm").value)
        self._kp_alt_thrust = float(self.get_parameter("kp_alt_thrust").value)
        self._kd_alt_thrust = float(self.get_parameter("kd_alt_thrust").value)
        self._thrust_min = float(self.get_parameter("thrust_min").value)
        self._thrust_max = float(self.get_parameter("thrust_max").value)
        self._max_atti_sp = float(self.get_parameter("max_atti_setpoint_rad").value)
        self._takeoff_z_threshold = float(
            self.get_parameter("takeoff_z_threshold").value
        )
        self._takeoff_thrust_norm = float(
            self.get_parameter("takeoff_thrust_norm").value
        )
        self._odom_truth_max_alt = float(
            self.get_parameter("odom_truth_max_alt").value
        )
        self._odom_truth_max_vz = float(
            self.get_parameter("odom_truth_max_vz").value
        )
        self._use_odom_truth_altitude = bool(
            self.get_parameter("use_odom_truth_altitude").value
        )
        self._latest_vz: float = 0.0    # for kd_alt damping
        self._truth_x: float = 0.0      # for the throttled status log + plot
        self._truth_y: float = 0.0
        # Previous (alt, t) pair for the world-Z finite-difference that
        # backs `_latest_vz`. Using a derivative of position keeps the
        # damping in world frame regardless of how gz-sim tags
        # twist.twist.linear.z.
        self._odom_truth_prev_z: float = 0.0
        self._odom_truth_prev_t: Optional[float] = None
        # Sim-only yaw inject: the firmware's residual mixer asymmetry
        # rotates the actual drone over a takeoff (~0.4 rad in r17), but
        # line_tracer has no IMU subscription so DR.yaw never sees it.
        # The yaw lock can't fight a drift it can't observe. /odom_truth
        # orientation is the sim stand-in for the proposal's eventual
        # IMU-derived heading. Real-flight builds set
        # use_odom_truth_altitude=false and this path is bypassed.
        self._odom_truth_yaw: Optional[float] = None

        # --- pubs/subs/srvs --------------------------------------------------
        # The flight controller (fc_sim_node in sim, real STM32 over USART2
        # later) consumes attitude/thrust setpoints, not body-frame Twist.
        # _build_setpoint() maps the planner's body-velocity intent through
        # a small-angle attitude map + altitude-hold P controller.
        if Setpoint is None:
            self.get_logger().warn(
                "fc_sim_msgs not available; falling back to /cmd_vel Twist."
            )
            self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
            self._setpoint_pub = False
        else:
            self._pub_cmd = self.create_publisher(Setpoint, "/fc/setpoint", 10)
            self._setpoint_pub = True
        self._pub_odom = self.create_publisher(Odometry, "/odom_dr", 10)
        self._pub_markers = self.create_publisher(
            MarkerArray, "/waypoints/aruco", 10
        )
        self._pub_debug = self.create_publisher(
            Image, "/line_tracer/debug_image", 10
        )

        self._sub_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self._on_color, SENSOR_QOS
        )
        self._sub_depth = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self._on_depth,
            SENSOR_QOS,
        )
        self._sub_info = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._on_camera_info,
            SENSOR_QOS,
        )
        self._sub_pixel_err = self.create_subscription(
            Vector3, "/line_tracer/pixel_error", self._on_pixel_error_external, 10
        )
        # /odom_truth is the sim stand-in for the lidar+IMU Z estimator
        # called out in the team proposal. Real-flight builds set
        # use_odom_truth_altitude=false and rely on the depth-camera median
        # (or, eventually, the proposal's 2-state KF on lidar).
        if self._use_odom_truth_altitude:
            self._sub_odom_truth = self.create_subscription(
                Odometry, "/odom_truth", self._on_odom_truth, 10
            )

        if SetState is not None:
            self._srv_set_state = self.create_service(
                SetState, "/line_tracer/set_state", self._handle_set_state
            )
        else:                           # pragma: no cover
            self.get_logger().warn(
                "line_tracer_msgs not available; /line_tracer/set_state disabled"
            )

        dt = float(self.get_parameter("dr_dt").value)
        self._dr_dt = dt
        self._timer = self.create_timer(dt, self._on_dr_tick)

        self.get_logger().info(
            f"line_tracer_node up (state={self._fsm.state.name}, "
            f"target_alt={target_alt}, dr_dt={dt})"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._intrinsics is None:
            self._intrinsics = CameraIntrinsics.from_camera_info(msg)
            self.get_logger().info(
                f"camera_info: fx={self._intrinsics.fx:.2f} fy={self._intrinsics.fy:.2f} "
                f"cx={self._intrinsics.cx:.2f} cy={self._intrinsics.cy:.2f} "
                f"size={self._intrinsics.width}x{self._intrinsics.height}"
            )

    def _on_depth(self, msg: Image) -> None:
        # When /odom_truth is the truth source (sim path) the depth camera
        # contribution would overwrite it on every callback — guard.
        if self._use_odom_truth_altitude:
            return
        try:
            arr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:                                # pragma: no cover
            self.get_logger().warn(f"depth conversion failed: {e}")
            return
        depth_m = _depth_to_meters(np.asarray(arr))
        alt = _central_median_depth(depth_m)
        if alt is None:
            return
        self._altitude_window.append(alt)
        # Median of the rolling window — kills single-frame outliers.
        vals = sorted(self._altitude_window)
        self._altitude_m = vals[len(vals) // 2]

    def _on_color(self, msg: Image) -> None:
        if self._intrinsics is None:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:                                # pragma: no cover
            self.get_logger().warn(f"color conversion failed: {e}")
            return
        result = process_image(bgr, self._intrinsics, self._perception_cfg)
        self._latest_perception = result

        # Publish ArUco waypoints in world frame (using current DR estimate).
        if result.aruco:
            self._publish_aruco_markers(result, msg.header.stamp)

        if bool(self.get_parameter("publish_debug_image").value):
            try:
                debug_bgr = draw_debug_overlay(bgr, self._intrinsics, result)
                debug_msg = self._bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
                debug_msg.header = msg.header
                self._pub_debug.publish(debug_msg)
            except Exception as e:                            # pragma: no cover
                self.get_logger().warn(f"debug publish failed: {e}")

    def _on_pixel_error_external(self, msg: Vector3) -> None:
        self._external_pixel_error = msg
        self._external_pixel_error_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_odom_truth(self, msg: Odometry) -> None:
        """Sim-only altitude + vz override.

        Real-flight builds set use_odom_truth_altitude=false and never call
        this. It exists so the demo doesn't depend on tuning the depth
        camera; the proposal's eventual LIDAR-based KF will plug into the
        same `_altitude_m` / `_latest_vz` slot.

        Garbage-frame gate: DartSim's ODE collision detector occasionally
        emits |z| in the millions during contact instability. Letting it
        through pollutes both the altitude reading and the derived vz.
        Refuse those frames and keep the last-good values.

        vz is derived from finite-differencing the altitude in world frame
        rather than reading msg.twist.twist.linear.z directly — the
        latter's frame convention in gz-sim's OdometryPublisher is body /
        z-flipped in ways that didn't fit the PD damping sign (r16
        showed positive `linear.z` while altitude was decreasing).
        """
        z = float(msg.pose.pose.position.z)
        if abs(z) > self._odom_truth_max_alt:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._odom_truth_prev_t is not None:
            dt = now - self._odom_truth_prev_t
            if 1e-3 < dt < 0.5:    # ignore unphysical timestep skips
                vz = (z - self._odom_truth_prev_z) / dt
                if abs(vz) <= self._odom_truth_max_vz:
                    self._latest_vz = vz
        self._odom_truth_prev_t = now
        self._odom_truth_prev_z = z
        self._altitude_m = z
        self._truth_x = float(msg.pose.pose.position.x)
        self._truth_y = float(msg.pose.pose.position.y)
        # Yaw from the orientation quaternion (ENU): standard formula
        # for yaw from (w, x, y, z). The result lives in `_odom_truth_yaw`
        # and is injected into DR.yaw at the top of _on_dr_tick — the
        # yaw lock can then drive wz against the actual drift instead of
        # against zero.
        qw = float(msg.pose.pose.orientation.w)
        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        self._odom_truth_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

    def _handle_set_state(self, request, response):
        try:
            new_state = self._fsm.set_state(request.state)
            response.success = True
            response.message = f"transitioned to {new_state.name}"
            self.get_logger().info(response.message)
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().warn(f"set_state rejected: {exc}")
        return response

    # ------------------------------------------------------------------
    # DR tick
    # ------------------------------------------------------------------

    def _resolved_pixel_error(
        self,
    ) -> tuple[Optional[float], Optional[float], Optional[float], str]:
        """Pick external override (if fresh) else perceived. Returns
        (du, dv, psi_err, source)."""
        ttl = float(self.get_parameter("external_override_ttl").value)
        now = self.get_clock().now().nanoseconds * 1e-9
        if (
            self._external_pixel_error is not None
            and self._external_pixel_error_t is not None
            and (now - self._external_pixel_error_t) <= ttl
        ):
            v = self._external_pixel_error
            return float(v.x), float(v.y), float(v.z), "external"
        if self._latest_perception is not None:
            r = self._latest_perception
            return r.du, r.dv, r.psi_err, "perception"
        return None, None, None, "none"

    def _on_dr_tick(self) -> None:
        du, dv, psi_err, source = self._resolved_pixel_error()

        # Sim-only yaw inject. /odom_truth orientation -> DR.yaw so the
        # yaw lock and the world_to_body rotation see the same heading
        # the simulated drone actually has. Real-flight builds set
        # use_odom_truth_altitude=false and never enter this branch.
        if (self._use_odom_truth_altitude
                and self._odom_truth_yaw is not None):
            self._dr.state.yaw = self._odom_truth_yaw

        # --- altitude resolution + FSM tick ---
        z_hat = self._dr.state.z
        altitude = self._altitude_m if self._altitude_m is not None else max(z_hat, 0.05)

        now = self.get_clock().now().nanoseconds * 1e-9
        tick_result = self._fsm.tick(
            now=now,
            dr_state=self._dr.state,
            perception=self._latest_perception,
            altitude=altitude,
        )
        behavior = tick_result.behavior

        # FSM events worth logging on the spot — these are what the verifier
        # greps for to confirm the demo walked all phases.
        if tick_result.state_changed:
            self.get_logger().info(
                f">> FSM: {self._fsm_state_prev.name if self._fsm_state_prev else 'NONE'} "
                f"-> {tick_result.state.name} (alt={altitude:.2f})"
            )
        if tick_result.snapped_record is not None:
            mid, mx, my = tick_result.snapped_record
            self.get_logger().info(
                f">> RECORD aruco id={mid} at ({mx:+.2f}, {my:+.2f})"
            )
        self._fsm_state_prev = tick_result.state

        # --- pixel-space (or FSM target_xy) → body-frame metric offsets ---
        intr = self._intrinsics
        dx_body = 0.0
        dy_body = 0.0
        psi = 0.0

        if tick_result.target_xy_world is not None:
            # Retrieval / return phases: navigate to a world-frame target.
            tx, ty = tick_result.target_xy_world
            dx_w = tx - self._dr.state.x
            dy_w = ty - self._dr.state.y
            dx_body, dy_body = world_to_body(dx_w, dy_w, self._dr.state.yaw)
        else:
            if behavior.use_lateral_error and du is not None and intr is not None:
                dy_body = -du * altitude / intr.fx     # +du (line right) → -y_body
            if behavior.use_heading_error and psi_err is not None:
                psi = float(psi_err)
            if behavior.use_forward_error and dv is not None and intr is not None:
                dx_body = -dv * altitude / intr.fy     # +dv (line behind) → -x_body
            elif behavior.cruise_vx != 0.0 and self._gains.kp_xy != 0.0:
                # Coerce a constant cruise into the P-controller by feeding
                # an offset that, after the kp gain, equals the requested vx.
                dx_body = behavior.cruise_vx / self._gains.kp_xy

        # Yaw lock fallback: if the behavior demands an initial-heading
        # lock and perception didn't supply a fresh psi_err on this tick,
        # drive yaw back toward MissionContext.start_yaw. Without this
        # the firmware's residual mixer/quat-sign asymmetry steadily
        # yaws the drone (~20° per takeoff in r15) and cruise_vx in
        # body +X ends up sending the drone diagonally off-grid.
        if (behavior.lock_yaw_to_initial
                and psi == 0.0
                and self._fsm.context.start_yaw is not None):
            psi = wrap_angle(self._fsm.context.start_yaw - self._dr.state.yaw)

        # --- target altitude per FSM ---
        gains_now = self._gains
        if behavior.target_altitude != gains_now.target_altitude:
            gains_now = Gains(
                kp_xy=gains_now.kp_xy, kp_yaw=gains_now.kp_yaw,
                kp_z=gains_now.kp_z, max_vxy=gains_now.max_vxy,
                max_wz=gains_now.max_wz,
                target_altitude=behavior.target_altitude,
            )
        self._dr.gains = gains_now

        # vz uses estimated z (z_hat) — for sim we substitute measured altitude
        # so that the loop closes against ground truth instead of integrated z.
        if self._altitude_m is not None:
            self._dr.state = State(
                x=self._dr.state.x, y=self._dr.state.y,
                z=self._altitude_m, yaw=self._dr.state.yaw,
            )

        vel, _ = self._dr.step(dx_body, dy_body, psi, self._dr_dt)

        # Always build a Twist for the /odom_dr debug message (DR state is
        # easier to inspect with Twist than with Setpoint).
        twist = Twist()
        twist.linear.x = float(vel.vx)
        twist.linear.y = float(vel.vy)
        twist.linear.z = float(vel.vz)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(vel.wz)

        if self._setpoint_pub:
            sp = self._build_setpoint(vel, behavior.target_altitude)
            self._pub_cmd.publish(sp)
        else:
            self._pub_cmd.publish(twist)

        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._dr.state.x
        odom.pose.pose.position.y = self._dr.state.y
        odom.pose.pose.position.z = self._dr.state.z
        odom.pose.pose.orientation = _yaw_to_quaternion(self._dr.state.yaw)
        odom.twist.twist = twist
        self._pub_odom.publish(odom)

        # Throttled debug log: every ~1 s (every 20 ticks at 20 Hz)
        if not hasattr(self, "_log_counter"):
            self._log_counter = 0
        self._log_counter = (self._log_counter + 1) % 20
        if self._log_counter == 0:
            self.get_logger().info(
                f"[{self._fsm.state.name}/{source}] "
                f"xy=({self._truth_x:+.2f},{self._truth_y:+.2f}) "
                f"alt={altitude:.2f} vz_truth={self._latest_vz:+.2f} | "
                f"du={du} dv={dv} psi_err={psi_err} | "
                f"vx={vel.vx:+.2f} vy={vel.vy:+.2f} vz={vel.vz:+.2f} wz={vel.wz:+.2f}"
            )

    # ------------------------------------------------------------------
    # Body-velocity intent -> FC attitude/thrust setpoint
    # ------------------------------------------------------------------

    def _build_setpoint(self, vel, target_altitude: float):
        """Thin rclpy wrapper around :func:`body_vel_to_atti_thr` — the
        actual sign / clamp / thrust-PD logic lives there so it can be
        unit-tested without rclpy or the fc_sim_msgs message class."""
        gains = SetpointGains(
            hover_thrust_norm=self._hover_thrust_norm,
            kp_alt_thrust=self._kp_alt_thrust,
            kd_alt_thrust=self._kd_alt_thrust,
            max_atti_setpoint_rad=self._max_atti_sp,
            thrust_min=self._thrust_min,
            thrust_max=self._thrust_max,
            takeoff_z_threshold=self._takeoff_z_threshold,
            takeoff_thrust_norm=self._takeoff_thrust_norm,
        )
        cmd = body_vel_to_atti_thr(
            vel=vel,
            target_alt=float(target_altitude),
            altitude=float(self._altitude_m if self._altitude_m is not None else 0.0),
            vz_truth=float(self._latest_vz),
            gains=gains,
        )
        sp = Setpoint()
        sp.mode = Setpoint.MODE_ATTITHR
        sp.arm = True
        sp.roll_sp = cmd.roll_sp
        sp.pitch_sp = cmd.pitch_sp
        sp.yawrate_sp = cmd.yawrate_sp
        sp.vz_sp = 0.0
        sp.thrust_norm = cmd.thrust_norm
        return sp

    # ------------------------------------------------------------------
    # ArUco markers → world frame
    # ------------------------------------------------------------------

    def _publish_aruco_markers(self, result: PerceptionResult, stamp) -> None:
        if self._intrinsics is None or self._altitude_m is None:
            return
        d = self._altitude_m
        intr = self._intrinsics
        s = self._dr.state

        ma = MarkerArray()
        for det in result.aruco:
            cu, cv = det.center_uv
            xc = (cu - intr.cx) * d / intr.fx
            yc = (cv - intr.cy) * d / intr.fy
            # camera optical → body FLU (mount: pitch=+π/2 around Y_body)
            xb = -yc
            yb = -xc
            zb = -d
            cy_, sy_ = math.cos(s.yaw), math.sin(s.yaw)
            xw = s.x + cy_ * xb - sy_ * yb
            yw = s.y + sy_ * xb + cy_ * yb
            zw = s.z + zb

            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = stamp
            m.ns = "aruco"
            m.id = int(det.id)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = xw
            m.pose.position.y = yw
            m.pose.position.z = zw
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.85
            m.lifetime.sec = 5
            ma.markers.append(m)
        self._pub_markers.publish(ma)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineTracerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
