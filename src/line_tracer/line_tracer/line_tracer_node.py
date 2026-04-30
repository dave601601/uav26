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
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
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

from .dead_reckoning import DeadReckoning, Gains, State
from .geom import CameraIntrinsics
from .perception import PerceptionConfig, PerceptionResult, process_image, draw_debug_overlay
from .state_machine import StateMachine, StateName


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
        # perception
        self.declare_parameter("canny_low", 60)
        self.declare_parameter("canny_high", 180)
        self.declare_parameter("hough_threshold", 60)
        self.declare_parameter("hough_min_line_length", 40)
        self.declare_parameter("hough_max_line_gap", 20)
        self.declare_parameter("marker_size", 0.5)
        # depth fallback altitude when no depth has arrived yet (TAKEOFF init)
        self.declare_parameter("default_altitude", 0.0)

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
        self._fsm = StateMachine(initial=StateName.TAKEOFF, target_altitude=target_alt)

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
        self._latest_perception: Optional[PerceptionResult] = None
        self._external_pixel_error: Optional[Vector3] = None
        self._external_pixel_error_t: Optional[float] = None

        # --- pubs/subs/srvs --------------------------------------------------
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
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
        try:
            arr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:                                # pragma: no cover
            self.get_logger().warn(f"depth conversion failed: {e}")
            return
        depth_m = _depth_to_meters(np.asarray(arr))
        alt = _central_median_depth(depth_m)
        if alt is not None:
            self._altitude_m = alt

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
        behavior = self._fsm.behavior()
        du, dv, psi_err, source = self._resolved_pixel_error()

        # TODO(search-on-no-line): when behavior.use_lateral_error 등이 True 인데
        # du/dv/psi_err 가 N 틱 (예: 1초 = 20틱) 연속으로 None 이면 slow yaw 나
        # 작은 정사각 search 패턴을 주입해 perception 이 다시 line 을 잡을 때까지
        # 휘젓는다. 지금은 None 이 들어오면 lateral/heading 보정만 0 이 되고
        # cruise_vx 만 그대로 적용 → 그냥 직진하다 line 을 영영 못 잡는 상황 발생.

        # --- pixel-space → body-frame metric offsets ---
        z_hat = self._dr.state.z
        altitude = self._altitude_m if self._altitude_m is not None else max(z_hat, 0.05)

        intr = self._intrinsics
        dx_body = 0.0
        dy_body = 0.0
        psi = 0.0

        if behavior.use_lateral_error and du is not None and intr is not None:
            dy_body = -du * altitude / intr.fx     # +du (line right) → -y_body
        if behavior.use_heading_error and psi_err is not None:
            psi = float(psi_err)
        if behavior.use_forward_error and dv is not None and intr is not None:
            dx_body = -dv * altitude / intr.fy     # +dv (line behind) → -x_body
        elif behavior.cruise_vx != 0.0 and self._gains.kp_xy != 0.0:
            # Coerce a constant cruise into the P-controller by feeding an
            # offset that, after the kp gain, equals the requested vx.
            dx_body = behavior.cruise_vx / self._gains.kp_xy

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

        twist = Twist()
        twist.linear.x = float(vel.vx)
        twist.linear.y = float(vel.vy)
        twist.linear.z = float(vel.vz)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(vel.wz)
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
                f"du={du} dv={dv} psi_err={psi_err} alt={altitude:.2f} "
                f"vx={vel.vx:+.2f} vy={vel.vy:+.2f} vz={vel.vz:+.2f} wz={vel.wz:+.2f}"
            )

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
