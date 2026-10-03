import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import sys
import select
import threading
import numpy as np
from std_msgs.msg import Bool, Float32MultiArray, String
from rclpy.executors import ExternalShutdownException

from amr_control.square_profiles import (
    corner_forward_speed,
    get_square_profile,
    scheduled_lateral_gain,
    snap_cardinal_heading,
    straight_integral_gate,
    straight_integral_output_gate,
)


class BSMCSquare(Node):
    """
    BSMC Controller for a square trajectory.

    Mặc định chạy liên tục trên đa tuyến vuông có góc nhọn: giảm tốc theo vị
    trí, đi tới đúng đỉnh rồi đổi hướng mong muốn 90 độ nhưng vẫn giữ vận tốc
    tiến dương. Hình chiếu vị trí lên đường đi ngăn tham chiếu chạy trước.
    Robot bắt đầu tại một đỉnh, đi ngược chiều kim đồng hồ (quẹo trái).
    Trên cạnh thẳng: v_d = VD, w_d = 0.
    """

    def __init__(self):
        super().__init__('bsmc_square')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.err_pub = self.create_publisher(Point, '/tracking_error', 10)
        self.desired_pub = self.create_publisher(Point, '/desired_trajectory', 10)
        self.desired_mode_pub = self.create_publisher(String, '/desired_trajectory_mode', 10)
        self.diagnostics_pub = self.create_publisher(
            Float32MultiArray, '/bsmc_diagnostics', 10
        )

        self.declare_parameter('square_profile', '2m')
        self.square_profile = str(self.get_parameter('square_profile').value)
        profile = get_square_profile(self.square_profile)

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('camera_topic', '/odom_camera')
        self.declare_parameter('robot_state_topic', '/robot_state')
        self.declare_parameter('camera_timeout', 0.50)
        self.declare_parameter('robot_state_timeout', 0.50)
        self.declare_parameter('require_camera_fresh', True)
        self.declare_parameter('require_robot_state_fresh', True)
        self.declare_parameter('side_length', profile['side_length'])
        self.declare_parameter('corner_speed', profile['corner_speed'])
        self.declare_parameter('desired_speed', profile['desired_speed'])
        self.declare_parameter('control_frequency', 25.0)
        self.declare_parameter('startup_delay', 1.0)
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('snap_start_yaw', True)
        self.declare_parameter('snap_start_yaw_tolerance_deg', 10.0)
        self.declare_parameter('k1', 0.80)
        self.declare_parameter('k2_straight', profile['k2'])
        self.declare_parameter('large_error_k2', profile['large_error_k2'])
        self.declare_parameter('large_error_ey', profile['large_error_ey'])
        self.declare_parameter('k3_straight', profile['k3'])
        self.declare_parameter('large_error_k3', profile['large_error_k3'])
        self.declare_parameter('kd_w_straight', profile['kd_w'])
        self.declare_parameter('kd_w_deadband', profile['kd_w_deadband'])
        self.declare_parameter('ki_y_straight', profile['ki_y_straight'])
        self.declare_parameter('ey_integral_limit', profile['ey_integral_limit'])
        self.declare_parameter(
            'initial_ey_integral', profile['initial_ey_integral']
        )
        self.declare_parameter(
            'integral_corner_distance', profile['integral_corner_distance']
        )
        self.declare_parameter(
            'integral_heading_limit_deg',
            profile['integral_heading_limit_deg'],
        )
        self.declare_parameter('ks1', 0.08)
        self.declare_parameter('ks2', 0.10)
        self.declare_parameter('phi1', 1.0)
        self.declare_parameter('phi2', 1.5)
        self.declare_parameter('max_v', 0.18)
        self.declare_parameter('max_w', profile['max_w'])
        self.declare_parameter('min_v', profile['min_v'])
        self.declare_parameter(
            'corner_min_wheel_ratio', profile['corner_min_wheel_ratio']
        )
        self.declare_parameter(
            'corner_preempt_start_deg', profile['corner_preempt_start_deg']
        )
        self.declare_parameter(
            'corner_preempt_end_deg', profile['corner_preempt_end_deg']
        )
        self.declare_parameter(
            'corner_preempt_ratio', profile['corner_preempt_ratio']
        )
        self.declare_parameter(
            'corner_preempt_w_scale', profile['corner_preempt_w_scale']
        )
        self.declare_parameter('corner_stall_rpm', profile['corner_stall_rpm'])
        self.declare_parameter(
            'corner_stall_recovery_ratio',
            profile['corner_stall_recovery_ratio'],
        )
        self.declare_parameter(
            'corner_stall_w_scale', profile['corner_stall_w_scale']
        )
        self.declare_parameter(
            'corner_stall_recovery_hold',
            profile['corner_stall_recovery_hold'],
        )
        self.declare_parameter('invert_ey', False)
        self.declare_parameter('sharp_corner_w', profile['sharp_corner_w'])
        self.declare_parameter(
            'sharp_corner_blend_start_deg',
            profile['sharp_corner_blend_start_deg'],
        )
        self.declare_parameter(
            'sharp_corner_blend_full_deg',
            profile['sharp_corner_blend_full_deg'],
        )
        self.declare_parameter('max_progress_rate', 0.30)
        self.declare_parameter(
            'corner_decel_distance', profile['corner_decel_distance']
        )

        self.odom_topic = self.get_parameter('odom_topic').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.robot_state_topic = self.get_parameter('robot_state_topic').value
        self.CAMERA_TIMEOUT = max(
            0.10, float(self.get_parameter('camera_timeout').value)
        )
        self.ROBOT_STATE_TIMEOUT = max(
            0.10, float(self.get_parameter('robot_state_timeout').value)
        )
        self.REQUIRE_CAMERA_FRESH = bool(
            self.get_parameter('require_camera_fresh').value
        )
        self.REQUIRE_ROBOT_STATE_FRESH = bool(
            self.get_parameter('require_robot_state_fresh').value
        )
        self.side_length = float(self.get_parameter('side_length').value)
        self.CORNER_SPEED = max(
            0.01, float(self.get_parameter('corner_speed').value)
        )
        self.VD = float(self.get_parameter('desired_speed').value)

        control_frequency = max(1.0, float(self.get_parameter('control_frequency').value))
        self.timer_period = 1.0 / control_frequency
        self.STARTUP_DELAY = float(self.get_parameter('startup_delay').value)
        self.SETTLE_TIME = float(self.get_parameter('settle_time').value)
        self.SNAP_START_YAW = bool(
            self.get_parameter('snap_start_yaw').value
        )
        self.SNAP_START_YAW_TOLERANCE_DEG = max(
            0.0,
            float(self.get_parameter('snap_start_yaw_tolerance_deg').value),
        )

        self.k1 = float(self.get_parameter('k1').value)
        self.k2_straight = float(self.get_parameter('k2_straight').value)
        self.LARGE_ERROR_K2 = max(
            self.k2_straight,
            float(self.get_parameter('large_error_k2').value),
        )
        self.LARGE_ERROR_EY = max(
            0.005, float(self.get_parameter('large_error_ey').value)
        )
        self.k3_straight = float(self.get_parameter('k3_straight').value)
        self.LARGE_ERROR_K3 = max(
            0.0, float(self.get_parameter('large_error_k3').value)
        )
        self.kd_w_straight = max(
            0.0, float(self.get_parameter('kd_w_straight').value)
        )
        self.kd_w_deadband = max(
            0.0, float(self.get_parameter('kd_w_deadband').value)
        )
        self.KI_Y_STRAIGHT = max(
            0.0, float(self.get_parameter('ki_y_straight').value)
        )
        self.EY_INTEGRAL_LIMIT = max(
            0.0, float(self.get_parameter('ey_integral_limit').value)
        )
        self.INITIAL_EY_INTEGRAL = max(
            -self.EY_INTEGRAL_LIMIT,
            min(
                self.EY_INTEGRAL_LIMIT,
                float(self.get_parameter('initial_ey_integral').value),
            ),
        )
        self.INTEGRAL_CORNER_DISTANCE = max(
            0.01, float(self.get_parameter('integral_corner_distance').value)
        )
        self.INTEGRAL_HEADING_LIMIT_DEG = max(
            0.1,
            float(self.get_parameter('integral_heading_limit_deg').value),
        )
        self.INTEGRAL_HEADING_LIMIT = math.radians(
            self.INTEGRAL_HEADING_LIMIT_DEG
        )
        self.Ks1 = float(self.get_parameter('ks1').value)
        self.Ks2 = float(self.get_parameter('ks2').value)
        self.phi1 = float(self.get_parameter('phi1').value)
        self.phi2 = float(self.get_parameter('phi2').value)
        self.MAX_V = float(self.get_parameter('max_v').value)
        self.MAX_W = float(self.get_parameter('max_w').value)
        self.MIN_V = float(self.get_parameter('min_v').value)
        self.CORNER_MIN_WHEEL_RATIO = max(
            0.0,
            min(
                0.80,
                float(self.get_parameter('corner_min_wheel_ratio').value),
            ),
        )
        self.CORNER_PREEMPT_START_DEG = max(
            0.0,
            float(self.get_parameter('corner_preempt_start_deg').value),
        )
        self.CORNER_PREEMPT_END_DEG = max(
            0.0,
            min(
                self.CORNER_PREEMPT_START_DEG,
                float(self.get_parameter('corner_preempt_end_deg').value),
            ),
        )
        self.CORNER_PREEMPT_RATIO = max(
            self.CORNER_MIN_WHEEL_RATIO,
            min(
                0.80,
                float(self.get_parameter('corner_preempt_ratio').value),
            ),
        )
        self.CORNER_PREEMPT_W_SCALE = max(
            0.10,
            min(
                1.0,
                float(self.get_parameter('corner_preempt_w_scale').value),
            ),
        )
        self.CORNER_STALL_RPM = max(
            0.0, float(self.get_parameter('corner_stall_rpm').value)
        )
        self.CORNER_STALL_RECOVERY_RATIO = max(
            self.CORNER_MIN_WHEEL_RATIO,
            min(
                0.80,
                float(
                    self.get_parameter(
                        'corner_stall_recovery_ratio'
                    ).value
                ),
            ),
        )
        self.CORNER_STALL_W_SCALE = max(
            0.10,
            min(
                self.CORNER_PREEMPT_W_SCALE,
                float(self.get_parameter('corner_stall_w_scale').value),
            ),
        )
        self.CORNER_STALL_RECOVERY_HOLD = max(
            0.0,
            float(self.get_parameter('corner_stall_recovery_hold').value),
        )
        self.INVERT_EY = bool(self.get_parameter('invert_ey').value)
        self.SHARP_CORNER_W = max(
            0.0, float(self.get_parameter('sharp_corner_w').value)
        )
        self.SHARP_CORNER_BLEND_START = math.radians(max(
            0.0,
            float(self.get_parameter('sharp_corner_blend_start_deg').value),
        ))
        self.SHARP_CORNER_BLEND_FULL = math.radians(max(
            math.degrees(self.SHARP_CORNER_BLEND_START) + 1.0,
            float(self.get_parameter('sharp_corner_blend_full_deg').value),
        ))
        self.MAX_PROGRESS_RATE = max(
            self.VD, float(self.get_parameter('max_progress_rate').value)
        )
        self.CORNER_DECEL_DISTANCE = max(
            0.05, float(self.get_parameter('corner_decel_distance').value)
        )

        # Validate
        S = self.side_length
        if S <= 0.0:
            self.get_logger().warn("Invalid side_length <= 0. Using 1.0m.")
            self.side_length = 1.0
            S = 1.0
        # Headroom check
        VD_max = self.MAX_V * 0.60
        if self.VD > VD_max:
            self.VD = VD_max
            self.get_logger().warn(
                f"desired_speed clamped to {self.VD:.3f}m/s (60% of max_v={self.MAX_V:.2f})"
            )

        # Build path segments
        self._build_path()

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Odometry, self.camera_topic, self.camera_callback, 10
        )
        self.robot_state_sub = self.create_subscription(
            Float32MultiArray,
            self.robot_state_topic,
            self.robot_state_callback,
            20,
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_v = 0.0
        self.current_w = 0.0

        self.odom_received = False
        self.last_odom_time = None
        self.last_camera_time = None
        self.last_robot_state_time = None
        self.rpm_left = 0.0
        self.rpm_right = 0.0
        self.corner_stall_recovery_until = 0.0
        self.start_time = None

        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0
        self.tracking_started = False
        self.path_progress = 0.0
        self.projection_distance = 0.0
        self.reference_segment_index = None
        self.ey_integral = self.INITIAL_EY_INTEGRAL

        # EKF Settling
        self.settling = False
        self.settle_samples_x = []
        self.settle_samples_y = []
        self.settle_samples_theta = []

        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Pause
        self.is_paused = False
        self.total_paused_time = 0.0
        self.pause_start_time = None
        self.pause_sub = self.create_subscription(Bool, '/pause_control', self.pause_cb, 10)

        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

        self.c = 1.0
        self.L = 0.17
        self.VL_MIN = 0.0
        self.DEADBAND_EX = 0.0
        self.DEADBAND_EY = 0.0
        self.DEADBAND_ETHETA = 0.0
        self.debug_counter = 0

        self.get_logger().info(
            f"EKF BSMC Square started: profile={self.square_profile}, "
            f"side={S:.2f}m, sharp_corners=true, "
            f"vd={self.VD:.3f}m/s, perimeter={self.loop_length:.3f}m, "
            f"lap_time={self.loop_length / self.VD:.1f}s, "
            f"max_v={self.MAX_V:.2f}, max_w={self.MAX_W:.2f}, "
            f"ki_y={self.KI_Y_STRAIGHT:.3f}, "
            f"i_y_seed={self.INITIAL_EY_INTEGRAL:+.2f}, "
            f"i_y_max={self.EY_INTEGRAL_LIMIT:.2f}, "
            f"k2={self.k2_straight:.1f}->{self.LARGE_ERROR_K2:.1f}, "
            f"large_error_ey={self.LARGE_ERROR_EY:.3f}m, "
            f"camera_timeout={self.CAMERA_TIMEOUT:.2f}s, "
            f"robot_state_timeout={self.ROBOT_STATE_TIMEOUT:.2f}s, "
            "mode=continuous-path-following"
        )
        self.get_logger().info(">>> Nhấn 'p' rồi Enter để TẠM DỪNG / CHẠY TIẾP <<<")

    # ─── Path builder ───────────────────────────────────────────────
    def _build_path(self):
        """Pre-compute the four exact Paper1-cu polyline segments."""
        side = self.side_length
        starts = ((0.0, 0.0), (side, 0.0), (side, side), (0.0, side))
        headings = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
        self.path_segments = [
            {
                'length': side,
                'start': starts[index],
                'heading': headings[index],
                's_start': index * side,
            }
            for index in range(4)
        ]
        self.loop_length = 4.0 * side
        self.lead_in_length = 0.0

    def _pose_on_loop(self, s_loop):
        """Return (x_local, y_local, heading) given arc-length on the loop."""
        s_rem = s_loop % self.loop_length
        for seg in self.path_segments:
            if seg['s_start'] + seg['length'] > s_rem + 1e-9:
                ds = s_rem - seg['s_start']
                heading = seg['heading']
                start_x, start_y = seg['start']
                return (
                    start_x + ds * math.cos(heading),
                    start_y + ds * math.sin(heading),
                    heading,
                    0.0,
                )
        return 0.0, 0.0, 0.0, 0.0

    def _project_onto_loop(self, x_local, y_local):
        """Return unwrapped path progress and distance to the square."""
        candidates = []
        for seg in self.path_segments:
            heading = seg['heading']
            start_x, start_y = seg['start']
            along = (
                (x_local - start_x) * math.cos(heading)
                + (y_local - start_y) * math.sin(heading)
            )
            ds = max(0.0, min(seg['length'], along))
            px = start_x + ds * math.cos(heading)
            py = start_y + ds * math.sin(heading)
            distance = math.hypot(x_local - px, y_local - py)
            candidates.append((seg['s_start'] + ds, distance))

        previous_loop = max(0.0, self.path_progress - self.lead_in_length)
        lap_guess = int(math.floor(previous_loop / self.loop_length))
        expanded = []
        for s_loop, distance in candidates:
            for lap in (lap_guess - 1, lap_guess, lap_guess + 1):
                progress = self.lead_in_length + lap * self.loop_length + s_loop
                if progress + 1e-9 < self.lead_in_length:
                    continue
                # Distance is primary. This small continuity term resolves
                # equal-distance ambiguity at adjacent segment endpoints.
                score = distance + 0.01 * abs(progress - self.path_progress)
                expanded.append((score, progress, distance))
        local = [
            item for item in expanded
            if abs(item[1] - self.path_progress) <= 0.15
        ]
        _, progress, distance = min(local or expanded, key=lambda item: item[0])
        return progress, distance

    def _update_path_progress(self):
        """Advance reference progress from robot projection, never the clock."""
        dx = self.current_x - self.x0
        dy = self.current_y - self.y0
        cos0 = math.cos(self.theta0)
        sin0 = math.sin(self.theta0)
        x_local = cos0 * dx + sin0 * dy
        y_local = -sin0 * dx + cos0 * dy

        measured, self.projection_distance = self._project_onto_loop(
            x_local, y_local
        )

        # Reject projection jumps across a corner or to an opposite edge. The
        # cap is faster than the robot, so ordinary projected progress is free.
        max_step = self.MAX_PROGRESS_RATE * self.timer_period
        measured = min(measured, self.path_progress + max_step)
        self.path_progress = max(self.path_progress, measured)
        return self.path_progress

    def _pose_at_progress(self, progress):
        return self._pose_on_loop(progress)

    def _segment_at_loop_progress(self, s_loop):
        s_rem = s_loop % self.loop_length
        for index, seg in enumerate(self.path_segments):
            if seg['s_start'] + seg['length'] > s_rem + 1e-9:
                return index, seg, max(0.0, s_rem - seg['s_start'])
        seg = self.path_segments[-1]
        return len(self.path_segments) - 1, seg, seg['length']

    def _straight_integral_gates(self, e_theta, w_d):
        """Return separate learning and output gates for steering bias.

        Learning is suppressed for large heading errors so corner recovery is
        not stored as bias.  Applying an already learned bias is suppressed
        only near vertices and while the heading reference is turning.
        """
        _, segment, ds = self._segment_at_loop_progress(self.path_progress)
        corner_distance = min(ds, max(0.0, segment['length'] - ds))
        return (
            straight_integral_gate(
                corner_distance,
                self.INTEGRAL_CORNER_DISTANCE,
                e_theta,
                self.INTEGRAL_HEADING_LIMIT,
                w_d,
            ),
            straight_integral_output_gate(
                corner_distance,
                self.INTEGRAL_CORNER_DISTANCE,
                w_d,
            ),
        )

    def _continuous_path_speed(self, progress, ramp):
        """Position-based continuous speed profile with a slow, tight corner."""
        s_loop = progress
        index, seg, ds = self._segment_at_loop_progress(s_loop)
        corner_speed = min(self.VD, self.CORNER_SPEED)
        remaining = max(0.0, seg['length'] - ds)
        decel_blend = min(1.0, remaining / self.CORNER_DECEL_DISTANCE)
        first_straight_first_lap = index == 0 and s_loop < self.loop_length
        if first_straight_first_lap:
            accel_blend = 1.0
        else:
            accel_blend = min(1.0, ds / self.CORNER_DECEL_DISTANCE)
        blend = min(accel_blend, decel_blend)
        # Smoothstep keeps commanded acceleration finite at both ends.
        blend = blend * blend * (3.0 - 2.0 * blend)
        return (corner_speed + (self.VD - corner_speed) * blend) * ramp

    # ─── Trajectory generator ──────────────────────────────────────
    def generate_path_following_trajectory(self, t):
        """Use the closest path pose so the reference cannot run ahead."""
        ramp = min(1.0, max(0.0, t / 2.5))
        progress = self._update_path_progress()
        v_d = self._continuous_path_speed(progress, ramp)
        x_local, y_local, theta_local, w_d = self._pose_at_progress(progress)

        cos0 = math.cos(self.theta0)
        sin0 = math.sin(self.theta0)
        x_d = self.x0 + cos0 * x_local - sin0 * y_local
        y_d = self.y0 + sin0 * x_local + cos0 * y_local
        theta_d = self.normalize_angle(self.theta0 + theta_local)
        segment_index, _, _ = self._segment_at_loop_progress(progress)

        if self.reference_segment_index is None:
            self.reference_segment_index = segment_index

        if segment_index != self.reference_segment_index:
            theta_d_old = self.normalize_angle(
                self.theta0 + self.path_segments[self.reference_segment_index]['heading']
            )
            raw_angle_error = theta_d - self.current_theta
            wrapped_angle_error = self.normalize_angle(raw_angle_error)
            self.get_logger().info(
                "WAYPOINT_TRANSITION "
                f"time={t:.3f}, waypoint_index={segment_index}, "
                f"theta_d_old={theta_d_old:.6f}, theta_d_new={theta_d:.6f}, "
                f"theta_est={self.current_theta:.6f}, "
                f"raw_angle_error={raw_angle_error:.6f}, "
                f"wrapped_angle_error={wrapped_angle_error:.6f}"
            )
            self.reference_segment_index = segment_index

        # Match Paper1-cu: theta_d jumps by 90 degrees at a vertex and w_d is
        # zero on both adjacent polyline edges. Sharp-corner blending below
        # handles the heading step entirely through feedback.
        return x_d, y_d, theta_d, v_d, w_d

    # ─── Utilities ──────────────────────────────────────────────────
    def pause_cb(self, msg):
        self.toggle_pause(msg.data)

    def keyboard_loop(self):
        while rclpy.ok():
            i, o, e = select.select([sys.stdin], [], [], 0.5)
            if i:
                key = sys.stdin.readline().strip().lower()
                if key == 'p':
                    self.toggle_pause(not self.is_paused)

    def toggle_pause(self, state):
        if self.is_paused != state:
            self.is_paused = state
            if self.is_paused:
                self.get_logger().warn(">>> PAUSED! Nhấn 'p'+Enter để chạy tiếp. <<<")
            else:
                self.get_logger().info(">>> RESUMED! <<<")

    def sat(self, z):
        return max(-1.0, min(1.0, z))

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def euler_from_quaternion(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    # ─── Odom callback ──────────────────────────────────────────────
    def odom_callback(self, msg):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        new_theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        now_s = self.get_clock().now().nanoseconds / 1e9

        # Accept finite EKF relocalization jumps. The EKF owns pose outlier
        # rejection; duplicating a 1 m gate here can deadlock the controller
        # forever at its old pose after a valid global correction.
        if not all(math.isfinite(value) for value in (new_x, new_y, new_theta)):
            self.get_logger().error("REJECTED non-finite filtered odometry")
            return

        self.current_x = new_x
        self.current_y = new_y
        self.current_theta = new_theta
        self.current_v = float(msg.twist.twist.linear.x)
        self.current_w = float(msg.twist.twist.angular.z)

        if not self.odom_received:
            self.start_time = now_s
            self.get_logger().info(
                f"Odometry received. Waiting {self.STARTUP_DELAY:.1f}s..."
            )
        self.odom_received = True
        self.last_odom_time = now_s

    def camera_callback(self, _msg):
        self.last_camera_time = self.get_clock().now().nanoseconds / 1e9

    def robot_state_callback(self, msg):
        self.last_robot_state_time = self.get_clock().now().nanoseconds / 1e9
        if len(msg.data) >= 2:
            self.rpm_left = float(msg.data[0]) / 10.0
            self.rpm_right = float(msg.data[1]) / 10.0

    # ─── EKF settling ───────────────────────────────────────────────
    def _check_ekf_freshness(self, now_s):
        if self.last_odom_time is None:
            return True
        return (now_s - self.last_odom_time) <= (3.0 / 20.0)

    def _check_source_freshness(self, now_s):
        """Reject a fresh EKF prediction when its physical inputs are stale."""
        stale = []
        if self.REQUIRE_CAMERA_FRESH and (
            self.last_camera_time is None
            or now_s - self.last_camera_time > self.CAMERA_TIMEOUT
        ):
            age = (
                math.inf
                if self.last_camera_time is None
                else now_s - self.last_camera_time
            )
            stale.append(f"camera age={age:.2f}s")
        if self.REQUIRE_ROBOT_STATE_FRESH and (
            self.last_robot_state_time is None
            or now_s - self.last_robot_state_time > self.ROBOT_STATE_TIMEOUT
        ):
            age = (
                math.inf
                if self.last_robot_state_time is None
                else now_s - self.last_robot_state_time
            )
            stale.append(f"robot_state age={age:.2f}s")
        if stale:
            self.get_logger().warn(
                "Physical feedback stale; stopping robot: " + ", ".join(stale),
                throttle_duration_sec=1.0,
            )
            return False
        return True

    def _settle_ekf(self, now_s):
        t = now_s - self.start_time - self.total_paused_time
        settle_t = t - self.STARTUP_DELAY

        self.settle_samples_x.append(self.current_x)
        self.settle_samples_y.append(self.current_y)
        self.settle_samples_theta.append(self.current_theta)

        if settle_t >= self.SETTLE_TIME:
            x_arr = np.array(self.settle_samples_x)
            y_arr = np.array(self.settle_samples_y)
            theta_arr = np.array(self.settle_samples_theta)

            x_mean = np.mean(x_arr)
            y_mean = np.mean(y_arr)
            dists = np.sqrt((x_arr - x_mean)**2 + (y_arr - y_mean)**2)
            threshold = np.percentile(dists, 80)
            good = dists <= threshold

            self.x0 = float(np.mean(x_arr[good]))
            self.y0 = float(np.mean(y_arr[good]))
            sin_sum = np.sum(np.sin(theta_arr[good]))
            cos_sum = np.sum(np.cos(theta_arr[good]))
            raw_theta0 = math.atan2(sin_sum, cos_sum)
            self.theta0 = (
                snap_cardinal_heading(
                    raw_theta0,
                    self.SNAP_START_YAW_TOLERANCE_DEG,
                )
                if self.SNAP_START_YAW
                else raw_theta0
            )

            self.tracking_started = True
            self.settling = False
            self.path_progress = 0.0
            self.projection_distance = 0.0
            self.reference_segment_index = 0
            self.ey_integral = self.INITIAL_EY_INTEGRAL

            n_total = len(self.settle_samples_x)
            n_good = int(good.sum())
            self.get_logger().info(
                f"=== TRACKING START! ({n_good}/{n_total} samples) ==="
                f" x0={self.x0:.4f}, y0={self.y0:.4f}, "
                f"theta0_raw={math.degrees(raw_theta0):.1f}deg, "
                f"theta0={math.degrees(self.theta0):.1f}deg, "
                f"snap={'on' if self.SNAP_START_YAW else 'off'}"
            )
            self.settle_samples_x = []
            self.settle_samples_y = []
            self.settle_samples_theta = []
            return True

        desired_msg = Point()
        desired_msg.x = float(self.current_x)
        desired_msg.y = float(self.current_y)
        desired_msg.z = float(self.current_theta)
        self.desired_pub.publish(desired_msg)

        if len(self.settle_samples_x) % 20 == 0:
            x_arr = np.array(self.settle_samples_x)
            y_arr = np.array(self.settle_samples_y)
            self.get_logger().info(
                f"Settling... {settle_t:.1f}/{self.SETTLE_TIME:.0f}s "
                f"({len(self.settle_samples_x)} samples)"
            )
        return False

    # ─── Control loop ───────────────────────────────────────────────
    def control_loop(self):
        if not self.odom_received:
            return

        now_s = self.get_clock().now().nanoseconds / 1e9

        if self.last_odom_time is not None and now_s - self.last_odom_time > 1.0:
            self.get_logger().warn("Odometry timeout >1s. Stopping.")
            self.cmd_pub.publish(Twist())
            return

        if self.is_paused:
            if self.pause_start_time is None:
                self.pause_start_time = now_s
            self.cmd_pub.publish(Twist())
            return
        else:
            if self.pause_start_time is not None:
                self.total_paused_time += (now_s - self.pause_start_time)
                self.pause_start_time = None

        if not self._check_source_freshness(now_s):
            self.cmd_pub.publish(Twist())
            return

        t = now_s - self.start_time - self.total_paused_time

        if t < self.STARTUP_DELAY:
            self.cmd_pub.publish(Twist())
            return

        if not self.tracking_started:
            if not self.settling:
                self.settling = True
                self.settle_samples_x = []
                self.settle_samples_y = []
                self.settle_samples_theta = []
                self.get_logger().info(
                    f"EKF settling ({self.SETTLE_TIME:.0f}s)..."
                )
            self.cmd_pub.publish(Twist())
            self._settle_ekf(now_s)
            return

        t_track = (
            now_s - self.start_time - self.total_paused_time
            - self.STARTUP_DELAY - self.SETTLE_TIME
        )

        x_d, y_d, theta_d, v_d, w_d = (
            self.generate_path_following_trajectory(t_track)
        )

        if not self._check_ekf_freshness(now_s):
            self.cmd_pub.publish(Twist())
            self.get_logger().warn(
                "Feedback stale during tracking; stopping robot.",
                throttle_duration_sec=1.0,
            )
            return

        desired_msg = Point()
        desired_msg.x = float(x_d)
        desired_msg.y = float(y_d)
        desired_msg.z = float(theta_d)
        self.desired_pub.publish(desired_msg)

        mode_msg = String()
        mode_msg.data = 'actual'
        self.desired_mode_pub.publish(mode_msg)

        # Error computation
        dx = x_d - self.current_x
        dy = y_d - self.current_y
        cos_th = math.cos(self.current_theta)
        sin_th = math.sin(self.current_theta)

        e_x = cos_th * dx + sin_th * dy
        e_y = -sin_th * dx + cos_th * dy
        if self.INVERT_EY:
            e_y = -e_y
        e_theta = self.normalize_angle(theta_d - self.current_theta)

        if abs(e_x) < self.DEADBAND_EX: e_x = 0.0
        if abs(e_y) < self.DEADBAND_EY: e_y = 0.0
        if abs(e_theta) < self.DEADBAND_ETHETA: e_theta = 0.0

        # Sliding surfaces
        s1 = e_x
        s2 = e_theta + self.c * e_y
        sat_s1 = self.sat(s1 / self.phi1)
        sat_s2 = self.sat(s2 / self.phi2)

        # V control
        v_cmd = (
            v_d * math.cos(e_theta)
            + self.k1 * e_x
            + self.Ks1 * sat_s1
        )
        # Learn only the persistent cross-track bias on established straight
        # portions. The clamp is anti-windup; the 10 cm guard rejects large
        # disturbances that should be handled by the ordinary BSMC feedback.
        integral_gate, integral_output_gate = self._straight_integral_gates(
            e_theta, w_d
        )
        if abs(e_y) <= 0.10 and self.EY_INTEGRAL_LIMIT > 0.0:
            self.ey_integral += (
                integral_gate * e_y * self.timer_period
            )
            self.ey_integral = max(
                -self.EY_INTEGRAL_LIMIT,
                min(self.EY_INTEGRAL_LIMIT, self.ey_integral),
            )
        w_integral = (
            integral_output_gate * self.KI_Y_STRAIGHT * self.ey_integral
        )
        # Angular control on exact square edges.
        w_ff = w_d
        k2_control = scheduled_lateral_gain(
            self.k2_straight,
            self.LARGE_ERROR_K2,
            e_y,
            self.LARGE_ERROR_EY,
        )
        k3_control = scheduled_lateral_gain(
            self.k3_straight,
            self.LARGE_ERROR_K3,
            e_y,
            self.LARGE_ERROR_EY,
        )
        w_ey = v_d * k2_control * e_y
        w_eth = v_d * k3_control * math.sin(e_theta)
        w_smc = self.Ks2 * sat_s2
        damping_rate = max(0.0, abs(self.current_w) - self.kd_w_deadband)
        w_damping = -self.kd_w_straight * math.copysign(
            damping_rate, self.current_w
        )
        w_cmd = w_ff + w_ey + w_eth + w_smc + w_integral + w_damping
        # Large unexpected heading errors still receive bounded recovery.
        # During nominal fillet tracking, w_ff supplies the turn continuously.
        corner_blend = 0.0
        if self.SHARP_CORNER_W > 0.0:
            heading_magnitude = abs(e_theta)
            corner_blend = (
                (heading_magnitude - self.SHARP_CORNER_BLEND_START)
                / (
                    self.SHARP_CORNER_BLEND_FULL
                    - self.SHARP_CORNER_BLEND_START
                )
            )
            corner_blend = max(0.0, min(1.0, corner_blend))
            corner_blend = (
                corner_blend
                * corner_blend
                * (3.0 - 2.0 * corner_blend)
            )
            corner_w = math.copysign(
                min(self.SHARP_CORNER_W, self.MAX_W), e_theta
            )
            w_cmd = (
                (1.0 - corner_blend) * w_cmd
                + corner_blend * corner_w
            )
        # Do not force motion during an actual stop command. In continuous
        # sharp-corner mode, v_d stays positive and MIN_V creates the intended
        # physical overshoot outside the vertex instead of pivoting in place.
        active_min_v = self.MIN_V if v_d > 1e-4 else 0.0
        v_cmd = max(active_min_v, min(self.MAX_V, v_cmd))
        corner_wheel_ratio = self.CORNER_MIN_WHEEL_RATIO
        corner_w_before_antistall = w_cmd
        antistall_w_scale = 1.0
        if corner_blend >= 0.80:
            heading_error_deg = math.degrees(abs(e_theta))
            if (
                self.CORNER_PREEMPT_END_DEG
                <= heading_error_deg
                <= self.CORNER_PREEMPT_START_DEG
            ):
                corner_wheel_ratio = max(
                    corner_wheel_ratio,
                    self.CORNER_PREEMPT_RATIO,
                )
                antistall_w_scale = min(
                    antistall_w_scale,
                    self.CORNER_PREEMPT_W_SCALE,
                )
            inner_rpm = min(abs(self.rpm_left), abs(self.rpm_right))
            outer_rpm = max(abs(self.rpm_left), abs(self.rpm_right))
            if (
                outer_rpm > 10.0
                and inner_rpm < self.CORNER_STALL_RPM
            ):
                self.corner_stall_recovery_until = (
                    now_s + self.CORNER_STALL_RECOVERY_HOLD
                )
            if now_s < self.corner_stall_recovery_until:
                corner_wheel_ratio = max(
                    corner_wheel_ratio,
                    self.CORNER_STALL_RECOVERY_RATIO,
                )
                antistall_w_scale = min(
                    antistall_w_scale,
                    self.CORNER_STALL_W_SCALE,
                )
        if antistall_w_scale < 1.0:
            # Preserve the forward-speed floor for the original sharp-corner
            # command, then reduce yaw demand. Hardware r21/r22 showed that
            # raising v alone while retaining w=0.55 can still pivot on one
            # wheel; reducing differential load lets the inner motor recover.
            v_cmd = corner_forward_speed(
                v_cmd,
                corner_w_before_antistall,
                self.L,
                corner_wheel_ratio,
                corner_blend,
            )
            w_cmd *= antistall_w_scale
        v_cmd = corner_forward_speed(
            v_cmd,
            w_cmd,
            self.L,
            corner_wheel_ratio,
            corner_blend,
        )
        v_cmd = min(self.MAX_V, v_cmd)

        if self.VL_MIN > 0.0:
            w_max_safe = (v_cmd - self.VL_MIN) / (self.L / 2.0)
            w_limit = min(self.MAX_W, max(0.0, w_max_safe))
        else:
            w_limit = self.MAX_W
        w_cmd_before_limit = w_cmd
        w_cmd = max(-w_limit, min(w_limit, w_cmd))

        # Publish
        err_msg = Point()
        err_msg.x = float(e_x)
        err_msg.y = float(e_y)
        err_msg.z = float(e_theta)
        self.err_pub.publish(err_msg)

        cmd_msg = Twist()
        cmd_msg.linear.x = float(v_cmd)
        cmd_msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(cmd_msg)

        diagnostics_msg = Float32MultiArray()
        diagnostics_msg.data = [
            float(w_ff),
            float(w_ey),
            float(w_eth),
            float(w_smc),
            float(w_damping),
            float(w_integral),
            float(w_cmd_before_limit),
            float(w_cmd),
            float(integral_gate),
            float(integral_output_gate),
            float(self.ey_integral),
        ]
        self.diagnostics_pub.publish(diagnostics_msg)

        self.debug_counter += 1
        if self.debug_counter >= 20:
            self.debug_counter = 0
            n_laps = max(
                0.0, self.path_progress - self.lead_in_length
            ) / self.loop_length
            self.get_logger().info(
                f"t={t_track:.1f}s lap={n_laps:.2f} | "
                f"ex={e_x:+.3f} ey={e_y:+.3f} eth={math.degrees(e_theta):+.1f}deg | "
                f"v={v_cmd:.3f} w={w_cmd:.3f} | "
                f"wey={w_ey:+.3f} weth={w_eth:+.3f} "
                f"wsmc={w_smc:+.3f} wiy={w_integral:+.3f} | "
                f"k2={k2_control:.1f} k3={k3_control:.1f} "
                f"cblend={corner_blend:.2f} "
                f"wheel_ratio={corner_wheel_ratio:.2f} "
                f"wscale={antistall_w_scale:.2f} | "
                f"iy={self.ey_integral:+.3f} "
                f"igate={integral_gate:.2f}/{integral_output_gate:.2f} | "
                f"des=({x_d:.2f},{y_d:.2f}) cur=({self.current_x:.2f},{self.current_y:.2f}) "
                f"proj_err={self.projection_distance:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = BSMCSquare()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
