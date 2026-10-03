import math

import numpy as np
import pytest
from nav_msgs.msg import Odometry

from amr_control.custom_ekf_node import (
    CustomEkfNode,
    PersistentInnovationTracker,
)


def make_tracker():
    return PersistentInnovationTracker(
        min_samples=40,
        min_duration=1.5,
        max_gap=0.25,
        xy_consistency=0.12,
        yaw_consistency=0.20,
    )


def test_short_camera_glitch_never_requests_relocalization():
    tracker = make_tracker()
    ready = [
        tracker.observe(index * 0.05, 0.55, -0.10, 0.0)
        for index in range(10)
    ]
    assert not any(ready)


def test_consistent_displacement_requests_relocalization():
    tracker = make_tracker()
    ready = False
    for index in range(40):
        # Small camera/EKF motion noise around one persistent offset.
        dx = 0.55 + 0.01 * math.sin(index)
        dy = -0.12 + 0.01 * math.cos(index)
        ready = tracker.observe(index * 0.05, dx, dy, 0.02)
    assert ready
    assert tracker.count == 40
    assert tracker.duration == pytest.approx(1.95)


def test_inconsistent_camera_jumps_restart_confirmation():
    tracker = make_tracker()
    ready = False
    for index in range(100):
        sign = 1.0 if index % 2 == 0 else -1.0
        ready = tracker.observe(index * 0.05, sign * 0.60, 0.0, 0.0)
        assert tracker.count == 1
    assert not ready


def test_camera_gap_restarts_confirmation():
    tracker = make_tracker()
    for index in range(20):
        assert not tracker.observe(index * 0.05, 0.50, 0.0, 0.0)

    assert not tracker.observe(2.0, 0.50, 0.0, 0.0)
    assert tracker.count == 1


def camera_odom(x, y=0.0, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
    msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
    return msg


class _Logger:
    def error(self, _message):
        return None


class _Publisher:
    def publish(self, _message):
        return None


class _RelocalizationHarness:
    handle_camera_mismatch = CustomEkfNode.handle_camera_mismatch
    record_accepted_camera = CustomEkfNode.record_accepted_camera

    def __init__(self):
        self.camera_relocalization_enabled = True
        self.camera_mismatch = make_tracker()
        self.use_camera_pose = True
        self.use_camera_yaw = True
        self.x = np.zeros((5, 1), dtype=float)
        self.p = np.eye(5, dtype=float)
        self.r_camera_x = 0.0036
        self.r_camera_y = 0.0036
        self.r_camera_yaw = 0.0030
        self.last_predict_time = 0.0
        self.last_v_state = 0.0
        self.last_w_state = 0.0
        self.camera_relocalization_count = 0
        self.last_camera_time = None
        self.last_aligned_camera_pose = None
        self.last_aligned_camera_time = None
        self.camera_aligned_pub = _Publisher()
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


def test_ekf_relocalizes_to_persistent_camera_pose():
    ekf_node = _RelocalizationHarness()
    relocated = False
    for index in range(40):
        relocated = ekf_node.handle_camera_mismatch(
            camera_odom(0.55, -0.10, 0.02),
            index * 0.05,
        )

    assert relocated
    assert ekf_node.camera_relocalization_count == 1
    assert ekf_node.x[0, 0] == pytest.approx(0.55)
    assert ekf_node.x[1, 0] == pytest.approx(-0.10)
    assert ekf_node.x[2, 0] == pytest.approx(0.02)


def test_ekf_does_not_relocalize_for_short_glitch():
    ekf_node = _RelocalizationHarness()
    for index in range(10):
        assert not ekf_node.handle_camera_mismatch(
            camera_odom(0.55),
            index * 0.05,
        )

    assert ekf_node.camera_relocalization_count == 0
    assert ekf_node.x[0, 0] == pytest.approx(0.0)
