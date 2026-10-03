import math

import numpy as np

from amr_control.three_controller_report import (
    _align_circle_disturbance_phase,
    _common_active_duration,
    _common_time_mask,
    _disturbance_anchor_index,
    _event_index,
    _trajectory_start_index,
)


def test_common_time_mask_uses_shortest_active_run_duration():
    runs = {
        "Backstepping": {
            "t": np.arange(7, dtype=float),
            "cmd_v": np.array([0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]),
        },
        "BSMC": {
            "t": np.arange(9, dtype=float),
            "cmd_v": np.array(
                [0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]
            ),
        },
        "SMC": {
            "t": np.arange(5, dtype=float),
            "cmd_v": np.array([0.0, 0.1, 0.1, 0.1, 0.0]),
        },
    }
    controllers = ("Backstepping", "BSMC", "SMC")

    duration = _common_active_duration(runs, controllers)

    assert duration == 2.0
    for controller in controllers:
        relative_t, mask = _common_time_mask(
            runs[controller], duration
        )
        assert relative_t[mask].min() == 0.0
        assert relative_t[mask].max() == duration


def _circle_data(phase, camera_offset):
    angle = np.arange(8, dtype=float) * (0.5 * math.pi) + phase
    desired_x = np.cos(angle)
    desired_y = 1.0 + np.sin(angle)
    paused = np.zeros(8, dtype=float)
    paused[4:6] = 1.0
    event = np.zeros(8, dtype=float)
    event[6] = 1.0
    return {
        "t": np.arange(8, dtype=float),
        "desired_x_local": desired_x,
        "desired_y_local": desired_y,
        "camera_x_local": desired_x + camera_offset[0],
        "camera_y_local": desired_y + camera_offset[1],
        "camera_age_s": np.zeros(8, dtype=float),
        "cmd_v": np.full(8, 0.1, dtype=float),
        "control_paused": paused,
        "disturbance_event": event,
    }


def test_circle_disturbance_phase_alignment_uses_common_camera_anchor():
    data = {
        "Backstepping": _circle_data(0.5 * math.pi, (0.02, -0.01)),
        "BSMC": _circle_data(0.0, (0.01, 0.02)),
        "SMC": _circle_data(math.pi, (-0.03, 0.01)),
    }

    aligned, metadata = _align_circle_disturbance_phase(
        data, ("Backstepping", "BSMC", "SMC")
    )

    common = np.array([
        metadata["common_anchor_x_m"],
        metadata["common_anchor_y_m"],
    ])
    common_start = np.array([
        metadata["common_start_x_m"],
        metadata["common_start_y_m"],
    ])
    for controller, transformed in aligned.items():
        event_index = _event_index(data[controller])
        anchor_index = _disturbance_anchor_index(
            data[controller], event_index
        )
        actual = np.array([
            transformed["camera_x_local"][anchor_index],
            transformed["camera_y_local"][anchor_index],
        ])
        np.testing.assert_allclose(actual, common, atol=1e-12)
        start_index = _trajectory_start_index(data[controller])
        actual_start = np.array([
            transformed["camera_x_local"][start_index],
            transformed["camera_y_local"][start_index],
        ])
        np.testing.assert_allclose(
            actual_start, common_start, atol=1e-12
        )

    assert metadata["reference_controller"] == "BSMC"
    np.testing.assert_allclose(
        data["Backstepping"]["desired_x_local"],
        np.cos(np.arange(8) * (0.5 * math.pi) + 0.5 * math.pi),
    )
