import math

import pytest

from amr_control.square_profiles import (
    corner_forward_speed,
    get_square_profile,
    profile_name_for_side,
    scheduled_lateral_gain,
    snap_cardinal_heading,
    straight_integral_gate,
    straight_integral_output_gate,
)


@pytest.mark.parametrize(
    ("heading_deg", "expected_deg"),
    ((1.5, 0.0), (-7.0, 0.0), (88.0, 90.0), (-92.0, -90.0), (178.0, 180.0)),
)
def test_near_cardinal_start_heading_is_snapped(heading_deg, expected_deg):
    snapped = snap_cardinal_heading(math.radians(heading_deg), 10.0)
    error = math.atan2(
        math.sin(snapped - math.radians(expected_deg)),
        math.cos(snapped - math.radians(expected_deg)),
    )
    assert error == pytest.approx(0.0)


def test_intentionally_diagonal_start_heading_is_preserved():
    heading = math.radians(20.0)
    assert snap_cardinal_heading(heading, 10.0) == pytest.approx(heading)


@pytest.mark.parametrize(
    ("name", "side_length", "corner_distance"),
    (("1m", 1.0, 0.15), ("2m", 2.0, 0.25)),
)
def test_square_profile_contains_guarded_integral_settings(
    name, side_length, corner_distance
):
    profile = get_square_profile(name)
    assert profile["side_length"] == side_length
    assert profile["sharp_corner_w"] == 0.55
    assert profile["max_w"] == 0.85
    expected_ki = 0.08
    expected_limit = 0.30 if name == "1m" else 0.25
    assert profile["ki_y_straight"] == expected_ki
    assert profile["ey_integral_limit"] == expected_limit
    assert profile["integral_corner_distance"] == corner_distance
    assert profile["integral_heading_limit_deg"] == 5.0
    expected_initial_integral = 0.0 if name == "1m" else 0.20
    assert profile["corner_min_wheel_ratio"] == 0.15
    assert profile["corner_preempt_start_deg"] == 80.0
    assert profile["corner_preempt_end_deg"] == 35.0
    assert profile["corner_preempt_ratio"] == 0.30
    assert profile["corner_preempt_w_scale"] == 0.82
    assert profile["corner_stall_rpm"] == 6.0
    assert profile["corner_stall_recovery_ratio"] == 0.35
    assert profile["corner_stall_w_scale"] == 0.64
    assert profile["large_error_k3"] == 7.0
    assert profile["initial_ey_integral"] == expected_initial_integral


def test_square_profile_result_is_independent():
    first = get_square_profile("2m")
    first["ki_y_straight"] = 99.0
    assert get_square_profile("2m")["ki_y_straight"] == 0.08


@pytest.mark.parametrize(("side", "name"), ((1.0, "1m"), (2.0, "2m")))
def test_profile_name_for_validated_side(side, name):
    assert profile_name_for_side(side) == name


def test_integral_gate_is_on_only_for_an_established_straight():
    heading_limit = 5.0 * 3.141592653589793 / 180.0
    middle = straight_integral_gate(0.30, 0.25, 0.0, heading_limit, 0.0)
    corner = straight_integral_gate(0.00, 0.25, 0.0, heading_limit, 0.0)
    misaligned = straight_integral_gate(
        0.30, 0.25, heading_limit, heading_limit, 0.0
    )
    turning = straight_integral_gate(0.30, 0.25, 0.0, heading_limit, 0.10)
    assert middle == 1.0
    assert corner == 0.0
    assert misaligned == 0.0
    assert turning == 0.0


def test_integral_gate_is_smooth_and_bounded():
    heading_limit = 5.0 * 3.141592653589793 / 180.0
    gate = straight_integral_gate(
        0.125, 0.25, 0.5 * heading_limit, heading_limit, 0.05
    )
    assert gate == pytest.approx(0.125)
    assert 0.0 <= gate <= 1.0


def test_integral_output_remains_active_for_opposing_heading_error():
    output_gate = straight_integral_output_gate(0.30, 0.25, 0.0)
    learn_gate = straight_integral_gate(
        0.30,
        0.25,
        5.0 * 3.141592653589793 / 180.0,
        5.0 * 3.141592653589793 / 180.0,
        0.0,
    )
    assert learn_gate == 0.0
    assert output_gate == 1.0


def test_integral_output_is_disabled_at_corners_and_during_reference_turn():
    assert straight_integral_output_gate(0.0, 0.25, 0.0) == 0.0
    assert straight_integral_output_gate(0.30, 0.25, 0.10) == 0.0


def test_lateral_gain_is_strong_only_for_large_error():
    assert scheduled_lateral_gain(6.0, 10.0, 0.02, 0.04) == 6.0
    assert scheduled_lateral_gain(6.0, 10.0, 0.08, 0.04) == 10.0
    middle = scheduled_lateral_gain(6.0, 10.0, 0.06, 0.04)
    assert middle == pytest.approx(8.0)


def test_corner_speed_keeps_inner_wheel_rolling():
    speed = corner_forward_speed(0.02, 0.55, 0.17, 0.15, 1.0)
    inner = speed - 0.5 * 0.55 * 0.17
    outer = speed + 0.5 * 0.55 * 0.17
    assert speed == pytest.approx(0.06325)
    assert inner / outer == pytest.approx(0.15)


def test_corner_speed_does_not_change_straight_command():
    assert corner_forward_speed(0.10, 0.02, 0.17, 0.25, 0.0) == 0.10
