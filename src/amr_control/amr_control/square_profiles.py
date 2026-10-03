"""Hardware profiles for continuous sharp-corner squares."""

import math
from copy import deepcopy


def _smoothstep01(value):
    value = max(0.0, min(1.0, float(value)))
    return value * value * (3.0 - 2.0 * value)


def snap_cardinal_heading(angle, tolerance_deg=10.0):
    """Snap a near-cardinal heading to the exact closest 90-degree axis.

    Headings outside the tolerance are preserved, allowing an intentionally
    diagonal square while removing small camera-yaw setup errors near
    0/90/180/-90 degrees.
    """
    angle = math.atan2(math.sin(float(angle)), math.cos(float(angle)))
    quarter_turn = 0.5 * math.pi
    nearest = round(angle / quarter_turn) * quarter_turn
    error = math.atan2(math.sin(angle - nearest), math.cos(angle - nearest))
    tolerance = math.radians(max(0.0, float(tolerance_deg)))
    if abs(error) <= tolerance:
        return math.atan2(math.sin(nearest), math.cos(nearest))
    return angle


def straight_integral_gate(
    corner_distance,
    corner_gate_distance,
    e_theta,
    heading_limit,
    reference_heading_rate,
):
    """Smoothly enable integral action only on an established straight."""
    distance_gate = _smoothstep01(
        corner_distance / max(0.01, corner_gate_distance)
    )
    heading_gate = _smoothstep01(
        1.0 - abs(e_theta) / max(1e-6, heading_limit)
    )
    reference_rate_gate = _smoothstep01(
        1.0 - abs(reference_heading_rate) / 0.10
    )
    return distance_gate * heading_gate * reference_rate_gate


def straight_integral_output_gate(
    corner_distance,
    corner_gate_distance,
    reference_heading_rate,
):
    """Enable an already learned steering bias on established straights.

    Heading error intentionally does not gate this output.  A persistent
    lateral offset commonly coexists with a few degrees of opposite heading
    error, so multiplying the learned bias by the heading-learning gate would
    remove the correction precisely when it is needed.
    """
    distance_gate = _smoothstep01(
        corner_distance / max(0.01, corner_gate_distance)
    )
    reference_rate_gate = _smoothstep01(
        1.0 - abs(reference_heading_rate) / 0.10
    )
    return distance_gate * reference_rate_gate


def scheduled_lateral_gain(base_gain, large_error_gain, lateral_error, threshold):
    """Use stronger recovery far from a line and the validated gain near it."""
    error = abs(float(lateral_error))
    threshold = max(1e-6, float(threshold))
    blend = _smoothstep01((error - threshold) / threshold)
    return float(base_gain) + blend * (
        float(large_error_gain) - float(base_gain)
    )


def corner_forward_speed(
    linear_command,
    angular_command,
    wheel_base,
    minimum_wheel_ratio,
    corner_blend,
):
    """Raise forward speed so the inner wheel keeps rolling through a corner.

    For differential drive, v_inner=v-|w|L/2 and v_outer=v+|w|L/2.
    The requested ratio is blended in with the corner law.  At zero blend the
    command is unchanged; at full blend, v_inner/v_outer is at least the
    requested positive ratio.
    """
    blend = _smoothstep01(corner_blend)
    ratio = max(0.0, min(0.80, float(minimum_wheel_ratio))) * blend
    if blend <= 0.0 or abs(float(angular_command)) <= 1e-9:
        return float(linear_command)
    half_delta = 0.5 * abs(float(angular_command)) * float(wheel_base)
    required = half_delta * (1.0 + ratio) / max(1e-6, 1.0 - ratio)
    return max(float(linear_command), required)


COMMON_SQUARE_PROFILE = {
    "corner_speed": 0.04,
    "desired_speed": 0.10,
    "min_v": 0.02,
    "max_w": 0.85,
    # Keep the corner law validated by Paper1-cu. Raising this to 0.85 rad/s
    # made the outer wheel run hard while the inner wheel was nearly stationary.
    "sharp_corner_w": 0.55,
    "sharp_corner_blend_full_deg": 35.0,
    "k3": 7.0,
    # Slow square-only integral action removes the repeatable lateral bias
    # that remains when k2*e_y and k3*sin(e_theta) cancel on a straight edge.
    "ki_y_straight": 0.08,
    "ey_integral_limit": 0.30,
    "initial_ey_integral": 0.0,
    "integral_heading_limit_deg": 5.0,
    # Preserve the tight Paper1-cu corner with a small positive baseline inner
    # wheel speed.  Hardware r21 showed that raising this baseline to 0.20
    # widened the corner but did not remove the delayed mid-turn stall.
    "corner_min_wheel_ratio": 0.15,
    # Pre-empt the measured high-load portion of the turn instead of waiting
    # for the inner-wheel RPM to reach zero.  The stronger reactive ratio below
    # remains as a fallback.
    "corner_preempt_start_deg": 80.0,
    "corner_preempt_end_deg": 35.0,
    "corner_preempt_ratio": 0.30,
    "corner_preempt_w_scale": 0.82,
    "corner_stall_rpm": 6.0,
    "corner_stall_recovery_ratio": 0.35,
    "corner_stall_w_scale": 0.64,
    "corner_stall_recovery_hold": 0.30,
    "large_error_ey": 0.04,
    # Hardware r19 showed that reducing k3 after a corner increased straight
    # waviness (2.79 cm vs 2.46 cm) and heading RMS (5.76 deg vs 4.58 deg).
    # Keep the validated Paper1-cu heading gain at every lateral error.
    "large_error_k3": 7.0,
}


SQUARE_PROFILES = {
    # Baseline 1 m run before integral action: path RMS 1.58 cm, ripple 0.67 cm,
    # straight heading RMS 1.91 deg.
    "1m": {
        "side_length": 1.0,
        "corner_decel_distance": 0.13,
        "sharp_corner_blend_start_deg": 12.0,
        "k2": 4.0,
        "kd_w": 0.0,
        "kd_w_deadband": 0.20,
        "integral_corner_distance": 0.15,
        "large_error_k2": 4.0,
    },
    # Baseline 2 m run before integral action: path RMS 2.88 cm, ripple 0.90 cm,
    # straight heading RMS 1.93 deg.
    "2m": {
        "side_length": 2.0,
        "corner_speed": 0.04,
        "corner_decel_distance": 0.25,
        "sharp_corner_blend_start_deg": 8.0,
        # Paper1-cu accepted hardware profile. Larger k2=10 corrected faster
        # after a corner but increased straight-edge waviness.
        "k2": 6.0,
        "kd_w": 0.18,
        "kd_w_deadband": 0.20,
        "ki_y_straight": 0.08,
        "ey_integral_limit": 0.25,
        # r15 learned approximately +0.20 on the first edge, after which
        # edges 2-4 had sub-centimetre mean lateral bias. Seed that repeatable
        # steering bias so edge 1 does not spend its full length learning it.
        "initial_ey_integral": 0.20,
        "integral_corner_distance": 0.25,
        "large_error_k2": 10.0,
    },
}


def get_square_profile(name):
    """Return an independent, complete profile dictionary."""
    if name not in SQUARE_PROFILES:
        choices = ", ".join(sorted(SQUARE_PROFILES))
        raise ValueError(f"unknown square profile {name!r}; choose {choices}")
    profile = deepcopy(COMMON_SQUARE_PROFILE)
    profile.update(SQUARE_PROFILES[name])
    profile["name"] = name
    return profile


def profile_name_for_side(side_length):
    """Map the two validated side lengths to their named profile."""
    side = float(side_length)
    for name, values in SQUARE_PROFILES.items():
        if abs(side - values["side_length"]) <= 1e-6:
            return name
    raise ValueError(
        f"no validated profile for side_length={side:g}; use 1.0 or 2.0"
    )
