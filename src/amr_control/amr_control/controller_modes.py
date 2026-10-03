#!/usr/bin/env python3
"""Controller-family configuration shared by the paper runners.

The trajectory generators, actuator limits, safety guards, and hardware
compensation remain identical.  Only the feedback family is changed:

* Backstepping: equivalent/feedforward + Backstepping feedback.
* BSMC: Backstepping + bounded sliding-mode injection.
* SMC: equivalent/feedforward + bounded sliding-mode injection, with the
  Backstepping proportional feedback removed.
"""


def force_smc(node, trajectory):
    """Configure an existing BSMC trajectory node as the pure-SMC baseline.

    ``ks1``, ``ks2``, ``phi1``, and ``phi2`` are deliberately retained from
    ROS parameters so the SMC boundary layer can be tuned without changing
    source code.  Controller-independent path generation, command limits,
    corner guards, and yaw-rate actuator compensation are also retained.
    """
    trajectory = str(trajectory).strip().lower()
    if trajectory not in {"circle", "eight", "square"}:
        raise ValueError(f"unsupported SMC trajectory: {trajectory!r}")

    # Circle's direct-node defaults are the nominal Backstepping gains and are
    # therefore zero.  Give the standalone SMC executable a usable,
    # comparison-matched injection unless the operator supplied nonzero gains.
    if trajectory == "circle" and abs(node.Ks1) < 1e-12 and abs(node.Ks2) < 1e-12:
        node.Ks1 = 0.024
        node.Ks2 = 0.050

    # Remove longitudinal Backstepping feedback.
    if hasattr(node, "k1"):
        node.k1 = 0.0

    if trajectory == "circle":
        node.k2 = 0.0
        node.k3 = 0.0
        # These terms are nonlinear/adaptive path feedback rather than the
        # equivalent SMC control, so disable them for a clean SMC baseline.
        node.yaw_bias_gain = 0.0
        node.yaw_bias_integral = 0.0
        node.yaw_feedforward = 0.0
        node.radius_feedback_gain = 0.0
        node.radius_position_gain = 0.0
    elif trajectory == "eight":
        node.k2 = 0.0
        node.k3 = 0.0
        node.CENTER_K1 = 0.0
        node.CENTER_K2 = 0.0
        node.CENTER_K3 = 0.0
        node.KI_Y = 0.0
        node.i_y = 0.0
        node.yaw_bias_gain = 0.0
        node.yaw_bias_integral = 0.0
        node.yaw_feedforward = 0.0
    else:
        node.k2_straight = 0.0
        node.k3_straight = 0.0
        node.LARGE_ERROR_K2 = 0.0
        node.LARGE_ERROR_K3 = 0.0
        node.KI_Y_STRAIGHT = 0.0
        node.ey_integral = 0.0

    node.get_logger().info(
        "Pure SMC baseline active: Backstepping proportional feedback is "
        "disabled; bounded sliding injection and shared trajectory/safety "
        f"logic remain active (Ks1={node.Ks1:.6g}, Ks2={node.Ks2:.6g}, "
        f"phi1={node.phi1:.6g}, phi2={node.phi2:.6g})."
    )
