from amr_control.controller_modes import force_smc


class _Logger:
    def info(self, _message):
        return None


class _Node:
    k1 = 1.0
    k2 = 2.0
    k3 = 3.0
    k2_straight = 4.0
    k3_straight = 5.0
    LARGE_ERROR_K2 = 6.0
    LARGE_ERROR_K3 = 7.0
    Ks1 = 0.08
    Ks2 = 0.10
    phi1 = 1.0
    phi2 = 1.5
    yaw_bias_gain = 1.0
    yaw_bias_integral = 1.0
    yaw_feedforward = 1.0
    radius_feedback_gain = 1.0
    radius_position_gain = 1.0
    CENTER_K1 = 1.0
    CENTER_K2 = 2.0
    CENTER_K3 = 3.0
    KI_Y = 1.0
    i_y = 1.0
    KI_Y_STRAIGHT = 1.0
    ey_integral = 1.0

    def get_logger(self):
        return _Logger()


def test_circle_smc_removes_backstepping_and_adaptive_terms():
    node = _Node()
    force_smc(node, "circle")
    assert (node.k1, node.k2, node.k3) == (0.0, 0.0, 0.0)
    assert node.radius_feedback_gain == 0.0
    assert node.radius_position_gain == 0.0
    assert (node.Ks1, node.Ks2) == (0.08, 0.10)


def test_circle_smc_supplies_comparison_gains_for_zero_defaults():
    node = _Node()
    node.Ks1 = 0.0
    node.Ks2 = 0.0
    force_smc(node, "circle")
    assert (node.Ks1, node.Ks2) == (0.024, 0.050)


def test_eight_smc_removes_center_backstepping_terms():
    node = _Node()
    force_smc(node, "eight")
    assert (node.k1, node.k2, node.k3) == (0.0, 0.0, 0.0)
    assert (node.CENTER_K1, node.CENTER_K2, node.CENTER_K3) == (0.0, 0.0, 0.0)
    assert node.KI_Y == 0.0


def test_square_smc_removes_scheduled_backstepping_terms():
    node = _Node()
    force_smc(node, "square")
    assert node.k1 == 0.0
    assert (node.k2_straight, node.k3_straight) == (0.0, 0.0)
    assert (node.LARGE_ERROR_K2, node.LARGE_ERROR_K3) == (0.0, 0.0)
    assert node.KI_Y_STRAIGHT == 0.0
