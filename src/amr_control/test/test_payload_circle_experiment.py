import math
from types import SimpleNamespace

from amr_control.payload_circle_experiment import (
    condition_metadata,
    experiment_cases,
    payload_jz_com,
)


def _args(**overrides):
    values = {
        "base_mass": 1.55,
        "base_jz": None,
        "payload_length_m": None,
        "payload_width_m": None,
        "payload_jz_com": None,
        "offset_direction": "forward",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_seven_cases_have_requested_mass_and_offset_layout():
    cases = experiment_cases()
    assert len(cases) == 7
    assert [case["payload_mass_kg"] for case in cases] == [
        0.0, 0.5, 1.0, 1.5, 0.5, 1.0, 1.5
    ]
    assert [case["payload_offset_m"] for case in cases] == [
        0.0, 0.0, 0.0, 0.0, 0.05, 0.05, 0.05
    ]


def test_parallel_axis_increment_for_1kg_at_5cm():
    case = experiment_cases()[5]
    data = condition_metadata(case, _args(base_jz=0.02))
    assert math.isclose(data["total_mass_kg"], 2.55)
    assert math.isclose(data["payload_delta_jz_kg_m2"], 0.0025)
    assert math.isclose(data["total_jz_kg_m2"], 0.0225)
    assert math.isclose(data["payload_offset_x_m"], 0.05)
    assert math.isclose(data["payload_offset_y_m"], 0.0)


def test_rectangular_payload_centroidal_inertia():
    value, model = payload_jz_com(1.5, 0.20, 0.10, None)
    assert model == "rectangle"
    assert math.isclose(value, 1.5 * (0.20 ** 2 + 0.10 ** 2) / 12.0)


def test_explicit_payload_inertia_takes_priority():
    value, model = payload_jz_com(1.0, 0.20, 0.10, 0.004)
    assert value == 0.004
    assert model == "provided"
