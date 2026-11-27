"""EQ-0 algebra counterexamples; stdlib only, no simulator/production function imports.

Success means the documented counterexamples reproduce, not environment qualification PASS.
This reads the audit's equations/evidence and prints results; it never edits input evidence.
"""
import json
import math
from pathlib import Path


def reproduce():
    directory = Path(__file__).resolve().parent
    cases = json.loads((directory / 'supporting/g1/motor_algebra.json').read_text())['cases']
    motor = []
    for case in cases:
        dt, tau = case['dt'], case['tau']
        k = 1.0 / (dt + tau)
        h = k * dt
        # Independent integration of y'=k*(reference-y): exact exponential and its Taylor polynomial.
        exact = 1.0 - math.exp(-h)
        rk4 = h - h*h/2 + h**3/6 - h**4/24
        # Algebraic expansion of the recorded plus-error stages, without calling that routine.
        recorded_stages_gain = h + h*h/2 + h**3/6 + h**4/24
        nominal = 1.0 - math.exp(-dt / tau)
        assert math.isclose(recorded_stages_gain, case['production_increment_fraction'], abs_tol=1e-14)
        assert math.isclose(rk4, case['classical_RK4_same_coefficient_increment_fraction'], abs_tol=1e-14)
        assert not math.isclose(recorded_stages_gain, rk4, abs_tol=1e-12)
        motor.append({'dt': dt, 'tau': tau, 'recorded_stages_gain': recorded_stages_gain,
                      'classical_RK4_gain': rk4, 'exact_same_ODE_gain': exact,
                      'exact_nominal_tau_gain': nominal})
    width, hfov = 160, math.radians(87)
    cx = (width - 1) / 2
    fx = cx / math.tan(hfov / 2)
    theta = math.radians(20)
    pinhole_u = cx - fx * math.tan(theta)
    linear_u = round((hfov / 2 - theta) / hfov * (width - 1))
    selected_theta = math.atan((cx - linear_u) / fx)
    bearing_error = abs(math.degrees(selected_theta - theta))
    pixel_error = abs(pinhole_u - linear_u)
    assert bearing_error > 3 and pixel_error > 5
    stop_distance = 2.0 * 0.1 + 2.0**2 / (2.0 * 2.9608856678)
    clearance, margin = 0.1, 0.45
    # Riskcap at clearance<3 holds its configured 2 m/s cap; stopping admissibility is a separate oracle.
    required_clearance = margin + stop_distance
    stopping_margin = max(0.0, clearance - margin) - stop_distance
    assert required_clearance > 1.3 and stopping_margin < 0
    return {'scope': 'Independent algebra and fixture counterexamples only; no EQ-1 rollout.',
            'named_RK4_gate': 'FAIL', 'motor_cases': motor,
            'calibrated_camera_lidar_registration_gate': 'FAIL',
            'camera_fixture': {'correct_pinhole_u': pinhole_u, 'selected_linear_u': linear_u,
                               'angular_error_deg': bearing_error, 'pixel_error': pixel_error,
                               'aligned_front_plane_origin_bias_m': -0.1},
            'riskcap_stopping_guarantee_gate': 'FAIL',
            'stopping_fixture': {'clearance_m': clearance, 'capped_request_mps': 2.0,
                                 'required_clearance_m': required_clearance,
                                 'source_telemetry_stopping_margin_m': stopping_margin}}


if __name__ == '__main__':
    print(json.dumps(reproduce(), indent=2, sort_keys=True))
