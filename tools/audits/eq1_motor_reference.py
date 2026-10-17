#!/usr/bin/env python3
"""EQ-1B motor — the frozen production motor integrator against the intended continuous-time model (CPU).

    python -B tools/audits/eq1_motor_reference.py [--out docs/evidence/.../eq1b_motor.json]

Validation only. Policy F flies `navrl_quad` on `base_sim` (dt 0.01 s, one substep) with
`use_rps=True`, `use_discrete_approximation=True`, the default `integration_scheme='rk4'`, motor time
constants 0.04 s (min = max, no randomization) and `max_thrust_rate=1e5`, so the active routine is
`compute_thrust_with_rpm_time_constant_rk4`: the rotor speed w = sqrt(T / k_T) is advanced by
`rk4_integration(w_ref - w, 1/(dt + tau), max_rate, dt)`.

Intended continuous-time model (declared here, before execution): a first-order lag in rotor speed,
dw/dt = clamp((w_ref - w) / tau, -max_rate, max_rate), tau = 0.04 s, thrust T = k_T w^2. Its exact
solution over a step with constant reference is w_ref + (w - w_ref) exp(-dt / tau) while the rate
limit is inactive. References here are analytic; the production functions are only executed.

Gates (fixed before execution):
  M1 named-method algebra: the routine called rk4 equals classical RK4 for its own ODE
     dw/dt = k (w_ref - w) to 1e-12 per step (it is a polynomial identity for a linear ODE);
  M2 convergence: the step-response error against the intended model at t = 0.2 s shrinks with
     observed order >= 3.5 between dt = 10/5/2.5/1.25 ms (classical RK4 on the intended ODE: 4);
  M3 fixed-timestep preservation (for any proposed correction): per-step gain at dt = 0.01 s within
     0.5 % of the current production gain, so the frozen Policy F plant response is kept;
  M4 rate-limited regime: while saturated the increment equals dt * max_rate.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eq1_common as common  # noqa: E402

TAU = 0.04
DT = 0.01
MAX_RATE = 1.0e5
MAX_THRUST = 2.0
K_T = 9.26312e-6  # motor_thrust_constant_min; k_T cancels from the normalized response (checked below)
M1_TOL = 1e-12
M2_ORDER = 3.5
M3_REL = 0.005
DTS = (0.01, 0.005, 0.0025, 0.00125)
HORIZON_S = 0.2


# ------------------------------------------------------------------ references (no production import)
def exact_gain(dt, tau=TAU):
    """Fraction of the rotor-speed error removed in one step by the intended continuous model."""
    return 1.0 - math.exp(-dt / tau)


def classical_rk4_gain(dt, k):
    """Classical RK4 on dw/dt = k (w_ref - w): the error e obeys de/dt = -k e."""
    h = k * dt
    return h - h * h / 2 + h ** 3 / 6 - h ** 4 / 24


def classical_rk4_step(e, k, dt, max_rate):
    """Independent classical RK4 for dw/dt = clamp(k e, +-max_rate) with e = w_ref - w (returns dw)."""
    def rate(err):
        return max(-max_rate, min(max_rate, k * err))
    k1 = rate(e)
    k2 = rate(e - 0.5 * dt * k1)
    k3 = rate(e - 0.5 * dt * k2)
    k4 = rate(e - dt * k3)
    return dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def zoh_step(w, w_ref, dt, tau=TAU):
    """Proposed correction (reference code only): exact discretization of the intended lag."""
    return w_ref + (w - w_ref) * math.exp(-dt / tau)


def sign_flip_gain(dt, tau=TAU):
    """The naive 'replace + by -' edit that keeps k = 1/(dt + tau): a different, slower plant."""
    return classical_rk4_gain(dt, 1.0 / (dt + tau))


# ------------------------------------------------------------------ production execution
def production():
    mm = common.load_production("aerial_gym.control.motor_model")
    return mm


def production_gain(mm, dt, tau=TAU):
    e = torch.tensor([[1.0]], dtype=torch.float64)
    k = mm.discrete_mixing_factor(dt, torch.tensor([[tau]], dtype=torch.float64))
    return float(mm.rk4_integration(e, k, torch.tensor([[MAX_RATE]], dtype=torch.float64), dt))


def production_step_response(mm, dt, t0=0.25 * MAX_THRUST, tref=MAX_THRUST, k_t=K_T, horizon=HORIZON_S):
    """Run the active production thrust update for a thrust step; returns rotor-speed fraction at the horizon."""
    steps = int(round(horizon / dt))
    thrust = torch.tensor([[t0]], dtype=torch.float64)
    ref = torch.tensor([[tref]], dtype=torch.float64)
    kt = torch.tensor([[k_t]], dtype=torch.float64)
    k = mm.discrete_mixing_factor(dt, torch.tensor([[TAU]], dtype=torch.float64))
    rate = torch.tensor([[MAX_RATE]], dtype=torch.float64)
    for _ in range(steps):
        thrust = mm.compute_thrust_with_rpm_time_constant_rk4(ref, thrust.clone(), k, kt, rate, dt)
    w0, wr, w = math.sqrt(t0 / k_t), math.sqrt(tref / k_t), math.sqrt(float(thrust) / k_t)
    return (w - w0) / (wr - w0), float(thrust)


def exact_step_fraction(horizon=HORIZON_S):
    return 1.0 - math.exp(-horizon / TAU)


def observed_orders(errors):
    return [math.log(abs(a) / abs(b), 2) if b != 0 and a != 0 else None for a, b in zip(errors[:-1], errors[1:])]


def run():
    with common.production():
        return _run()


def _run():
    mm = production()
    # M1 algebra: production per-step gain against classical RK4 of the same ODE and coefficient.
    algebra = []
    for dt, tau in ((0.01, 0.04), (0.005, 0.04), (0.0025, 0.04), (0.04, 0.04), (0.1, 0.04), (0.01, 0.02)):
        k = 1.0 / (dt + tau)
        g = production_gain(mm, dt, tau)
        h = k * dt
        algebra.append({"dt": dt, "tau": tau, "k": k, "h": h, "production_gain": g,
                        "production_closed_form": h + h * h / 2 + h ** 3 / 6 + h ** 4 / 24,
                        "classical_rk4_same_ode": classical_rk4_gain(dt, k),
                        "exact_same_ode": 1 - math.exp(-h), "exact_intended_tau": exact_gain(dt, tau),
                        "backward_euler_intended_tau": dt / (dt + tau),
                        "production_effective_tau_s": (-dt / math.log(1 - g)) if g < 1 else None,
                        "abs_error_vs_classical_rk4": abs(g - classical_rk4_gain(dt, k))})
    m1_pass = all(r["abs_error_vs_classical_rk4"] <= M1_TOL for r in algebra)

    # M2 convergence against the intended continuous model (thrust step 25 % -> 100 %).
    target = exact_step_fraction()
    conv = []
    for dt in DTS:
        frac, thrust = production_step_response(mm, dt)
        conv.append({"dt": dt, "production_fraction": frac, "error": frac - target, "thrust_N": thrust})
    orders = observed_orders([r["error"] for r in conv])
    ref_err = []
    for dt in DTS:
        w = 0.0
        for _ in range(int(round(HORIZON_S / dt))):
            w += classical_rk4_step(1.0 - w, 1.0 / TAU, dt, MAX_RATE)
        ref_err.append(w - target)
    ref_orders = observed_orders(ref_err)
    m2_pass = all(o is not None and o >= M2_ORDER for o in orders)

    # k_T cancellation (the normalized transient does not depend on the thrust constant).
    kt_check = [production_step_response(mm, DT, k_t=k)[0] for k in (9.26312e-6, 1.826312e-5)]

    # M3 preservation of the frozen fixed-timestep response.
    g_prod = production_gain(mm, DT)
    candidates = {
        "zoh_exact_nominal_tau": exact_gain(DT),
        "naive_sign_flip_keep_coefficient": sign_flip_gain(DT),
        "classical_rk4_continuous_coefficient": classical_rk4_gain(DT, 1.0 / TAU),
        "backward_euler_discrete_coefficient": DT / (DT + TAU),
    }
    preservation = {name: {"gain_dt_0.01": g, "relative_change": g / g_prod - 1.0,
                           "effective_tau_s": -DT / math.log(1 - g),
                           "status": "PASS" if abs(g / g_prod - 1.0) <= M3_REL else "FAIL"}
                    for name, g in candidates.items()}
    zoh_conv = []
    for dt in DTS:
        w = 0.0
        for _ in range(int(round(HORIZON_S / dt))):
            w = zoh_step(w, 1.0, dt)
        zoh_conv.append({"dt": dt, "error": w - target})

    # M4 rate-limited regime: small max_rate so every RK4 stage saturates.
    e = torch.tensor([[100.0]], dtype=torch.float64)
    k = torch.tensor([[20.0]], dtype=torch.float64)
    sat = float(mm.rk4_integration(e, k, torch.tensor([[50.0]], dtype=torch.float64), DT))
    max_error_rps = math.sqrt(MAX_THRUST / K_T)
    return {
        "schema": "motar.eq1b-motor.v1", "package": "EQ-1B motor integrator (CPU, validation only)",
        "source": common.source_binding(["aerial_gym/control/motor_model.py",
                                         "aerial_gym/config/robot_config/base_quad_config.py",
                                         "aerial_gym/config/robot_config/navrl_quad_config.py",
                                         "aerial_gym/config/sim_config/base_sim_config.py"]),
        "config": {"dt_s": DT, "tau_s": TAU, "use_rps": True, "use_discrete_approximation": True,
                   "integration_scheme": "rk4 (default)", "max_thrust_rate": MAX_RATE, "max_thrust_N": MAX_THRUST,
                   "rate_limit_active_in_policy_f": (1.0 / (DT + TAU)) * max_error_rps > MAX_RATE,
                   "largest_rate_rps_per_s": (1.0 / (DT + TAU)) * max_error_rps},
        "M1_named_method_algebra": {"rows": algebra, "gate": "|production - classical RK4| <= %g" % M1_TOL,
                                    "production": common.verdict(m1_pass, "k2..k4 evaluate e + dt k/2: the error grows "
                                                                 "instead of shrinking; gain is the e^h - 1 series")},
        "M2_convergence": {"horizon_s": HORIZON_S, "exact_fraction": target, "rows": conv, "observed_orders": orders,
                           "classical_rk4_intended_ode_errors": ref_err, "classical_rk4_observed_orders": ref_orders,
                           "gate": "observed order >= %.1f" % M2_ORDER,
                           "production": common.verdict(m2_pass, "global order 3: with x = dt/tau the gain is "
                                                        "x - x^2/2 + x^3/6 + x^4/24 against 1 - exp(-x); the sign error "
                                                        "and k = 1/(dt + tau) cancel through x^3, local error x^4/12"),
                           "proposed_correction_errors": zoh_conv,
                           "proposed_correction": common.verdict(max(abs(r["error"]) for r in zoh_conv) <= 1e-12,
                                                                 "exact for a constant reference at every dt")},
        "k_T_cancellation": {"fractions": kt_check, "equal": abs(kt_check[0] - kt_check[1]) < 1e-12},
        "M3_fixed_timestep_preservation": {"production_gain_dt_0.01": g_prod,
                                           "production_effective_tau_s": -DT / math.log(1 - g_prod),
                                           "gate": "|gain / production - 1| <= %.3f" % M3_REL, "candidates": preservation},
        "M4_rate_limited": {"increment": sat, "expected": DT * 50.0,
                            "production": common.verdict(abs(sat - DT * 50.0) < 1e-12, "saturated stages are exact"),
                            "note": "not exercised by Policy F: the largest rate is below max_thrust_rate"},
        "composite_response_note": "at dt = 0.01 s the production gain 0.2214 implies tau_eff = 0.03996 s, within 0.1 % "
                                   "of the nominal 0.04 s: the frozen OCC plant behaves like the intended lag at its own "
                                   "timestep (a third-order-accurate composite by cancellation), but the routine named "
                                   "rk4 is not classical RK4 for its ODE; the naive sign flip gives a 0.05 s motor",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    record = run()
    if args.out:
        common.write_record(args.out, record)
    print("M1", record["M1_named_method_algebra"]["production"]["status"],
          "| M2", record["M2_convergence"]["production"]["status"], record["M2_convergence"]["observed_orders"],
          "| M3", {k: v["status"] for k, v in record["M3_fixed_timestep_preservation"]["candidates"].items()})
    return 0


if __name__ == "__main__":
    sys.exit(main())
