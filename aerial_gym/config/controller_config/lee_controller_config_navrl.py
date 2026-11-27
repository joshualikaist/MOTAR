import os

from aerial_gym.config.controller_config.lee_controller_config import control as _base

try:
    # Same env var the task reads, so the two ceilings stay in lockstep (raise to fly faster).
    _NAVRL_YAW_RATE_MAX = float(os.environ.get("NAVRL_YAW_RATE_MAX", "").strip() or "2.5")
except ValueError:
    _NAVRL_YAW_RATE_MAX = 2.5




def _gain_rand_from_env():
    """Opt-in controller-gain randomization (D-2026-11-04-1; docs/plans/prereg_gain_randomization_gr0_v1.md).

    Unset or "0": None, and the config is byte-for-byte the historical one (randomize_params False, midpoint gains).
    "1": (scale, seed). At every robot reset each env's K_vel, K_rot and K_angvel elements are drawn independently and
    uniformly in nominal * [1 - scale, 1 + scale], nominal being the midpoint gains Policy F was trained with, from a
    dedicated torch.Generator seeded with NAVRL_GAIN_RAND_SEED, so the global RNG stream is not consumed. Both
    NAVRL_GAIN_RAND_SCALE (0 <= scale < 1) and NAVRL_GAIN_RAND_SEED are required; anything else refuses."""
    raw = os.environ.get("NAVRL_GAIN_RAND", "").strip()
    if raw in ("", "0"):
        return None
    if raw != "1":
        raise ValueError("NAVRL_GAIN_RAND must be unset, 0 or 1, got %r" % raw)
    try:
        scale = float(os.environ["NAVRL_GAIN_RAND_SCALE"])
        seed = int(os.environ["NAVRL_GAIN_RAND_SEED"])
    except (KeyError, ValueError) as exc:
        raise ValueError("NAVRL_GAIN_RAND=1 needs NAVRL_GAIN_RAND_SCALE and NAVRL_GAIN_RAND_SEED") from exc
    if not (0.0 <= scale < 1.0) or seed < 0:
        raise ValueError("NAVRL_GAIN_RAND_SCALE must be in [0, 1) and NAVRL_GAIN_RAND_SEED >= 0")
    return scale, seed


GAIN_RAND = _gain_rand_from_env()


class control(_base):
    """NavRL-scoped Lee velocity controller config.

    Raises ONLY the yaw-rate ceiling from the shared default pi/3 (~1.047 rad/s). The task fallback
    is 2.5 rad/s; canonical v2 launchers pin 3.0 rad/s so the
    learned 4th action (yaw-rate, option b) has the authority to track the ~1.75-2.4 rad/s velocity-
    direction slew in the tight weaves that cause corner-clips. Everything else is inherited from
    lee_controller_config. Scoped here (rather than editing the shared config) so position_setpoint_task
    and every other task using lee_velocity_control keeps the pi/3 clamp unchanged.
    """

    max_yaw_rate = _NAVRL_YAW_RATE_MAX  # [rad/s]; MUST equal navrl_task_config.yaw_rate_max (same env var)

    # Altitude-priority thrust: T = f_z / cos(tilt) instead of T = f·b3 (see velocity_control.py).
    # Cancels the deterministic vertical-force sag during attitude-lag transients (weave reversals),
    # measured as `below` floor strikes -- worst under NAVRL_GENERAL_TRAIN's random spawns, where
    # sharp initial turns happen while the task-level altitude-PI integral is still zero.
    # NavRL-scoped opt-in; every other task keeps the shared Lee projection unchanged.
    # Keep boolean semantics identical to NavRLTask checkpoint provenance. Previously "false"
    # meant controller=ON but checkpoint=OFF, making a same-shape control mismatch look verified.
    tilt_thrust_compensation = os.environ.get("NAVRL_TILT_COMP", "1").strip().lower() not in (
        "0",
        "false",
        "no",
        "off",
    )

    # Maximum commanded tilt from vertical [deg], 0 disables. The base Lee controller derives its
    # desired body-z axis straight from the force vector, so nothing otherwise stops a large
    # horizontal command from asking the drone to pitch past 90 deg and flip -- which a from-scratch
    # policy does constantly (floor strikes were 73-87% of all crashes in a fresh run's first
    # epochs). 45 deg still allows g*tan(45) = 9.8 m/s^2 of lateral acceleration, a 0.64 m turn
    # radius at 2.5 m/s, so it bounds attitude without costing the agility the weave needs.
    # NavRL-scoped: every other task keeps the unlimited default.
    max_tilt_angle_deg = float(os.environ.get("NAVRL_MAX_TILT_DEG", "").strip() or 45.0)

    if GAIN_RAND is not None:
        randomize_params = True
        gain_rand_scale, gain_rand_seed = GAIN_RAND
