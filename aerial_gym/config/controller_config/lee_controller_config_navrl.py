import os

from aerial_gym.config.controller_config.lee_controller_config import control as _base

try:
    # Same env var the task reads, so the two ceilings stay in lockstep (raise to fly faster).
    _NAVRL_YAW_RATE_MAX = float(os.environ.get("NAVRL_YAW_RATE_MAX", "").strip() or "2.5")
except ValueError:
    _NAVRL_YAW_RATE_MAX = 2.5


class control(_base):
    """NavRL-scoped Lee velocity controller config.

    Raises ONLY the yaw-rate ceiling from the shared default pi/3 (~1.047 rad/s) to 2.5 rad/s so the
    learned 4th action (yaw-rate, option b) has the authority to track the ~1.75-2.4 rad/s velocity-
    direction slew in the tight weaves that cause corner-clips. Everything else is inherited from
    lee_controller_config. Scoped here (rather than editing the shared config) so position_setpoint_task
    and every other task using lee_velocity_control keeps the pi/3 clamp unchanged.
    """

    max_yaw_rate = _NAVRL_YAW_RATE_MAX  # [rad/s]; MUST equal navrl_task_config.yaw_rate_max (same env var)
