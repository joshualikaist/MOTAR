"""Opt-in vehicle (mass / motor) randomization for the real-vehicle study (2026-11-08; survey
~/motar_tmp/realveh_design_20261108/SURVEY.md section 1, "Mass/motor switch"; IMPLEMENTATION.md part 3).

Default OFF: nothing imports this module unless NAVRL_VEH_RAND=1, and then only tools/realveh_entry.py does. Every
value is required from the environment; there are no defaults here, so an incomplete or out-of-range setting refuses
the run:

  NAVRL_VEH_RAND_SEED         int >= 0, seeds this module's own torch.Generator (the global RNG is never consumed)
  NAVRL_VEH_RAND_TW_NOMINAL   nominal thrust-to-weight in [1, MAX_TW_NOMINAL = 6]. The per-motor thrust limit becomes
                              limit = TW_NOMINAL * m * |g| / n_motors, with m and g read at run time from the robot's
                              controller (`controller.mass`, `controller.gravity`: the PhysX total mass and the sim
                              gravity the Lee controller itself uses), n_motors from the motor model. Against the
                              robot's own T/W (sum of max_thrust / (m |g|) from the robot's own tensors; navrl_quad
                              3.2619774 for m = 0.25 kg and the float32 gravity 9.81):
                                below it ("lower", CAM-K2 v1): the reference is capped at the limit, min(ref, limit);
                                above it, only with NAVRL_VEH_RAND_LIMIT_MODE=raise ("raise", CAM-K2 v2, 2026-11-08):
                                  the motor model's run-time max_thrust tensor, which clamps the reference
                                  (motor_model.py update_motor_thrusts), is replaced by the limit after init, so
                                  thrust up to the limit is delivered; the file and the config are not edited;
                                  without it, refuses (as committed for v1);
                                equal to it (relative 1e-6): no change ("none"). In raise mode "cap" in the record
                                  and log field names (cap_applied, cap_per_motor_n) is the raised limit.
  NAVRL_VEH_RAND_LIMIT_MODE   optional, "lower" or "raise" (CAM-K2 v2 review r1 m-1: the raise is opt-in). Unset or
                              "lower" keeps the committed v1 behaviour: a TW_NOMINAL above the robot's own T/W
                              refuses at init. Only "raise" lets a TW_NOMINAL above it raise the limit, and "raise"
                              with a TW_NOMINAL below it refuses. The value is part of the settings (and the receipt).
  NAVRL_VEH_RAND_TW_SCALE     s_tw in [0, 0.5]: per-env delivered-thrust factor 1 + s_tw (2u - 1)
  NAVRL_VEH_RAND_MOTOR_IMBAL  s_m in [0, 0.2]: per-motor factor 1 + s_m (2v - 1) on top (motor-to-motor spread)
  NAVRL_VEH_RAND_TAU_MIN/_MAX motor time constant [s] in [0.005, 0.2], MIN <= MAX: one value per env, drawn uniformly,
                              written to both the increasing and decreasing constants of all motors.

Delivered thrust of motor i in env e: MotorModel(min(ref, limit_e)), clamped by the motor model at its (raised)
max_thrust, * (1 + s_tw(2u_e - 1)) * (1 + s_m(2v_ei - 1)); the
yaw torque cq * T follows it. Why the capped navrl_quad is equivalent to the 0.50 kg "Option A" placeholder, under the
conditions listed in the CAM-K2 preregistration (x2 Lee attitude gains of a uniform-density x2 body, zero custom drag,
no disturbance, no IMU, a kinematic target, mass-free PhysX damping, terminal contacts; the absolute 0.05 N contact
threshold is not scaled): Option A is navrl_quad's
geometry with every mass and inertia element x2, the Lee attitude gains x2 and max thrust 2.7590625 N (T/W 2.25).
The velocity loop's forces are proportional to mass and the attitude torques to the gains, so every commanded force
and torque of Option A is exactly 2x navrl_quad's at the same state, the allocation (arm 0.13 m) is the same, and
the motor model is linear in thrust scale (its k cancels). Option A's delivered forces are therefore 2x the capped
navrl_quad's, with 2x the mass and inertia: identical accelerations (gravity and the PhysX damping are mass-free,
INFERRED for the damping; contacts are terminal). tests/test_realveh.py checks this numerically on the CPU motor and
allocation path. The factors then scale delivered thrust around that nominal (mass / motor-strength uncertainty),
while the controller keeps the nominal mass, as on the real vehicle.

install() wraps, at run time and without editing the files (they are F's dynamics):
  * BaseMultirotor.init_tensors: after the original, compute the limit, draw factors and tau for every env (the
    construction-time motor state is rescaled from [min_thrust, max_thrust] to [min_thrust, limit]) and, in "raise",
    replace the motor model's run-time max_thrust tensor by the limit;
  * BaseMultirotor.reset_idx: after the original (whose global-RNG draws are unchanged in number), redraw factors and
    tau for exactly `env_ids`; in "lower", rescale the motor state the original drew in [min_thrust, max_thrust] to
    [min_thrust, limit] (as the capped vehicle's own reset would draw it); in "raise" the original already draws in
    [min_thrust, limit], because its max_thrust is the limit;
  * MotorModel.update_motor_thrusts: return orig(min(ref, limit)) * factor, out of place.
CAM-K2 v2 ("raise", TW_NOMINAL 4.0 on navrl_quad; review r1 m-2): the limit is 4.0 x 0.25 x 9.81 / 4 = 2.4525 N per
motor, so the thrust-to-weight (maximum thrust acceleration 4.0 g, hover fraction 0.25) is that of a 1.1 kg vehicle at
T/W 4.0; the rotational dynamics and the drag are navrl_quad's, not the real 5-inch frame's (CAM-K2 v2
preregistration). In raise mode the update wrapper also counts, on the device and without a per-step sync, the motor
commands (the reference passed to update_motor_thrusts, before any clamp) above the robot's own max_thrust and at the
raised limit, their running maximum and the number of command elements (review r1 M-1); raise_counters() reads them,
with a live read-back of the motor model's max_thrust tensor, only at the log points.
With TW_NOMINAL equal to the robot's own T/W, 8 / (m |g|) from the robot's tensors (navrl_quad on the CPU stand-in,
m = 0.25 kg and the float32 gravity 9.81: 3.261977434, printed 3.2619774; "3.2620" is 6.9e-6 away and does not
select "none"), within relative 1e-6, s_tw = s_m = 0 and TAU_MIN = TAU_MAX = the config's tau, the run is bitwise
identical to OFF (no cap, factor 1.0 exactly, tau rewritten with the same float32 value). A thrust update before
init_tensors refuses, and so does a second initialize() on the same motor model (review r1 L-3).

Telemetry: one JSON object per line, prefix LOG_PREFIX, written to the stream that was sys.stderr when install() ran
(review r2 N-1). rl_games' quiet startup (quiet_rl_io.devnull_stdio) swaps sys.stderr for /dev/null while it builds
the env, which is exactly when init_tensors runs; the stream bound at install, outside that context, still reaches
the log, as the task's logging handlers do. Lines: "install" with the settings; "init" with the cap, mass, gravity,
own T/W and the first draw's factor / tau range; "reset" for the first RESET_LOG_FIRST redraw calls and every
RESET_LOG_EVERY-th, with the call count, the envs redrawn in that call and in total, the factor / tau range of that
call and of the run so far, and the robot's motor-update call count; "update" at the first motor update and every
UPDATE_LOG_EVERY-th, with that count (review r2 N-7). tools/camk2_train.py requires these lines after a run. They cost
nothing on the GPU: the draws are made on the CPU and the counter is a Python int. The "init" line and the record
carry a live read-back of max_thrust in every mode. In raise mode only, the "reset" and "update" lines also carry
raise_counters(): three small reductions per motor update stay on the device and are synchronised only at those log
points (tools/camk2v2_train.py requires them).
"""
import json
import os
import sys

# torch is imported inside the functions: tools/realveh_entry.py validates the settings before Isaac Gym is imported,
# and Isaac Gym must be imported before torch.

ENV = "NAVRL_VEH_RAND"
MAX_TW_NOMINAL = 6.0                       # CAM-K2 v2 (2026-11-08): the raise is bounded here
LIMITS = {"NAVRL_VEH_RAND_TW_NOMINAL": (1.0, MAX_TW_NOMINAL), "NAVRL_VEH_RAND_TW_SCALE": (0.0, 0.5),
          "NAVRL_VEH_RAND_MOTOR_IMBAL": (0.0, 0.2), "NAVRL_VEH_RAND_TAU_MIN": (0.005, 0.2),
          "NAVRL_VEH_RAND_TAU_MAX": (0.005, 0.2)}
REQUIRED = ("NAVRL_VEH_RAND_SEED",) + tuple(LIMITS)
OWN_TW_RTOL = 1e-6
LIMIT_RTOL = 1e-6                          # a command "at the limit" is >= limit x (1 - LIMIT_RTOL)
LIMIT_MODE_KEY = "NAVRL_VEH_RAND_LIMIT_MODE"
LIMIT_MODES = ("lower", "raise")
LOG_PREFIX = "[veh-rand] "
RESET_LOG_FIRST = 3
RESET_LOG_EVERY = 1000
UPDATE_LOG_EVERY = 100000
_INSTALLED = {}
_STREAM = {"stream": None}                 # bound at install(); see the module docstring


def log(event, **fields):
    stream = _STREAM["stream"] if _STREAM["stream"] is not None else sys.stderr
    stream.write(LOG_PREFIX + json.dumps(dict(event=event, **fields), sort_keys=True) + "\n")
    stream.flush()


def settings_from_env(environ=None):
    """Validated settings; refuses (ValueError) unless NAVRL_VEH_RAND=1 and every value is present and in bounds."""
    env = os.environ if environ is None else environ
    if env.get(ENV, "").strip() != "1":
        raise ValueError("%s must be 1 to use navrl_vehicle_rand, got %r" % (ENV, env.get(ENV)))
    missing = [k for k in REQUIRED if not env.get(k, "").strip()]
    if missing:
        raise ValueError("NAVRL_VEH_RAND=1 needs %s" % ", ".join(missing))
    out = {}
    for k, (lo, hi) in LIMITS.items():
        try:
            v = float(env[k])
        except ValueError:
            raise ValueError("%s is not a number: %r" % (k, env[k])) from None
        if not lo <= v <= hi:          # also refuses nan
            raise ValueError("%s=%r outside [%g, %g]" % (k, v, lo, hi))
        out[k] = v
    if out["NAVRL_VEH_RAND_TAU_MIN"] > out["NAVRL_VEH_RAND_TAU_MAX"]:
        raise ValueError("NAVRL_VEH_RAND_TAU_MIN > NAVRL_VEH_RAND_TAU_MAX")
    try:
        seed = int(env["NAVRL_VEH_RAND_SEED"])
    except ValueError:
        raise ValueError("NAVRL_VEH_RAND_SEED is not an integer: %r" % env["NAVRL_VEH_RAND_SEED"]) from None
    if seed < 0:
        raise ValueError("NAVRL_VEH_RAND_SEED must be >= 0")
    out["NAVRL_VEH_RAND_SEED"] = seed
    mode = env.get(LIMIT_MODE_KEY, "").strip()
    if mode:                                   # optional; absent keeps the committed v1 settings unchanged
        if mode not in LIMIT_MODES:
            raise ValueError("%s must be one of %s, got %r" % (LIMIT_MODE_KEY, ", ".join(LIMIT_MODES), mode))
        out[LIMIT_MODE_KEY] = mode
    return out


def worst_case_thrust_to_weight(s):
    """TW_NOMINAL x (1 - TW_SCALE) x (1 - MOTOR_IMBAL): the weakest env with all four motors at their weakest."""
    return (float(s["NAVRL_VEH_RAND_TW_NOMINAL"]) * (1.0 - float(s["NAVRL_VEH_RAND_TW_SCALE"]))
            * (1.0 - float(s["NAVRL_VEH_RAND_MOTOR_IMBAL"])))


def initialize(robot, s, record_path=None):
    """Compute the per-motor cap from the robot's controller mass and gravity, check it against the robot's own T/W,
    draw factors and tau for every env and (optionally) record the computation to `record_path` (mode "x")."""
    import torch

    mm = robot.control_allocator.motor_model
    if getattr(mm, "_veh_rand_ready", False):  # review r1 L-3: after a raise, max_thrust is no longer the robot's own
        raise RuntimeError("vehicle randomization: initialize called twice on one motor model")
    mass = robot.controller.mass.reshape(-1).to(torch.float64)
    g = robot.controller.gravity.reshape(int(robot.num_envs), -1).to(torch.float64).norm(dim=1)
    if mass.numel() != int(robot.num_envs) or not bool(torch.isfinite(mass).all()) or not bool((mass > 0).all()):
        raise RuntimeError("vehicle randomization: controller mass is not a positive per-env value: %s" % mass)
    n = int(mm.num_motors_per_robot)
    max_total = mm.max_thrust.to(torch.float64).sum(dim=1)
    own_tw = max_total / (mass * g)
    tw = float(s["NAVRL_VEH_RAND_TW_NOMINAL"])
    if not 1.0 <= tw <= MAX_TW_NOMINAL:
        raise RuntimeError("NAVRL_VEH_RAND_TW_NOMINAL=%g outside [1, %g]" % (tw, MAX_TW_NOMINAL))
    requested = s.get(LIMIT_MODE_KEY)          # None: as committed for v1 (lower or none; above refuses)
    if bool((abs(tw - own_tw) <= own_tw * OWN_TW_RTOL).all()):
        mode = "none"
    elif bool((tw < own_tw).all()):
        if requested == "raise":
            raise RuntimeError("NAVRL_VEH_RAND_LIMIT_MODE=raise, but NAVRL_VEH_RAND_TW_NOMINAL=%g is below the robot's "
                               "own thrust-to-weight %s" % (tw, own_tw.tolist()[:4]))
        mode = "lower"
    elif bool((tw > own_tw).all()):
        if requested != "raise":
            raise RuntimeError("NAVRL_VEH_RAND_TW_NOMINAL=%g exceeds the robot's own thrust-to-weight %s (a raise needs "
                               "NAVRL_VEH_RAND_LIMIT_MODE=raise)" % (tw, own_tw.tolist()[:4]))
        mode = "raise"
    else:
        raise RuntimeError("NAVRL_VEH_RAND_TW_NOMINAL=%g is neither above nor below every env's own T/W %s"
                           % (tw, own_tw.tolist()[:4]))
    applied = mode != "none"
    cap64 = tw * mass * g / n
    own_max = [float(mm.max_thrust.min()), float(mm.max_thrust.max())]
    if applied:
        mm._veh_rand_cap = cap64.to(dtype=mm.max_thrust.dtype, device=mm.max_thrust.device).view(-1, 1).expand(-1, n)
    else:
        mm._veh_rand_cap = None
    mm._veh_rand_ready = True
    redraw(robot, torch.arange(int(robot.num_envs)), s)   # rescales the construction-time state to [min, limit]
    if mode == "raise":                        # CAM-K2 v2: the motor model's own clamp is raised to the limit
        mm._veh_rand_own_max_thrust = mm.max_thrust
        mm.max_thrust = mm._veh_rand_cap.clone()
        mm._veh_rand_limit_threshold = mm._veh_rand_cap * (1.0 - LIMIT_RTOL)
        mm._veh_rand_limit_raised = True
    live_max = [float(mm.max_thrust.min()), float(mm.max_thrust.max())]   # read back from the live tensor (M-1)
    record = {"tw_nominal": tw, "cap_applied": applied, "mode": mode, "num_motors": n,
              "cap_per_motor_n": [float(cap64.min()), float(cap64.max())],
              "mass_kg": [float(mass.min()), float(mass.max())], "gravity_mps2": [float(g.min()), float(g.max())],
              "own_thrust_to_weight": [float(own_tw.min()), float(own_tw.max())],
              "max_thrust_per_motor_n": own_max,
              "thrust_limit_per_motor_n": [float(cap64.min()), float(cap64.max())] if applied else own_max,
              "live_max_thrust_per_motor_n": live_max,
              "worst_case_thrust_to_weight": worst_case_thrust_to_weight(s), "settings": dict(s)}
    robot._veh_rand_record = record
    stats = getattr(robot, "_veh_rand_stats", {})
    log("init", num_envs=int(robot.num_envs), cap_applied=applied, mode=mode, cap_per_motor_n=record["cap_per_motor_n"],
        mass_kg=record["mass_kg"], gravity_mps2=record["gravity_mps2"],
        own_thrust_to_weight=record["own_thrust_to_weight"], tw_nominal=tw, live_max_thrust_n=live_max,
        factor_min=stats.get("factor_min"), factor_max=stats.get("factor_max"),
        tau_min=stats.get("tau_min"), tau_max=stats.get("tau_max"))
    robot._veh_rand_stats = {}                 # the reset counters start after the initial draw
    if record_path:
        with open(record_path, "x") as fh:
            json.dump(record, fh, indent=1, sort_keys=True)
            fh.write("\n")
    return record


def _factor(motor_model):
    """Per-env/motor delivered-thrust factor of one MotorModel (1.0 before the first draw)."""
    import torch

    factor = getattr(motor_model, "_veh_rand_factor", None)
    if factor is None:
        factor = torch.ones((motor_model.num_envs, motor_model.num_motors_per_robot), dtype=torch.float32,
                            device=motor_model.current_motor_thrust.device)
        motor_model._veh_rand_factor = factor
    return factor


def redraw(robot, env_ids, s):
    """Draw new factors and tau for `env_ids` of `robot` (a BaseMultirotor) from the module's own generator and, in
    "lower" (and once at init before a raise), rescale those envs' motor state from [min, max_thrust] to [min, cap].
    After a raise the original reset already draws in [min, limit], so no rescale."""
    import torch

    mm = robot.control_allocator.motor_model
    if not getattr(mm, "_veh_rand_ready", False):
        raise RuntimeError("vehicle randomization: redraw before initialize (init_tensors)")
    gen = getattr(robot, "_veh_rand_generator", None)
    if gen is None:
        gen = torch.Generator(device="cpu")
        gen.manual_seed(int(s["NAVRL_VEH_RAND_SEED"]))
        robot._veh_rand_generator = gen
    ids = torch.as_tensor(env_ids, dtype=torch.long).reshape(-1).cpu()
    n, m = int(ids.numel()), int(mm.num_motors_per_robot)
    if n == 0:
        return
    u_tw = torch.rand((n, 1), generator=gen, dtype=torch.float32)
    u_m = torch.rand((n, m), generator=gen, dtype=torch.float32)
    u_tau = torch.rand((n, 1), generator=gen, dtype=torch.float32)
    s_tw, s_m = float(s["NAVRL_VEH_RAND_TW_SCALE"]), float(s["NAVRL_VEH_RAND_MOTOR_IMBAL"])
    lo, hi = float(s["NAVRL_VEH_RAND_TAU_MIN"]), float(s["NAVRL_VEH_RAND_TAU_MAX"])
    factor = (1.0 + s_tw * (2.0 * u_tw - 1.0)) * (1.0 + s_m * (2.0 * u_m - 1.0))
    tau = (torch.full((n, 1), lo, dtype=torch.float32) + (hi - lo) * u_tau).expand(n, m)
    _account(robot, n, factor, tau)
    device = mm.current_motor_thrust.device
    dev_ids = ids.to(device)
    _factor(mm)[dev_ids] = factor.to(device)
    mm.motor_time_constants_increasing[dev_ids] = tau.to(device=device, dtype=mm.motor_time_constants_increasing.dtype)
    mm.motor_time_constants_decreasing[dev_ids] = tau.to(device=device, dtype=mm.motor_time_constants_decreasing.dtype)
    cap = getattr(mm, "_veh_rand_cap", None)
    if cap is not None and not getattr(mm, "_veh_rand_limit_raised", False):
        # lower, and the raise's initial draw: the original drew in [min, own max_thrust]; after a raise the original
        # draws in [min, limit] itself.
        lo_t, hi_t = mm.min_thrust[dev_ids], mm.max_thrust[dev_ids]
        state = mm.current_motor_thrust[dev_ids]
        mm.current_motor_thrust[dev_ids] = lo_t + (state - lo_t) * (cap[dev_ids] - lo_t) / (hi_t - lo_t)


def _account(robot, n, factor, tau):
    """Running telemetry of the draws (CPU tensors, so no device synchronisation); logs per the module docstring."""
    st = getattr(robot, "_veh_rand_stats", None)
    if st is None:
        st = robot._veh_rand_stats = {}
    f_lo, f_hi, t_lo, t_hi = float(factor.min()), float(factor.max()), float(tau.min()), float(tau.max())
    st["calls"] = st.get("calls", 0) + 1
    st["envs_total"] = st.get("envs_total", 0) + n
    st["factor_min"] = min(st.get("factor_min", f_lo), f_lo)
    st["factor_max"] = max(st.get("factor_max", f_hi), f_hi)
    st["tau_min"] = min(st.get("tau_min", t_lo), t_lo)
    st["tau_max"] = max(st.get("tau_max", t_hi), t_hi)
    if getattr(robot, "_veh_rand_record", None) is None:
        return                                 # the initial draw inside initialize(); reported by its "init" line
    if st["calls"] <= RESET_LOG_FIRST or st["calls"] % RESET_LOG_EVERY == 0:
        log("reset", calls=st["calls"], envs=n, envs_total=st["envs_total"], factor_min_call=f_lo,
            factor_max_call=f_hi, factor_min=st["factor_min"], factor_max=st["factor_max"], tau_min=st["tau_min"],
            tau_max=st["tau_max"],
            motor_updates=int(getattr(robot.control_allocator.motor_model, "_veh_rand_updates", 0)),
            **(raise_counters(robot.control_allocator.motor_model) or {}))


def install(s, multirotor_cls=None, motor_model_cls=None, record_path=None):
    """Wrap `multirotor_cls.init_tensors` / `.reset_idx` and `motor_model_cls.update_motor_thrusts` (production:
    BaseMultirotor and MotorModel; import isaacgym first). Refuses a second install."""
    import functools

    if multirotor_cls is None:
        from aerial_gym.robots.base_multirotor import BaseMultirotor as multirotor_cls
    if motor_model_cls is None:
        from aerial_gym.control.motor_model import MotorModel as motor_model_cls
    for cls in (multirotor_cls, motor_model_cls):
        if getattr(cls, "_veh_rand_settings", None) is not None:
            raise RuntimeError("vehicle randomization already installed on %r" % cls)
    s = dict(s)
    _STREAM["stream"] = sys.stderr             # bound now, outside rl_games' quiet construction (review r2 N-1)
    orig_init, orig_reset = multirotor_cls.init_tensors, multirotor_cls.reset_idx
    orig_update = motor_model_cls.update_motor_thrusts

    @functools.wraps(orig_init)
    def init_tensors(self, *args, **kwargs):
        out = orig_init(self, *args, **kwargs)
        initialize(self, s, record_path)
        return out

    @functools.wraps(orig_reset)
    def reset_idx(self, env_ids):
        out = orig_reset(self, env_ids)
        if len(env_ids) > 0:
            redraw(self, env_ids, s)
        return out

    @functools.wraps(orig_update)
    def update_motor_thrusts(self, ref_thrust):
        if not getattr(self, "_veh_rand_ready", False):
            raise RuntimeError("vehicle randomization: motor update before initialize (init_tensors)")
        if getattr(self, "_veh_rand_limit_raised", False):
            _count_commands(self, ref_thrust)  # review r1 M-1: on the device, before any clamp
        cap = self._veh_rand_cap
        if cap is not None:
            ref_thrust = torch_minimum(ref_thrust, cap)
        out = orig_update(self, ref_thrust) * _factor(self)
        self._veh_rand_updates = getattr(self, "_veh_rand_updates", 0) + 1
        if self._veh_rand_updates == 1 or self._veh_rand_updates % UPDATE_LOG_EVERY == 0:
            log("update", motor_updates=self._veh_rand_updates, cap_applied=cap is not None,
                **(raise_counters(self) or {}))
        return out

    multirotor_cls.init_tensors = init_tensors
    multirotor_cls.reset_idx = reset_idx
    motor_model_cls.update_motor_thrusts = update_motor_thrusts
    multirotor_cls._veh_rand_settings = s
    motor_model_cls._veh_rand_settings = s
    _INSTALLED["classes"] = (multirotor_cls, motor_model_cls, orig_init, orig_reset, orig_update)
    log("install", settings=s)
    lo = (1.0 - s["NAVRL_VEH_RAND_TW_SCALE"]) * (1.0 - s["NAVRL_VEH_RAND_MOTOR_IMBAL"])
    hi = (1.0 + s["NAVRL_VEH_RAND_TW_SCALE"]) * (1.0 + s["NAVRL_VEH_RAND_MOTOR_IMBAL"])
    return {"settings": s, "multirotor": "%s.%s" % (multirotor_cls.__module__, multirotor_cls.__qualname__),
            "motor_model": "%s.%s" % (motor_model_cls.__module__, motor_model_cls.__qualname__),
            "factor_bounds": [lo, hi],
            "thrust_to_weight_range": [lo * s["NAVRL_VEH_RAND_TW_NOMINAL"], hi * s["NAVRL_VEH_RAND_TW_NOMINAL"]],
            "record_file": record_path}


def _count_commands(mm, ref):
    """Raise mode: accumulate on ref's device (no .item(), so no per-step synchronisation) the number of command
    elements, those above the robot's own max_thrust, those at the raised limit and the running maximum command."""
    import torch

    c = getattr(mm, "_veh_rand_counters", None)
    if c is None:
        c = mm._veh_rand_counters = {
            "elements": 0, "above_own": torch.zeros((), dtype=torch.long, device=ref.device),
            "at_limit": torch.zeros((), dtype=torch.long, device=ref.device),
            "max_ref": torch.full((), float("-inf"), dtype=ref.dtype, device=ref.device)}
    c["elements"] += int(ref.numel())
    c["above_own"] += (ref > mm._veh_rand_own_max_thrust).sum()
    c["at_limit"] += (ref >= mm._veh_rand_limit_threshold).sum()
    c["max_ref"] = torch.maximum(c["max_ref"], ref.detach().max())


def raise_counters(mm):
    """None unless the limit was raised. Otherwise the command counters (cmd_elements, cmd_above_own_max, cmd_at_limit,
    cmd_max_n; zeros and None before the first update) and live_max_thrust_n, [min, max] read back from the motor
    model's live max_thrust tensor. Synchronises the device; called only at the log points."""
    if not getattr(mm, "_veh_rand_limit_raised", False):
        return None
    live = [float(mm.max_thrust.min()), float(mm.max_thrust.max())]
    c = getattr(mm, "_veh_rand_counters", None)
    if c is None:
        return {"cmd_elements": 0, "cmd_above_own_max": 0, "cmd_at_limit": 0, "cmd_max_n": None,
                "live_max_thrust_n": live}
    return {"cmd_elements": int(c["elements"]), "cmd_above_own_max": int(c["above_own"].item()),
            "cmd_at_limit": int(c["at_limit"].item()), "cmd_max_n": float(c["max_ref"].item()),
            "live_max_thrust_n": live}


def torch_minimum(a, b):
    import torch

    return torch.minimum(a, b)
