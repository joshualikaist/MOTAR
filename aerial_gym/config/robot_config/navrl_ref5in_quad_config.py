from aerial_gym.config.robot_config.navrl_quad_config import NavRLQuadCfg


class NavRLRef5inQuadCfg(NavRLQuadCfg):
    """Internally consistent 5-inch *simulation reference candidate* for NavRL.

    The legacy body mixes a 0.28 m collision proxy, motors at +/-0.13 m and the stock 0.25 kg
    Aerial Gym dynamics.  This opt-in candidate instead uses a 220 mm motor diagonal, 1.20 kg
    nominal all-up mass, an estimated central-box-plus-motors inertia and 9.60 N/motor so nominal
    thrust-to-weight stays 3.262.  Its upright XY collision proxy remains 0.28 m; the calculated
    5-inch prop-tip AABB is 0.2826 m, so the proxy is 0.9% smaller, not an exact CAD envelope.

    This is not yet a flight-proven or CAD-validated aircraft.  The 1.20 kg budget is a modelling
    allowance, the inertia is analytic rather than measured, and carrier/cooling/power/cabling,
    centre of gravity, prop clearance, LiDAR occlusion, endurance and thermal limits remain open.
    The 0.12 m collision height is also a conservative payload-stack proxy; when the vehicle tilts
    it changes the horizontal contact envelope relative to the legacy 0.08 m box.  Therefore the
    learning smoke measures viability of the whole candidate, not a dynamics-only causal effect.

    Nominal T/W, tilt and motor lag are matched, but that does not make trajectories bit-identical:
    mass/inertia, yaw authority, contact envelope and disturbance response differ.  In particular,
    the algebraic constant-thrust roll acceleration is about 0.586x legacy.  A policy trained here
    forms a new lineage and must never be numerically spliced into legacy checkpoint curves.

    See `docs/reference_platform_proposal_2026-08.md` for the evidence levels and required hardware
    gates, and `tests/test_navrl_ref5in_platform.py` for simulator-internal consistency checks.
    """

    class robot_asset(NavRLQuadCfg.robot_asset):
        file = "quad_navrl_ref5in.urdf"

    class control_allocator_config(NavRLQuadCfg.control_allocator_config):
        # 5-inch frame: motors at (+/-0.0777817, +/-0.0777817) m, i.e. a 0.110 m motor radius and a
        # 220 mm motor-to-motor diagonal. MUST equal the joint origins in quad_navrl_ref5in.urdf --
        # the allocation matrix and the URDF are two independent statements of the same geometry and
        # nothing at runtime checks that they agree (test_navrl_ref5in_platform.py does).
        allocation_matrix = [
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0, 1.0],
            [-0.0777817, -0.0777817, 0.0777817, 0.0777817],
            [-0.0777817, 0.0777817, 0.0777817, -0.0777817],
            [-0.01, 0.01, -0.01, 0.01],
        ]

        class motor_model_config(NavRLQuadCfg.control_allocator_config.motor_model_config):
            # T/W 3.2617 preserved: 3.2617 * 1.20 kg * 9.81 / 4 = 9.599 N per motor.
            # Sanity: a 2207-class motor on 6S with a 5x4.3x3 prop makes 1.6-1.8 kgf static, so
            # 9.6 N = 0.98 kgf is a ~55% duty ceiling -- comfortably inside a real motor's envelope
            # rather than an aspirational number.
            max_thrust = 9.6

            # thrust = k * rps^2 with use_rps=True, so k must be rescaled with max_thrust or the
            # implied rotor speed becomes physically impossible. Anchored at 467 rps (28,020 RPM),
            # which a 1750 kV motor on 6S reaches at ~70% of no-load: k = 9.6 / 467^2 = 4.401e-5.
            # In this simulator's unconstrained use_rps path k algebraically cancels from the
            # normalized thrust transient; varying it is NOT motor-strength randomization.  Keep a
            # single coordinate calibration instead of pretending the spread adds domain randomization.
            motor_thrust_constant_min = 4.401e-05
            motor_thrust_constant_max = 4.401e-05

            # Unchanged at the stock 0.04 s. Rotor spin-up lag is a property of the rotor, not the
            # airframe, and 20-45 ms is the measured band for 5-inch props; adding payload mass does
            # not slow it. (A 10-inch platform would need ~0.08 s -- one more reason to stay at 5".)
            #   motor_time_constant_{increasing,decreasing}_{min,max} = 0.04

            # thrust_to_torque_ratio 0.01 m is inherited: kQ/kT for a 5-inch prop is 0.008-0.016 m.
