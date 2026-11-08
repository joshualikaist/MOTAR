from aerial_gym.config.robot_config.navrl_quad_config import NavRLQuadCfg

# DOCUMENTATION OF "OPTION A" ONLY (2026-11-08; survey ~/motar_tmp/realveh_design_20261108/SURVEY.md section 1,
# IMPLEMENTATION.md part 3). No robot is registered with this config and no launcher or entry uses it: CAM-K2 keeps
# navrl_quad's identity and models this vehicle at run time with the thrust cap of navrl_vehicle_rand.py
# (NAVRL_VEH_RAND_TW_NOMINAL=2.25), which tests/test_realveh.py shows equivalent to this file and its URDF.
# Every number below is a modelling choice, not a measurement.
NOMINAL_MASS_KG = 0.50
THRUST_TO_WEIGHT = 2.25
GRAVITY_MPS2 = 9.81
NUM_MOTORS = 4
MOTOR_ARM_M = 0.13
# Per-motor maximum thrust that gives THRUST_TO_WEIGHT at NOMINAL_MASS_KG: 2.25 * 0.50 * 9.81 / 4 = 2.7590625 N.
MAX_THRUST_PER_MOTOR_N = THRUST_TO_WEIGHT * NOMINAL_MASS_KG * GRAVITY_MPS2 / NUM_MOTORS
assert abs(MAX_THRUST_PER_MOTOR_N - 2.7590625) < 1e-12, MAX_THRUST_PER_MOTOR_N
assert abs(NUM_MOTORS * MAX_THRUST_PER_MOTOR_N / (NOMINAL_MASS_KG * GRAVITY_MPS2) - THRUST_TO_WEIGHT) < 1e-12
# Assembled inertia relative to navrl_quad (quad_navrl_collide.urdf), per body axis (x, y, z). Option A doubles every
# link mass and inertia element, so the ratio is exactly 2 on every axis (tests/test_realveh.py). Flying this body
# would need the Lee attitude gains x2 as well; with them, every force and torque is 2x navrl_quad's at 2x the mass
# and inertia, which is why the capped navrl_quad is the same vehicle.
INERTIA_RATIO_VS_NAVRL_QUAD = (2.0, 2.0, 2.0)


class NavRLRv05QuadCfg(NavRLQuadCfg):
    """0.50 kg "Option A" vehicle of the real-vehicle study: DOCUMENTATION AND TEST ORACLE ONLY, not registered.

    PLACEHOLDER GEOMETRY until the user gives the real frame, arm length and thrust table. This is the survey's
    "Option A": the geometry of navrl_quad (motors at +/-0.13 m, 0.28 x 0.28 x 0.08 m collision box) with every link
    mass and every inertia element doubled (quad_navrl_rv05.urdf), so the total mass is 0.50 kg and the assembled
    inertia is 2x navrl_quad's: Ixx = Iyy = 1.69e-3, Izz = 3.38e-3 kg m^2. The real vehicle's arm length is unknown,
    so its inertia (and therefore the attitude-gain ratio) is unknown too.

    Actuators (all modelling choices, none measured):
      * max thrust 2.7590625 N per motor = thrust-to-weight 2.25 at 0.50 kg (navrl_quad: 2 N, T/W 3.26). At T/W 2.25 a
        45 deg tilt already takes 1.41 mg, leaving 0.84 mg for attitude torque, and the per-motor clamp has no
        thrust/attitude priority (survey section 1, risks).
      * motor time constant 0.04 s up and down (inherited, restated); thrust-to-torque ratio 0.01 m (inherited,
        restated). Small motors are often 0.02-0.06 s and 0.01-0.016 m (INFERRED, survey).
      * one thrust constant: in the use_rps path k cancels from the normalized thrust transient
        (control/motor_model.py), so a k spread is not motor-strength randomization; see navrl_ref5in_quad_config.py.
        Delivered-thrust randomization is the separate opt-in NAVRL_VEH_RAND (navrl_vehicle_rand.py).
    The allocation matrix arm (0.13 m) MUST equal the URDF motor joint origins: nothing at run time checks it,
    tests/test_realveh.py does. The Lee attitude gains are absolute torques, so this body would need K_R and K_omega
    x INERTIA_RATIO_VS_NAVRL_QUAD to keep both attitude poles. CAM-K2 does not fly this config: it flies navrl_quad
    (robot identity and checkpoint lineage unchanged) with the per-motor cap TW 2.25 x m x g / 4 = 1.3795 N, which
    gives the same accelerations (tests/test_realveh.py, TestOptionAEquivalence).
    """

    class robot_asset(NavRLQuadCfg.robot_asset):
        file = "quad_navrl_rv05.urdf"

    class control_allocator_config(NavRLQuadCfg.control_allocator_config):
        # Motors at (+/-0.13, +/-0.13, 0) m, the joint origins of quad_navrl_rv05.urdf (unchanged from navrl_quad).
        # Row 3 is the roll torque y_i, row 4 the pitch torque -x_i, row 5 the yaw torque -/+ thrust_to_torque_ratio.
        allocation_matrix = [
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0, 1.0],
            [-MOTOR_ARM_M, -MOTOR_ARM_M, MOTOR_ARM_M, MOTOR_ARM_M],
            [-MOTOR_ARM_M, MOTOR_ARM_M, MOTOR_ARM_M, -MOTOR_ARM_M],
            [-0.01, 0.01, -0.01, 0.01],
        ]

        class motor_model_config(NavRLQuadCfg.control_allocator_config.motor_model_config):
            max_thrust = MAX_THRUST_PER_MOTOR_N
            min_thrust = 0
            # A single coordinate calibration (the base config's midpoint); it cancels in the use_rps path.
            motor_thrust_constant_min = 1.376312e-05
            motor_thrust_constant_max = 1.376312e-05
            motor_time_constant_increasing_min = 0.04
            motor_time_constant_increasing_max = 0.04
            motor_time_constant_decreasing_min = 0.04
            motor_time_constant_decreasing_max = 0.04
            thrust_to_torque_ratio = 0.01
