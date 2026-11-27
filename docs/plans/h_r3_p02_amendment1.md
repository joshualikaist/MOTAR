# P02 amendment1 — selected-row reset cache isolation, before GPU measurement

Written after independent code audit and before any P02 GPU execution. Gates, seeds, metrics,
thresholds, arm definitions and sample counts are unchanged.

Original robot/task reset methods call robot.update_states(), which refreshes derived kinematic
buffers for **all** environments. Between normal physics ticks those buffers can still reflect
the preceding controller frame. Thus an unrelated env reset can refresh another env's orientation/
velocity even if random draws are isolated. The P02-only reset wrapper therefore preserves and
restores unaffected rows of exactly robot_euler_angles, robot_vehicle_orientation,
robot_vehicle_linvel, robot_body_linvel and robot_body_angvel around each original reset.

This changes no production source, physical state/dynamics/controller/sensor correction lineage or
P01 record. It removes an evaluation batching side effect alongside RNG coupling. Fixed-N scope
and actual forced-other-reset state/raw-sensor equality gates remain mandatory. If additional
physics coupling remains, the gate fails and P02 does not run; no tolerance is relaxed.
