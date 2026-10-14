"""Simulator-independent evaluation core for the KRoC 2027 study (Experiment A and conditional B).

Nothing in this package imports ``aerial_gym`` or Isaac Gym, so CPU tests, offline audits and the
evaluation CLI load it directly. The Aerial Gym side reaches it only through
``aerial_gym/task/navrl_task/kroc_eval_adapter.py``, which is off by default.

Modules
-------
contracts      time/frame/unit contract and the target-motion gates G1 (generator state) and G2
               (written-back float32 positions)
geometry       line of sight, camera frustum and swept proxy clearance against bar footprints
initial_view   the Experiment A initial condition: fixed range, clear line of sight, yaw 0 or 180 deg
events         acquisition chronology (measurement, track, loss, reacquisition, censoring)
scene_record   episode keys and hashes of the scene and initial state (recording only, no replay)
protocol       cells, quota, seed-level statistics, decision labels, the B arm rule and gate
"""

PACKAGE_CONTRACT = "KROC2027_EVAL_CORE_V1"
