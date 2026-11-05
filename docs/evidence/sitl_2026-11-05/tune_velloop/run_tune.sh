#!/usr/bin/env bash
# PX4 velocity-loop sweep on navrl_quad (iters-100 world, rangefinder height) against G9 attempt 2 env 1
# (Lee rise 0.61 s, onset 0.08 s, overshoot ~1 %). S3b runner gains plus MPC_XY_VEL_P_ACC in {2.5, 1.8, 1.4, 1.1}.
cd /home/fair/motar_tmp/sitl_tune_20261105 || exit 2
PYX=/home/fair/miniconda3/envs/px4/bin/python
for p in 2.5 1.8 1.4 1.1; do
  echo "tune P_ACC=$p window start $(date -Is)" >> /home/fair/motar_tmp/sitl_g8_20261105/cpu_windows.log
  ( ulimit -v 16000000; SITL_MODEL=navrl_quad SITL_WORLD=motar_dbg_iters100 SITL_PARAMS="{\"MC_ROLLRATE_P\": 0.013, \"MC_ROLLRATE_I\": 0.017, \"MC_ROLLRATE_D\": 0.00026, \"MC_PITCHRATE_P\": 0.013, \"MC_PITCHRATE_I\": 0.017, \"MC_PITCHRATE_D\": 0.00026, \"MPC_XY_VEL_P_ACC\": $p, \"MPC_XY_VEL_I_ACC\": 0.1, \"MPC_XY_VEL_D_ACC\": 0.0, \"THR_MDL_FAC\": 1.0}" CUDA_VISIBLE_DEVICES="" nice -n 10 taskset -c 6,7,12,13 timeout -k 20 600 $PYX -B s12_offboard_mavlink_w.py tune_pacc_$p.json > tune_pacc_$p.out 2>&1 )
  echo "tune P_ACC=$p rc=$? window end $(date -Is)" >> /home/fair/motar_tmp/sitl_g8_20261105/cpu_windows.log
  sleep 5
done
