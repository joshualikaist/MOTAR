#!/usr/bin/env bash
# Check P_ACC 2.1 (interpolated rise match) with the current yaw-rate gains and with faster ones.
cd /home/fair/motar_tmp/sitl_tune_20261105 || exit 2
PYX=/home/fair/miniconda3/envs/px4/bin/python
BASE='"MC_ROLLRATE_P": 0.013, "MC_ROLLRATE_I": 0.017, "MC_ROLLRATE_D": 0.00026, "MC_PITCHRATE_P": 0.013, "MC_PITCHRATE_I": 0.017, "MC_PITCHRATE_D": 0.00026, "MPC_XY_VEL_P_ACC": 2.0, "MPC_XY_VEL_I_ACC": 0.1, "MPC_XY_VEL_D_ACC": 0.0, "THR_MDL_FAC": 1.0'
for v in "yawP0.2:, \"MC_YAWRATE_P\": 0.2, \"MC_YAWRATE_I\": 0.1"; do
  name=${v%%:*}; extra=${v#*:}
  echo "tune2 P_ACC=2.0 $name window start $(date -Is)" >> /home/fair/motar_tmp/sitl_g8_20261105/cpu_windows.log
  ( ulimit -v 16000000; SITL_MODEL=navrl_quad SITL_WORLD=motar_dbg_iters100 SITL_PARAMS="{$BASE$extra}" CUDA_VISIBLE_DEVICES="" nice -n 10 taskset -c 6,7,12,13 timeout -k 20 600 $PYX -B s12_offboard_mavlink_w.py tune_pacc_2.0_$name.json > tune_pacc_2.0_$name.out 2>&1 )
  echo "tune2 P_ACC=2.0 $name rc=$? window end $(date -Is)" >> /home/fair/motar_tmp/sitl_g8_20261105/cpu_windows.log
  sleep 5
done
