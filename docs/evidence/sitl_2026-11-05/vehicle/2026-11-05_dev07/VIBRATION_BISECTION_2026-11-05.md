# navrl_quad EKF divergence: bisection (2026-11-05 09:24-09:28 KST, windows in ~/motar_tmp/sitl_g8_20261105/cpu_windows.log)

All runs: `dbg_vib_world2.py navrl_quad <world>` (empty world, 10 s on the ground motors off, then arm + 12 s position
hold at 1 m), cores 6,7,12,13 nice 10, CUDA hidden, during the G3 GPU run. RTF over each run 0.999-1.0 (PX4 sim time /
wall). Logs `dbg_vib_navrl_quad_<variant>.log`.

| Variant | VIBRATION z ground / flight | EKF vel test ratio, flight | Reading |
| --- | --- | --- | --- |
| original navrl_quad, empty.world (quick, iters 10) | 0.57 / 0.43 (04:20 session) | 0.83 | defect |
| IMU link 0.002 kg, inertia 1.5e-7; base 0.223 kg | 0.573 / 0.431 | 0.848 | IMU-link hypothesis rejected |
| iris IMU noise params added | 0.574 / 0.424 | 0.872 | IMU noise hypothesis rejected |
| base mass/inertia set to iris (1.5 kg, 0.029/0.055) | 0.055 / 0.058 | 0.04 | the light base is involved |
| original model, world iters 100 | 0.055 / 0.058 | 0.045 | **fixes it** |
| original model, world solver type `world` | 0.055 / 0.058 | 0.049 | also fixes it (mag ratio 0.17-0.22) |

Earlier (04:20): rotor inertia 1e-6/1e-5 and a 1-2 ms physics step did not remove the vibration.
Mechanism (consistent with): ODE quickstep with 10 iterations leaves constraint residuals that a 0.21 kg base with
light attached links (IMU 0.015 kg, rotors 4 x 0.00625 kg) feels as accelerometer noise.
Next: S3b worlds use iters 100; rerun S3b dev with the bridge's mu-scale fix.
