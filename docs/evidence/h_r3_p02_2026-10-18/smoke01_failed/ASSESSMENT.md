# P02 SMOKE01 — FAIL, main not launched

Source: `125690c3c02c619575ec2ed868d2a126734811fc`. Eight processes were planned;
normal B0 and forced-reset B0 completed, then the fail-closed runner stopped.
No B1–B5 process or two-seed pilot launched. No effects are estimated.

The forced-reset diagnostic resets env15 at step40. First unaffected state and raw-sensor
SHA mismatch: step99, env8; first action mismatch: step100. Normal B0 resets env15 at99;
forced B0 does not. Both reset env0/env8 at98. RNG probes and target positions of unaffected
envs are unchanged. This localizes a non-RNG side effect consistent with the full PhysX
actor-root setter used by every reset; the proposed indexed-write repair still needs proof.
Production and P01 files remain unchanged. SMOKE01 is preserved and will not be overwritten.
A fresh mechanics run is required after a committed runner repair, with unchanged equality,
safety and sample gates. H-R3 remains PROVISIONAL — NOT CONFIRMED.

Raw path: `../aerial_gym_simulator/results/h_r3_p02_v1/HR3-P02-SMOKE01`.
The adjacent manifest and report are compact copies; raw payload stays outside Git.
No external backup is established here. Historical raw and checkpoint remain preserved.
