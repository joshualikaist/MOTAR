# S3c v1 — heavier vehicle and one-step command delay through PX4 SITL (preregistration, 2026-11-06)

**State:** preregistered, **NOT_RUN**. Runner `tools/sitl_s3c.py`, tests `tests/test_sitl_s3c_s4.py`. Plan stage S3c of
[`sitl_validation_plan_v1.md`](sitl_validation_plan_v1.md). Amendments go in a new dated file.

## Question

How much does Frozen Policy F's closed loop through PX4 SITL change in two ways?
- With a heavier, lower-thrust vehicle than the one it was trained on: PX4's iris, 1.5 kg and T/W about 1.9,
  against navrl_quad's 0.25 kg and T/W 3.26.
- With every command arriving 100 ms late.

The plan's S3c is **reported, not gated**: "the gap reported; it decides whether GR-1b/GR-2 policies are needed".

## Fixed treatment

Everything is [S3b formal v1](prereg_sitl_s3b_formal_v1_2026-11-06.md)'s, through the same functions of
`tools/sitl_s3b_formal.py`:
- Frozen F, the bridge, MOTAR's own sensors, the riskcap governor, 10 Hz control and MOTAR's termination rules;
- the owned fresh-rootfs session, parameter read-back and the 0.15 s lateness bound;
- the G8 r2 inputs and their pins, and the launch refusals.

Only the condition differs, in two arms run in this order:

| Arm | Vehicle | PX4 parameters set (read back) | Command |
| --- | --- | --- | --- |
| `iris` | `navrl_iris`: PX4's iris model plus navrl_quad's downward rangefinder; airframe 10091 = iris 10015 plus MOTAR's speed, tilt and yaw-rate limits and the rangefinder height reference, as 10090 | `MPC_XY_VEL_P_ACC 2.5`, `_I_ACC 0.1`, `_D_ACC 0` (the S3b velocity loop). Rate loops and `THR_MDL_FAC` stay at iris's PX4 defaults | as S3b |
| `delay1` | navrl_quad, exactly as S3b | as S3b | each governed command is sent one 0.1 s step late; the first step commands hover |

- The navrl_iris model and airframe were created outside every repository, in the PX4 tree, on 2026-11-06.
- The PX4 binary's SHA-256 did not change in the rebuild (`b1da06f0…`).

## Development smoke before the attempt

- **smoke01:** the first scheduled episode (arm `iris`, the first scene), run once with the same code into
  `~/motar_tmp/s3c_20261106/smoke01`. Iris has never flown in this harness.
- **Not part of the reading.** attempt01 refuses unless smoke01 is COMPLETE with its one episode VALID. A failed smoke
  stops the unit for repair and a new review; attempt01 is not consumed.
- **Approval:** the approval names both smoke01 and attempt01.

## Scenes

- **Arms:** the first 40 eligible 70-bar G8 r2 scenes (S3b block A's first 40), one boot each, in each arm.
- **Isaac:** these scenes' Isaac outcomes come from the same ledger.
- **Matched S3b scenes:** S3b's outcomes on these scenes come from its attempt01, where VALID.

## Validity and stop rules

- **Episode validity:** as S3b. The same function runs it, with an idle check before every episode (HOST_BUSY) and a
  runtime binding of the PX4/Gazebo stack. The binding is compared within each arm against that arm's first VALID
  episode (BINDING_CHANGED).
- **The unit stops (ABORTED)** in any of these cases (`tools/sitl_s3b_formal.run_unit` with every arm primary):
  - the kernel fault count rises;
  - HEAD or a tracked file changes;
  - the first episode is INVALID;
  - three consecutive episodes are INVALID;
  - more than 12 of the 80 episodes are INVALID;
  - a scene lacks a full-horizon target.
- Any exception, termination request or SIGTERM is recorded in `final.json` as ABORTED. The analyzer reads a missing
  `final.json` as `NO_FINAL_RECORD`.
- **No retries.**

## Reported (fixed analyzer: `tools/sitl_s3c.py --analyze`)

For each arm:
- capture, crash and timeout counts and rates;
- Δcrash and Δcapture against Isaac on the same VALID scenes;
- per-scene outcome agreement with Isaac;
- the matched-scene table against S3b (pairs, same outcome, crashes in each);
- timing per arm: lateness, truth age, compute, real-time factor, EKF error and parameter-problem episodes;
- integrity problems (VOID-like, reported): ledger order against the schedule, kernel faults, tree changes;
- the S3b reference, bound by its ledger's SHA-256 and reading label, or `S3B_NOT_AVAILABLE`;
- iris's crashes at its own footprint (about 0.235 m half span), recomputed from `steps.npy` clearances, ungated.

There is no pass or fail label.

## What follows, and what does not

- It estimates, on 40 scenes, how F's closed loop changes on PX4's iris and with 100 ms of extra command latency. That
  informs whether gain-randomized or latency-robust policies (GR-1b/GR-2) are needed before real flight. There is no
  decision rule.
  - Iris differs from navrl_quad in mass, thrust margin, inertia, geometry, motors, rate gains and collision size
    together. The arm does not separate them.
- **It does not follow:**
  - real-vehicle behaviour: iris is a heavier, slower proxy, not the ref5in design point (1.20 kg, T/W 3.26), whose
    Gazebo model is S5's;
  - a delay tolerance bound: one delay value, command-side only;
  - S3b acceptance.

## Known limits

- **Crash footprint:** a crash is MOTAR's footprint rule (0.14 m half width). Iris's arms reach farther, so iris can
  touch a bar physically without a MOTAR crash. Gazebo contacts are not recorded.
- **Delay scope:** only the command is delayed. Sensing and PX4's own latency are unchanged.
- **Scope:** one seed, first episodes only, 40 scenes per arm. The rate differences are descriptive.

## Execution envelope

- **Process:** as S3b. CPU only; cores 6,7,12,13; `nice -n 10`; one thread; `timeout -k 30 7200`; idle host; frozen
  tree.
- **Single use:** attempt `~/motar_tmp/s3c_20261106/attempt01`.
- **Approval:** `~/motar_tmp/s3c_20261106/execution_approval.json`, naming `tools/sitl_s3c.py`'s `SCOPE` and the
  commit.
- **Duration:** expected about 70 min.
