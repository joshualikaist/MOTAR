# C-Planner charter v1 (fixed 2026-11-04, before any C-Planner gate measurement)

Branch `cplanner/dev` (worktree `src/MOTAR-cplanner`), base `62e59c031ad27c4d00e7bef3110fc17d3f0f6249`
(`codex/mt-p1`). Code: [`cplanner/`](../../cplanner/README.md). Decision record: `D-2026-11-04-0` in
[`DECISIONS.md`](../DECISIONS.md). The roadmap this charter fixes was proposed on 2026-11-03
(`~/motar_tmp/cplanner_roadmap_20261103/ROADMAP_PROPOSAL.md`) and approved by the user on 2026-11-04.

## 1. Research question

How much mission performance does a sound static-bar certificate built from a sparse 72x4 LiDAR scan cost, and
how much collision risk does it remove, when a planner intercepts a moving target among bars?

- Arms of the eventual matched comparison: **C-OFF** (predictive planner, certificate computed and logged as a
  label only), **C-FILTER** (only certified segments are executed; otherwise the fallback), Frozen Policy F, B-PP.
- C-FILTER − C-OFF is the certificate's own effect. A high refusal rate is a result, not a failure of the study.
- The final approach is outside the certificate by construction (a certified segment keeps >= 0.45 + 0.20 =
  0.65 m from the target centre; capture needs < 0.5 m; `cplanner/docs/TERMINAL_INTERCEPTION_SPEC.md`).
  Capture is reported twice: with certified navigation only, and in total. They are never merged.

## 2. Claims

Allowed, each with its measured scope: conditional static-catalog-bar freedom of a certified segment under the
declared assumptions (§4); measured certification rates, refusal reasons and runtimes; measured capture, crash and
collision-free capture rates in simulation.

Not allowed: overall safety, real-flight safety, a certified capture rate, real-time guarantees, novelty or
state-of-the-art claims before the prior-art check and a matched comparison exist, and any number not produced by a
machine-readable record.

## 3. Milestones and gates

| M | Content | Gate (verdict label) | Sessions |
| --- | --- | --- | --- |
| M0 | charter, branch, host mitigation, R2' preregistration | committed | S1 |
| M1 | P0-R r1 full qualification R2' + independent review R3' | R2' CONDITIONAL, then R3' PASS under §5 | S2-S3 |
| M2 | MOTAR 72x4 adapter: V1 hit-bit qualification, CPU warp replay of B1-R poses/layouts | V1 PASS, replay FALSE_CERT 0, **G-alpha** | S4-S6 |
| M3 | per-packet exclusion grid (equivalence to `_coverage_B`), predictive planner, simulator arm | equivalence PASS, **G-beta**, DEV smoke completes | S7-S10 |
| M4 | eval-only control logs, `collision_free_capture`, tracking-error bound, terminal mode | non-interference OFF/ON, eps(t) bound recorded | S11-S13 |
| M5 | GPU shadow (C computes beside F, does not act) and closed-loop pilot | **G-gamma** | S14-S15 |
| M6 | preregistered matched comparison and independent review | preregistered labels | S16-S18 |

Gate thresholds (fixed here; each gate's own preregistration repeats them before its measurement):

- **G-alpha (certifiability on replay).** On CPU replay of B1-R trajectories at 160 and 205 bars, the fraction of
  decision steps outside the terminal zone (target centre > 0.9 m) at which at least one candidate segment that
  reduces the distance to the predicted intercept point is STATIC_BAR_CERTIFIED.
  - Any FALSE_CERT against the replay ground truth: stop and repair before anything else.
  - >= 60 %: FILTER stays a pilot arm. 20-60 %: the pilot runs C-OFF with certificate labels only, and one
    map/candidate improvement is allowed. < 20 %: the certificate is not used as a filter (Track A paused); it
    stays a reported label.
  - Limitation recorded with the result: replayed trajectories are F's or B-PP's, not C's own.
- **G-beta (compute).** Candidates K = 8-16 on 72x4, single core: p95 of step + certify <= 50 ms on the replay.
  Otherwise the evaluation scale is reduced by a recorded decision before any pilot. Reason: B1-R scale is about
  1.7e7 decisions; at the historical 0.33-0.41 s per 8-candidate call (hardware-faulted gate, diagnostic only) that
  is about 1,600-2,000 CPU-h, at 50 ms about 240 CPU-h.
- **G-gamma (pilot).** If C-OFF capture is more than 10 pp below B-PP in the pilot, one design repair before the
  matched comparison. If C-FILTER loses capture without reducing crashes relative to C-OFF, that is reported as the
  price of certification, not repaired away.

## 4. Certificate assumptions (unchanged from P0-R)

Bars are static, upright, full-height (0-3 m), axis-aligned and from the declared `bars_h3` catalog; pose is exact;
declared beam statuses are true. B1-R ran `NAVRL_BAR_POOL=bars_h3` (every cell receipt). MOTAR's default pool is the
2 m `bars`; the M2 adapter must read the pool from the run's receipt and refuse any other pool.

## 5. Review scope and threat model (decision D3)

Independent reviews (Opus 5.5, fresh context, read-only, writing only to their audit directory) classify findings
as P0-blocking only inside this scope:

1. a STATIC_BAR_CERTIFIED result for a packet that truthfully represents the scene under §4;
2. evidence semantics: completeness, status validation, sensor-height eligibility, free-ray clipping, map
   veto-only behaviour, incomplete-packet handling;
3. temporal binding: a certificate bound to its own packet and step, staleness, replay;
4. monotonicity: camera input never changes a certificate; a track only removes certificates;
5. vacuous tests or provenance gaps that would hide 1-4.

Out of scope, recorded as limitations and never P0-blocking: in-process adversarial Python (object subclasses,
metaclasses, NumPy or warnings hooks, signals, threads, asynchronous mutation of caller arrays); an adapter that
declares false statuses, health or pose (declarations are trusted by design, `DECLARED_NOT_AUTHENTICATED`);
non-catalog and dynamic objects (`UNVERIFIED` scope); real-time guarantees; exact boundary grazing. This
follows the cooperative-caller trust model the user chose on 2026-10-29 (model A).

Stop rule (kept): a new in-scope structural defect stops the cycle and goes back to the user; no automatic next
version. In-scope local defects get one repair round and a re-review.

## 6. Approvals (D-2026-11-04-0)

| # | Decision | State |
| --- | --- | --- |
| D1 | KRoC 2027 manuscript does not include C-Planner results | approved |
| D2 | hybrid track: C-OFF predictive planner with certificate labels; FILTER only after G-alpha | approved |
| D3 | review scope and threat model of §5 | approved |
| D4 | development on `cplanner/dev`, local commits; no push, no merge | approved |
| D5 | M1-M3 CPU work proceeds through its gates without intermediate approvals; each gate verdict is reported; a FAIL stops at the stop rule | approved |
| D6 | DEV GPU envelope: eval-only, Frozen F unchanged, no PPO, <= 16 envs, <= 2 GPU-h in total, results labelled DEV and never evidence; each run still has a committed preregistration | approved as an envelope |
| D7 | eval-only instrumentation (`collision_free_capture`, per-step command/governor/Lee/velocity logs), opt-in and OFF by default, with F and B-PP re-measured for the matched comparison | approved as scope; GPU runs under their own preregistration |
| D8 | BIOS update before runs | declined by the user; replaced by the host mitigation of §7 |

## 7. Host mitigation (no BIOS update, no sudo in agent sessions)

- Every C-Planner CPU process is pinned with `taskset -c 0-3,8-11,14-23`: it excludes cores 2, 3 and 6
  (cpu4-7, cpu12-13), the cores of every machine-check record since 2026-10-28.
- At most 12 worker processes; `concurrent.futures` (a dead worker fails the suite instead of hanging).
- The kernel fault record and boot id are compared before and after every command. A change voids the run
  (AGENTS.md); it is recorded as aborted and reported, not silently rerun.
- Deterministic suites run twice; randomized suites rerun a seed prefix with per-seed SHA-256 digests.
- The host stays `NOT_REMEDIATED` (BIOS 0806, microcode 0x12f). This is a disclosed limitation of every
  C-Planner result.

## 8. Information access (to be repeated in the M6 preregistration)

- F: target-carved, camera-fused 4x72 scan, 5-step history, tracker features, no raw image.
- B-PP / B-PN: exact own state, tracker mean and active flag, raw `_lidar_distance_m()` scan, `last_target_like`.
- C: the raw scan with an explicit per-beam status (after V1), declared pose, tracker, camera labels, and the
  `bars_h3` catalog prior (axis-aligned, full height), which F and the B arms do not get. This extra prior is
  disclosed with every comparison.
