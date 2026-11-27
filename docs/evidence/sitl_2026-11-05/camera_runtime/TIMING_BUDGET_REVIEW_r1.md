# REVIEW r1: contract-derived capture-age bound (MOTAR-camonly camonly/dev @ 571eb3e + uncommitted diff)

Reviewer: Claude Opus 5.5, read-only, 2026-11-23. No repository edits, no git changes, CPU only. Probe scripts are in
the session scratchpad, not in the repo.

## Verdict: PASS_WITH_LIMITS

There are no P0 or P1 findings. CAM-K2 v3 keeps 0.15 s on every path. All capture-age checks in the three modules use
the derived bound. Every cadence, clock, inference and gateway estimator check keeps 0.15 s, with one exception: the
session's finish() receipt check (P2-2). A wider bound cannot be reached today: both pinned training reports lack the
key, and a new lineage needs new PINS in code. Three P2 points must be closed before any lineage with a nonzero delay
is used.

## Tests run (conda aerialgym, CUDA_VISIBLE_DEVICES="", OMP_NUM_THREADS=1, PYTHONNOUSERSITE=1, from tests/)
- test_camera_timing_budget: 10 OK. test_camera_ros2_px4: 19 OK. test_camera_ros2_session: 9 OK.
  test_camera_policy_contract: 8 OK. test_camera_policy_runtime: 7 OK.
  test_camera_runtime_outdoor_r1.SessionHoldOnRefusal: 7 OK (it asserts valid_until == capture + .15).
- Runtime probe (object.__new__ runtime, _validate_timing): no delay accepts 0.150 and refuses 0.151. Delay "2"
  accepts 0.151 and 0.30 and refuses 0.351.
- Session probe (see P2-2): with delay "2", a receipt 0.20 s old at publication is proposed. With no delay it expires.

## Q1: classification of every MAX_AGE_S / bound use
- runtime:1176 rgb/depth/state capture age uses the derived bound (correct). Session odometry is capture-aligned within
  25 ms, so state['t'] must share the camera bound. The runtime's MAX_AGE_S (line 26) is now unused there.
- session:221 (ROS clock pause), 230 (inference duration), 233 (output stream) and the prepare period check keep
  0.15 s (correct).
- session:239 (newest stamp derived, newest receipt 0.15), 255-256 (stamp derived, receipt 0.15), 291 (odometry
  bracket: stamp derived, receipt 0.15) and 326 (finish stamps derived) are all correct. 352 and 372 (valid_until) are
  correct but informational: no code reads valid_until_s, and TwistStamped carries only the capture stamp.
- session:327 (finish RECEIPTS) uses the derived bound. This is inconsistent; see P2-2.
- gateway:326/331/332 (estimator receipt, publication and sample), 464 (update cadence), 597 and 634 (command receipt)
  keep 0.15 s (correct). 587 and 636 (command capture/generation, on accept and on reuse) use self.capture_age_s
  (correct). Generation time is never earlier than capture time, so widening it together with capture changes nothing.
- Missed capture checks outside the three files: tools/oakd_camera.py:302,313 (direct OAK-D path) and
  tools/sitl_camera.py:84,180 still use 0.15. They are narrower, so they refuse rather than widen (P3-5).

## Findings

P2-1 The trained-delay key is not produced by any training code.
`NAVRL_ACTION_DELAY_STEPS_MAX` (camera_policy_contract.py:190) occurs only in camera_policy_contract.py: grep finds it
in no aerial_gym/ file and in no other worktree. `runtime_env` is `realveh_receipt.env_navrl`, an echo of the NAVRL_*
variables at launch, not proof that the simulator applied a delay. If a future run sets the variable on a trainer that
ignores it, or implements it in physics substeps rather than 0.1 s control steps, the flight bound widens to
0.25-0.45 s for a policy that never met that delay. Training's real latency knob, NAVRL_DETECTION_LATENCY_S, is ignored;
that direction is safe. This is dormant today, because pinned v3 lacks the key.
Fix: until the training-side delay exists, refuse any nonzero value in trained_action_delay_steps. Afterwards, also
require a report field written by the delay implementation itself, e.g.
`report["action_delay"] = {"steps_max": k, "unit": "control_step", "applied": true}`, checked against env. Add the
implementing file to SOURCE_PATHS and add a test that fails when the key is not consumed under aerial_gym/.

P2-2 finish() widens a monotonic receipt check (camera_ros2_session.py:327).
This contradicts the session's own comment (lines 111-112) and the stated design ("receipt ... keep 0.15 s"). Measured:
with delay "2", stamps 0.15 s old and receipts 0.20 s old at finish, a proposal is made. The pre-change code (and
delay 0) raises "inference result expired". The effective bound becomes min(bound, 0.15 at prepare + 0.15 watchdog) =
0.30 s, so the widening is partial anyway.
Answer to the brief's question: receipts should keep 0.15 s.
(a) Receipt age at publication is local queue wait plus inference. The trained delay describes the total from capture
to actuation, which the stamp check already bounds.
(b) When ROS clocks are consistent, receipt age is at most capture age, so the wider receipt check permits nothing
extra. Its only job is to guard against ROS-stamp or clock faults (wrong driver offset, sim time), and widening it
weakens exactly that guard.
(c) Camera transport (35-77 ms) happens before receipt, so it does not use up the receipt budget.
Fix: revert line 327 to `MAX_AGE_S+1e-9` for item.receipts. Add positive and negative tests: receipt age 0.14 at finish
is proposed and 0.16 expires, both under delay "2". No current test reaches this line (the new test uses a receipt age
of 0.05).

P2-3 The gateway bound is a free number, not tied to the policy (camera_ros2_px4.py:424-436).
Any caller may pass any value in [0.15, 0.5]. The gateway process has no contract (camera_ros2_px4_node.py:169 knows
none), and nothing checks the value against the session's bound. Today it is safe: the only caller uses the 0.15
default. But if the planned ROS 1 port exposes this as a launch parameter, it becomes operator-editable, and the last
gate in the flight path could be wider than the policy's training. A gateway wider than the session is the unsafe
mismatch. In that case reuse is limited only by receipt ≤ 0.15, i.e. capture age ≤ publication age + 0.15.
Fix: give the gateway (or its node) the contract path and derive the bound there, as
`capture_age_bound_s(load_contract(path))`, instead of accepting a float. Report the contract SHA and bound in the
gateway diagnostics. Test that the port passes the derived value and that the parameter cannot be overridden.

P3-1 Mismatch handling, ROS 2 node at 0.15 with a session at 0.35 (Q3). This fails closed. In PRIMING, the first
command older than 0.15 s at OFFBOARD entry raises in _accept_command and revokes. In ACTIVE, a reused or new command
older than 0.15 s revokes, which triggers COM_OF_LOSS_T and then COM_OBL_RC_ACT with the pilot. The revoke is latched,
so there is no intermittent acceptance. It is discovered only at OFFBOARD entry, though, and valid_until_s cannot help
because it is never transmitted. Fix: publish capture_age_s in both diagnostics. Have prime() refuse when the policy's
diagnostic bound differs. State in ros2/motar_camera_policy/README.md (around line 405, "checked within 0.15 s") that
the ROS 2 gateway supports delay-0 lineages only.

P3-2 validate_contract wiring is untested. test_camera_timing_budget.py:53 ("contract check reports a bad delay") calls
capture_age_bound_s directly, not validate_contract (camera_policy_contract.py:557-560). Fix: build a contract with
build_contract(seed=1), set runtime_env[KEY]="9", and assert that a "timing:" problem and "runtime_env differs" are
both reported. As a control, with "1" assert that only the runtime_env mismatch appears, which shows that the pinned
training record is the gate.

P3-3 The runtime path is covered only by source-text assertions (test lines 113-119). Fix: add the behavioural test
above (object.__new__(CameraPolicyRuntime), contract with delay None or "2", ages 0.151, 0.30 and 0.351). It passes
on the current diff.

P3-4 Floating point. (0+1)*0.1+0.05 = 0.15000000000000002, one ulp above 0.15. The threshold bound+1e-9 differs
from the old 0.15+1e-9 (0.15000000100000002 vs 0.150000001), and valid_until_s can differ by one ulp for small stamps
(0.3+bound = 0.45 vs 0.44999999999999996). At real ROS stamp magnitudes it is identical. In practice the behaviour is
unchanged, but it is not bit-identical. Tests use places=9, which hides this. Fix: `bound = round(..., 9)`, or integer
milliseconds, plus `assertEqual(C.capture_age_bound_s(None), 0.15)`.

P3-5 Smaller hardening.
- raw.isdigit() at line 201 accepts non-ASCII digits ("٣" gives 0.45) and "03". Use `raw in ("0", "1", "2", "3")`.
- dt ≤ 0 yields a bound of 0.05 or 0.04 silently (narrower, and dt is pinned at 0.1 by the equality check).
  Require dt == 0.1.
- The two 0.5 s caps (MAX_CAPTURE_AGE_S in the contract, MAX_COMMAND_CAPTURE_AGE_S in the gateway) are duplicated.
  Assert them equal in a test.
- The runtime recomputes the bound on every frame and imports inside the property. Cache it in __init__ after
  validation.
- The oakd_camera.py and sitl_camera.py paths keep 0.15 s (fail closed). Note this limitation.

P3-6 Physical coupling for future lineages. The riskcap governor in v3 assumes NAVRL_SPEED_GOVERNOR_REACTION_S = 0.1.
At the 0.45 s cap and 3.54 m/s, about 1.2 m more distance is covered on a stale observation than the 0.45 m margin
assumes. For a lineage with a nonzero delay, require REACTION_S ≥ (k+1)·dt in validate_contract, or record that the
mismatch is accepted. The 0.5 s cap and the 3-step limit are otherwise reasonable: the largest derivable bound is
0.45 s.

## Context (not a defect of this diff)
- For v3 at 0.15 s: the gateway reuses each 10 Hz command until the next one. The stored command must therefore satisfy
  L + 0.1 + δ ≤ 0.15, where L is the latency from capture to publication. That means L must be at most about 0.05 s.
  Measured camera transport alone is 35-77 ms, plus unmeasured inference. Expect gateway revokes on hardware with v3 in
  the slower stream profiles. This follows from the user's decision; record it before the desk/field test.
- The diff changes three files listed in SOURCE_PATHS. Every contract JSON generated earlier now fails with "runtime
  source changed" and must be rebuilt after the commit (by design). No contract JSON is committed in the repo.
