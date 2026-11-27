# Dev08 amendment4: independent targeted review r7

Reviewer: Codex fresh-context delegated reviewer `/root/dev08_review_r7`.
Model: GPT-6 family per the supplied role; exact runtime model variant **NOT_RECORDED**. This review is not attributed to Opus.
Date: 2026-11-05, approximately 12:31–12:39 UTC.

Reviewed commit: `821706de308ad8ef739c124d73c972dace16d778`.
Parent: `c4974f416249e543384f7dedf8128d805c408b68`.
Worktree: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`.
Observed HEAD: `80bb687f5db092cb67f92848bcf159bb1679aa43`; its changes after the reviewed commit are documentation only. Runtime/test bytes inspected match the reviewed commit's bound sources. `git status --porcelain` was empty.

Root clock checked, never reset or closed: original start 08:17:36.500168 UTC; finish-by 13:07:36.500168 UTC. This was a brief closeout review.

## Verdict: PASS_WITH_LIMITS, CPU/runtime implementation and reviewed proposal only

No BLOCKER or MAJOR was found in the amendment4 delta. The previously unreviewed amendment4 changes are now independently reviewed. The accumulated component reviews plus this targeted review are sufficient for closing **motar-5md's CPU/runtime implementation acceptance scope**, with the permanent-test MINOR below and inherited limits recorded. This does not authorize execution or establish a live result.

That conclusion does not treat old reviews as reviewing later bytes. In addition to the actual `c4974f4..821706d` diff, I compared the owned-session and link r3 snapshots with the submitted source, inspected their small later changes, and probed the relevant repairs. Those limited checks are described below. This is not a new exhaustive audit of every earlier source line.

The concrete 42-member proposal is reviewed as amendment2 + amendment3 + amendment4, together with D-2026-11-05-2. A separate scoped live approval and a reviewed CLI activation remain necessary. `main()` still refuses. Measurement remains NOT_RUN; live assumptions remain NOT_VERIFIED. No formal S3b/S3c/S4/S5 gate, S3a equivalence, candidate gain selection or real-flight claim follows.

## Findings

### BLOCKER / MAJOR

None in the reviewed delta or the small earlier runtime repairs inspected here.

### MINOR m7-a: the new torch branch lacks permanent suite regression coverage

Locations: `tools/sitl_s3b_dev08_prefix.py:433–444`; `tests/test_sitl_s3b_dev08_prefix.py:374–378`.

The committed Execute fixture always passes an injected `B`, so it skips the production import branch containing `torch.set_num_threads(1)` and the get-count refusal. Updating its environment fixture does not test these four new lines. A regression deleting the torch enforcement can therefore pass the existing Execute tests.

I independently exercised that exact production branch using fake imported bridge/torch/aerial_gym/pymavlink modules, an empty synthetic schedule and a recorder that writes nothing. The valid case reaches warm-up only after setting one thread; a torch count of 8 is refused before warm-up/open; OMP4 and MKL4 are refused before bridge import. Removing the added thread guard in memory makes this probe fail. Thus current behavior is verified, but the permanent regression gap remains. A future CPU test should retain these positive and rejection controls. No real bridge, policy or simulator was loaded by this probe.

## Checks and acceptance scope

| Acceptance area | Assessment and evidence |
| --- | --- |
| One compute thread | `sitl_dev08_attempt.py:235–255` records and requires exact string `1` for OMP/MKL. `execute():412–414` checks before heavy imports; `:440–444` sets/checks torch intra-op count before `warm_up():448`. One accepted environment, 14 invalid/missing/non-string controls rejected; fake production branch positive/refusal controls pass and kill a removed-guard mutant. The setting controls compute parallelism, not the total number of receiver/watchdog/library OS threads. |
| Positive/rejection lifecycle; unrelated sentinel preservation | Read added X18/X20 fixtures and their ownership checks. Independently ran normal cleanup with unrelated sentinels, cleanup escape resampling, and journal recovery including the new escape case: 3/3 pass. Only own fake Python processes were launched/signalled. |
| Later owned-session repairs after r3 | Snapshot diff shows only the declared journal-limit wording, malformed-marker ValueError-to-LockBusy mapping (`:518–521`), and cleanup SIGINT deferral (`:767–771`). Independently injected SIGINT during termination: it was deferred, receipt was written, prior handler received it afterwards, and unrelated sentinel survived. Marker ordering/unreadable-marker positive/refusal test and journal-survivor reporting test also pass: 3/3. The previously declared handler/receipt windows remain inherited limitations. |
| Parameters/source/freshness/clock controls | Link r3 snapshot vs current executable behavior is unchanged: later source edits are wording/docstrings. Read current parameter read/set/fresh checks and TelemetryGate source/clock/skip/freshness checks, plus corresponding fixture controls. Reviewed corrected pass-offset wording and receiver-promptness assumption. Independently ran three selected current link controls; removing the next-position order bound and removing settle calls each makes an added test fail. This confirms the earlier N10/N11 coverage repairs are non-vacuous; it does not establish live MAVLink timing. |
| Fixed scenes and ledger | `scene_specs():311–331` admits the fixed twenty verified scenes and exact target windows; unchanged prefix contract retains 208 checkpoints for 70/env5 and 600 elsewhere. Independently checked schedule: 42 unique members, exactly 2 native smokes, 40 main records, 20 scene identities. Read ledger sequence/hash-root/final validation and complete-pair summary rules. The root separately verifies full fake-member integration; no rollout ledger was produced here. |
| Native command-prefix identity | Amendment4 changes neither step/pacing/terminal semantics nor historical sources. Read added floor/y-bound/minimum-distance assertions against the historical comparator. Existing real bridge + Policy F identity evidence and its script are hash-bound and still available; CPU scripted-stream scope and stream-phase differences remain disclosed. I did not rerun real-policy identity or claim live/native outcome identity. |
| Source/platform/host/resource/fresh output guards | Read source list (amendment4 added at attempt `:36,:51`), clean/source binding and fresh external-directory checks, platform environment/header recording, host/fault checks and resource/timeout proposal. Added guard test cases cover unrelated tracked dirt, missing/dirty external source, inside-Git output rejection, fault-change and bad-record exclusion. Root's focused suite supplies their execution verification. The existing sampled RSS/cleanup-detection/between-member-budget limits remain. |
| Concrete bounded execution proposal awaiting approval | Amendment4 replaces OMP/MKL4 with 1; amendment3's outer timeout remains `timeout -s TERM -k 90 5800`; fixed member order, output path, no substitution, episode/budget/resource/source/host stops remain. D-2026-11-05-2 consistently fixes skip_max_s=0.15 for CLOCK_SKIP and RUNNER_LATE before smoke and across both arms, without authorizing CLI activation or a live run. The future approving CLI must call execute in the main thread and verify the live smoke requirements under the committed bounds. |

## Record bindings

`probe_results.json` records an independent rehash against commit blobs and on-disk evidence:

- 13/13 `validation_amendment4.json` source hashes match the exact reviewed commit blobs.
- 75/75 external evidence hashes and 13/13 external review hashes match.
- Both focused/public test-log hashes and the validation-generator hash match.
- The committed Fable analysis equals its bound external original.
- The schedule and new environment/torch controls pass.

The historical amendment3 record has exactly the two disclosed external mismatches: `step_mutants/results.json` and `step_mutants/run_mutants.py`. Their old bytes remain unavailable. Amendment4 §5 explicitly preserves that loss and binds the new contents; it does not retroactively repair the old binding. The new record therefore verifies; the old external evidence is still partially unreproducible.

The timing benchmark JSON/log/script/plan hashes verify. The one-thread JSON reports n=5000, max=13.8 ms and zero above 60/100/150 ms, with the plan hash matching the fixed plan. I inspected the timing script and rule but did not rerun that benchmark or reconstruct quantiles from raw per-iteration timings. It measures scripted CPU observe/policy/governor compute, not live simulator/receiver timing, a worst-case latency guarantee or candidate outcomes. D-2026-11-05-2 preserves the measured rule's 0.10 s conclusion while selecting the user's 0.15 s tolerance.

## Inherited limits retained

This review does not resolve step M20, owned-session R2_N27, r6 m6-a/m6-b, the handler-swap/receipt windows and first-signal semantics, numpy arrays/scalars serialized through default=str, or budgets checked only between members. The earlier component reports' further notes and declared sampling/ownership limits still apply. Old review verdicts remain tied to their snapshots/commits.

Live checks still include fresh-rootfs boot, PARAM echo/readback, stream phase/arrival/freshness, prompt receiver, real-time factor, and shutdown. The current timing-only measurements and fake fixtures cannot establish those. The approving entry and smoke protocol require their own review/verification; the present default entry still refuses.

## What ran and repository integrity

- `probe_r7.py`: hash checks, schedule/environment checks, fake production-branch acceptance/refusal/mutation checks, and 3 lifecycle fixtures. Lifecycle part: 3 tests OK in 3.601 s.
- `probe_prior_deltas.py`: 3 selected link controls OK; two in-memory mutants rejected; 3 owned-session delta/marker/survivor controls OK in 5.563 s.
- Commands used CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE=1, OMP_NUM_THREADS=1, MKL_NUM_THREADS=1, nice10, cores6/7/12/13, 8 GiB address-space limit and timeout60/kill5. Temporary outputs were confined to this audit directory. No real Policy F, PX4, Gazebo, SITL, GPU or real episode runner was launched. All fixture processes ended; no owned process remains.
- Git use was read-only. No Beads mutation. No file was written inside any repository. `git diff --check c4974f4 821706d` passes. Current tracked worktree remains clean.
- Parent reports its separate focused validation: 131 OK in 58.429 s, host faults0→0, and a synthetic 0.15 propagation check across42 members/40main+2excluded smoke. Those are parent-run checks, not claimed as my independent test runs.

Git Traceability: branch camonly/dev; base c4974f416249e543384f7dedf8128d805c408b68; reviewed commit821706de308ad8ef739c124d73c972dace16d778; observed HEAD80bb687f5db092cb67f92848bcf159bb1679aa43; new commit NONE (read-only reviewer); changed repository files NONE; tests/probes as above; working tree clean; remote push NO; main merge NO; raw/checkpoint integrity preserved (no writes or checkpoint deserialization).
