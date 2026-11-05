# Dev08 amendment5 — authorized CLI, numeric records and pre-main smoke timing, 2026-11-05

Parents: immutable amendments1–4 of `prereg_sitl_dev08_candidate_gains_v1`. This amendment is written before
any dev08 PX4/Gazebo measurement. It changes no scene, target prefix, physical terminal, arm parameter,
Frozen Policy F, member order, analysis or `skip_max_s=0.15` decision.

## Authorization and execution unit

The user explicitly instructed: "오케이. 본 측정까지 하고 나한테 말해. 얼마나 걸려?", then chose
"지금부터 새 5시간 작업 세션으로 진행". This authorizes essential CPU repairs/CLI, independent source review,
the existing two-native-smoke +40-main diagnostic, analysis/review and local records. It authorizes no GPU,
formal stage advance, flight, push or main merge. The new session starts 2026-11-05 13:45:22 UTC; finish-by
18:35:22 UTC. The receipt is `/home/fair/motar_tmp/dev08_execute_20261105/root/user_authorization.json`.

The original fixed42-member schedule is retained. A new smoke-only phase split is unnecessary under this
full-unit authorization. Smoke failure still stops before main; no failed member is substituted or automatically
rerun. The CLI approval file must name the reviewed clean execution commit, exact existing scope, user/date and
`skip_max_s=0.15`. It is created after the source commit and bound into the attempt header by path and SHA-256.

## Guarded entry point

`tools/sitl_s3b_dev08_prefix.py` defaults to refusal. Execution requires all of:

- `--execute --approval-file <record.json> --attempt-dir <fresh external directory>`;
- the main Python thread;
- the existing scope/clean-commit, environment, source/input/platform/host/resource guards;
- the selected float `skip_max_s=0.15`, refused by the CLI if different.

The entry returns0 only for COMPLETE, and nonzero for refusal, interruption or ABORTED. CLI availability is not
live verification. All tracked files are committed and frozen before launch; no tracked edit/commit occurs until
the guarded process has ended. Failure output is preserved.

## Numeric ledger records

Per-member ledger values are converted to JSON numeric primitives: numpy integers/reals to int/float, numpy
arrays/booleans through recursive `tolist()`. Non-finite and unsupported values are retained as null with an
explicit record problem and stop through existing SMOKE_FAILED/RECORD_INVALID rules. The ledger no longer uses
`default=str`. Header/final metadata retain their existing serializer; this change concerns per-member records.
No analysis value is silently coerced to a string or a success status. Historical ledgers are not edited.

## Live smoke timing, fixed before measurement

The original proposal requires approximately real-time simulation and compute inside the0.1s command interval,
but VALID alone did not numerically assess those assumptions before main. Add these predeclared development
checks for each of the two smoke members:

| Recorded quantity | Required value | Failure behavior |
| --- | --- | --- |
| `real_time_factor` | finite,0.95 through1.05 inclusive | member HARNESS_INVALID, attempt SMOKE_FAILED; no main |
| `compute_ms_max` | finite,0 through less than100ms | same |

The RTF tolerance is an engineering allowance of5% around1, not a statistical test or transfer criterion. It is
fixed here before seeing live data. Compute starts after the telemetry checkpoint wait and covers the policy/
governor step, checkpoint metadata and command send; terminal checks are recorded too. Every checkpoint stores
`compute_ms`; the ledger stores its maximum. The historical frozen runner's misindexed compute fields are not
reused. These smoke checks do not change the main endpoint estimator or claim receiver promptness: PX4 stream
phase/receipt ambiguity remains as disclosed in amendment2 and the link reviews.

Existing stream freshness/source/order and parameter checks remain mandatory. Session receipts must record
owned-only cleanup; empty-rootfs boot and native readback are assessed from the live evidence. No timing bound
is widened after a failed smoke.

## Bounds and command

Keep OMP/MKL/torch compute threads1, CUDA hidden, PYTHONNOUSERSITE1, worktree PYTHONPATH, idle host,
cores6/7/12/13 and nice10. Keep12GiB sampled aggregate RSS,240s per member,2400s per-density main and5400s overall
between-member budgets. Keep the outer `timeout -s TERM -k 90 5800` from amendment3. In addition, bound each
inherited process's virtual address space to16GiB (`ulimit -v 16777216`); this is an execution resource cap,
not a statement about measured RSS. A cap can abort the unit; completion is not guaranteed.

From the clean approved worktree, the external launcher sets these environment/limits and calls:

```bash
timeout -s TERM -k 90 5800 nice -n 10 taskset -c 6,7,12,13 \
  /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/sitl_s3b_dev08_prefix.py \
  --execute --approval-file /home/fair/motar_tmp/dev08_execute_20261105/root/execution_approval.json \
  --attempt-dir /home/fair/motar_tmp/sitl_s3b_20261105/dev08_prefix_attempt01/
```

## CPU/source verification before launch

- The CLI refuses by default and on missing/wrong approval; a synthetic valid invocation reaches execution,
  carries the exact bound/file digest, and reports COMPLETE versus ABORTED correctly. Off-main-thread entry fails.
- Permanent tests exercise the real import branch with fake bridge/torch modules: one thread reaches warm-up
  and42 fake members; wrong torch count and OMP/MKL4 fail before warm-up/output creation.
- Finite numpy records retain numeric types; non-finite arrays/unsupported values remain visible and stop.
- Interruptions after member return versus after ledger append name the correct recorded/unrecorded member.
  Held-signal tests cover SIGTERM/SIGHUP/SIGINT and direct delivery to another thread.
- Positive smoke timing controls pass; invalid/missing/non-finite timing stops before the first main member.
- Focused CPU and PUBLIC_CPU must end with0failures/0errors; documentation/index checks pass. Independent review
  covers actual source/plan before the execution commit is bound and launched. Results get a separate review.

Unchanged limits include other surviving mutants, handler-swap/receipt windows, sampled ownership/RSS limits,
the two unrecoverable historical external evidence files, S3a NOT_EQUIVALENT and strict600 BLOCKED. This run is
an observed-prefix development diagnostic; no full60s outcome, gain selection or formal S3b acceptance follows.
