# S5 safety v2, amendment 2: stable MAVLink dependency binding (2026-11-10; NOT_RUN)

- **Amends:** `docs/plans/prereg_sitl_s5_safety_v2_2026-11-10.md` and amendment 1
  `docs/plans/prereg_sitl_s5_safety_v2_amendment1_2026-11-10.md`. Both remain immutable and in force except below.
- **Status:** NOT_RUN under this amendment. CPU verification and independent review precede a local commit and a new
  scoped user approval naming the final HEAD. No live PX4/Gazebo smoke or attempt is authorized by this document.

## Reason and historical evidence

At `023998f`, smoke02 completed N0 (VALID, PASSED), then N0R was INVALID `BINDING_CHANGED`; no attempt01 started.
The ledger at `~/motar_tmp/sitl_next_20261110/s5_safety_v2/smoke02/episodes.jsonl` and committed
`docs/evidence/sitl_2026-11-05/s5_safety_v2/smoke02_episodes.jsonl` bind the first decoder to
`pymavlink/dialects/v10/all.py` and the second to `pymavlink/dialects/v20/all.py`.
Installed pymavlink 2.4.50's `mavutil.auto_mavlink_version` switches to v20 on the first v2 byte and retains the global
selection. This is a harness binding defect; no safety-function failure is inferred from it.
`SF.binding_core` already excludes the generated world and compares source SHA values only. Per-episode file paths,
log directories and world text therefore do not cause the comparison to change. Its code is unchanged.

## Changes fixed before the new smoke

1. `pymavlink_files(link)` records **three** files on every real link: `mavutil.py` and the installed
   `dialects/v10/<current_dialect>.py` and `dialects/v20/<current_dialect>.py`. It reads their path, size and SHA-256
   without importing a dialect, forcing MAVLink 2, changing the environment, or calling `set_dialect`.
   Missing sources or an invalid dialect name fail closed. The old single `pymavlink_dialect` key is replaced by
   `pymavlink_dialect_v10` and `pymavlink_dialect_v20`. Both source digests remain compared across scenarios and are
   checked against disk during analysis. Existing binding keys retain their meaning.
2. `mavlink_decoder_at_connect` and `mavlink_decoder_after_pin` record the selected decoder descriptively outside
   `runtime_binding.files`; decoder selection may change but source files may not. A binding failure closes the link
   through the existing `finally` path.
3. A new single-use **smoke03** replaces smoke02 as the launch gate. `attempt01` remains the never-created attempt
   folder. Both prior smokes remain ABORTED and unchanged. No continuation from N0R and no retries are allowed.
4. The new SCOPE and amendment are source-bound; amendment 1 remains in `OWN_SOURCES`. Prior approvals are refused.

## CPU verification contract

- A fresh subprocess with no `MAVLINK20` opens two consecutive real `mavutil.mavlink_connection` instances over actual
  MAVLink 2 wire bytes. The first starts on v10 and switches to v20 on receive; the second starts on v20.
  The new three-file binding is identical before/after receiving and between connections; both records complete
  through `run_unit`. The old active-decoder-only binding reproduces `BINDING_CHANGED` on record 2.
- Changes to the bytes of either dialect file or mavutil still cause `BINDING_CHANGED`; a missing dialect is refused.
  These checks include a positive two-record COMPLETE control with unchanged source bytes.
- Per-episode world/path/log variation is excluded, while a source digest change is detected.
- The fake-PX4 live path uses the installed mavutil dependency set where available. Existing positive and negative
  safety/telemetry/recording controls and PUBLIC_CPU must pass. No live discrimination result is claimed.

## SCOPE and execution

The new approval must quote exactly:

```
S5 safety v2 amendment2: PX4 v1.14.3 SITL on navrl_quad (stand-in vehicle), fixed safety parameter set v1, scenarios N0 OBL GFH GFV OVJ OVR RCL DLL KIL FDS once each in this order, one fresh-rootfs boot per scenario; development smoke03 (nulls N0 N0R N0J, read for harness validity only) then attempt01; no retries; SITL only, no claim about real hardware
```

The launch commands, CPU affinity, idle-host checks, single-thread environment, 8 GiB RSS guard, 3 GiB free-disk floor,
outer timeouts, no-retry and first-invalid stop rules, ten-scenario attempt schedule, three-null smoke schedule,
parameter values, observation contract and all validity/acceptance criteria remain as preregistered.
Both `--smoke` and `--launch` require the final committed HEAD and a new scoped approval. A failed smoke refuses the
attempt. Historical evidence is not reanalyzed under the new source contract.
