# P0-R r1: repair contract, fixed before measurement (2026-11-03 KST)

User instruction: resume the interrupted P0-R work; exclude manuscript work. Original P0-R sources, preregistrations,
R2 results and R3 FAIL remain unchanged at `/home/fair/motar_tmp/p0r_20261029` and
`/home/fair/motar_tmp/p0r_independent_20261029`. BASELINE_MANIFEST.json binds the copied starting files.

Scope is CPU source repair and bounded focused validation in this new directory. No full R2 qualification, P1,
MOTAR production integration, GPU, PPO, checkpoint change, push or main merge. Prior no-commit external-work scope
is preserved. An independently reviewed local repair is still NOT_QUALIFIED until a new approved full gate.

## Predicted failures and repairs

- F1: `core._extent` gives planar rays free evidence outside bar height. Apply strict `0 < z < 3` eligibility
  to every layout. At/beyond either boundary no free-ray evidence may be produced. Keep coverage_b byte-identical.
- F2: suppress synchronous numpy floating-point/warning callbacks during public entry points, restore their process
  settings afterwards, validate map report points/time, and bind results to the packet captured by certify.
- F3: validate the owned copy's dtype/shape after copying, including paths; asynchronous mutation remains outside
  trust model A and is not claimed solved in general.
- F4: correct the documented track contract to float64 (5,), and identify existing check_static.py as the actual
  catalog/AST check. Only certificate result dictionaries carry the certificate scope. Step ingestion reports and
  map-state labels are diagnostic outputs, never free-volume or safety certificates; retain their API.
- F5: if segment computation completes beyond a requested decision deadline, return DEADLINE_UNKNOWN rather than
  a late positive certificate. This does not interrupt Python computation or establish a hard runtime bound.
- F6: add deterministic multi-packet controls that actually cross N_STATIC/T_STATIC, produce static/conflict cells,
  exercise a track-induced certificate rejection, and retain accepted valid certificates. Expand height coverage.
  Do not silently change the historical R2 sample count/rules or claim a full random-scene gate has run.

## Focused acceptance checks

1. Reproduce F1 on baseline: truthful z=3.05 planar scan may wrongly certify a bar-crossing path. The same case
   must be refused after repair. For both layouts check z below/at/inside/at/above (0,3), and non-zero in-height
   open-scene accepted certificates. Ground truth stays in the test harness.
2. Adversarial np.seterrcall / warnings.showwarning callbacks are never invoked by the repaired public APIs;
   caller error/warning settings are restored, invalid map inputs refuse, and ordinary map/certificate calls work.
   Result binding survives an artificial trusted test-side state mutation during a certificate calculation.
3. Controlled test-side copy mutation is rejected for packet arrays and paths; ordinary copies are private/read-only.
4. A deterministic injected slow segment is refused after its decision deadline; an untimed matching case certifies.
5. The temporal positive control records >0 static cells, >0 conflict cells, >0 lost certificates with the track,
   >0 accepted baseline certificates, 0 gained certificates from a track, and identical results with camera labels.
6. Existing static, attack, scenario and map suites are inspected for cost before use. Execute only a small bounded
   CPU subset; log exact commands, limits, outputs and exception counts. No global/public suite or full R2 by default.
7. Fresh-context independent review inspects actual source, tests and this contract read-only, writing only audit/.
   Record the reviewer/model accurately. Findings remain open unless fixed and rechecked; no automatic GO/PASS.
8. Check host kernel MCE/segfault/general-protection records before and after every validation command. Use one worker
   and one numerical-library thread, CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE=1, explicit timeouts. Any new host fault
   invalidates measurements and stops execution. A clean short check does not resolve the historical host hold.

These focused checks do not replace the old randomized R2 regressions/noninterference/timing or an approved new
qualification. Evidence is written to new result subdirectories and never overwrites an earlier measurement.
