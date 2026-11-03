# P0-R r1 current interface corrections (2026-11-03)

This dated amendment corrects the historical copied R0 description. The original R0/R2 documents and old source
comments remain byte-preserved to show exactly what was reviewed. For r1, this amendment and the repair contract
are the current interface reference.

- `track` is an exact float64 ndarray of shape **(5,)**: active flag, acquisition time, world x, world y, radius.
  The old R0 `(6,)` statement is incorrect. A malformed track is ignored and cannot grant a certificate.
- Every **certificate result dictionary** has the declared `scope` fields. Step ingestion reports and `map_states`
  lists are diagnostic outputs, not certificates. A label such as VALIDATED_FREE is map evidence state, not an
  overall safety claim. No real-flight, dynamic-object, pose-error or tracking-error guarantee follows.
- The actual catalog/AST checks are both in `tests/check_static.py`. Historical comments in registry.py and the
  byte-identical coverage_b.py mention nonexistent check_catalog.py/check_coverage_ast.py filenames; use
  check_static.py. The coverage algorithm and its bytes are unchanged.
- All layouts require sensor height strictly between the declared static-bar bounds before any free-ray evidence
  is used. Exact boundary grazing remains unqualified. This binds only the synthetic 0–3 m catalog; connecting to
  MOTAR's actual bar pool (whose default differs) remains a P1 prerequisite, not a repaired runtime integration.
- `deadline_s` is a decision-expiry budget. An over-budget segment returns DEADLINE_UNKNOWN. The Python computation
  is not preempted; this API does not promise return within the requested wall-clock deadline or real-time control.
- The copied old R2 driver and two-step noninterference script are historical, not a new r1 gate. The new focused
  temporal control is mandatory evidence for addressing F6; a future full R2 must explicitly include it and a
  separately fixed non-vacuous random-sequence design. No R2/R3 PASS or P1 entry is claimed here.
