# S-L1C v2 — independent near-range recall qualification

Authority: the user's 2026-10-18 S-L1C v2 instruction, base `aed6affbd2567feee79e658fbe5f07510b039f36`.
The [JSON](s_l1c_v2.json) governs exact algorithms, populations, margins, selection and stop rules.
This is fixed before candidate comparison; original A/B/C/D sources, preregistrations and evidence
are preserved. RGB-D + LiDAR, Frozen Policy F and its 898-D actor remain the contract.

## Calculation before experiment

A radius-0.20m sphere can hit adjacent vertical rows 10 degrees apart up to
`0.20 / sin(5 deg) = 2.29474265m`. Equal slant sphere returns at symmetric row offsets have
different horizontal ranges, whereas a vertical face has equal horizontal ranges. The difference
is only centimetres; a declared noise bound can make these hypotheses indistinguishable. A
positive geometric test must require a residual advantage exceeding that noise, and otherwise
abstain from adding an association. It cannot claim hardware discrimination from exact simulation.

V2-0 freezes v1. V2-A fits sphere versus vertical-face hypotheses using ranges and the track's own
predicted direction. V2-B adds the proposed azimuth-extent discriminator and abstains when its
angular-size premise fails. V2-C is a CPU-only identity-unconfirmed range-update comparison; it
is ineligible for automatic selection because its recency/expiry contract is not established.
Existing v1 acceptances and their row/range are retained by A/B; recovered observations must also
be genuine first hits in independent evaluation, not merely more association calls.

## Populations and gates

CPU: 2,928 clear-sphere geometries × three radial prediction offsets = 8,784 positive prediction
cases. Negatives: 105 tall, 480 narrow, 384 mixed foreground and seven frozen EQ-1D negatives,
each at three offsets = 2,928 prediction cases. Five noise bounds with nine total perturbation
patterns produce **105,408 matched cases**. A separate 1,152-case angular-prediction-error
diagnostic is reported. Required ranges include 1.16/1.50/2.00/2.28/2.30m and 3/6m controls;
the row-centre/inter-row/elevation/bearing grid is in JSON. Attitude duplicates are not independent
scenes. No-hit opportunities and abstentions remain explicit in the denominators.

All primary strata preserve existing genuine acceptances, pooled near-range recall strictly
improves, and no matched negative/mixed scene adds a false BOX acceptance. Newly recovered range
estimates have fixed error limits. A/B can be selected only when all gates pass; otherwise stop
with **NO-GO before production or GPU work**. No tolerance is relaxed after observing results.

If a candidate qualifies, add a separate `s_l1c_v2` switch inheriting A/B and v1 covariance/clocks.
Never replace v1 or make v2 numerically enable S-L1D automatically. CPU regression checks precede
a fixed-source GPU canary: 144 unique rendered positives and 31 negatives, radial offsets, and
the seven closed-loop arms per static/moving configuration specified in JSON. Original modes
must match frozen qualified digest logs; E-L0 P1/P2 must match on v2. Any GO remains limited to
these declared tests and the simulator's measurement precision.

The sequence is preregistration commit → independent CPU oracle/tests → selection commit →
opt-in implementation/CPU regression → source commit → GPU canary → result commit → independent
Fable 5.1 read-only review. Preserve failed and partial evidence. Publish only ordinary fast-forward
updates of `codex/mt-p1` after history/security/remote checks; main and history rewriting are forbidden.

Afterward write plans only for realistic S-L1D belief error, S-L1B carve retention and camera
centre-range bias. No B/D production changes, plant/governor work, PPO, Planner or H-R test.
The session reset timer is not observable; started 14:52 KST, conservative wrap-up 18:52 KST,
with verification/commit/handoff time reserved rather than starting unfinished long work.
