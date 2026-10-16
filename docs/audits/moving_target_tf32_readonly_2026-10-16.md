# Historical moving-target distance audit — 2026-10-16

**Read-only finding:** old moving-target performance remains a record of its original simulator
contract. It is not evidence that the target satisfied a physically bounded, collision-free motion
contract. The observed-invalid cells, merely exposed cells, static bypasses and new direct-distance
measurements must remain separate. No old result, verdict, preregistration or checkpoint was edited.

Machine-readable inventory: [moving_target_tf32_readonly_2026-10-16.json](moving_target_tf32_readonly_2026-10-16.json).
It contains family × density × model × classification counts, input fingerprints, source commits and
Git-blob hashes. The audit worktree was at `c038ba50193fb6401a473d8eb9ff4be097883e0e`.

## Scope and classification

The source is the original repository's `results/`, including ignored metadata still present on disk.
The scan paired receipts with sibling result/summary JSON, without reading checkpoints, episode
rollouts, archives or long logs. It found **881 receipt-linked records in 104 families**; all 881
result/summary hashes matched their receipts. It hashed 2,006 selected metadata files. Their sorted
path/SHA fingerprint is in the JSON. This is a metadata audit, not a new measurement or full raw-archive
verification. Repeated canaries, VOID families and diagnostic summaries are included; counts below
are **not independent experiments or episodes**. In particular, the unknown category is not a count
of proven moving-target runs. Forty-nine receipts lacked a sibling result and were outside this scan.
The TF32 classification is an additional axis, not a replacement verdict. Existing VOID, withdrawn,
failed or otherwise excluded records retain their original exclusion; even a nonexposed static
record is not thereby cleared for paper reuse.

| Classification | Records | Meaning and reuse limit |
| --- | ---: | --- |
| `OBSERVED_INVALID` | 12 | A recorded swept proxy or wall violation occurred in that cell. This is not proof of a PhysX contact or that every run at that density fails. |
| `TF32_EXPOSED_VALIDITY_UNVERIFIED` | 372 | Moving point-centre `cdist` screening is eligible for matmul; runtime TF32/kernel choice and GPU motion validity are not attested. Exposure is not a measured failure. |
| `NOT_EXPOSED_THIS_DEFECT` | 187 | 181 static-target records bypass the moving executor; six source-checked physical records use an AABB local screen. Spawn/goal queries and unrelated defects are not exonerated. |
| `DIRECT_LINEAGE_VALIDATED` | 72 | New direct-distance measurements passed their recorded G1/G2, swept-clearance and wall gates. The finite recorded contract is validated, not universal safety. |
| `INSUFFICIENT_PROVENANCE` | 238 | The relevant model/backend is absent from this metadata scope. Do not infer it from a filename, Policy F, or current defaults. |

Additional physical-family aggregate summaries are inventoried separately in the JSON and are not
added to those counts. An aggregate's physical-looking name alone is insufficient provenance.

## Exposure is not restricted to multiples of four

Historical `bounded_drone_target_step` uses `torch.cdist(pos, bars_xy)` with 73 candidate points.
Historical `steer_target_step`, used by H, also uses `torch.cdist` with ten steering candidates.
The default Euclidean `cdist` selects its matrix-multiplication approach when **either point axis
exceeds 25**. Thus the bounded call is eligible even with at most 25 nonempty bar slots; every study
density 70/115/160/205 is eligible. With zero bars the query is bypassed. H is not exempt simply
because it belongs to the legacy lineage.

The multiple-of-four pattern was observed in one RTX 3070 / torch 2.4.1 CUDA shape scan. It describes
that kernel selection, not a safe-density whitelist. Even that scan recorded nonzero errors at other
densities, and TF32-off `cdist` retains the squared-norm cancellation issue. The direct variant uses
the norm of the coordinate difference instead. No global policy matmul setting needs to change.

Thirty-five receipt-named Git versions of `target_motion.py` were read and hashed; all retain the
historical `cdist` sites. New versions add a direct opt-in alongside them. A Git blob is not a
byte-level runtime attestation for a dirty receipt. Representative old commits `ae286e5` and
`b1c0bc6` have the same motion-source SHA
`a7179fb9fd6b9e19ae9d44f7eba3c7b5e3f758ff0568c8e16220c6d5c8dc9c5e`.

## Three-seed and seven-seed historical campaigns

| Campaign | Main cells | H | E0 | E1 | E2 | Classification |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `target_motion_e0_e2_evaluation_2026-09-18` | 48 | 12 | 12 | 12 | 12 | H/E1/E2: 36 exposed/unverified; E0: 12 moving-executor bypasses |
| `target_motion_replication_7seed` | 112 | 28 | 28 | 28 | 28 | H/E1/E2: 84 exposed/unverified; E0: 28 moving-executor bypasses |

Each campaign covers 70/115/160/205 bars. The first records commit `ae286e5` and a dirty source tree;
the replication records measurement commit `b1c0bc6`, clean preflight, torch 2.4.1+cu121 and RTX 3070.
These metadata improve provenance but do **not** record the old calls' TF32 flags, dtype, ordered
obstacle tensor or strides. Candidate counts are source-derived; batch and active-bar counts come
from conditions. Current runner flags must not be retroactively described as measured old flags.

The old `analysis_report.json` stage 2 labels its CPU executor audit PASS while explicitly saying
target-side penetration/violation counts were not remeasured per GPU cell. That CPU PASS and zero
reflection counters cannot validate GPU distance screening. Conversely, the new 4190-seed
counterexamples are not direct evidence that a specific old 4101–4110 cell penetrated an obstacle.
Those historical GPU trajectories remain **unverified**, not newly assigned a failure verdict.

E0 alone bypasses this moving screen, but an **E0 minus H contrast still inherits the exposed H
reference**. Do not describe the whole contrast as unaffected merely because its E0 arm is static.

Keep historical capture contrasts and their estimator/seed limitations as conditional simulator
performance. Do not use them to establish a real-drone target model, safe trajectory, acceleration
envelope or generalization to the new direct-distance lineage. No performance figures were
recomputed, pooled or substituted in this audit.
Receipt/hash traceability and moving-UAV motion validity are separate axes. Matching stored bytes
does not itself establish bitwise trajectory reproducibility across machines or numerical backends.

## What is actually observed invalid

Both original and rerun KROC2027 canaries recorded swept-clearance failures in A3/A4 × E1/E2 at
160 bars, seed 4190: four cells in each campaign. The packet run repeats those four cases, making
12 receipt-linked invalid records, not 12 independent discoveries.

The historical packet set contains 606 capped violation packets. The task called 604 of them safe.
TF32-on batch-two reconstruction matched all 606 flags and 598 executed steps; eight remain
`UNRESOLVED_INPUTS`. The old fallback-only hypothesis is rejected. Local-bar packets omit full
ordered geometry, original turn sign and original batch/layout information, so their CPU replay
cannot establish exact old CUDA trajectory identity. These conclusions are read from hashed
`kroc2027_moving_target_packets_v1/{tf32_replay,packets_report,RAW_MANIFEST}.json`; no replay ran here.

## New direct lineage: finite evidence, separate identity

| Run | Cells | Lineage / seeds | Recorded validity |
| --- | ---: | --- | --- |
| `MT-P2-C01` | 16 | E1/E2 direct × A3/A4 × four densities; seed 4190 | CANARY_PASS; G1/G2/swept/wall violations 0; minimum swept surface clearance 0.2254725 m |
| `KROC27-A-M02` | 56 | E2 direct × A3/A4 × four densities × seeds 4201–4207 | COMPLETE; G1/G2/swept/wall violations 0; minimum swept surface clearance 0.2226601 m |

Source commits are `4bba8836d3bab86b87351a47b70e674d6e86cfaa` and
`ea677289cecf637ebe8d3d473d29cd0b20cc47d9`. Their motion-source SHA is
`d8e243f375fd9dc8944a3ae2731900372045f538b051d07834bdcb054939b46f` and model ID is
`bounded_planar_drone_v1_rollout_direct_l2_v1`. The new summaries explicitly record TF32=True and
matmul precision=high: the target screen changes, not the neural policy's global precision contract.
This audit checked 48 canary and 168 main-run receipt/result/KROC-summary files against the respective
raw manifests, with no mismatch. It did not reread every episode or all raw-manifest members.

Main-run maximum finite-difference target speed is 1.5000035763 m/s, within its preregistered
float32 G2 tolerance, rather than a claim of an exact real-number 1.5 m/s bound. No-violation runs
write no full violation packets, so runtime tensor dtype is not separately attested by such a packet.
The bounded fallback remains; a finite canary does not provide a terminal-braking or global
collision-free guarantee. These results do not repair old receipts or validate H/E1/E2 retroactively.

## Physical/AABB boundary and next use

For the six cells in `navrl_corrected_nonoverlap_physical_off_heldout_seed313`, the archived
`navrl_task.py` and `target_motion.py` source snapshots were found in the referenced braking-route
worktree and matched the recorded source-manifest hashes. The physical branch supplies obstacle
half extents plus target support; its local screen takes direct rectangle distances instead of the
point-centre `cdist` branch. This establishes the narrow `NOT_EXPOSED_THIS_DEFECT` classification.
Separate target-spawn/goal `cdist` sites and pre-existing physical validity findings remain open.

For future use: keep all five classifications in evidence tables; preregister any additional
historical remeasurement separately; record backend/model ID, full call shape/dtype/layout and
runtime precision; use new direct motion only within its assessed contract. An unresolved old
trajectory is never made valid by replacing its bytes or by relabelling it as direct.

Only this appendix and its JSON were created. No GPU query, GPU calculation, simulator, PPO,
checkpoint deserialization, commit, merge, push, stash operation or historical-result edit occurred.
