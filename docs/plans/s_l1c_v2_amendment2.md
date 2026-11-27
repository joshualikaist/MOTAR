# C-v2 execution addendum 2 — task-level actor GT firewall

Fixed before any C-v2 GPU execution. In addition to the original 22 processes, run one
`firewall_v2` process: four environments, seed 4401, 320 control steps on the selected v2 mode.
Reuse the unchanged EQ-1D D3 packet-replay intervention and its positive control. At scheduled
steps 50/150/300 and the first two reset boundaries, replay the same sensors and pre-perception
state while changing evaluator/privileged target state, semantic buffers and critic fields.
All/all-permuted/subset mutations must preserve every 898-D actor row and deterministic action;
at least one reset boundary must be observed. Re-rendering after moving the target is a separate
positive control and must change observations when the target is in view. RMS remains frozen.

This supplies task-level evidence beyond the association-only firewall. The independent analysis
recomputes the row/action equality from the saved branch arrays and verifies the recorded RMS and
controls. Source/clock/fixture gates are not relaxed. Original plan and amendment 1 remain intact;
144 positive/31 negative scenes, seven closed-loop variants, 900s per-process and 7200s total
caps are unchanged. The extra process brings the matrix to **23**. No GPU result has been inspected.
