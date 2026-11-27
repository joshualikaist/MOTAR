# C-v2 execution clarification 1 — original negative sensor replay

Before any C-v2 GPU run, the seven original EQ-1D negative fixtures are fixed to their preserved
GPU-rendered packets (`EQ1D-R1/d2_scenes/d2_renders.npz`, unique level pass), rather than an
approximate reconstruction using a newly randomized bar asset inventory. These exact packets
are replayed through the actual GPU association/tracker alongside the 24 newly rendered negative
fixtures. The new files record the packet source, original layout, raw manifest and first-hit
labels separately. They do not overwrite any original raw or inject labels into association.

The original [plan](s_l1c_v2.json) remains byte-identical. This clarifies execution of its existing
seven negatives; counts (31 negative/144 positive), gates, margins, candidate, seed and error
limits do not change. Report replay and newly rendered negative results separately. Do not claim
these seven sensor packets were newly rendered by C-v2. CPU selection is already recorded, but no
GPU outcome has been inspected and no statistical tolerance is changed.
