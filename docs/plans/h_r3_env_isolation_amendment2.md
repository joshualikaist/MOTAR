# Amendment 2 — nested completion paths and fresh ISO-Q02

ISO-Q01 source d5c91cec21c7e35917409b612957ecfb30f74a84 ran only legacy_normal (40 steps, 52.0201 process seconds). No natural reset occurred at step2. Supervisor then stopped before launching legacy_forced: inherited flat-directory completion validator rejected valid frames/*.npz paths. All ISO-Q01 raw/log/manifest remain preserved. No mechanics outcome is inferred from this incomplete attempt.

Only the new evaluation supervisor completion checker is corrected: allow nested relative paths confined to the output root, reject absolute/parent traversal/symlink/duplicate paths, verify each hash/size and complete file inventory. Added focused nested-path tamper/traversal regression. Geometry wrapper, RNG, original source, launcher, analysis criteria and all experimental bounds remain unchanged.

Fresh RunID ISO-Q02 uses all five original processes and gates with 600s per process/3600s total. CLI accepts only ISO-Q02 for this amended source; ISO-Q01 is never resumed or overwritten. Source commit and clean-tree preflight precede launch. This is correction of diagnostic bookkeeping, not a new pilot or threshold change.
