# E-L0 preregistration — amendment 1 (attachment point), before any official E-L0 run

Date: 2026-10-18, before run `EL0-R1`. The JSON contract `e_l0_evaluation_lineage_v1.json` (commit
`37e6b19`) stays byte-identical. This amendment supersedes only its `switch.attachment` text.

**Change.** The plan said `kroc_eval_adapter.KrocRecorder` builds the E-L0 recorder, which would need
hooks added to `aerial_gym/task/navrl_task/kroc_eval_adapter.py`. That file is frozen: the existing
OCC-P2 contract test `test_occlusion_confirmatory.StatisticalTest.test_preexisting_cpu_event_contract_is_unchanged`
requires it to stay byte-identical to the OCC G0 commit. The adapter edits failed that test in
the pre-commit CPU suite, were reverted unpublished, and the test was not changed.

**Instead.** `motar_eval.e_l0.attach(adapter)` wraps the hook methods of the running
`KrocRecorder` *instance* when `NAVRL_KROC2027_E_L0=1`. The evaluation launcher attaches before the
first reset. Every wrapped hook still calls the original with the same arguments and returns its
value. E-L0 runs first in `after_physics` and `on_episode_end` (before the adapter advances its
step and episode counters) and after the original elsewhere. The recorder, the physics-step
callback chain and the call-through association wrapper are otherwise unchanged. No source file
of the simulator, task, perception, adapter or controller is modified.

**Unchanged.** Switch variable and values, records, outputs, the V1/V2/V3 fixture matrix, every
gate, the stop rules and the resource limits. The only consequence is that E-L0 is attached by an
evaluation launcher rather than by the task's own adapter construction. Making it available to
the standard OCC eval scripts would need a separately approved entry point.
