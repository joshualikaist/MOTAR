"""Q02-A: CPU-only sensor-evidence layer for evidence-aware reacquisition research.

Consumes copied sensor packets (Q01 SensorPacket or saved arrays); never holds a task,
simulator, renderer, policy or ground-truth handle, never re-calls a sensor getter and
never draws from a random generator. Outputs are explainable evidence records and
hypotheses, not calibrated probabilities and not actions. See
docs/research/q02_evidence_aware_reacquisition/Q02A_SPEC.md.
"""
