"""Unit tests for the CPU shadow benchmark metrics (declared synthetic labels; no simulator outcomes)."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.q02.benchmark_metrics import LABEL_SEMANTICS, Row, summarize


def row(s, k, decision, warranted, policy="H1", in_loss=True, reasons=(), l2=1):
    return Row(s, policy, k, 0.1 * k, decision, reasons, "OCCLUDED", "NOT_APPLICABLE", True, False, None, l2, in_loss, warranted, 0.01 * (k + 1))


class MetricTests(unittest.TestCase):
    def test_trigger_latency_missed_and_unnecessary(self):
        rows = [row("A", k, "PROPOSE_REACQUIRE_SHADOW" if k == 4 else "ABSTAIN", k >= 2) for k in range(6)]
        rows += [row("B", k, "ABSTAIN", k >= 1) for k in range(4)]                       # warranted, never proposed
        rows += [row("C", k, "PROPOSE_REACQUIRE_SHADOW" if k == 1 else "CONTINUE_F", False) for k in range(3)]
        s = summarize(rows)["H1"]
        self.assertEqual(s["trigger_latency_s"], [0.2]); self.assertEqual(s["missed_trigger_scenarios"], ["B"])
        self.assertEqual((s["unnecessary_triggers"], s["unnecessary_trigger_scenarios"]), (1, ["C"]))
        self.assertEqual(s["trigger_count"], 2); self.assertEqual(s["warranted_scenarios"], 2)
        self.assertEqual(s["label_semantics"], LABEL_SEMANTICS)

    def test_policies_are_never_pooled_and_runtime_quantiles(self):
        rows = [row("A", k, "ABSTAIN", False, policy=p, reasons=("INPUT_INVALID",) if k == 0 else ()) for p in ("F0", "H3") for k in range(5)]
        s = summarize(rows)
        self.assertEqual(set(s), {"F0", "H3"}); self.assertEqual(s["F0"]["decisions"], 5)
        self.assertEqual(s["F0"]["invalid_input_rejections"], 1); self.assertAlmostEqual(s["F0"]["runtime_s"]["median"], 0.03)


if __name__ == "__main__":
    unittest.main()
