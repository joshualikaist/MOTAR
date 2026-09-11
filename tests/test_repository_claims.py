"""Guard current prose against resurrection of withdrawn claims; no experiment runs."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


class RepositoryClaimsTest(unittest.TestCase):
    def test_current_verification_has_completed_and_blocked_outcomes(self):
        page = read("VERIFICATION.md").split("## 역사 기록:")[0]
        for word in ("P3–P10", "S4", "R-C", "INCONCLUSIVE", "WITHDRAWN",
                     "ATTITUDE_NOT_RELIABLE", "BLOCKED", "FAIL 유지"):
            self.assertIn(word, page)

    def test_completed_runs_are_not_current_authority(self):
        page = read("OPERATIONS.md")
        self.assertIn("완료된 실행 기록 — P10", page)
        self.assertNotIn("## 허가된 실행 — P10", page)
        self.assertIn("소비된 실행 계약", page)

    def test_current_r4_prose_does_not_blame_criterion(self):
        for name in ("README.md", "docs/status/index.html"):
            page = read(name)
            for withdrawn in ("What failed was the criterion:", "C가 실패한 진짜 이유",
                              "<strong>기준 설계 결함</strong>"):
                self.assertNotIn(withdrawn, page)

    def test_c3_summary_is_withdrawn(self):
        page = read("docs/plans/confirmation_phase_plan_2026-09-06.md")
        summary = page.split("## 0. 한 문장")[1].split("## 1.")[0]
        self.assertIn("철회", summary)
        self.assertNotIn("정책이 무엇과 함께 학습됐는지가 정하며", summary)

    def test_e3p_latest_test_is_not_reported_unrun(self):
        page = read("docs/plans/eth_ds5_e3_2026-09-10.md")
        self.assertIn("ATTITUDE_NOT_RELIABLE", page)
        self.assertIn("third reliability test has already run and failed", page)
        self.assertNotIn("where the six\narms are resolved", read(
            "results/eth_ds5_e3p_reliability_2026-09-10/README.md"))

    def test_smoothness_is_not_accuracy(self):
        page = read("results/eth_ds5_e3s_2026-09-10/README.md")
        self.assertIn("trajectory-smoothness residual", page)
        self.assertIn("not a bound on measurement", page)
        self.assertIn("no temporal embargo", page)

    def test_test_access_ledger_is_experiment_specific(self):
        page = read("VERIFICATION.md").split("## 역사 기록:")[0]
        self.assertIn("Det-Fly 020", page)
        self.assertIn("P4/P5", page)
        self.assertIn("never-observed holdout 아님", page)

    def test_memory_measure_has_scope(self):
        page = read("docs/status/index.html")
        self.assertIn("183.9 MiB Torch peak allocated", page)
        self.assertIn("전체 프로세스 VRAM은 아닙니다", page)


if __name__ == "__main__":
    unittest.main()
