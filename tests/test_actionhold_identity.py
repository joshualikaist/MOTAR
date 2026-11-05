"""G7's native identity control compares raw ledger bytes, not only JSON values."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("actionhold_run_identity_test", ROOT / "tools/actionhold_run.py")
G7 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(G7)


class LedgerIdentityTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.reference = Path(self.tmp.name) / "reference"
        self.cell = Path(self.tmp.name) / "cell"
        self.reference.mkdir()
        self.cell.mkdir()

    def test_same_bytes_accept(self):
        payload = b'{"episode_id":1,"outcome":"capture"}\n'
        for directory in (self.reference, self.cell):
            (directory / "episodes.jsonl").write_bytes(payload)
        self.assertTrue(G7.episodes_byte_equal(self.reference, self.cell))

    def test_same_json_different_bytes_reject(self):
        original = b'{"episode_id":1,"outcome":"capture"}\n'
        reformatted = b'{ "outcome": "capture", "episode_id": 1 }\n'
        self.assertEqual(json.loads(original), json.loads(reformatted))
        (self.reference / "episodes.jsonl").write_bytes(original)
        (self.cell / "episodes.jsonl").write_bytes(reformatted)
        self.assertFalse(G7.episodes_byte_equal(self.reference, self.cell))

    def test_missing_ledger_reject(self):
        self.assertFalse(G7.episodes_byte_equal(self.reference, self.cell))
        (self.reference / "episodes.jsonl").write_text('{"episode_id":1}\n')
        self.assertFalse(G7.episodes_byte_equal(self.reference, self.cell))
        (self.reference / "episodes.jsonl").unlink()
        (self.cell / "episodes.jsonl").write_text('{"episode_id":1}\n')
        self.assertFalse(G7.episodes_byte_equal(self.reference, self.cell))


if __name__ == "__main__":
    unittest.main()
