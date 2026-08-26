import importlib.util
import json
from pathlib import Path
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "check_research_authority", ROOT / "tools" / "check_research_authority.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_frozen_authority_matches_result_summaries():
    receipt = MODULE.verify_authority()
    assert receipt["track_a"]["stage2_authorised"] is False
    assert receipt["track_b"]["long_training_authorized"] is False
    assert receipt["hardware_state"]["real_flights"] == 0


def test_track_b_prohibits_training_and_retuning():
    receipt = MODULE.verify_authority()
    prohibited = set(receipt["track_b"]["prohibited"])
    assert {"ppo_training", "gain_or_margin_retune", "cell_or_grid_rerun"} <= prohibited


def test_evidence_sha_drift_fails_closed():
    receipt = json.loads(MODULE.DEFAULT_RECEIPT.read_text(encoding="utf-8"))
    receipt["evidence"][0]["sha256"] = "0" * 64
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "authority.json"
        path.write_text(json.dumps(receipt), encoding="utf-8")
        try:
            MODULE.verify_authority(path)
        except RuntimeError as error:
            assert "SHA drift" in str(error)
        else:
            raise AssertionError("modified evidence SHA was accepted")
