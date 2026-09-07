import base64
import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def load_tool(name):
    path = ROOT / "tools" / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DOWNLOAD = load_tool("download_detfly_dataset")
EVALUATE = load_tool("eval_detfly_zeroshot")


class DetFlyDownloadTest(unittest.TestCase):
    def test_stable_manifest_excludes_signed_urls_and_item_ids(self):
        files = [{
            "relative": "JPEGImages/010/0100001.jpg",
            "size": 42,
            "quick_xor_hash": "stable",
            "item_id": "remote-id",
            "download_url": "https://example.invalid/?tempauth=secret",
            "share_url": "https://example.invalid/share",
        }]
        rows, digest = DOWNLOAD.stable_manifest(files)
        self.assertEqual(rows, [{
            "relative": "JPEGImages/010/0100001.jpg",
            "size": 42,
            "quick_xor_hash": "stable",
        }])
        self.assertEqual(len(digest), 64)
        self.assertNotIn("secret", json.dumps(rows))

    def test_embedded_expiry_reads_sharepoint_style_token(self):
        payload = base64.urlsafe_b64encode(json.dumps({"exp": "12345"}).encode()).decode().rstrip("=")
        self.assertEqual(DOWNLOAD.embedded_expiry("v1." + payload + ".signature"), 12345)


class DetFlyEvaluationTest(unittest.TestCase):
    def test_native_scale_origins_cover_flush_edges(self):
        self.assertEqual(
            EVALUATE.tile_origins(3840, 640, 128),
            [0, 512, 1024, 1536, 2048, 2560, 3072, 3200],
        )
        self.assertEqual(EVALUATE.tile_origins(2160, 640, 128), [0, 512, 1024, 1520])

    def test_greedy_match_and_ranked_metrics(self):
        ground_truth = [[0, 0, 10, 10]]
        predictions = [[0, 0, 10, 10, 0.9], [50, 50, 60, 60, 0.8]]
        matches = EVALUATE.greedy_matches(predictions, ground_truth, 0.5)
        metric = EVALUATE.MetricSlice()
        metric.add(matches, [0])
        report = metric.report(0.25)
        self.assertEqual(report["true_positives"], 1)
        self.assertEqual(report["false_positives"], 1)
        self.assertEqual(report["false_negatives"], 0)
        self.assertAlmostEqual(report["average_precision"], 1.0)

    def test_p4_schema_freezes_top_k_and_embedding_dimension(self):
        schema_path = ROOT / "docs" / "specs" / "motar_perception_candidates_v1.schema.json"
        schema = json.loads(schema_path.read_text())
        candidates = schema["properties"]["candidates"]
        self.assertEqual(candidates["maxItems"], 5)
        embedding = candidates["items"]["properties"]["appearance_64d"]
        self.assertEqual(embedding["minItems"], 64)
        self.assertEqual(embedding["maxItems"], 64)


if __name__ == "__main__":
    unittest.main()
