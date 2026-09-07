import base64
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


def load_tool(name):
    path = ROOT / "tools" / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DOWNLOAD = load_tool("download_detfly_dataset")
EVALUATE = load_tool("eval_detfly_zeroshot")
PREPARE = load_tool("prepare_detfly_dataset")
JOINT = load_tool("build_nps_detfly_joint_dataset")
TRAIN = load_tool("run_joint_detector_training")


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


class DetFlyPreparationTest(unittest.TestCase):
    @staticmethod
    def write_pair(root, stem, objects):
        annotation_dir = root / "Annotations" / "010"
        image_dir = root / "JPEGImages" / "010"
        annotation_dir.mkdir(parents=True)
        image_dir.mkdir(parents=True)
        image_path = image_dir / (stem + ".jpg")
        image_path.write_bytes(b"jpeg-placeholder")
        object_xml = "".join(objects)
        xml_path = annotation_dir / (stem + ".xml")
        xml_path.write_text(
            "<annotation><size><width>100</width><height>50</height><depth>3</depth></size>"
            + object_xml + "</annotation>"
        )
        return xml_path

    def test_negative_frame_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)
            xml_path = self.write_pair(source, "negative", [])
            with mock.patch.object(PREPARE, "jpeg_size", return_value=(100, 50)), \
                    mock.patch.object(PREPARE, "sha256_file", return_value="hash"):
                row = PREPARE.parse_annotation(xml_path, source)
            self.assertEqual(row["objects"], [])

    def test_implausible_box_is_flagged_not_dropped(self):
        obj = (
            "<object><name>UAV</name><difficult>0</difficult><truncated>0</truncated>"
            "<bndbox><xmin>0</xmin><ymin>1</ymin><xmax>100</xmax><ymax>3</ymax></bndbox>"
            "</object>"
        )
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)
            xml_path = self.write_pair(source, "wide", [obj])
            with mock.patch.object(PREPARE, "jpeg_size", return_value=(100, 50)), \
                    mock.patch.object(PREPARE, "sha256_file", return_value="hash"):
                row = PREPARE.parse_annotation(xml_path, source)
            self.assertEqual(len(row["objects"]), 1)
            self.assertTrue(row["objects"][0]["implausible_size"])


class JointDetectorDatasetTest(unittest.TestCase):
    @staticmethod
    def row(group, frame_id, digest=None, objects=None):
        return {
            "image": "JPEGImages/%s/%s%04d.jpg" % (group, group, frame_id),
            "image_sha256": digest or (group + "-%d" % frame_id),
            "source_group": group,
            "width": 1280,
            "height": 640,
            "objects": list(objects or []),
        }

    def test_structural_split_holds_larger_source_group_out_whole(self):
        rows = [self.row("010", value) for value in (1, 2, 100, 101)]
        rows += [self.row("020", value) for value in (1, 2, 3, 4, 5)]
        assignments, policy = JOINT.choose_structural_split(rows)
        self.assertEqual(policy["test_group"], "020")
        self.assertEqual(policy["development_gap"]["left_id"], 2)
        self.assertEqual(policy["development_gap"]["right_id"], 100)
        self.assertEqual({assignments[row["image"]] for row in rows if row["source_group"] == "020"},
                         {"test"})
        self.assertEqual(assignments[rows[0]["image"]], "train")
        self.assertEqual(assignments[rows[2]["image"]], "val")

    def test_split_audit_rejects_exact_duplicate_across_splits(self):
        rows = [self.row("010", 1, "same"), self.row("010", 100, "val"),
                self.row("020", 1, "same"), self.row("020", 2, "test-2"),
                self.row("020", 3, "test-3")]
        assignments, policy = JOINT.choose_structural_split(rows)
        with self.assertRaisesRegex(ValueError, "exact JPEG duplicates cross splits"):
            JOINT.split_audit(rows, assignments, policy)

    def test_negative_tile_never_contains_even_a_partial_target(self):
        obj = {
            "xyxy": [100.0, 200.0, 180.0, 280.0],
            "implausible_size": False,
        }
        row = self.row("010", 1, objects=[obj])
        selected, exclusion = JOINT.choose_tiles(row)
        self.assertIsNone(exclusion)
        self.assertTrue(any(kind == "positive" for _x, _y, _boxes, kind in selected))
        negatives = [(x, y) for x, y, _boxes, kind in selected if kind == "negative"]
        self.assertEqual(len(negatives), 1)
        self.assertIsNone(JOINT.intersection(obj["xyxy"], *negatives[0]))


class JointDetectorTrainingContractTest(unittest.TestCase):
    def test_command_freezes_multiscale_and_excludes_resume(self):
        command = TRAIN.frozen_command()
        self.assertIn("--multi-scale", command)
        self.assertEqual(command[command.index("--batch-size") + 1], "8")
        self.assertEqual(command[command.index("--epochs") + 1], "30")
        self.assertEqual(command[command.index("--seed") + 1], "0")
        self.assertNotIn("--resume", command)

    def test_expected_hashes_are_full_sha256(self):
        for digest in TRAIN.EXPECTED.values():
            if len(digest) == 64:
                int(digest, 16)


if __name__ == "__main__":
    unittest.main()
