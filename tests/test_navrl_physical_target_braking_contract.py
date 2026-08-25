import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
import sys
sys.path.insert(0, str(TOOLS))

import probe_navrl_physical_target_braking as probe
import run_navrl_physical_target_braking_v2_fresh as launcher
import verify_navrl_physical_target_braking as verifier


def fake_cell(speed):
    rows = []
    for env_id in range(probe.REGISTERED_ENVS):
        rows.append({
            "env_id": env_id,
            "requested_speed_mps": speed,
            "measured_initial_speed_mps": speed,
            "stop_time_s": 0.20 + env_id * 0.001,
            "stop_distance_m": 0.08 + env_id * 0.0001,
            "effective_deceleration_mps2": speed * speed / (2.0 * (0.08 + env_id * 0.0001)),
            "contact": False,
            "invalid_obb": False,
            "motor_saturation_fraction": 0.01,
            "max_tilt_deg": 12.0,
        })
    return {
        "schema": probe.SCHEMA,
        "cell": {"speed_mps": speed, "envs": probe.REGISTERED_ENVS, "seed": 827},
        "contract": probe.FROZEN_CONTRACT,
        "instantiated": {"sim_name": "base_sim", "envs": probe.REGISTERED_ENVS},
        "raw_samples": rows,
        "physics_samples": [],
        "provenance": {
            "python_executable": "synthetic",
            "python_version": "3.8",
            "torch_version": "synthetic",
            "torch_cuda_version": "synthetic",
            "cuda_device": "synthetic",
            "nvidia_smi_path": "synthetic",
            "nvidia_smi_sha256": "0" * 64,
            "nvidia_smi_identity": "synthetic",
            "gpu_driver_version": "synthetic",
            "ninja_path": "synthetic",
            "ninja_sha256": "1" * 64,
            "ninja_version": "synthetic",
            "imported_modules": {"synthetic": {"path": "synthetic", "sha256": "2" * 64}},
            "tool_hashes": {path: "3" * 64 for path in probe.TOOL_SOURCE_PATHS},
        },
    }


class PhysicalTargetBrakingContractTest(unittest.TestCase):
    def test_preflight_is_cpu_safe_and_has_exact_grid(self):
        completed = subprocess.run(
            [sys.executable, str(TOOLS / "probe_navrl_physical_target_braking.py"), "--preflight", "--output", "/tmp/unused"],
            cwd=str(ROOT), text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout)
        payload = json.loads(completed.stdout)
        self.assertEqual(payload["contract"]["envs"], 32)
        self.assertEqual(payload["contract"]["max_bars"], 300)
        self.assertEqual(payload["contract"]["physics_substeps"], 10)
        self.assertEqual(payload["contract"]["rl_step_dt_s"], 0.1)

    def test_raw_validator_recomputes_speed_lookup(self):
        cells = [fake_cell(speed) for speed in probe.REGISTERED_SPEEDS]
        summary = verifier.summarize_cells(cells)
        self.assertEqual(summary["speeds_mps"], list(probe.REGISTERED_SPEEDS))
        lookup = summary["measured_speed_to_p95_lookup"]
        self.assertEqual(sorted(lookup), ["0.6", "0.9", "1.2", "1.5"])
        expected = verifier.quantile_stats([row["stop_distance_m"] for row in cells[0]["raw_samples"]])["p95"]
        self.assertEqual(lookup["0.6"]["p95_stop_distance_m"], expected)
        cells[0]["raw_samples"][0]["contact"] = True
        with self.assertRaises(ValueError):
            verifier.validate_cell(cells[0])

    def test_summary_and_receipt_are_immutable_and_relative(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "braking"
            output.mkdir()
            (output / "cells").mkdir()
            cells = []
            for speed in probe.REGISTERED_SPEEDS:
                path = output / "cells" / ("speed_%s.json" % format(speed, ".1f").replace(".", "p"))
                path.write_bytes(probe.canonical_json_bytes(fake_cell(speed)))
                cell_payload = json.loads(path.read_text(encoding="utf-8"))
                cells.append({
                    "path": str(path.relative_to(output)),
                    "speed_mps": speed,
                    "sha256": probe.sha256_file(path),
                    "provenance_sha256": probe.sha256_bytes(probe.canonical_json_bytes(cell_payload["provenance"])),
                })
            manifest = probe.source_manifest(ROOT, launcher.TOOL_PATHS)
            (output / "source_manifest.json").write_bytes(probe.canonical_json_bytes(manifest))
            summary = verifier.summarize_cells([fake_cell(speed) for speed in probe.REGISTERED_SPEEDS])
            (output / "summary.json").write_bytes(probe.canonical_json_bytes(summary))
            receipt = {
                "schema": probe.RECEIPT_SCHEMA,
                "probe_schema": probe.SCHEMA,
                "contract": probe.FROZEN_CONTRACT,
                "git_head": probe.git_head(ROOT),
                "source_manifest": "source_manifest.json",
                "source_manifest_sha256": probe.sha256_file(output / "source_manifest.json"),
                "cells": cells,
                "summary_path": "summary.json",
                "summary_sha256": probe.sha256_file(output / "summary.json"),
                "summary": summary,
                "decel_p05_mps2": min(row["p05_effective_deceleration_mps2"] for row in summary["measured_speed_to_p95_lookup"].values()),
                "stop_time_p95_s": max(row["p95_stop_time_s"] for row in summary["measured_speed_to_p95_lookup"].values()),
                "measured_speed_to_p95_lookup": summary["measured_speed_to_p95_lookup"],
                "fresh_only": True,
                "process_isolation": "one fresh Isaac Gym child per registered speed",
            }
            (output / "receipt.json").write_bytes(probe.canonical_json_bytes(receipt))
            (output / "complete.marker").write_text("COMPLETE\n", encoding="utf-8")
            verified = verifier.verify_receipt(output, ROOT)
            self.assertTrue(verified["verified"])
            self.assertEqual(receipt["cells"][0]["path"], "cells/speed_0p6.json")
            # A producer-side summary cannot hide raw tampering.
            mutated = json.loads((output / "cells/speed_0p6.json").read_text(encoding="utf-8"))
            mutated["raw_samples"][0]["stop_distance_m"] = 99.0
            (output / "cells/speed_0p6.json").write_bytes(probe.canonical_json_bytes(mutated))
            with self.assertRaises(ValueError):
                verifier.verify_receipt(output, ROOT)

    def test_fresh_launcher_rejects_continuation_arguments(self):
        completed = subprocess.run(
            [sys.executable, str(TOOLS / "run_navrl_physical_target_braking_v2_fresh.py"), "--output", "/tmp/not-created", "--checkpoint", "/tmp/x"],
            cwd=str(ROOT), text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
        )
        self.assertEqual(completed.returncode, 1)
        self.assertIn("fresh-only", completed.stdout)

    def test_core_files_are_not_modified_by_this_lineage(self):
        changed = subprocess.run(
            ["git", "diff", "--name-only", "HEAD^", "HEAD", "--", *probe.CORE_PATHS],
            cwd=str(ROOT), text=True, stdout=subprocess.PIPE, check=True,
        ).stdout.splitlines()
        self.assertEqual(changed, [])


if __name__ == "__main__":
    unittest.main()
