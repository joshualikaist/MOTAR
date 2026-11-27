#!/usr/bin/env python3
"""Write ``index.json`` for the EQ-1 CPU validation evidence folder (file SHA-256, commands, environment).

    python -B tools/audits/eq1_build_index.py [--folder docs/evidence/eq1_validation_2026-10-18]
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eq1_common as common  # noqa: E402

FOLDER = "docs/evidence/eq1_validation_2026-10-18"
TOOLS = {
    "eq1a_geometry.json": "tools/audits/eq1_geometry_reference.py",
    "eq1b_motor.json": "tools/audits/eq1_motor_reference.py",
    "eq1b_scene.json": "tools/audits/eq1_scene_feasibility.py",
    "eq1b_envelope.json": "tools/audits/eq1_flight_envelope.py",
    "eq1c_tracking.json": "tools/audits/eq1_tracking_reference.py",
    "eq1c_information_flow.json": "tools/audits/eq1_information_flow.py",
}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--folder", default=FOLDER)
    args = parser.parse_args(argv)
    folder = common.ROOT / args.folder
    import numpy
    import scipy
    import torch

    files = {p.name: {"bytes": p.stat().st_size, "sha256": common.sha256_file(p)}
             for p in sorted(folder.iterdir()) if p.is_file() and p.name != "index.json"}
    record = {
        "schema": "motar.eq1-evidence-index.v1",
        "package": "EQ-1A/B/C CPU independent validation (validation only)",
        "measurement_source": common.MEASUREMENT_SOURCE,
        "source_at_generation": common.source_binding(sorted(set(TOOLS.values()) | {"tools/audits/eq1_common.py"})),
        "policy_f": {"path": common.POLICY_F, "sha256": common.POLICY_F_SHA256},
        "raw_root_read_only": str(common.OCC_P2),
        "receipt_environment": str(common.RECEIPT),
        "environment": {"python": platform.python_version(), "torch": torch.__version__, "numpy": numpy.__version__,
                        "scipy": scipy.__version__, "cpu_count": os.cpu_count(), "cuda_visible_devices": "''"},
        "commands": {name: "PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' PYTHONPATH=$PWD python -B %s --out %s/%s"
                     % (tool, args.folder, name) for name, tool in TOOLS.items()},
        "derived_files": {"eq1b_scene_episodes.csv.gz": "written by tools/audits/eq1_scene_feasibility.py next to "
                          "eq1b_scene.json (gzip mtime 0)"},
        "reproducibility": "every JSON/CSV record was generated twice from the same code into separate folders; "
                           "all seven files were byte-identical",
        "files": files,
    }
    common.write_record(folder / "index.json", record)
    print(json.dumps({k: v["sha256"][:16] for k, v in files.items()}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
