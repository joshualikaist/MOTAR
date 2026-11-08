"""After the CAM-K2 v2 chain ends: the compact committed record of the two training seeds.

  nice -n 10 python3 -B ~/motar_tmp/camk2v2_20261108/make_camk2v2_record.py --out <dir>

Writes into <dir>, refusing to overwrite: train_report_v2_s{1,2}.json and the checkpoint sidecar manifests
manifest_v2_s{1,2}.json (byte copies), and training_curve_v2.json (per-100-epoch training-rollout statistics parsed
from each seed's launcher.log with CAM-K1's committed parser; CAM-K1's last window is included as a reference). The
rollout numbers are under exploration noise and are not an evaluation (that is CAM-K2-E).
"""
import argparse
import hashlib
import importlib.util
import json
import pathlib
import shutil

U = pathlib.Path.home() / "motar_tmp" / "camk2v2_20261108"
TREE = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly")
K1REC = TREE / "docs/evidence/camonly_2026-11-04/camk1/make_camk1_record.py"
K1CURVE = TREE / "docs/evidence/camonly_2026-11-04/camk1/training_curve.json"


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    out = pathlib.Path(ap.parse_args().out)
    out.mkdir(parents=True, exist_ok=True)
    spec = importlib.util.spec_from_file_location("make_camk1_record", K1REC)
    k1 = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(k1)
    names = ["train_report_v2_s1.json", "train_report_v2_s2.json", "training_curve_v2.json"]
    for n in names:
        if (out / n).exists():
            raise SystemExit("refusing to overwrite %s" % (out / n))
    curve = {"written_by": str(pathlib.Path(__file__)), "script_sha256": sha(__file__), "parser": str(K1REC),
             "parser_sha256": sha(K1REC), "window_epochs": k1.WINDOW,
             "note": "training-rollout statistics printed by the runner, under exploration noise; not an evaluation",
             "seeds": {}}
    for seed in (1, 2):
        d = U / ("train_s%d" % seed)
        rep = d / "train_report.json"
        if rep.is_file():
            shutil.copyfile(rep, out / ("train_report_v2_s%d.json" % seed))
            r = json.loads(rep.read_text())
            man = pathlib.Path(r.get("checkpoint", {}).get("copy", "")).parent
            for m in sorted(man.glob("*manifest*.json")) if man.is_dir() else []:
                shutil.copyfile(m, out / ("manifest_v2_s%d.json" % seed))
        log = d / "launcher.log"
        if log.is_file():
            rows = k1.parse(log)
            curve["seeds"][str(seed)] = {"launcher_log": str(log), "launcher_log_sha256": sha(log),
                                         "train_report_sha256": sha(rep) if rep.is_file() else None,
                                         "blocks": len(rows), "first_epoch": rows[0]["epoch"] if rows else None,
                                         "last_epoch": rows[-1]["epoch"] if rows else None,
                                         "windows": k1.windows(rows, 28001)}
    k1c = json.loads(K1CURVE.read_text())
    curve["reference_camk1_last_window"] = {s: v["windows"][-1] for s, v in k1c["seeds"].items()}
    (out / "training_curve_v2.json").write_text(json.dumps(curve, indent=1, sort_keys=True) + "\n")
    for s, c in curve["seeds"].items():
        w = c["windows"]
        print("seed", s, "blocks", c["blocks"], "first", w[0] if w else None, "last", w[-1] if w else None)
    print("camk1 ref", curve["reference_camk1_last_window"])


if __name__ == "__main__":
    main()
