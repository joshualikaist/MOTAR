"""After the CAM-K1 chain ends: build the compact committed record from ~/motar_tmp/camk1_20261106/train_s{1,2}/.

  nice -n 10 python3 -B ~/motar_tmp/camk1_20261106/make_camk1_record.py --out <dir>

Writes, refusing to overwrite, into <dir> (staging first; then <worktree>/docs/evidence/camonly_2026-11-04/camk1):
  train_report_s1.json, train_report_s2.json   byte copies of each seed's train_report.json (when present)
  training_curve.json                          per-100-epoch training-rollout statistics parsed from launcher.log,
                                               with GR-1 ctrl seed 1 (camera + LiDAR, same contract) as a reference
The training-curve numbers are rollout statistics under PPO's exploration noise, as printed by the runner. They are
NOT an evaluation and support no capture, crash or transfer claim (prereg: the evaluation is CAM-K1-E).
"""
import argparse
import hashlib
import json
import pathlib
import re
import shutil

U = pathlib.Path.home() / "motar_tmp" / "camk1_20261106"
GR1_CTRL_LOG = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/"
                            "gain_randomization_gr1_v1/train_ctrl/launcher.log")
WINDOW = 100
PAT = {
    "epoch": re.compile(r"^\s*epoch\s*:\s*(\d+)\s*/\s*(\d+)"),
    "step_time_s": re.compile(r"^\s*step time \(s\)\s*:\s*([-\d.]+)"),
    "mean_reward": re.compile(r"^\s*mean reward\s*:\s*([-\d.]+)"),
    "captured": re.compile(r"^\s*captured \(success\)\s*:\s*[\d.]+%\s*\((\d+)/(\d+)\)"),
    "crash": re.compile(r"^\s*crash\s*:\s*[\d.]+%\s*\((\d+)/(\d+)\)"),
    "timeout": re.compile(r"^\s*timeout \(no capture\)\s*:\s*[\d.]+%\s*\((\d+)/(\d+)\)"),
    "mean_ep_length": re.compile(r"^\s*mean ep length\s*:\s*([-\d.]+)"),
}


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def parse(log):
    rows, cur = [], None
    for line in pathlib.Path(log).read_text(errors="replace").replace("\r", "\n").splitlines():
        m = PAT["epoch"].match(line)
        if m:
            if cur and "captured" in cur:
                rows.append(cur)
            cur = {"epoch": int(m.group(1))}
            continue
        if cur is None:
            continue
        for k in ("step_time_s", "mean_reward", "mean_ep_length"):
            m = PAT[k].match(line)
            if m:
                cur[k] = float(m.group(1))
        for k in ("captured", "crash", "timeout"):
            m = PAT[k].match(line)
            if m:
                cur[k] = [int(m.group(1)), int(m.group(2))]
    if cur and "captured" in cur:
        rows.append(cur)
    return rows


def windows(rows, first_epoch):
    out = []
    by = {}
    for r in rows:
        by.setdefault((r["epoch"] - first_epoch) // WINDOW, []).append(r)
    for w in sorted(by):
        rs = by[w]
        n = sum(r["captured"][1] for r in rs)
        out.append({
            "epochs": [rs[0]["epoch"], rs[-1]["epoch"]], "blocks": len(rs), "episodes": n,
            "capture_pct": round(100.0 * sum(r["captured"][0] for r in rs) / n, 2) if n else None,
            "crash_pct": round(100.0 * sum(r["crash"][0] for r in rs) / n, 2) if n else None,
            "timeout_pct": round(100.0 * sum(r.get("timeout", [0, 0])[0] for r in rs) / n, 2) if n else None,
            "mean_reward_mean": round(sum(r.get("mean_reward", 0.0) for r in rs) / len(rs), 3),
            "mean_ep_length_mean": round(sum(r.get("mean_ep_length", 0.0) for r in rs) / len(rs), 2)})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    out = pathlib.Path(ap.parse_args().out)
    out.mkdir(parents=True, exist_ok=True)
    names = ["train_report_s1.json", "train_report_s2.json", "training_curve.json"]
    for n in names:
        if (out / n).exists():
            raise SystemExit("refusing to overwrite %s" % (out / n))
    curve = {"written_by": str(pathlib.Path(__file__)), "script_sha256": sha(__file__), "window_epochs": WINDOW,
             "note": "training-rollout statistics printed by the runner, under exploration noise; not an evaluation",
             "seeds": {}}
    for seed in (1, 2):
        d = U / ("train_s%d" % seed)
        rep = d / "train_report.json"
        if rep.is_file():
            shutil.copyfile(rep, out / ("train_report_s%d.json" % seed))
            assert sha(rep) == sha(out / ("train_report_s%d.json" % seed))
        log = d / "launcher.log"
        if log.is_file():
            rows = parse(log)
            curve["seeds"][str(seed)] = {
                "launcher_log": str(log), "launcher_log_sha256": sha(log),
                "train_report_sha256": sha(rep) if rep.is_file() else None,
                "blocks": len(rows), "first_epoch": rows[0]["epoch"] if rows else None,
                "last_epoch": rows[-1]["epoch"] if rows else None,
                "windows": windows(rows, 25001)}
    if GR1_CTRL_LOG.is_file():
        rows = parse(GR1_CTRL_LOG)
        curve["reference_gr1_ctrl_s1"] = {
            "what": "GR-1 ctrl seed 1: camera + LiDAR, 87 deg camera, F's reward weights, same 205-bar contract, "
                    "epochs 25001..26000",
            "launcher_log": str(GR1_CTRL_LOG), "launcher_log_sha256": sha(GR1_CTRL_LOG),
            "blocks": len(rows), "windows": windows(rows, 25001)}
    (out / "training_curve.json").write_text(json.dumps(curve, indent=1, sort_keys=True) + "\n")
    for s, c in curve["seeds"].items():
        w = c["windows"]
        print("seed", s, "blocks", c["blocks"], "first", w[0] if w else None, "last", w[-1] if w else None)
    if "reference_gr1_ctrl_s1" in curve:
        w = curve["reference_gr1_ctrl_s1"]["windows"]
        print("gr1 ctrl first", w[0], "last", w[-1])


if __name__ == "__main__":
    main()
