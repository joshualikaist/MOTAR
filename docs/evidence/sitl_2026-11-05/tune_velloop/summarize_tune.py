"""Summarise the PX4 velocity-loop sweep on navrl_quad (2026-11-05) against G9 attempt 2 env 1 (MOTAR's Lee loop).
Reads the tune_pacc_*.json next to this file and the G9 JSON path given; writes tune_summary.json.
usage: python summarize_tune.py G9_JSON"""
import json
import pathlib
import statistics as S
import sys

HERE = pathlib.Path(__file__).resolve().parent
g = json.loads(pathlib.Path(sys.argv[1]).read_text())
ref = {k: S.median(st["per_env"][1][k] for ax in ("x", "y") for st in g["A3"][ax])
       for k in ("rise_10_90_s", "onset_latency_s", "overshoot_pct", "settling_5pct_s")}
ref["yaw_rise_10_90_s"] = S.median(a["per_env"][1]["rise_10_90_s"] for a in g["A4"])
out = {"reference_g9_env1": ref, "runs": {}}
for f in sorted(HERE.glob("tune_pacc_*.json")):
    j = json.loads(f.read_text())
    st = [s for ax in ("x", "y") for s in j["A3"][ax]]
    med = lambda k: S.median(s[k] for s in st if s.get(k) is not None)  # noqa: E731
    out["runs"][f.stem] = {"overrides": j["param_overrides"], "rise_10_90_s": med("rise_10_90_s"),
                           "rise_range_s": [min(s["rise_10_90_s"] for s in st), max(s["rise_10_90_s"] for s in st)],
                           "onset_latency_s": med("onset_latency_s"), "overshoot_pct": med("overshoot_pct"),
                           "settling_5pct_s": med("settling_5pct_s"),
                           "yaw_rise_10_90_s": S.median(a["rise_10_90_s"] for a in j["A4"]),
                           "diag_tilt_max_deg": j["A5"]["tilt_max_deg"], "rtf": j["real_time_factor"],
                           "status": j["status"], "host_faults": [j["host_faults_before"], j["host_faults_after"]]}
(HERE / "tune_summary.json").write_text(json.dumps(out, indent=1) + "\n")
print(json.dumps({k: {kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in v.items() if kk != "overrides"}
                  for k, v in out["runs"].items()}, indent=0)[:1500])
