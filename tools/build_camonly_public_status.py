#!/usr/bin/env python3
"""Build docs/status/data/camonly-status.json: the camera-only line's public numbers and verdicts.

Every value is read from a committed record; the public pages (README, docs/status/index.html,
docs/EVIDENCE.md) quote the `display` strings, and tests/test_camonly_public_status.py checks both
directions. Usage: python tools/build_camonly_public_status.py [--check]
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EV = "docs/evidence/camonly_2026-11-04"
SOURCES = {
    "c2": f"{EV}/c1c2/c2_analysis.json",
    "cams3e": f"{EV}/cams3/eval_analysis_aerialgym.json",
    "cams3e_review": f"{EV}/cams3/REVIEW_E_results_r1.md",
    "rt_s3w_review": f"{EV}/rt_s3w/REVIEW_r1.md",
    "outdoor": f"{EV}/outdoor_1126/sky_scan_analysis.json",
}
OUT = ROOT / "docs/status/data/camonly-status.json"


def pct(x, digits=1):
    return f"{x:.{digits}f} %"


def signed(x, digits=1):
    return ("+" if x > 0 else "−") + f"{abs(x):.{digits}f} pp"


def interval(lo, hi, digits=1):
    def s(v):
        return ("−" if v < 0 else "+") + f"{abs(v):.{digits}f}"
    return f"[{s(lo)}, {s(hi)}]"


def build(root=ROOT):
    rd = lambda key: (root / SOURCES[key]).read_text(encoding="utf-8")
    c2 = json.loads(rd("c2"))
    e = json.loads(rd("cams3e"))
    out = json.loads(rd("outdoor"))
    review_e = re.search(r"\*\*Verdict: ([A-Z_]+)\.\*\*", rd("cams3e_review")).group(1)
    review_rt = re.search(r"## Verdict\n([A-Z_]+) for a Jetson desk / hand-held shadow test", rd("rt_s3w_review")).group(1)

    w, s0 = e["cells"]["W"], e["cells"]["S0"]
    q1 = e["Q1"]["W"]
    ws0 = e["Q2"]["W-S0"]
    g15 = out["q4_other"]["ground_filter_sweep"]["by_config"]
    sky = out["q1_sky"]["G1.5"]["overall"]
    values = {
        "c2_total_crash_pp": c2["contrasts"]["total"]["crash"]["mean_pp"],
        "c2_out_fov_crash_pp": c2["contrasts"]["out_fov"]["crash"]["mean_pp"],
        "w_capture_pct": w["capture"]["pooled_pct"],
        "w_crash_pct": w["crash"]["pooled_pct"],
        "w_crash_wilson_lo_pct": 100 * q1["all_crashes"]["wilson_lower_one_sided_95"],
        "w_crash_wilson_hi_pct": 100 * q1["all_crashes"]["wilson_upper_one_sided_95"],
        "w_bar_contact_pct": w["crash_bar"]["pooled_pct"],
        "w_target_contact_pct": w["crash_target"]["pooled_pct"],
        "s0_capture_pct": s0["capture"]["pooled_pct"],
        "s0_crash_pct": s0["crash"]["pooled_pct"],
        "s0_bar_contact_pct": s0["crash_bar"]["pooled_pct"],
        "s0_target_contact_pct": s0["crash_target"]["pooled_pct"],
        "w_minus_s0_crash_pp": ws0["crash"]["mean_pp"],
        "w_minus_s0_crash_ci_lo_pp": ws0["crash"]["ci95_pp"][0],
        "w_minus_s0_crash_ci_hi_pp": ws0["crash"]["ci95_pp"][1],
        "outdoor_ground_near_pct_at_1p5m": 100 * g15["G1.5"]["native"]["T5_near_not_in_Gm0.5_rate"],
        "outdoor_ground_near_pct_at_1p3m": 100 * g15["G1.3"]["native"]["T5_near_not_in_Gm0.5_rate"],
        "outdoor_ground_near_pct_at_1p7m": 100 * g15["G1.7"]["native"]["T5_near_not_in_Gm0.5_rate"],
        "outdoor_sky_phantom_pct": 100 * sky["native"]["T5"]["sky_phantom_rate"],
        "outdoor_frame_bins": out["q1_sky"]["G1.5"]["overall"]["native"]["T5"]["near_count"],
    }
    v = values
    display = {
        "c2_total": signed(v["c2_total_crash_pp"], 2),
        "c2_out_fov": signed(v["c2_out_fov_crash_pp"], 2),
        "w_capture": pct(v["w_capture_pct"]),
        "w_crash": pct(v["w_crash_pct"]),
        "w_crash_wilson": f"[{v['w_crash_wilson_lo_pct']:.1f}, {v['w_crash_wilson_hi_pct']:.1f}]",
        "w_bar_contact": pct(v["w_bar_contact_pct"]),
        "w_target_contact": pct(v["w_target_contact_pct"]),
        "s0_capture": pct(v["s0_capture_pct"]),
        "s0_crash": pct(v["s0_crash_pct"]),
        "s0_bar_contact": pct(v["s0_bar_contact_pct"]),
        "s0_target_contact": pct(v["s0_target_contact_pct"]),
        "w_minus_s0_crash": signed(v["w_minus_s0_crash_pp"]),
        "w_minus_s0_crash_ci": interval(v["w_minus_s0_crash_ci_lo_pp"], v["w_minus_s0_crash_ci_hi_pp"]),
        "outdoor_ground_1p5": pct(v["outdoor_ground_near_pct_at_1p5m"]),
        "outdoor_ground_range": f"{v['outdoor_ground_near_pct_at_1p3m']:.1f}–{v['outdoor_ground_near_pct_at_1p7m']:.1f} %",
        "outdoor_sky_phantom": pct(v["outdoor_sky_phantom_pct"], 2),
    }
    labels = {
        "c2_reading": c2.get("reading", {}).get("label") if isinstance(c2.get("reading"), dict) else c2.get("reading"),
        "q1_label": q1["label"],
        "q1_wilson": q1["all_crashes"]["wilson_label"],
        "w_minus_s0_status": ws0["status"],
        "w_minus_s0m_status": e["Q2"]["W-S0M"]["status"],
        "pointer": e["use_of_result"]["pointer"],
        "results_review": review_e,
        "runtime_review": review_rt,
    }
    return {"schema": "camonly_public_status_v1", "sources": SOURCES, "values": values,
            "display": display, "labels": labels,
            "scope": "Simulation evaluation (CAM-S3-E) and a hand-held outdoor depth recording analysis; "
                     "no autonomous real flight. Runtime review GO covers a desk / hand-held shadow test only."}


def main():
    data = build()
    text = json.dumps(data, indent=1, ensure_ascii=False) + "\n"
    if "--check" in sys.argv:
        ok = OUT.is_file() and OUT.read_text(encoding="utf-8") == text
        print("camonly status: " + ("PASS" if ok else "STALE"))
        return 0 if ok else 1
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text, encoding="utf-8")
    print(OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
