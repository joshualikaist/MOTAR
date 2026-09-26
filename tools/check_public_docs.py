#!/usr/bin/env python3
"""Check declared public surfaces; not every historical Markdown page or external URL."""
import argparse
import json
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
PUBLIC = ("README.md", "THIRD_PARTY_LICENSES.md", "docs/results_overview_2026-09-12.md",
          "docs/renderer_cpu_quickstart_2026-09-12.md", "docs/renderer_public_tools.md",
          "docs/PUBLIC_RELEASE_CHECKLIST.md", "docs/public_release_audit_2026-09-12.md",
          "docs/plans/moving_target_rendezvous_master_plan_2026-09-12.md",
          "docs/status/overview-sources-2026-09-13.md", "docs/REPRODUCIBILITY.md",
          "docs/external_data/ETH_DS5.md", "docs/eth_ds5_public_release_inventory_2026-09-14.md",
          "docs/public_release_status_2026-09-14.md",
          "docs/relation_to_published_systems_2026-09-16.md",
          "docs/target_behavior_ladder_2026-09-16.md",
          "docs/status/arena_motion_audit_2026-09-17.md")
STATUS_VALUES = {"COMPLETE", "TECHNICAL_PASS", "TECHNICAL_GO", "FAIL", "PARTIAL_EVIDENCE", "PENDING",
                 "BLOCKED_BY_POLICY", "NOT_RUN", "GO", "INCONCLUSIVE", "NOT_STARTED"}


def manifest_errors(data, root=ROOT):
    errors = []
    if data.get("schema_version") != 2 or data.get("real_flight_validated") is not False:
        errors.append("manifest version/real-flight scope invalid")
    entries = data.get("track_d", {})
    if set(entries) != {"D" + str(i) for i in range(1, 10)}:
        errors.append("Track D entries missing or extra")
    for key, row in entries.items():
        if not isinstance(row, dict) or set(row) != {"label", "status", "evidence"}:
            errors.append(key + ": invalid entry")
            continue
        path = (root / row["evidence"]).resolve()
        if row["status"] not in STATUS_VALUES or not row["label"]:
            errors.append(key + ": invalid state/label")
        if root.resolve() not in path.parents or not path.is_file():
            errors.append(key + ": missing/escaping evidence")
    # These are protected negative outcomes, not test-count or performance pins.
    for key, status in {"D4": "FAIL", "D5": "PARTIAL_EVIDENCE", "D6": "INCONCLUSIVE", "D7": "GO", "D8": "TECHNICAL_GO", "D9": "NOT_RUN"}.items():
        if entries.get(key, {}).get("status") != status:
            errors.append(key + ": protected evidence boundary changed")
    expected = {"e3p": "ATTITUDE_NOT_RELIABLE", "c3": "WITHDRAWN", "ppo_replication": "INCONCLUSIVE",
                "integrated_simulator_cost": "D8A_TECHNICAL_ONLY"}
    if data.get("limits") != expected:
        errors.append("negative/withdrawn evidence limits changed")
    return errors


def registry_errors(data, root=ROOT):
    errors = []
    lifecycle = {
        "COMPLETED", "PLANNED", "BLOCKED", "NOT_TESTED", "ARCHIVED_WITHDRAWN"
    }
    if data.get("schema_version") != 1:
        errors.append("component registry version invalid")
    if set(data.get("status_semantics", {})) != lifecycle:
        errors.append("component registry lifecycle semantics invalid")
    entries = data.get("components", {})
    required = {
        "P6", "P7", "P8", "P9", "P10", "LIVE_RGB_POLICY", "BEARING_DEG",
        "TRUE_METRIC_RANGE", "PERSISTENT_ID_SWITCH", "SAM_IN_SIM",
        "D8A", "D8B", "D8C", "D9", "TM_E0", "TM_E1", "TM_E2", "TM_E3", "TM_E4",
    }
    if set(entries) != required:
        errors.append("component registry entries missing or extra")
    for key, row in entries.items():
        if not isinstance(row, dict) or set(row) != {
            "label", "lifecycle_status", "evidence_status", "evidence"
        }:
            errors.append(key + ": invalid component entry")
            continue
        if row["lifecycle_status"] not in lifecycle:
            errors.append(key + ": invalid lifecycle")
        path = (root / row["evidence"]).resolve()
        if root.resolve() not in path.parents or not path.is_file():
            errors.append(key + ": missing/escaping evidence")
    protected = {
        "P10": ("COMPLETED", "INCONCLUSIVE"),
        "D8B": ("COMPLETED", "MATERIAL_LOSS"),
        "D8C": ("PLANNED", "NOT_STARTED"),
        "D9": ("PLANNED", "NOT_RUN"),
        "SAM_IN_SIM": ("ARCHIVED_WITHDRAWN", "SUPERSEDED"),
        "TM_E3": ("PLANNED", "NOT_IMPLEMENTED"),
        "TM_E4": ("PLANNED", "NOT_IMPLEMENTED"),
    }
    for key, (lifecycle_status, evidence_status) in protected.items():
        row = entries.get(key, {})
        if row.get("lifecycle_status") != lifecycle_status or row.get(
            "evidence_status"
        ) != evidence_status:
            errors.append(key + ": protected lifecycle/evidence boundary changed")
    return errors


def local_link_errors(text, path, root=ROOT):
    text = re.sub(r"```.*?```", "", text, flags=re.S)
    errors = []
    for raw in re.findall(r"!?\[[^\]\n]*\]\(([^)]+)\)", text):
        target = raw.strip().strip("<>").split(' "', 1)[0]
        parsed = urlsplit(target)
        if parsed.scheme or parsed.netloc or not parsed.path:
            continue
        resolved = (path.parent / unquote(parsed.path)).resolve()
        if root.resolve() not in resolved.parents and resolved != root.resolve():
            errors.append("link escapes repository: " + parsed.path)
        elif not resolved.exists():
            errors.append("broken local link: " + parsed.path)
    return errors


# The README is a short landing page (user decision, 2026-09-24): ten fixed sections, headline
# numbers only in the opening block, Key Results and the replication section. Every decimal
# must match a canonical machine-readable record (landing_number_errors). The quick start may
# report the required CPU validation outcome; detailed execution logs stay in WORKLOG.
LANDING_HEADINGS = ("Why MOTAR?", "Key Results", "System", "Target-motion replication", "Repository Structure", "Quick Start",
                    "Research Evidence", "Documentation", "Limitations", "Citation and license")
NUMBER_SECTIONS = ("", "Key Results", "Target-motion replication")


def landing_sections(text):
    """(heading, body) pairs; the block before the first `## ` heading has heading ''."""
    prose = re.sub(r"```.*?```", "", text, flags=re.S)
    parts = re.split(r"^## (.+)$", prose, flags=re.M)
    sections = [("", parts[0])]
    sections += [(parts[i].strip(), parts[i + 1]) for i in range(1, len(parts) - 1, 2)]
    return sections


def landing_errors(text):
    errors = []
    for heading in LANDING_HEADINGS:
        if "## " + heading not in text:
            errors.append("missing heading: " + heading)
    prose = re.sub(r"```.*?```", "", text, flags=re.S)
    if re.search(r"\b\d[\d,]*\s+(?:tests|executed|passed)\b|\bseed[- ]?\d", prose, re.I):
        errors.append("volatile test count or seed identifier on landing page")
    for heading, body in landing_sections(text):
        if heading not in NUMBER_SECTIONS and re.search(r"\d+(?:\.\d+)?\s*(?:%|pp\b)", body):
            errors.append("result number outside declared evidence sections: " + heading)
    for scope in ("simulation-only", "no real-flight validation claim", "historical", "interception"):
        if scope not in text.lower():
            errors.append("missing scope: " + scope)
    if "moving-target rendezvous" not in text.lower():
        errors.append("public terminology missing")
    return errors


def headline_values(root=ROOT):
    """Every value a landing page may show, read from the canonical records."""
    values = set()
    registry = json.loads((root / "docs/quantitative_positioning_registry.json").read_text())
    for row in registry["internal_motar"]:
        for key in ("value_pp", "value_percent", "secondary_value_pp", "value_utility"):
            if isinstance(row.get(key), (int, float)):
                values.add(float(row[key]))
        for key in ("ci95_pp", "seed_level_ci95_pp"):
            for bound in row.get(key) or []:
                values.add(float(bound))
    motion = json.loads((root / "docs/results/target_motion_generalization_2026-09-19.json").read_text())
    for row in motion["contrasts_vs_H"] + motion["contrasts_within_E"]:
        values.add(100 * row["mean_diff"])
        values.update(100 * bound for bound in row["bca95"])
    for arm in motion["arm_means"].values():
        values.update(100 * rate for rate in arm.values())
    # Current target-motion evidence: the independent bias-corrected seven-seed replication.
    replication = root / "results/target_motion_replication_7seed/analysis.json"
    if replication.is_file():
        rep = json.loads(replication.read_text())
        values.update((rep["minimum_exact_two_sided_p"], rep["holm_floor_three_contrasts"]))
        for block in rep["estimators"].values():
            values.update(100 * rate for rate in block["arm_mean_capture"].values())
            for row in block["contrasts"]:
                if row["metric"] == "min_relative_distance_m":
                    continue       # failed its semantic check; never a headline value
                values.add(100 * row["mean_diff"])
                values.update(100 * bound for bound in row["bca95"])
    audit = json.loads((root / "docs/results/target_motion_visibility_reacquisition_audit_2026-09-19.json").read_text())
    for row in audit["per_arm_outcome"].values():
        values.add(100 * row["never_acquired_rate"])
    # The P10 record carries the per-seed readaptation values behind the registry rows.
    p10 = root / "results/perception_p10_seed_replication_2026-09-10/summary.json"
    if p10.is_file():
        for seed in json.loads(p10.read_text())["per_training_seed"].values():
            for key in ("p9_cost_on_source", "p9_cost_on_adapted", "primary_capture"):
                values.add(seed[key]["delta_pp"])
                values.update(seed[key]["ci95"])
    return values


def landing_number_errors(text, values):
    """Each decimal shown in a number-bearing section must round from a canonical value."""
    errors = []
    for heading, body in landing_sections(text):
        if heading not in NUMBER_SECTIONS:
            continue
        for token in re.findall(r"[−+-]?\d+\.\d+", body):
            shown = float(token.replace("−", "-"))
            digits = len(token.split(".")[1])
            if not any(abs(abs(shown) - abs(v)) <= 0.5 * 10 ** -digits + 1e-9 and
                       (shown == 0 or (shown > 0) == (v > 0) or token[0] not in "−+-") for v in values):
                errors.append("landing number not bound to a canonical record: " + token)
    return errors


def check(root=ROOT):
    errors = []
    for name in PUBLIC:
        path = root / name
        if not path.is_file():
            errors.append(name + ": missing")
        else:
            errors += [name + ": " + e for e in local_link_errors(path.read_text(), path, root)]
    readme = (root / "README.md").read_text()
    errors += landing_errors(readme)
    errors += landing_number_errors(readme, headline_values(root))
    citation = (root / "CITATION.cff").read_text().lower()
    if "moving-target rendezvous" not in citation or "simulation-only" not in citation:
        errors.append("CFF public terminology/scope mismatch")
    title = (
        "motar: moving object tracking and rendezvous"
    )
    if title not in readme.lower() or title not in citation:
        errors.append("MOTAR full title mismatch")
    # The method subtitle is canonical: README, the citation record and the live
    # page must carry the SAME wording, so drift in any one of them is an error
    # rather than something only a reader notices.
    subtitle = (
        "reinforcement learning for uav tracking and close approach "
        "in random obstacle fields"
    )
    site = (root / "docs/status/index.html").read_text().lower()
    evidence_page = (root / "docs/status/evidence.html").read_text().lower()
    for name, text in (("README.md", readme.lower()), ("CITATION.cff", citation),
                       ("docs/status/index.html", site)):
        if subtitle not in text:
            errors.append("MOTAR method subtitle missing: " + name)
    data = json.loads((root / "docs/status_manifest.json").read_text())
    errors += manifest_errors(data, root)
    registry = json.loads((root / "docs/research_status_registry.json").read_text())
    errors += registry_errors(registry, root)
    verification = (root / "VERIFICATION.md").read_text().split("## 역사 기록:")[0]
    for value in ("ATTITUDE_NOT_RELIABLE", "WITHDRAWN", "INCONCLUSIVE", "FAIL 유지", "PARTIAL_EVIDENCE"):
        if value not in verification:
            errors.append("Verification disagrees with status manifest: " + value)
    # Lifecycle and Track D cells are loaded from the shared status on the full-evidence page.
    if "status_manifest.js" not in evidence_page:
        errors.append("site does not load shared status")
    return errors


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--schema", action="store_true", help="Also validate Draft-7 schema using jsonschema")
    args = p.parse_args()
    errors = check()
    if args.schema:
        import jsonschema
        jsonschema.validate(json.loads((ROOT / "docs/status_manifest.json").read_text()),
                            json.loads((ROOT / "docs/status_manifest.schema.json").read_text()))
        jsonschema.validate(
            json.loads((ROOT / "docs/research_status_registry.json").read_text()),
            json.loads((ROOT / "docs/research_status_registry.schema.json").read_text()),
        )
    print(json.dumps({"status": "FAIL" if errors else "PASS", "errors": errors,
                      "scope": "Declared public surfaces; local path existence, not external URLs or heading anchors"}, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
