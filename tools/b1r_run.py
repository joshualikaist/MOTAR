#!/usr/bin/env python3
"""B1-R runner (docs/plans/prereg_b1r_classical_baseline_v1.md): preflight, stages A/B/C, selection.

    python -B tools/b1r_run.py preflight --stage A --approved-commit SHA
    python -B tools/b1r_run.py run --stage A --approved-commit SHA     # G0: F_SHADOW, 28 cells
    python -B tools/b1r_run.py select                                  # after stage B: selection + G1
    python -B tools/b1r_run.py analyse --out FILE                      # after stage C

`preflight` never launches a simulation. `run` launches only the preregistered cells of one stage,
after every condition passes, one launcher invocation per cell through the byte-identical OCC-P2
path; no resume, no overwrite, no retry. Frozen values come from the JSON block in §12 of the
preregistration, never from this file.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PREREG = ROOT / "docs/plans/prereg_b1r_classical_baseline_v1.md"
HOOK_FILES = ("motar_eval/classical_baseline.py", "tools/b1r_hook.py", "tools/b1r_site/sitecustomize.py",
              "tools/b1r_run.py", "tools/b1r_analysis.py")
SITE_DIR = ROOT / "tools/b1r_site"
TARGET_MODEL = "E2_obstacle_aware"
STAGES = ("A", "B", "C-PN", "C-PP")



def load_tool(name):
    spec = importlib.util.spec_from_file_location("_b1r_" + name, ROOT / "tools" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_plan(path=PREREG):
    """The single JSON block of §12, parsed; the preregistration's own SHA-256 alongside."""
    text = Path(path).read_text(encoding="utf-8")
    section = text.split("## 12.", 1)
    if len(section) != 2:
        raise ValueError("preregistration has no section 12")
    blocks = re.findall(r"```json\n(.*?)\n```", section[1], flags=re.S)
    if len(blocks) != 1:
        raise ValueError("section 12 must hold exactly one JSON block, found %d" % len(blocks))
    plan = json.loads(blocks[0])
    plan["_prereg_sha256"] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    return plan


def cell_id(density, seed):
    return "%s__%dbars__seed%d" % (TARGET_MODEL, int(density), int(seed))


def config_label(controller, n_gain, v_nom):
    return ("%s_N%g_v%g" % (controller, n_gain, v_nom)) if controller == "B-PN" else ("%s_v%g" % (controller, v_nom))


def stage_cells(plan, stage, selection=None):
    """Cells of one stage in seed-major order; each holds the hook arm and controller parameters."""
    ev, tu, shadow = plan["evaluation"], plan["tuning"], plan["shadow_config"]
    cells = []
    if stage == "A":
        for s in ev["seeds"]:
            for b in ev["densities"]:
                cells.append(dict(stage="A", arm="F_SHADOW", controller=shadow["arm"], n_gain=shadow["N"],
                                  v_nom=shadow["v_nom"], density=b, seed=s, quota=ev["quota"], group="F_SHADOW"))
    elif stage == "B":
        configs = [("B-PN", n, v) for n in tu["grid"]["B-PN"]["N"] for v in tu["grid"]["B-PN"]["v_nom"]]
        configs += [("B-PP", 0, v) for v in tu["grid"]["B-PP"]["v_nom"]]
        for b in tu["densities"]:
            for ctrl, n, v in configs:
                cells.append(dict(stage="B", arm=ctrl, controller=ctrl, n_gain=n, v_nom=v, density=b, seed=tu["seed"],
                                  quota=tu["quota"], group=config_label(ctrl, n, v)))
    elif stage in ("C-PN", "C-PP"):
        if selection is None:
            raise ValueError("stage C needs the frozen selection")
        ctrl = stage.replace("C-", "B-")
        sel = selection["selected"][ctrl]
        for s in ev["seeds"]:
            for b in ev["densities"]:
                cells.append(dict(stage=stage, arm=ctrl, controller=ctrl, n_gain=sel["N"], v_nom=sel["v_nom"],
                                  density=b, seed=s, quota=ev["quota"], group=ctrl))
    else:
        raise ValueError("unknown stage %r" % stage)
    for c in cells:
        c["cell_dir"] = "%s/%s/%s" % (c["stage"], c["group"], cell_id(c["density"], c["seed"]))
    return cells


def contract_env(legacy, base, protocol, occ_plan, plan, cell, out_dir):
    """OCC-P2 moving-A3 variables for one cell, plus the opt-in hook variables."""
    group = occ_plan["design"]["groups"][plan["contract_source"]["group"]]
    env = legacy.cell_env(base, protocol, target_model=group["target_model"], density=cell["density"],
                          seed=cell["seed"], out_dir=out_dir, quota=cell["quota"], view=group["view"], recorder=True,
                          run_id=plan["run_id"], arm=plan["contract_source"]["kroc_arm_label"])
    env.update(group["extra_env"])
    env.update(occ_plan["telemetry_env"])
    env["PYTHONPATH"] = os.pathsep.join([str(ROOT), str(SITE_DIR)])
    env.update(MOTAR_B1R_ARM=cell["arm"], MOTAR_B1R_CONTROLLER=cell["controller"],
               MOTAR_B1R_N=repr(float(cell["n_gain"])), MOTAR_B1R_VNOM=repr(float(cell["v_nom"])))
    return env


# ------------------------------------------------------------------ checks (pure; CPU-tested)
def read_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def g0_compare(legacy, new_dir, old_dir, quota, fields):
    """G0 (§6): historical_match MATCH (ledger fields + every common recorder field) and equal quota counts."""
    match = legacy.historical_match(new_dir, old_dir, quota, fields)
    out = g0_counts(new_dir, old_dir)
    for key in ("keys_only_new", "keys_only_old"):
        if match.get(key):
            out["problems"].append("recorder %s: %s" % (key, match[key]))
    if match.get("verdict") != "MATCH":
        out["problems"].append("historical_match %s: %s %s" % (match.get("verdict"), match.get("ledger_mismatches")[:3]
                               if match.get("ledger_mismatches") else "", match.get("recorder_mismatches", [])[:3]))
    out["historical_match"] = {k: match.get(k) for k in ("verdict", "ledger_rows_compared", "recorder_records_compared",
                                                         "keys_only_new", "keys_only_old")}
    return out


def g0_counts(new_dir, old_dir):
    """Quota counts and in-quota outcome keys; diagnostics beside."""
    out = {"problems": [], "diagnostics": {}}
    try:
        sums = [json.loads((Path(d) / "episode_ledger_summary.json").read_text()) for d in (new_dir, old_dir)]
        rows = [{(r["env_index"], r["env_episode_index"]): r for r in read_jsonl(Path(d) / s["rows_file"]) if r["in_quota"]}
                for d, s in zip((new_dir, old_dir), sums)]
    except (OSError, ValueError, KeyError) as exc:
        out["problems"].append("%s: %s" % (type(exc).__name__, exc))
        return out
    if sums[0]["quota"]["counts"] != sums[1]["quota"]["counts"]:
        out["problems"].append("quota counts %s != OCC-P2 %s" % (sums[0]["quota"]["counts"], sums[1]["quota"]["counts"]))
    if set(rows[0]) != set(rows[1]):
        out["problems"].append("in-quota episode keys differ (%d vs %d)" % (len(rows[0]), len(rows[1])))
    common = set(rows[0]) & set(rows[1])
    outcome = sum(1 for k in common if rows[0][k]["outcome"] != rows[1][k]["outcome"])
    if outcome:
        out["problems"].append("%d in-quota outcomes differ" % outcome)
    out["diagnostics"] = {
        "completion_step_differs": sum(1 for k in common if rows[0][k]["completion_step"] != rows[1][k]["completion_step"]),
        "min_distance_differs": sum(1 for k in common
                                    if rows[0][k]["min_relative_distance_m"] != rows[1][k]["min_relative_distance_m"]),
        "rows_sha256_equal": sums[0].get("rows_sha256") == sums[1].get("rows_sha256"),
    }
    return out


def template_dir(plan, cell):
    """The OCC-P2 cell whose environment this cell must reproduce (§3)."""
    seed = plan["contract_source"]["tuning_template_seed"] if cell["stage"] == "B" else cell["seed"]
    return Path(plan["contract_source"]["occ_p2_root"]) / cell_id(cell["density"], seed)


def env_problems(new_dir, template, plan, stage):
    """Environment identity (§3): equal to the template except the declared keys, which must hold their values."""
    try:
        new = json.loads((Path(new_dir) / "receipt.json").read_text())["env_snapshot_navrl"]
        old = json.loads((Path(template) / "receipt.json").read_text())["env_snapshot_navrl"]
    except (OSError, ValueError, KeyError) as exc:
        return ["%s: %s" % (type(exc).__name__, exc)]
    src = plan["contract_source"]
    declared = dict(src["env_keys_allowed_to_differ"])
    if stage == "B":
        declared.update(src["tuning_env_extra"])
    problems = []
    for key in sorted(set(new) | set(old)):
        if key in declared:
            want = declared[key]
            if key == "NAVRL_BULK_EVAL_JSON":
                if Path(new.get(key, "")).parent.resolve() != Path(new_dir).resolve():
                    problems.append("env %s %r is not this cell's result.json" % (key, new.get(key)))
            elif want != "any" and new.get(key) != want:
                problems.append("env %s is %r, expected %r" % (key, new.get(key), want))
        elif new.get(key) != old.get(key):
            problems.append("env %s is %r, template %r" % (key, new.get(key), old.get(key)))
    return problems


def hook_problems(directory, cell, root=ROOT):
    path = Path(directory) / "b1r_hook.json"
    if not path.is_file():
        return ["hook sidecar missing"]
    side = json.loads(path.read_text())
    problems = []
    for key, want in (("arm", cell["arm"]), ("controller", cell["controller"]), ("n_gain", float(cell["n_gain"])),
                      ("v_nom", float(cell["v_nom"])), ("patch_applied", True)):
        if side.get(key) != want:
            problems.append("hook %s is %r, expected %r" % (key, side.get(key), want))
    counts = side.get("counts", {})
    if side.get("get_action_calls", 0) <= 0:
        problems.append("hook never called")
    for key in ("non_finite", "raw_out_of_bounds", "nonzero_z"):
        if counts.get(key, 1) != 0:
            problems.append("hook %s = %r" % (key, counts.get(key)))
    expected_site = (Path(root) / "tools/b1r_site/sitecustomize.py").resolve()
    if side.get("sitecustomize") != str(expected_site):
        problems.append("sitecustomize origin mismatch")
    for name in HOOK_FILES[:3]:
        if side.get("source_sha256", {}).get(name) != sha(Path(root) / name):
            problems.append("hook source SHA mismatch: " + name)
    for module in ("aerial_gym", "motar_eval"):
        origin = side.get("import_origins", {}).get(module)
        if not origin or Path(root).resolve() not in Path(origin).resolve().parents:
            problems.append("hook saw %s from %r" % (module, origin))
    return problems


def select(tuning, plan):
    """Per controller: max total capture, then min total crash, min v_nom, min N (§5)."""
    tuning = list(tuning)
    signature = lambda c: (c["controller"], c["n_gain"], c["v_nom"], c["density"], c["seed"], c["quota"])
    expected = [signature(c) for c in stage_cells(plan, "B")]
    actual = [signature(c) for c, _ in tuning]
    if len(actual) != len(expected) or set(actual) != set(expected):
        raise ValueError("incomplete or duplicate tuning grid")
    totals = {}
    for cell, counts in tuning:
        if sum(counts.values()) != plan["num_envs"] * cell["quota"]:
            raise ValueError("tuning quota counts differ")
        key = (cell["controller"], float(cell["n_gain"]), float(cell["v_nom"]))
        t = totals.setdefault(key, {"capture": 0, "crash": 0, "timeout": 0, "cells": 0, "capture_115": None})
        for k in ("capture", "crash", "timeout"):
            t[k] += counts[k]
        t["cells"] += 1
        if int(cell["density"]) == 115:
            t["capture_115"] = counts["capture"] / sum(counts.values())
    expected = len(plan["tuning"]["densities"])
    selected = {}
    for ctrl in ("B-PN", "B-PP"):
        ranked = sorted((k for k in totals if k[0] == ctrl and totals[k]["cells"] == expected),
                        key=lambda k: (-totals[k]["capture"], totals[k]["crash"], k[2], k[1]))
        if not ranked:
            raise ValueError("no complete tuning configuration for %s" % ctrl)
        best = ranked[0]
        selected[ctrl] = {"N": best[1], "v_nom": best[2], **totals[best]}
    table = [{"controller": k[0], "N": k[1], "v_nom": k[2], **v} for k, v in sorted(totals.items())]
    return {"schema": "motar.b1r-selection.v1", "selected": selected, "table": table}


def defect_signatures(cell_dirs, gates):
    """G1 (§6): D0-D3 over one configuration's tuning cells; returns the signatures present."""
    g = gates["G1"]
    found, detail = [], []
    captures = 0
    for directory in cell_dirs:
        directory = Path(directory)
        summary = json.loads((directory / "episode_ledger_summary.json").read_text())
        quota = {(r["env_index"], r["env_episode_index"]) for r in read_jsonl(directory / summary["rows_file"]) if r["in_quota"]}
        recs = [r for r in read_jsonl(directory / "kroc2027_episodes.jsonl") if (r["env_index"], r["env_episode_index"]) in quota]
        if len(recs) != len(quota) or len({(r["env_index"], r["env_episode_index"]) for r in recs}) != len(quota) or not quota:
            raise ValueError("incomplete/duplicate G1 quota records")
        n = len(recs)
        captures += summary["quota"]["counts"]["capture"]
        never = sum(1 for r in recs if not r.get("ever_track_active")) / n
        share, below = g["D2_max_share_timeout_with_swept_min_below_m"]
        cam_min = g["D2_min_camera_valid_share"]
        standoff = sum(1 for r in recs if r["outcome"] == "timeout" and r.get("swept_min_distance_m") is not None
                       and r["swept_min_distance_m"] < below and r.get("observations")
                       and r["obs_camera_valid"] / r["observations"] >= cam_min) / n
        side = json.loads((directory / "b1r_hook.json").read_text())
        steps = float(side["counts"]["steps_env"])
        if steps <= 0:
            raise ValueError("empty G1 process window")
        no_belief = float(side["counts"].get("no_belief", 0)) / steps
        detail.append({"cell": directory.name, "never_track_active": never, "standoff": standoff, "no_belief": no_belief})
        if never > g["D1_max_share_never_track_active"]:
            found.append("D1:%s" % directory.name)
        if standoff > share:
            found.append("D2:%s" % directory.name)
        if no_belief > g["D3_max_share_no_belief_steps"]:
            found.append("D3:%s" % directory.name)
    if g["D0_zero_capture_both_cells"] and captures == 0:
        found.append("D0")
    return found, detail


def seed_census(roots, seed):
    """Inspect complete JSON/JSONL contents and names; follow links once, fail closed on gaps."""
    pattern = re.compile(r'seed[_-]?(\d{3,5})\b|_s(\d{3,5})\b')
    hits, errors, visited, files, dangling = set(), [], set(), 0, []

    def contains(v):
        if isinstance(v, bool):
            return False
        if isinstance(v, int):
            return v == seed
        if isinstance(v, str):
            return seed in [int(x) for x in re.findall(r'-?\d+', v)]
        return isinstance(v, list) and any(contains(x) for x in v)

    def walk(o, path):
        if isinstance(o, dict):
            for k, v in o.items():
                if 'seed' in str(k).lower() and contains(v):
                    hits.add('%s:%s' % (path, k))
                walk(v, path)
        elif isinstance(o, list):
            for v in o:
                walk(v, path)

    def visit(p):
        nonlocal files
        if p.is_symlink() and not p.exists():
            # A dangling link has no content: inspect its name and target text, record it, move on.
            target = os.readlink(p)
            dangling.append('%s -> %s' % (p, target))
            for text in (p.name.lower(), target.lower()):
                for m in pattern.finditer(text):
                    if int(m.group(1) or m.group(2)) == seed:
                        hits.add(str(p))
            return
        try:
            key = p.resolve(strict=True)
            if key in visited:
                return
            visited.add(key)
            for m in pattern.finditer(p.name.lower()):
                if int(m.group(1) or m.group(2)) == seed:
                    hits.add(str(p))
            if p.is_dir():
                for child in sorted(p.iterdir()):
                    visit(child)
            elif p.suffix in ('.json', '.jsonl'):
                files += 1
                with p.open() as stream:
                    if p.suffix == '.jsonl':
                        for line in stream:
                            if line.strip():
                                walk(json.loads(line), p)
                    else:
                        walk(json.load(stream), p)
        except (OSError, ValueError, UnicodeError, RuntimeError) as exc:
            errors.append('%s: %s' % (p, exc))
    for root in roots:
        visit(Path(root))
    return {'seed': seed, 'hits': sorted(hits), 'files_parsed': files,
            'parse_errors': len(errors), 'errors': errors,
            'dangling_links': len(dangling), 'dangling': dangling}


# ---------------------------- supervised execution; no production imports
PREREG_SHA = 'e4607aa21c638f8498676f00e4a5c6b868df3b1aff083874f1ce07e9570636e3'
SELECTION_EVIDENCE = ROOT / 'docs/evidence/b1r_v1/selection_binding.json'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def emit(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n')


def atomic(path, value):
    tmp = Path(str(path) + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')
    os.replace(tmp, path)


def stage_path(plan, stage):
    return Path(plan['output_root']) / stage


def verify_manifest(directory):
    manifest = read(directory / 'RAW_MANIFEST.json')
    files = manifest['files']
    actual = {str(p.relative_to(directory)) for p in directory.rglob('*')
              if p.is_file() and p.name != 'RAW_MANIFEST.json'}
    if actual != set(files) or manifest['file_count'] != len(files):
        raise ValueError('manifest inventory mismatch: ' + str(directory))
    for name, expected in files.items():
        p = directory / name
        if directory.resolve() not in p.resolve().parents:
            raise ValueError('manifest path outside root')
        if sha(p) != expected:
            raise ValueError('manifest SHA mismatch: ' + str(p))
    return sha(directory / 'RAW_MANIFEST.json')


def complete_report(plan, stage, selection=None):
    directory = stage_path(plan, stage)
    verify_manifest(directory)
    report = read(directory / 'run_report.json')
    expected = {c['cell_dir'] for c in stage_cells(plan, stage, selection)}
    if (report.get('verdict') != 'COMPLETE' or report.get('stop_reason') or
            set(report['cells']) != expected or any(v['issues'] for v in report['cells'].values()) or
            report.get('checkpoint_sha256_after') != plan['policy_f_sha256'] or
            report.get('prereg_sha256') != PREREG_SHA):
        raise ValueError('stage not complete and qualified: ' + stage)
    if stage == 'A' and any(v.get('g0', {}).get('problems', ['missing G0']) for v in report['cells'].values()):
        raise ValueError('G0 not PASS')
    return report


def selected_plan(plan, committed=True):
    path = Path(plan['output_root']) / 'selection.json'
    value = read(path)
    if value.get('verdict') != 'PASS' or value.get('scene_overlap') != []:
        raise ValueError('selection gate not PASS')
    if committed:
        legacy = load_tool('kroc2027_eval')
        rel = str(SELECTION_EVIDENCE.relative_to(ROOT))
        binding = json.loads(legacy.git('show', 'HEAD:' + rel))
        if binding['selection_sha256'] != sha(path):
            raise ValueError('committed selection SHA mismatch')
        if binding['stage_b_manifest_sha256'] != sha(stage_path(plan, 'B') / 'RAW_MANIFEST.json'):
            raise ValueError('selection Stage B binding differs')
    return value


def cpu_evidence():
    path = ROOT / 'docs/evidence/b1r_v1/cpu_validation.json'
    value = read(path)
    legacy = load_tool('kroc2027_eval')
    committed = json.loads(legacy.git('show', 'HEAD:' + str(path.relative_to(ROOT))))
    if committed != value or value['PUBLIC_CPU']['failures'] or value['PUBLIC_CPU']['errors'] or value['PUBLIC_CPU']['status'] != 'PASS':
        raise ValueError('committed CPU gate missing or not PASS')
    for name in (*HOOK_FILES, 'tests/test_b1r_classical_baseline.py'):
        if value['source_sha256'][name] != sha(ROOT / name):
            raise ValueError('CPU-tested source differs: ' + name)


def preflight(plan, stage, approved_commit):
    legacy = load_tool('kroc2027_eval')
    protocol = legacy.kp.load_protocol(legacy.PROTOCOL)
    occ = read(ROOT / plan['contract_source']['plan'])
    head = legacy.git('rev-parse', 'HEAD').strip()
    problems, checks = [], {}
    selection = None
    if stage in ('C-PN', 'C-PP'):
        try:
            selection = selected_plan(plan)
        except Exception as exc:
            problems.append(str(exc))
    if approved_commit != head:
        problems.append('approved full SHA differs from HEAD')
    if legacy.git('status', '--porcelain', '--untracked-files=no').strip():
        problems.append('tracked tree dirty')
    if sha(PREREG) != PREREG_SHA:
        problems.append('preregistration bytes differ')
    for rel in (*HOOK_FILES, 'tests/test_b1r_classical_baseline.py', str(PREREG.relative_to(ROOT))):
        try:
            committed = legacy.git('rev-parse', 'HEAD:' + rel).strip()
            working = legacy.git('hash-object', rel).strip()
            if committed != working:
                problems.append('source not committed: ' + rel)
        except Exception:
            problems.append('source missing from HEAD: ' + rel)
    try:
        cpu_evidence()
    except Exception as exc:
        problems.append('CPU evidence: ' + str(exc))
    problems += legacy.static_problems(protocol)
    if legacy.policy_f_sha(protocol) != plan['policy_f_sha256']:
        problems.append('Policy F SHA mismatch')
    gpu = legacy.gpu_names()
    if gpu != [protocol['hardware']['gpu']]:
        problems.append('GPU differs: ' + repr(gpu))
    output = stage_path(plan, stage)
    if output.exists() or output.is_symlink():
        problems.append('stage output exists; no resume/overwrite')
    ancestor = output
    while not ancestor.exists():
        ancestor = ancestor.parent
    free = legacy.free_disk_gb(ancestor)
    if free < plan['gates']['preflight_free_disk_gb'][stage]:
        problems.append('insufficient preflight free disk')
    base = legacy.base_env(inherited={k: v for k, v in os.environ.items()
                                     if not k.startswith(('NAVRL_', 'MOTAR_B1R_'))})
    # Use the actual first-cell hook configuration, including the same PYTHONPATH.
    probe_cell = stage_cells(plan, 'A')[0] if selection is None and stage.startswith('C-') else stage_cells(plan, stage, selection)[0]
    variables = contract_env(legacy, base, protocol, occ, plan, probe_cell,
                             Path(plan['output_root']) / stage / probe_cell['group'])
    issues, origins = legacy.import_problems(variables, legacy.launcher_path(protocol).parent)
    problems += issues
    for cell in stage_cells(plan, stage, selection) if not stage.startswith('C-') or selection else []:
        if not template_dir(plan, cell).is_dir():
            problems.append('missing historical template: ' + str(template_dir(plan, cell)))
    if stage != 'A':
        try:
            a = complete_report(plan, 'A')
            problems += legacy.simulation_source_problems(a['source_commit'], head, plan['runtime_paths'])
            checks['G0'] = 'PASS'
        except Exception as exc:
            problems.append('Stage A: ' + str(exc))
    if stage == 'B':
        roots = [ROOT / 'results', ROOT / '../../.codex_worktrees', ROOT / 'docs',
                 ROOT / 'aerial_gym/rl_training/rl_games/runs']
        census = seed_census(roots, plan['tuning']['seed'])
        checks['seed_census'] = census
        if census['hits'] or census['parse_errors']:
            problems.append('tuning seed census not clean/complete')
    if stage.startswith('C-'):
        try:
            complete_report(plan, 'B')
            if stage == 'C-PP':
                complete_report(plan, 'C-PN', selection)
        except Exception as exc:
            problems.append('previous stage: ' + str(exc))
    return ({'status': 'PASS' if not problems else 'REFUSED', 'problems': problems,
             'stage': stage, 'source_commit': head, 'prereg_sha256': sha(PREREG),
             'checkpoint_sha256': legacy.policy_f_sha(protocol), 'gpu': gpu,
             'import_origins': origins, 'free_disk_gb': free, 'checks': checks},
            protocol, occ, base, selection)


def guard_reason(legacy, output, head, deadline):
    if time.monotonic() >= deadline:
        return 'WALL_CLOCK_CAP'
    if legacy.free_disk_gb(output) < 8.2:
        return 'FREE_DISK_BELOW_8P2_GB'
    if legacy.git('rev-parse', 'HEAD').strip() != head or legacy.git('status', '--porcelain', '--untracked-files=no').strip():
        return 'TRACKED_SOURCE_OR_WORKLOG_CHANGED'
    return None


def validate_cell(plan, cell, head, protocol, legacy):
    directory = Path(plan['output_root']) / cell['cell_dir']
    expect_cell = dict(target_model=TARGET_MODEL, density=cell['density'], seed=cell['seed'],
                       view='visible', search=None, arm='A3')
    issues = legacy.check_cell(directory, protocol, cell['quota'], require_motion=True,
                               expect=legacy.cell_expect(expect_cell, plan['run_id'], head, cell['quota']))
    issues += legacy.occlusion_trace_problems(directory)
    issues += legacy.mt_identity_problems(directory, read(ROOT / 'docs/plans/mt_p1_precision_v1.json'))
    issues += env_problems(directory, template_dir(plan, cell), plan, cell['stage'])
    issues += hook_problems(directory, cell)
    # Complete quota joins are required even before an analysis is requested.
    metrics = load_tool('b1r_analysis').cell_metrics(directory)
    if metrics['episodes'] != plan['num_envs'] * cell['quota']:
        issues.append('quota episode count mismatch')
    return issues


def run_stage(plan, stage, pre, protocol, occ, base, selection):
    legacy, supervisor = load_tool('kroc2027_eval'), load_tool('occlusion_confirmatory')
    output = stage_path(plan, stage)
    output.mkdir(parents=True, exist_ok=False)
    emit(output / 'launch_receipt.json', pre)
    report = dict(run_id=plan['run_id'], stage=stage, source_commit=pre['source_commit'],
                  prereg_sha256=sha(PREREG), cells={}, verdict='RUNNING', stop_reason=None)
    start = time.monotonic()
    deadline = start + plan['gates']['stage_caps_h'][stage] * 3600
    cells = stage_cells(plan, stage, selection)
    fields = read(ROOT / 'docs/plans/occlusion_pilot_v2.json')['g0_noninterference']['ledger_fields']
    guard = lambda: guard_reason(legacy, output, pre['source_commit'], deadline)
    atomic(output / 'run_report.json', report)
    try:
        for cell in cells:
            stopped = guard()
            if stopped:
                report['stop_reason'] = stopped
                break
            directory = Path(plan['output_root']) / cell['cell_dir']
            if directory.exists() or directory.is_symlink():
                raise FileExistsError('cell exists: ' + str(directory))
            env = contract_env(legacy, base, protocol, occ, plan, cell, directory.parent)
            rc, seconds, stopped = supervisor.supervised_launch(
                ['bash', str(legacy.launcher_path(protocol))], env, output / 'runner.log',
                min(plan['gates']['cell_cap_s'], deadline - time.monotonic()), guard)
            issues = [] if rc == 0 else ['launcher exit %s' % rc]
            if stopped:
                issues.append(stopped)
            final_guard = guard()
            if final_guard:
                issues.append(final_guard)
            g0 = None
            try:
                issues += validate_cell(plan, cell, pre['source_commit'], protocol, legacy)
                if stage == 'A':
                    g0 = g0_compare(legacy, directory, template_dir(plan, cell), cell['quota'], fields)
                    issues += g0['problems']
            except Exception as exc:
                issues.append('%s: %s' % (type(exc).__name__, exc))
            report['cells'][cell['cell_dir']] = dict(issues=issues, seconds=seconds, launcher_rc=rc,
                g0=g0, output_bytes=legacy.dir_bytes(directory), free_disk_gb_after=legacy.free_disk_gb(output))
            if issues:
                report['stop_reason'] = ('VOID_CONTRACT_MISMATCH' if g0 and g0['problems']
                                         else stopped or 'FIRST_FAILED_CELL')
            atomic(output / 'run_report.json', report)
            print(json.dumps({'stage': stage, 'completed': len(report['cells']), 'total': len(cells),
                              'cell': cell['cell_dir'], 'issues': issues, 'seconds': seconds}), flush=True)
            if issues:
                break
    except BaseException as exc:
        report['stop_reason'] = '%s: %s' % (type(exc).__name__, exc)
    report.update(wall_hours=(time.monotonic() - start) / 3600,
                  checkpoint_sha256_after=legacy.policy_f_sha(protocol))
    if report['checkpoint_sha256_after'] != plan['policy_f_sha256']:
        report['stop_reason'] = 'CHECKPOINT_CHANGED'
    report['verdict'] = 'COMPLETE' if len(report['cells']) == len(cells) and not report['stop_reason'] else 'STOPPED'
    atomic(output / 'run_report.json', report)
    supervisor.write_manifest(output)
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['verdict'] == 'COMPLETE' else 1


def scene_hashes(paths, num_envs):
    """Union of recorded layout hashes; one per layout (every reset), so at least one per environment."""
    import numpy as np

    out = set()
    for path in paths:
        with np.load(Path(path) / 'occlusion_layouts.npz', allow_pickle=False) as d:
            h = d['hashes']
            if len(h) < num_envs:
                raise ValueError('fewer scene hashes than environments: ' + str(path))
            out.update(str(x) for x in h)
    return out


def select_stage(plan):
    complete_report(plan, 'A')
    complete_report(plan, 'B')
    root = Path(plan['output_root'])
    cells = stage_cells(plan, 'B')
    choice = select([(c, read(root / c['cell_dir'] / 'episode_ledger_summary.json')['quota']['counts']) for c in cells], plan)
    issues, details = {}, {}
    for ctrl, selected in choice['selected'].items():
        selected_dirs = [root / c['cell_dir'] for c in cells if c['controller'] == ctrl and
                         c['n_gain'] == selected['N'] and c['v_nom'] == selected['v_nom']]
        issues[ctrl], details[ctrl] = defect_signatures(selected_dirs, plan['gates'])
    tuning = scene_hashes([root / c['cell_dir'] for c in cells], plan['num_envs'])
    evaluation = scene_hashes([template_dir(plan, c) for c in stage_cells(plan, 'A')], plan['num_envs'])
    overlap = sorted(tuning & evaluation)
    verdict = 'STOP_TUNING_CONTAMINATED' if overlap else 'STOP_BASELINE_DEFECT' if any(issues.values()) else 'PASS'
    choice.update(verdict=verdict, signatures=issues, details=details, scene_overlap=overlap,
                  tuning_scene_count=len(tuning), evaluation_scene_count=len(evaluation),
                  stage_b_manifest_sha256=sha(stage_path(plan, 'B') / 'RAW_MANIFEST.json'))
    emit(root / 'selection.json', choice)
    print(json.dumps(choice, indent=2))
    return 0 if verdict == 'PASS' else 1


CPU_EVIDENCE = ROOT / 'docs/evidence/b1r_v1/cpu_validation.json'


def last_json_object(text):
    """The final JSON object printed by tools/run_tests.py (its summary report)."""
    start = text.rfind('\n{')
    start = 0 if start < 0 and text.startswith('{') else start + 1
    return json.loads(text[start:])


def record_cpu(receipt):
    """Write the CPU evidence that every GPU preflight checks against HEAD (commit it afterwards)."""
    receipt = Path(receipt).resolve()
    report = last_json_object(receipt.read_text())
    if report.get('profile') != 'PUBLIC_CPU':
        raise ValueError('receipt is not a PUBLIC_CPU run')
    names = (*HOOK_FILES, 'tests/test_b1r_classical_baseline.py')
    value = {'schema': 'motar.b1r-cpu-validation.v1', 'prereg_sha256': PREREG_SHA,
             'PUBLIC_CPU': {k: report.get(k) for k in ('status', 'ran', 'failures', 'errors', 'skipped', 'excluded')},
             'receipt': str(receipt.relative_to(ROOT)) if ROOT in receipt.resolve().parents else str(receipt),
             'receipt_sha256': sha(receipt), 'source_sha256': {n: sha(ROOT / n) for n in names}}
    CPU_EVIDENCE.parent.mkdir(parents=True, exist_ok=True)
    emit(CPU_EVIDENCE, value)
    print(json.dumps(value, indent=2))
    return 0 if value['PUBLIC_CPU']['status'] == 'PASS' and not value['PUBLIC_CPU']['failures'] \
        and not value['PUBLIC_CPU']['errors'] else 1


def bind_selection(plan):
    """After `select` PASS: the docs-only binding that Stage C preflights read from HEAD (§5)."""
    path = Path(plan['output_root']) / 'selection.json'
    value = selected_plan(plan, committed=False)
    binding = {'schema': 'motar.b1r-selection-binding.v1', 'selection_sha256': sha(path),
               'stage_b_manifest_sha256': sha(stage_path(plan, 'B') / 'RAW_MANIFEST.json'),
               'selected': value['selected'], 'verdict': value['verdict'], 'prereg_sha256': PREREG_SHA}
    SELECTION_EVIDENCE.parent.mkdir(parents=True, exist_ok=True)
    emit(SELECTION_EVIDENCE, binding)
    print(json.dumps(binding, indent=2))
    return 0


def analyse_stages(plan, out):
    selection = selected_plan(plan)
    reports = {s: complete_report(plan, s, selection) for s in STAGES}
    root = Path(plan['output_root'])
    destination = root / 'analysis.json'
    if out is not None and out.resolve() != destination.resolve():
        raise ValueError('analysis uses one canonical output to prevent repeated analysis')
    if destination.exists():
        raise FileExistsError('analysis already exists; no rerun')
    result = load_tool('b1r_analysis').analyse(
        {'F_SHADOW': root / 'A/F_SHADOW', 'B-PN': root / 'C-PN/B-PN', 'B-PP': root / 'C-PP/B-PP'}, plan)
    result['provenance'] = {s: {'source_commit': r['source_commit'], 'manifest_sha256': sha(stage_path(plan, s) / 'RAW_MANIFEST.json')} for s, r in reports.items()}
    result['selection_sha256'] = sha(root / 'selection.json')
    emit(destination, result)
    print(json.dumps(result['contrasts'], indent=2))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('command', choices=('plan', 'preflight', 'run', 'select', 'bind-selection', 'record-cpu',
                                        'analyse'))
    ap.add_argument('--stage', choices=STAGES)
    ap.add_argument('--approved-commit', default='')
    ap.add_argument('--out', type=Path)
    ap.add_argument('--receipt', type=Path, help='record-cpu: saved stdout of tools/run_tests.py')
    args = ap.parse_args(argv)
    plan = load_plan()
    if plan['_prereg_sha256'] != PREREG_SHA:
        raise ValueError('preregistration differs from approved bytes')
    if args.command == 'record-cpu':
        if not args.receipt:
            ap.error('--receipt required')
        return record_cpu(args.receipt)
    if args.command == 'bind-selection':
        return bind_selection(plan)
    if args.command == 'plan':
        dummy = {'selected': {'B-PN': {'N': 0, 'v_nom': 0}, 'B-PP': {'N': 0, 'v_nom': 0}}}
        print(json.dumps({s: len(stage_cells(plan, s, dummy)) for s in STAGES}))
        return 0
    if args.command == 'select':
        return select_stage(plan)
    if args.command == 'analyse':
        return analyse_stages(plan, args.out)
    if not args.stage:
        ap.error('--stage required')
    pre, protocol, occ, base, selection = preflight(plan, args.stage, args.approved_commit)
    if args.out:
        emit(args.out, pre)
    print(json.dumps(pre, indent=2), flush=True)
    if pre['status'] != 'PASS':
        return 2
    if args.command == 'preflight':
        return 0
    return run_stage(plan, args.stage, pre, protocol, occ, base, selection)


if __name__ == '__main__':
    sys.exit(main())
