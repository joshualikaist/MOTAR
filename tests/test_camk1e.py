"""CPU tests of CAM-K1-E's launcher copy, runner checks and analysis (docs/plans/prereg_camonly_k1_eval_v1_2026-11-08.md).

No simulator and no GPU. Every negative case has a positive control. The end-to-end analysis smoke reads GR-0's S00
and C2's no_lidar raw cells (read-only) and is skipped where they are absent.
"""
import hashlib
import json
import pathlib
import random
import shutil
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import camk1e_analyse as E                                    # noqa: E402
import camk1e_run as R                                        # noqa: E402

RG = ROOT / "aerial_gym/rl_training/rl_games"
RAW = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results")
GR0 = RAW / "gain_randomization_gr0_v1/gr0_main"
C2 = RAW / "camonly_c1c2_v1/c2"
CELL = "E2_obstacle_aware__%dbars__seed%d"


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


class LauncherCopy(unittest.TestCase):
    def test_copy_differs_from_pinned_base_only_in_marked_changes(self):
        base = (RG / "eval_navrl_target_behavior_arms.sh").read_text()
        copy = (RG / "eval_navrl_camk1e_arms.sh").read_text()
        lines = copy.splitlines(keepends=True)
        start = lines.index("# Everything else is byte-for-byte the pinned launcher, which follows:\n")
        body = "".join([lines[0]] + lines[start + 2:])
        i = body.index("# [camk1e] 1:")
        j = body.index('EXPECT_SHA="${CAMK1E_CKPT_SHA:?CAMK1E_CKPT_SHA is required}"\n') + len(
            'EXPECT_SHA="${CAMK1E_CKPT_SHA:?CAMK1E_CKPT_SHA is required}"\n')
        body = body[:i] + ('\nCKPT_REL="runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/'
                           'last_gen_ppo_ep_25000_rew_39.742134.pth"\nCKPT="${HERE}/${CKPT_REL}"\n'
                           'EXPECT_SHA="f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"\n') + body[j:]
        body = body.replace('      export NAVRL_CAMTRAIN_RECEIPT="${CELL_DIR}/camtrain_receipt.json"   # [camk1e] 3\n', "")
        body = body.replace('PYTHON="${CAMEVAL_PY}" \\\n        ./play_navrl.sh "${CKPT}" > "${CELL_LOG}" 2>&1   # [camk1e] 4',
                            '\\\n        ./play_navrl.sh "${CKPT}" > "${CELL_LOG}" 2>&1')
        self.assertEqual(body, base)
        self.assertEqual(copy.count("[camk1e]"), 7)          # header 1; changes 1-4, two of change 1's messages

    def test_runner_pins_the_launcher_sha(self):
        self.assertEqual(R.LAUNCHER_SHA, sha(ROOT / R.LAUNCHER))


class Approval(unittest.TestCase):
    def good(self):
        return {"approved_by": "user", "approved_at": "2026-11-08T11:50:00+09:00", "scope": R.SCOPE,
                "commit": "abcdef1234567"}

    def test_good_approval_passes_and_bad_ones_fail(self):
        self.assertEqual(R.approval_problems(self.good(), "abcdef1234567"), [])
        self.assertEqual(R.approval_problems(dict(self.good(), commit="abcdef1"), "abcdef1234567"), [])
        self.assertTrue(R.approval_problems(dict(self.good(), scope="other"), "abcdef1234567"))
        self.assertTrue(R.approval_problems(dict(self.good(), commit="1234567"), "abcdef1234567"))
        self.assertTrue(R.approval_problems(dict(self.good(), approved_by=""), "abcdef1234567"))
        self.assertTrue(R.approval_problems([], "abcdef1234567"))


class Receipt(unittest.TestCase):
    MOD, ENT = "m" * 64, "e" * 64

    def cell(self, d, values, condition="no_lidar", after=None, log=True, mod=None, task=None, cam=None):
        d.mkdir(parents=True)
        after = after if after is not None else {f: float(values[k]) for k, f in R.AFTER_KEYS.items()}
        rec = {"ablation": {"condition": condition, "hbeams": 72, "vbeams": 4,
                            "task": task if task is not None else str(ROOT / "aerial_gym/task/navrl_task/navrl_task.py"),
                            "perception": str(ROOT / "aerial_gym/task/navrl_task/navrl_perception.py")},
               "ablation_module_sha256": mod or self.MOD, "entry_sha256": self.ENT, "entry": str(ROOT / R.ENTRY),
               "script": str(RG / "runner.py"), "overrides_after": after}
        (d / "camtrain_receipt.json").write_text(json.dumps(rec))
        snap = {"NAVRL_SENSOR_ABLATION": condition, **values}
        (d / "receipt.json").write_text(json.dumps({"env_snapshot_navrl": snap}))
        if log:
            (d / "cell.log").write_text("[navrl] camera hfov=%ddeg\n" % round(float(values["NAVRL_CAMTRAIN_HFOV_DEG"])))
        cam = cam if cam is not None else {"hfov_deg": float(values["NAVRL_CAMTRAIN_HFOV_DEG"]),
                                           "vfov_deg": float(values["NAVRL_CAMTRAIN_VFOV_DEG"])}
        (d / "kroc2027_summary.json").write_text(json.dumps({"camera": cam}))
        return d

    def test_positive_controls_pass(self):
        with tempfile.TemporaryDirectory() as t:
            for i, v in enumerate((R.CAM_VALUES, R.F_VALUES)):
                d = self.cell(pathlib.Path(t) / str(i), v)
                self.assertEqual(R.receipt_problems(d, v, self.MOD, self.ENT), [])

    def test_known_bad_cells_fail(self):
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            v = R.CAM_VALUES
            bad_after = {f: float(v[k]) for k, f in R.AFTER_KEYS.items()}
            bad_after["detector_hfov_deg"] = 87.0
            cases = {"after": self.cell(t / "a", v, after=bad_after), "cond": self.cell(t / "b", v, condition="full"),
                     "log": self.cell(t / "c", v, log=False), "mod": self.cell(t / "d", v, mod="x" * 64),
                     "values": self.cell(t / "e", R.F_VALUES), "task_missing": self.cell(t / "g", v, task=""),
                     "task_relative": self.cell(t / "h", v, task="aerial_gym/task/navrl_task/navrl_task.py"),
                     "vfov": self.cell(t / "i", v, cam={"hfov_deg": 66.0, "vfov_deg": 58.0})}
            for name, d in cases.items():
                self.assertTrue(R.receipt_problems(d, v, self.MOD, self.ENT), name)
            (t / "f").mkdir()
            self.assertTrue(R.receipt_problems(t / "f", v, self.MOD, self.ENT))


    @unittest.skipUnless((pathlib.Path.home() / "motar_tmp/camk1_20261106/train_s1/camtrain_receipt.json").is_file(),
                         "CAM-K1 receipt not on this host")
    def test_the_real_entry_receipt_passes(self):
        """Review r1 M1: CAM-K1 seed 1's real camtrain receipt (66 x 54, 0.6, 0.03) passes the receipt check from any
        working directory."""
        import os
        real = pathlib.Path.home() / "motar_tmp/camk1_20261106/train_s1/camtrain_receipt.json"
        with tempfile.TemporaryDirectory() as t:
            d = self.cell(pathlib.Path(t) / "c", R.CAM_VALUES)
            shutil.copy(real, d / "camtrain_receipt.json")
            mod, ent = sha(ROOT / R.MODULE), sha(ROOT / R.ENTRY)
            cwd = os.getcwd()
            try:
                os.chdir(t)
                self.assertEqual(R.receipt_problems(d, R.CAM_VALUES, mod, ent), [])
            finally:
                os.chdir(cwd)


class Checkpoints(unittest.TestCase):
    def report(self, d, seed, ck_bytes, smoke=False, verdict="TRAINED", copy_sha=None):
        (d / ("train_s%d" % seed) / "checkpoint").mkdir(parents=True)
        copy = d / ("train_s%d" % seed) / "checkpoint" / "c.pth"
        copy.write_bytes(ck_bytes)
        s = hashlib.sha256(ck_bytes).hexdigest()
        rep = {"seed": seed, "verdict": verdict, "development_smoke": smoke,
               "checkpoint": {"sha256": s, "copy": str(copy), "copy_sha256": copy_sha or s}}
        (d / ("train_s%d" % seed) / "train_report.json").write_text(json.dumps(rep))

    def run_case(self, pinned=None, **kw):
        from unittest import mock
        pins = pinned or {1: hashlib.sha256(b"one").hexdigest(), 2: hashlib.sha256(b"two").hexdigest()}
        with tempfile.TemporaryDirectory() as t, mock.patch.dict(R.K1_SHA, pins):
            t = pathlib.Path(t)
            self.report(t, 1, b"one", **kw)
            self.report(t, 2, b"two")
            out, problems = R.checkpoints(t)
            return out, [p for p in problems if not p.startswith("Policy F")]

    @unittest.skipUnless((pathlib.Path.home() / "motar_tmp/camk1_20261106/train_s2/train_report.json").is_file(),
                         "CAM-K1 train reports not on this host")
    def test_pinned_shas_match_the_real_train_reports(self):
        for seed in (1, 2):
            rep = json.loads((pathlib.Path.home() / ("motar_tmp/camk1_20261106/train_s%d/train_report.json" % seed))
                             .read_text())
            self.assertEqual(rep["checkpoint"]["sha256"], R.K1_SHA[seed])

    def test_trained_reports_pass(self):
        out, problems = self.run_case()
        self.assertEqual(problems, [])
        self.assertEqual(out["K1s1"][1], hashlib.sha256(b"one").hexdigest())

    def test_bad_reports_fail(self):
        self.assertTrue(self.run_case(smoke=True)[1])
        self.assertTrue(self.run_case(verdict="TRAINING_FAIL")[1])
        self.assertTrue(self.run_case(copy_sha="0" * 64)[1])
        self.assertTrue(self.run_case(pinned={1: "0" * 64, 2: hashlib.sha256(b"two").hexdigest()})[1])


class Gates(unittest.TestCase):
    def test_gate_r(self):
        self.assertEqual(E.gate_r([-30, -15], [-31, -12], -22), "MATERIAL_RECOVERY")
        self.assertEqual(E.gate_r([-30, -15], [-31, -12], -22, timeout_mean=5.0), "CRASH_TO_TIMEOUT")
        self.assertEqual(E.gate_r([-30, -5], [-31, -12], -15), "SMALL_RECOVERY")
        self.assertEqual(E.gate_r([-1.0, -0.4], [-1.1, -0.3], -0.7), "NEGLIGIBLE")    # review r1 m7: 2 pp floor
        self.assertEqual(E.gate_r([2, 9], [1, 10], 5), "WORSE")
        self.assertEqual(E.gate_r([0.2, 0.9], [0.1, 1.1], 0.6), "NEGLIGIBLE")
        self.assertEqual(E.gate_r([-5, 3], [-6, 4], -1), "NO_DETECTED_RECOVERY")
        self.assertEqual(E.gate_r([-5, -1], [-6, 1], -3), "UNRESOLVED")
        self.assertEqual(E.gate_r([0, 0], [0, 0], 0), "NEGLIGIBLE")          # a null contrast never recovers

    def test_gate_p(self):
        self.assertEqual(E.gate_p([0.3, 1.0], [0.1, 1.1]), "NO_MATERIAL_LOSS")
        self.assertEqual(E.gate_p([3.4, 4.0], [3.3, 4.1]), "MATERIAL_LOSS")
        self.assertEqual(E.gate_p([2, 5], [1.5, 5.5]), "LOSS_INCONCLUSIVE")
        self.assertEqual(E.lineage("A", "A"), "A")
        self.assertEqual(E.lineage("A", "B"), "SEEDS_DISAGREE")

    def test_stratified_intervals(self):
        blocks = [(d, s) for d in (70, 115, 160, 205) for s in (1, 2, 3)]
        xs = [10.0 * (i // 3) + (i % 3) * 0.1 for i in range(12)]   # density effect large, seed effect small
        ci = E.strat_ci(xs, blocks, random.Random(1))
        t = E.strat_t(xs, blocks)
        mean = sum(xs) / 12
        self.assertTrue(ci[0] <= mean <= ci[1] and t[0] < mean < t[1])
        self.assertLess(t[1] - t[0], 1.0)        # the fixed density effect does not widen the interval
        self.assertLess(ci[1] - ci[0], 1.0)

    def test_file_matches_commit(self):
        self.assertTrue(E.file_matches_commit(ROOT / "tools/gr_analyse.py", "tools/gr_analyse.py", "HEAD"))
        self.assertFalse(E.file_matches_commit(ROOT / "tools/gr_run.py", "tools/gr_analyse.py", "HEAD"))
        self.assertFalse(E.file_matches_commit(ROOT / "tools/gr_analyse.py", "tools/gr_analyse.py", "0" * 40))


@unittest.skipUnless((GR0 / "gr0_run_report.json").is_file() and (C2 / "c2_run_report.json").is_file(),
                     "GR-0 / C2 raw cells not on this host")
class AnalysisSmoke(unittest.TestCase):
    """Synthetic identity and eval folders built from C2's no_lidar cells with outcomes moved by a known share."""

    def build(self, t, shares, k1s1_equals_f66=False, bad_override=False, extra_snap_key=False, bad_condition=False):
        ident, ev = t / "identity_01", t / "eval_01"
        ck = {"F": ["f", "F" * 64], "K1s1": ["k1", "1" * 64], "K1s2": ["k2", "2" * 64]}
        records = []
        rng = random.Random(7)
        for d in (70, 115, 160, 205):
            for s in (4501, 4502, 4503):
                src = C2 / "no_lidar" / (CELL % (d, s))
                rows = [json.loads(x) for x in (src / "episodes.jsonl").read_text().splitlines() if x.strip()]
                made = {}
                for arm in ("F66", "K1s1", "K1s2"):
                    out = ev / arm / (CELL % (d, s))
                    out.mkdir(parents=True)
                    shutil.copy(src / "result.json", out / "result.json")
                    if arm == "K1s1" and k1s1_equals_f66:
                        new = made["F66"]
                    else:
                        frm, to, share = shares[arm]
                        new = [dict(r, outcome=to) if r.get("outcome") == frm and rng.random() < share else dict(r)
                               for r in rows]
                    made[arm] = new
                    (out / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in new))
                    after = dict(E.CAM_AFTER, detector_hfov_deg=87.0) if bad_override and arm == "K1s2" else E.CAM_AFTER
                    (out / "camtrain_receipt.json").write_text(json.dumps(
                        {"ablation": {"condition": "no_lidar"}, "ablation_module_sha256": "m", "entry_sha256": "e",
                         "overrides_after": after}))
                    key = "F" if arm == "F66" else arm
                    snap = dict(json.loads((src / "receipt.json").read_text())["env_snapshot_navrl"])
                    snap.pop("NAVRL_SENSOR_ABLATION_RECEIPT", None)
                    snap.update({"NAVRL_CAMTRAIN_RECEIPT": str(out / "camtrain_receipt.json"),
                                 "NAVRL_CAMTRAIN_HFOV_DEG": "66.0", "NAVRL_CAMTRAIN_VFOV_DEG": "54.0",
                                 "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03",
                                 "NAVRL_EVAL_CHECKPOINT": ck[key][1]})
                    if extra_snap_key and arm == "K1s1":
                        snap["NAVRL_GAIN_RAND"] = "1"
                    (out / "receipt.json").write_text(json.dumps(
                        {"env_snapshot_navrl": snap, "checkpoint_sha256": ck[key][1]}))
                    (out / "kroc2027_summary.json").write_text(json.dumps({"camera": {"hfov_deg": 66.0,
                                                                                     "vfov_deg": 54.0}}))
                    if bad_condition and arm == "K1s2":
                        res = json.loads((out / "result.json").read_text())
                        res["condition"]["bars"] = -1
                        (out / "result.json").write_text(json.dumps(res))
                    records.append({"arm": arm, "cell_id": CELL % (d, s), "density": d, "seed": s,
                                    "checkpoint_sha256": ck[key][1]})
                    if arm in ("F66", "K1s1") and (d, s) == (205, 4501):
                        neg = ident / ("F66_negative" if arm == "F66" else "K1s1_smoke") / (CELL % (d, s))
                        neg.mkdir(parents=True)
                        shutil.copy(out / "episodes.jsonl", neg / "episodes.jsonl")
        env = {"PATH": "/x"}
        (ident / "identity_report.json").write_text(json.dumps(
            {"verdict": "IDENTITY_PASS", "commit": "c0ffee", "env_allowlisted": env, "c2_dir": str(C2.resolve())}))
        (ev / "eval_run_report.json").write_text(json.dumps(
            {"verdict": "COMPLETE", "cells_expected": 36, "commit": "c0ffee", "env_allowlisted": env,
             "c2_dir": str(C2.resolve()), "checkpoints": ck, "module_sha256": "m", "entry_sha256": "e",
             "identity_report": str(ident / "identity_report.json"), "records": records}))
        return ident, ev

    def analyse(self, **kw):
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            ident, ev = self.build(t, **kw)
            from unittest import mock
            with mock.patch.object(E, "file_matches_commit", lambda *a, **k: True):
                rc = E.main([str(GR0), str(C2), str(ident), str(ev), str(t / "out.json")])
            return rc, json.loads((t / "out.json").read_text())

    SHARES = {"F66": ("capture", "crash", 0.10), "K1s1": ("crash", "capture", 0.80),
              "K1s2": ("crash", "capture", 0.80)}

    def test_known_good_recovery_is_labelled_and_control_reads_material_loss(self):
        rc, res = self.analyse(shares=self.SHARES)
        self.assertEqual(rc, 0, res.get("binding"))
        self.assertEqual(res["reading"], "REPORTED")
        self.assertTrue(all(c["ok"] for c in res["controls"].values()), res["controls"])
        self.assertEqual(res["controls"]["P_on_C87_minus_A"]["label"], "MATERIAL_LOSS")
        self.assertEqual(res["controls"]["R_on_A_minus_C87"]["label"], "MATERIAL_RECOVERY")
        self.assertEqual(res["gate_R"], "MATERIAL_RECOVERY")
        # the analysis's contrast mean equals the mean computed here from the rates it reports per block
        c = res["contrasts"]["R_K1s1"]["crash"]
        self.assertAlmostEqual(c["mean_pp"], sum(c["blocks_pp"]) / 12, places=9)
        self.assertLess(c["mean_pp"], -20)

    def test_unrecovered_seed_disagrees(self):
        rc, res = self.analyse(shares=dict(self.SHARES, K1s2=("crash", "capture", 0.02)))
        self.assertEqual(rc, 0, res.get("binding"))
        self.assertEqual(res["gates_per_seed"]["R_K1s1"], "MATERIAL_RECOVERY")
        self.assertNotEqual(res["gates_per_seed"]["R_K1s2"], "MATERIAL_RECOVERY")
        self.assertEqual(res["gate_R"], "SEEDS_DISAGREE")

    def test_known_bad_inputs_fail_binding(self):
        for kw in ({"k1s1_equals_f66": True}, {"bad_override": True}, {"extra_snap_key": True},
                   {"bad_condition": True}):
            rc, res = self.analyse(shares=self.SHARES, **kw)
            self.assertEqual((rc, res["reading"]), (1, "EVAL_BINDING_FAIL"), kw)


@unittest.skipUnless((C2 / "c2_run_report.json").is_file(), "C2 raw cells not on this host")
class RunnerSmoke(unittest.TestCase):
    """tools/camk1e_run.py identity and eval end to end with the preflight and gr_run.run_cell patched: run_cell copies
    C2's no_lidar cell and writes what the launcher and the entry point would write."""

    def fake_run_cell(self, mode):
        def run_cell(root, protocol, out_root, level, density, seed, quota, run_id, scale, head, start, records,
                     timeout_s, extra=None, launcher=None, ckpt_sha=None):
            cid = CELL % (density, seed)
            out = out_root / level / cid
            out.mkdir(parents=True)
            src = C2 / "no_lidar" / cid
            rows = (src / "episodes.jsonl").read_text()
            if (level != "F87_identity" or mode == "f87_differs") and not (mode == "f66_same" and level == "F66_negative"):
                rows = rows.replace('"outcome": "capture"', '"outcome": "crash"', 1 + len(level))
            (out / "episodes.jsonl").write_text(rows)
            shutil.copy(src / "result.json", out / "result.json")
            values = {k: extra[k] for k in R.F_VALUES}
            after = {f: float(values[k]) for k, f in R.AFTER_KEYS.items()}
            if mode == "wrong_fov" and level == "K1s2":
                after["detector_hfov_deg"] = 87.0
            (out / "camtrain_receipt.json").write_text(json.dumps(
                {"ablation": {"condition": extra["NAVRL_SENSOR_ABLATION"], "hbeams": 72, "vbeams": 4,
                              "module": str(ROOT / R.MODULE), "task": str(ROOT / R.MODULE),
                              "perception": str(ROOT / R.MODULE)},
                 "ablation_module_sha256": sha(ROOT / R.MODULE), "entry_sha256": sha(ROOT / R.ENTRY),
                 "entry": str(ROOT / R.ENTRY), "script": str(RG / "runner.py"), "overrides_after": after}))
            (out / "receipt.json").write_text(json.dumps(
                {"env_snapshot_navrl": {"NAVRL_SENSOR_ABLATION": extra["NAVRL_SENSOR_ABLATION"], **values},
                 "checkpoint_sha256": ckpt_sha}))
            (out / "cell.log").write_text("hfov=%ddeg\n" % round(float(values["NAVRL_CAMTRAIN_HFOV_DEG"])))
            (out / "kroc2027_summary.json").write_text(json.dumps({"camera": {
                "hfov_deg": float(values["NAVRL_CAMTRAIN_HFOV_DEG"]), "vfov_deg": float(values["NAVRL_CAMTRAIN_VFOV_DEG"])}}))
            self.assertEqual(launcher, R.LAUNCHER)
            self.assertEqual(extra["CAMK1E_CKPT_SHA"], ckpt_sha)
            rec = {"level": level, "cell_id": cid, "density": density, "seed": seed, "problems": [], "verdict": "OK"}
            records.append(rec)
            return rec
        return run_cell

    def run_stage(self, t, stage, mode=""):
        from unittest import mock
        ck = {"F": ("f", "F" * 64), "K1s1": ("k1", "1" * 64), "K1s2": ("k2", "2" * 64)}
        with mock.patch.object(R, "preflight", lambda p, a, o: ("c0ffee", ["output already exists"] if o.exists()
                                                                else [], {})), \
                mock.patch.object(R, "checkpoints", lambda: (ck, [])), \
                mock.patch.object(R.G, "run_cell", self.fake_run_cell(mode)), \
                mock.patch.object(R.G, "host", lambda: {"fault_count": 0}), \
                mock.patch.object(R.K, "GpuBudget", lambda h: type("B", (), {"exhausted": lambda s: False})()):
            return R.main([stage, "--approved-commit", "c0ffee", "--c2-dir", str(C2), "--out", str(t / stage)])

    def test_identity_then_eval_complete(self):
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            self.assertEqual(self.run_stage(t, "identity"), 0)
            rep = json.loads((t / "identity" / "identity_report.json").read_text())
            self.assertEqual(rep["verdict"], "IDENTITY_PASS")
            self.assertEqual(self.run_stage(t, "eval"), 0)
            ev = json.loads((t / "eval" / "eval_run_report.json").read_text())
            self.assertEqual((ev["verdict"], len(ev["records"])), ("COMPLETE", 36))
            self.assertEqual([r["arm"] for r in ev["records"][:3]], ["F66", "K1s1", "K1s2"])

    def test_known_bad_runs(self):
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            self.assertEqual(self.run_stage(t, "eval"), 2)                # no identity pass yet
            self.assertEqual(self.run_stage(t, "identity", "f87_differs"), 1)
            self.assertEqual(json.loads((t / "identity" / "identity_report.json").read_text())["verdict"],
                             "IDENTITY_FAIL")
            self.assertEqual(self.run_stage(t, "eval"), 2)                # a failed identity in the parent refuses
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            self.assertEqual(self.run_stage(t, "identity", "f66_same"), 1)  # the FOV override had no effect
            self.assertEqual(json.loads((t / "identity" / "identity_report.json").read_text())["verdict"],
                             "IDENTITY_FAIL")
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            self.assertEqual(self.run_stage(t, "identity"), 0)
            self.assertEqual(self.run_stage(t, "eval", "wrong_fov"), 1)    # stops at the first bad K1s2 cell
            ev = json.loads((t / "eval" / "eval_run_report.json").read_text())
            self.assertEqual(ev["verdict"], "INCOMPLETE")
            self.assertTrue(any("override detector_hfov_deg" in p for p in ev["records"][-1]["problems"]))


if __name__ == "__main__":
    unittest.main()
