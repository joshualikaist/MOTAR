"""CPU statistical and process-supervision checks for the preregistered OCC-P2 package."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
from scipy import stats

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location("_test_" + name, ROOT / "tools" / (name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


runner = load("occlusion_confirmatory")
analysis = load("occlusion_confirmatory_analysis")
PLAN = json.loads(runner.PLAN.read_text())


class DesignTest(unittest.TestCase):
    def test_exact_matrix_independence_and_no_shared_output(self):
        cells = runner.matrix(PLAN)
        self.assertEqual(len(cells), 112)
        self.assertEqual(len({c["cell_dir"] for c in cells}), 112)
        self.assertEqual({c["seed"] for c in cells}, set(range(4301,4315)))
        self.assertFalse({c["seed"] for c in cells} & set(PLAN["design"]["pilot_seeds"]))
        self.assertEqual({(c["group"],c["arm"],c["lineage"]) for c in cells},
                         {("static","A1","E0_static"),("moving","A3","E2_direct_l2_v1")})
        self.assertEqual(112*PLAN["design"]["num_envs"]*PLAN["design"]["quota_per_env"],57344)
        self.assertEqual(len(PLAN["analysis"]["primary_contrasts"]+PLAN["analysis"]["secondary_contrasts"]),6)
        self.assertIsNone(PLAN["analysis"]["equivalence_margin"])
        self.assertEqual(PLAN["safety"]["min_free_disk_gb"],8.)

    def test_scene_reuse_is_detected_without_resampling(self):
        with tempfile.TemporaryDirectory() as directory:
            np.savez(Path(directory)/"occlusion_layouts.npz", hashes=np.array(["pilot_scene","new_scene"]))
            overlap, count = runner.scene_overlap(directory,{"pilot_scene"})
            self.assertEqual(overlap,["pilot_scene"])
            self.assertEqual(count,2)

    def test_no_output_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"record.json"
            runner.emit_new(path,{"old":True})
            before = path.read_bytes()
            with self.assertRaises(FileExistsError):
                runner.emit_new(path,{"new":True})
            self.assertEqual(path.read_bytes(),before)


class SupervisionTest(unittest.TestCase):
    def test_term_ignoring_grandchild_dies_when_group_leader_exits(self):
        with tempfile.TemporaryDirectory() as directory:
            ready=Path(directory)/"grandchild.pid"
            child="import signal,time,os,pathlib; signal.signal(signal.SIGTERM,signal.SIG_IGN); pathlib.Path(%r).write_text(str(os.getpid())); time.sleep(30)" % str(ready)
            parent="import subprocess,sys,time; subprocess.Popen([sys.executable,'-c',%r]); time.sleep(30)" % child
            _,_,reason=runner.supervised_launch([sys.executable,"-B","-c",parent],os.environ,
                Path(directory)/"runner.log",3.,lambda:"STOP_TEST" if ready.exists() else None,.01)
            self.assertEqual(reason,"STOP_TEST")
            pid=int(ready.read_text())
            proc=Path("/proc")/str(pid)/"stat"
            deadline=time.monotonic()+1.
            while time.monotonic()<deadline:
                try:
                    state=proc.read_text().split()[2]
                except FileNotFoundError:
                    break
                if state=="Z":
                    break
                time.sleep(.01)
            else:
                self.fail("grandchild survived the owned group kill")

    def test_disk_crossing_kills_child_and_keeps_partial_file(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory)/"partial.bin"
            marker.write_bytes(b"preserve")
            polls = []
            def guard():
                polls.append(1)
                return "FREE_DISK_BELOW_8_GB" if len(polls)>=2 else None
            code, elapsed, reason = runner.supervised_launch(
                [sys.executable,"-B","-c","import time; time.sleep(30)"],os.environ,
                Path(directory)/"runner.log",10.,guard,.01)
            self.assertNotEqual(code,0)
            self.assertEqual(reason,"FREE_DISK_BELOW_8_GB")
            self.assertLess(elapsed,3.)
            self.assertEqual(marker.read_bytes(),b"preserve")

    def test_wall_cap_terminates_instead_of_hanging(self):
        with tempfile.TemporaryDirectory() as directory:
            code, elapsed, reason = runner.supervised_launch(
                [sys.executable,"-B","-c","import time; time.sleep(30)"],os.environ,
                Path(directory)/"runner.log",.03,lambda:None,.01)
            self.assertNotEqual(code,0)
            self.assertEqual(reason,"CELL_OR_WALL_CLOCK_CAP")
            self.assertLess(elapsed,3.)

    def test_nonzero_child_exit_recorded(self):
        with tempfile.TemporaryDirectory() as directory:
            code, _, reason = runner.supervised_launch([sys.executable,"-B","-c","raise SystemExit(17)"],
                os.environ,Path(directory)/"runner.log",2.,lambda:None,.01)
            self.assertEqual(code,17)
            self.assertIsNone(reason)

    def test_first_failure_stops_matrix_and_writes_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            plan = copy.deepcopy(PLAN)
            plan["output_root"] = str(Path(directory)/"new")
            pre={"source_commit":"a"*40,"checkpoint_sha256":"f"*64}
            def fake_git(*args,**kwargs):
                return "a"*40 if args[:2]==("rev-parse","HEAD") else ""
            with mock.patch.object(runner,"supervised_launch",return_value=(17,.1,None)) as launch, \
                 mock.patch.object(runner.legacy,"git",side_effect=fake_git), \
                 mock.patch.object(runner.legacy,"free_disk_gb",return_value=20.), \
                 mock.patch.object(runner.legacy,"check_cell",return_value=[]), \
                 mock.patch.object(runner.legacy,"occlusion_trace_problems",return_value=[]), \
                 mock.patch.object(runner.legacy,"policy_f_sha",return_value="f"*64), \
                 mock.patch.object(runner,"verify_preserved",return_value=({"problems":[]},set())):
                result = runner.run(plan,pre,runner.legacy.kp.load_protocol(runner.legacy.PROTOCOL),os.environ,set())
            self.assertEqual(result,1)
            self.assertEqual(launch.call_count,1)
            root=Path(plan["output_root"])
            self.assertEqual(len(json.loads((root/"run_report.json").read_text())["cells"]),1)
            self.assertTrue((root/"RAW_MANIFEST.json").is_file())


class StatisticalTest(unittest.TestCase):
    def test_end_to_end_seed_weighting_and_fixed_decision_family(self):
        def fixture(directory, cell, plan):
            seed_index=cell["seed"]-4301
            count=10+(cell["density"]-70)//15+(20 if cell["group"]=="moving" else 0)
            exposure=1.+seed_index/7
            intervals=[{"episode":"0/0","duration_s":.2,"end":"restored","censored":False}
                       for _ in range(count)]
            original={"_intervals":{"center_occlusion_in_view":intervals},"_exposures":[exposure]*512,
                      "parity":{},"indicators":{}}
            episodes=[{"episode":str(i),"outcome":"capture","acquired":True,"exposure_s":exposure,
                       "intervals":count if i==0 else 0,"strict_incidents":count if i==0 else 0,
                       "onset_kinds":{},"windows":{str(w):{"count":0,"exposure_s":min(w,exposure)} for w in (1,2,5)}}
                      for i in range(512)]
            duration=[dict(r,end="geometric_restored",onset_kind="clear_to_blocked") for r in intervals]
            return original,episodes,duration,[]
        plan=copy.deepcopy(PLAN)
        plan["analysis"]["bootstrap_resamples"]=100
        with mock.patch.object(analysis,"extract_cell",side_effect=fixture), \
             mock.patch.object(analysis,"load",return_value=SimpleNamespace(matrix=runner.matrix)), \
             mock.patch("sys.stderr"):
            result=analysis.analyse(Path("/no-simulation"),plan)
        self.assertEqual(result["RQ1"],"SUPPORTED")
        self.assertEqual(result["RQ3"],"SUPPORTED")
        self.assertEqual(result["RQ2"],"INCONCLUSIVE_EQUIVALENCE_NOT_TESTED")
        self.assertEqual(len(result["contrasts"]),6)
        group=result["groups"]["static/70"]
        self.assertAlmostEqual(group["seed_equal_rate_per_min"],float(np.mean(group["per_seed_rates"])))
        self.assertNotAlmostEqual(group["seed_equal_rate_per_min"],60*group["intervals"]/group["exposure_s"])

    def test_paired_t_matches_scipy_and_uses_fourteen_seeds(self):
        values=np.linspace(.1,2.,14)
        result=analysis.contrast(values)
        reference=stats.ttest_1samp(values,0.)
        self.assertAlmostEqual(result["p_one_sided"],reference.pvalue/2)
        self.assertEqual(result["df"],13)
        self.assertEqual(result["sign_flip_p"],1/16384)
        self.assertLess(result["simultaneous_ci"][0],result["ci"][0])

    def test_constant_contrast_is_not_silently_given_a_valid_t_test(self):
        result=analysis.contrast(np.ones(14))
        self.assertIsNone(result["p_one_sided"])
        self.assertIsNone(result["ci"])
        self.assertEqual(result["inference_status"],"NOT_ESTIMABLE_ZERO_SEED_VARIANCE")

    def test_holm_family_cannot_select_favorable_subset(self):
        np.testing.assert_allclose(analysis.holm([.001,.01,.02,.03,.04,.05]),[.006,.05,.08,.09,.09,.09])
        np.testing.assert_allclose(analysis.holm([.1]*6),[.6]*6)

    def test_censored_short_interval_is_unknown_not_short(self):
        rows=[{"duration_s":.2,"end":"geometric_restored"},
              {"duration_s":.3,"end":"crash"},
              {"duration_s":1.2,"end":"geometric_restored"}]
        summary=analysis.duration_summary(rows,[1,2,5])
        self.assertEqual(summary["threshold_bounds"]["1"]["fraction_lower"],1/3)
        self.assertEqual(summary["threshold_bounds"]["1"]["fraction_upper"],2/3)
        self.assertEqual(summary["censored_for_geometry_duration"],1)
        self.assertEqual(summary["kaplan_meier_geometry_restoration"]["min_risk_set_at"]["5"],0)
        self.assertIsNone(summary["kaplan_meier_geometry_restoration"]["S_with_supported_risk_set_only"]["5"])

    def test_competing_fov_crash_and_restoration_partition_mass(self):
        rows=[{"duration_s":.5,"end":"geometric_restored"},
              {"duration_s":.5,"end":"out_of_view"},
              {"duration_s":1.,"end":"crash"},
              {"duration_s":2.,"end":"timeout"}]
        result=analysis.duration_summary(rows,[.5,1,2])
        cif=result["competing_incidence"]
        self.assertAlmostEqual(sum(cif[c]["2"] for c in cif),1.)
        self.assertAlmostEqual(cif["out_of_view"]["0.5"],.25)
        self.assertEqual(result["censored_for_geometry_duration"],3)

    def test_zero_intervals_remains_not_estimable(self):
        result=analysis.duration_summary([],[1,2])
        self.assertEqual(result["n"],0)
        self.assertEqual(result["censored_for_geometry_duration"],0)
        self.assertIsNone(result["observed_elapsed_quantiles_s"])
        self.assertIsNone(result["threshold_bounds"]["1"]["fraction_lower"])

    def test_preexisting_cpu_event_contract_is_unchanged(self):
        for path in ("motar_eval/occlusion_events.py","motar_eval/occlusion_geometry.py",
                     "motar_eval/occlusion_trace.py","aerial_gym/task/navrl_task/kroc_eval_adapter.py"):
            old=subprocess.check_output(["git","show",PLAN["frozen"]["g0_commit"]+":"+path],cwd=ROOT)
            self.assertEqual((ROOT/path).read_bytes(),old)


if __name__=="__main__":
    unittest.main()
