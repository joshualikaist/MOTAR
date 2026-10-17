"""CPU-only checks of S-L1A integrity supervision; no GPU or simulator subprocess."""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
import s_l1a_run as runner  # noqa: E402


PLAN = runner.load_plan()
HEAD = "a" * 40
RMS = "b" * 64


def fake_git(*args):
    return SimpleNamespace(returncode=0, stdout=HEAD + "\n" if args[:2] == ("rev-parse", "HEAD") else "")


def rows(path, count=2, change=False):
    values = []
    for step in range(count):
        row = {key: "a" * 64 for key in runner.DIGEST_KEYS}
        row.update(step=step, resets=[])
        if change and step == 1:
            row["obs"] = "c" * 64
        values.append(row)
    path.write_text("".join(json.dumps(value) + "\n" for value in values))


class ContractTest(unittest.TestCase):
    def test_effective_historical_seeds_and_sensor_modes(self):
        specs = runner.process_specs(PLAN)
        self.assertEqual(len(specs), 11)
        self.assertEqual([s["id"] for s in specs[:3]], ["sensors_legacy", "sensors_depth", "sensors_full"])
        self.assertEqual([s["lineage"] for s in specs[:3]], [None, "s_l1a_v1", "s_l1a_v1"])
        self.assertTrue(all(s["seed"] == 4401 and s["num_envs"] == 36 for s in specs[:3]))
        self.assertEqual({s["seed"] for s in specs[3:] if s["config"] == "static"}, {4501})
        self.assertEqual({s["seed"] for s in specs[3:] if s["config"] == "moving"}, {4502})
        self.assertTrue(all(s["num_envs"] == 8 and s["steps"] == 600 for s in specs[3:]))
        self.assertEqual([s["arm"] for s in specs[3:7]], ["P2", "P2", "P1", "P2"])

    def test_inherited_lineage_is_scrubbed_before_explicit_selection(self):
        spec = runner.process_specs(PLAN)[3]
        with mock.patch.dict(os.environ, {"NAVRL_SENSOR_LINEAGE": "unknown", "NAVRL_DETNOISE_RANGE_STD_M": "99"}):
            env = runner.process_env(PLAN, spec, Path("/tmp/unused"))
        self.assertNotIn("NAVRL_SENSOR_LINEAGE", env)
        self.assertNotIn("NAVRL_DETNOISE_RANGE_STD_M", env)
        self.assertEqual(env["NAVRL_SEED"], "4501")
        self.assertEqual(env["NAVRL_KROC2027_RUN_ID"], PLAN["run"]["run_id"])
        explicit = runner.process_env(PLAN, runner.process_specs(PLAN)[4], Path("/tmp/unused"))
        self.assertEqual(explicit["NAVRL_SENSOR_LINEAGE"], "off")

    def test_uncommitted_new_contract_refuses_official_preflight(self):
        with tempfile.TemporaryDirectory() as temp:
            receipt = Path(temp) / "receipt.json"
            receipt.write_text(json.dumps({"checkpoint_sha256": PLAN["policy_f_sha256"]}))
            with mock.patch.object(runner.base, "preflight", return_value=([], {"policy_f_sha256": PLAN["policy_f_sha256"]})), \
                    mock.patch.object(runner, "committed", return_value=False), \
                    mock.patch.object(runner.el0, "config_receipt", return_value=receipt), \
                    mock.patch.object(runner, "historical_bindings", return_value={}), \
                    mock.patch.object(runner.base, "git", side_effect=fake_git):
                problems, _ = runner.preflight(PLAN, HEAD, Path(temp) / "new", False)
        self.assertTrue(any("not committed" in problem for problem in problems))


class OutputAndDigestTest(unittest.TestCase):
    def test_all_fourteen_digests_compared_and_missing_key_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            a, b = Path(temp) / "a.jsonl", Path(temp) / "b.jsonl"
            rows(a)
            rows(b, change=True)
            result = runner.compare_digest_files(a, b, 2)
            self.assertFalse(result["identical"])
            self.assertEqual(result["differences"], [{"step": 1, "key": "obs"}])
            bad = json.loads(b.read_text().splitlines()[0])
            del bad["tracker"]
            b.write_text(json.dumps(bad) + "\n")
            with self.assertRaises(ValueError):
                runner.load_digests(b, 1)

    def test_completion_hash_mismatch_and_nan_are_integrity_failures(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            payload = out / "fixture.json"
            payload.write_text('{"qualification":"FAIL"}')
            done = {"process": "sensors_legacy", "files": [{"file": payload.name,
                    "bytes": payload.stat().st_size, "sha256": runner.base.sha256_file(payload)}], "nan_fields": []}
            (out / "process_done.json").write_text(json.dumps(done))
            spec = runner.process_specs(PLAN)[0]
            self.assertEqual(runner.validate_done(out, spec, {})["nan_fields"], [])
            payload.write_text("tampered")
            with self.assertRaises(ValueError):
                runner.validate_done(out, spec, {})
            (out / "nonfinite.json").write_text('{"x":NaN}')
            with self.assertRaises(ValueError):
                runner.read_json(out / "nonfinite.json")

    def test_path_escape_refused_in_completion(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp) / "process"
            out.mkdir()
            (out / "process_done.json").write_text(json.dumps({"process": "sensors_legacy", "nan_fields": [],
                "files": [{"file": "../outside", "bytes": 0, "sha256": "0" * 64}]}))
            with self.assertRaises(ValueError):
                runner.validate_done(out, runner.process_specs(PLAN)[0], {})

    def test_historical_seed_context_must_match_before_comparison(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            members = []
            for config, seed in (("static", 4501), ("moving", 4502)):
                directory = root / ("v3_" + config + "_p2")
                directory.mkdir()
                rows(directory / "v3_steps.jsonl")
                (directory / "v3_meta.json").write_text(json.dumps({"process": {
                    "config": config, "seed": seed, "steps": 600, "num_envs": 8}}))
                for path in directory.iterdir():
                    members.append({"path": str(path.relative_to(root)), "bytes": path.stat().st_size,
                                    "sha256": runner.base.sha256_file(path)})
            manifest = root / "RAW_MANIFEST.json"
            manifest.write_text(json.dumps({"members": members}))
            with mock.patch.object(runner, "HISTORICAL_ROOT", root), \
                    mock.patch.object(runner, "HISTORICAL_MANIFEST", manifest):
                self.assertIn("static", runner.historical_bindings(PLAN)["configs"])
                wrong = copy.deepcopy(PLAN)
                wrong["run"]["closed_loop_seed_by_config"]["static"] = 4401
                with self.assertRaisesRegex(ValueError, "configuration mismatch"):
                    runner.historical_bindings(wrong)


class GuardTest(unittest.TestCase):
    def guard(self, **kwargs):
        return runner.runtime_guard(PLAN, Path("/tmp"), HEAD, "", 30, **kwargs)

    def test_disk_output_clock_and_source_are_polled_not_only_preflight(self):
        with mock.patch.object(runner.time, "monotonic", return_value=10), \
                mock.patch.object(runner.base, "git", side_effect=fake_git), \
                mock.patch.object(runner.base, "free_gb", return_value=20) as disk, \
                mock.patch.object(runner.base, "dir_bytes", return_value=0) as size:
            self.assertIsNone(self.guard()[0])
            disk.return_value = 7.99
            self.assertEqual(self.guard()[0], "FREE_DISK_BELOW_8_GB")
            disk.return_value = 20
            size.return_value = int(2e9) + 1
            self.assertEqual(self.guard()[0], "OUTPUT_SIZE_CAP")
            size.return_value = 0
            self.assertEqual(self.guard(process_deadline=9)[0], "PROCESS_WATCHDOG")
            with mock.patch.object(runner.time, "monotonic", return_value=30):
                self.assertEqual(self.guard()[0], "TOTAL_WALL_CLOCK_CAP")
            with mock.patch.object(runner.base, "git", return_value=SimpleNamespace(returncode=0, stdout="changed")):
                self.assertEqual(self.guard()[0], "TRACKED_TREE_OR_HEAD_CHANGED")

    def test_gpu_measurement_failure_is_fail_closed(self):
        with mock.patch.object(runner.time, "monotonic", return_value=10), \
                mock.patch.object(runner.base, "git", side_effect=fake_git), \
                mock.patch.object(runner.base, "free_gb", return_value=20), \
                mock.patch.object(runner.base, "dir_bytes", return_value=0), \
                mock.patch.object(runner.base, "proc_tree_pids", return_value={12}), \
                mock.patch.object(runner.base, "gpu_state", side_effect=RuntimeError("monitor unavailable")) as gpu:
            reason, _ = self.guard(proc=SimpleNamespace(pid=12), cap_mib=7000)
            self.assertTrue(reason.startswith("GUARD_CHECK_FAILED"))
            gpu.side_effect = None
            gpu.return_value = {"compute_apps": [{"pid": 12, "used_mib": 7001}]}
            self.assertEqual(self.guard(proc=SimpleNamespace(pid=12), cap_mib=7000), ("GPU_MEMORY_CAP", 7001))


class CpuProcessTest(unittest.TestCase):
    def test_cpu_watchdog_stops_process_and_preserves_partial_output(self):
        with tempfile.TemporaryDirectory() as temp:
            marker = Path(temp) / "partial"
            command = [sys.executable, "-B", "-c",
                       "from pathlib import Path; import time; Path(%r).write_text('keep'); time.sleep(30)" % str(marker)]
            deadline = time.monotonic() + .15
            rc, elapsed, _, reason = runner.supervised_launch(command, dict(os.environ, CUDA_VISIBLE_DEVICES=""),
                Path(temp) / "process.log", lambda proc: ("PROCESS_WATCHDOG" if time.monotonic() >= deadline else None, 0), .01)
            self.assertNotEqual(rc, 0)
            self.assertEqual(reason, "PROCESS_WATCHDOG")
            self.assertLess(elapsed, 3)
            self.assertEqual(marker.read_text(), "keep")

    def test_term_ignoring_grandchild_is_killed_after_leader_exits(self):
        with tempfile.TemporaryDirectory() as temp:
            ready = Path(temp) / "pid"
            child = "import os,signal,time; from pathlib import Path; signal.signal(signal.SIGTERM,signal.SIG_IGN); Path(%r).write_text(str(os.getpid())); time.sleep(30)" % str(ready)
            parent = "import subprocess,sys,time; subprocess.Popen([sys.executable,'-B','-c',%r]); time.sleep(30)" % child
            deadline = time.monotonic() + 3
            def guard(proc):
                return ("STOP" if ready.exists() or time.monotonic() >= deadline else None), 0
            runner.supervised_launch([sys.executable, "-B", "-c", parent], dict(os.environ, CUDA_VISIBLE_DEVICES=""),
                                     Path(temp) / "process.log", guard, .01)
            self.assertTrue(ready.exists())
            status = Path("/proc") / ready.read_text() / "stat"
            until = time.monotonic() + 1
            while time.monotonic() < until:
                if not status.exists() or status.read_text().split()[2] == "Z":
                    break
                time.sleep(.01)
            else:
                self.fail("grandchild survived process group termination")


class MatrixStopTest(unittest.TestCase):
    def execute(self, root, launch_result, completion=None, comparisons=None):
        specs = runner.process_specs(PLAN)[:2]
        if comparisons is not None:
            specs = runner.process_specs(PLAN)[3:5]
        real_hash = runner.base.sha256_file
        def digest(path):
            return PLAN["policy_f_sha256"] if Path(path).suffix == ".pth" else real_hash(path)
        with mock.patch.object(runner.base, "git", side_effect=fake_git), \
                mock.patch.object(runner.base, "free_gb", return_value=20), \
                mock.patch.object(runner.base, "gpu_state", return_value={"name": "RTX 3070", "total_mib": 8192, "used_mib": 512}), \
                mock.patch.object(runner.base, "sha256_file", side_effect=digest), \
                mock.patch.object(runner, "supervised_launch", return_value=launch_result) as launch, \
                mock.patch.object(runner, "validate_done", return_value=completion), \
                mock.patch.object(runner, "noninterference", return_value=comparisons or {}):
            result = runner.run(PLAN, root, specs, False, {"rms_digest": RMS})
        return result, launch.call_count

    def test_nonzero_exit_stops_next_process_without_retry(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "run"
            result, calls = self.execute(root, (17, .1, 0, None))
            self.assertEqual(calls, 1)
            self.assertEqual(result["status"], "STOPPED_INTEGRITY")
            self.assertTrue((root / "RAW_MANIFEST.json").is_file())
            with self.assertRaises(FileExistsError):
                runner.run(PLAN, root, [], False, {})

    def test_qualification_fail_does_not_stop_independent_fixtures(self):
        with tempfile.TemporaryDirectory() as temp:
            result, calls = self.execute(Path(temp) / "run", (0, .1, 0, None), {"qualification": "FAIL"})
        self.assertEqual(calls, 2)
        self.assertEqual(result["status"], "COMPLETED")

    def test_noninterference_failure_stops_before_following_process(self):
        with tempfile.TemporaryDirectory() as temp:
            result, calls = self.execute(Path(temp) / "run", (0, .1, 0, None), {}, {"comparison": {"identical": False}})
        self.assertEqual(calls, 1)
        self.assertEqual(result["status"], "STOPPED_NONINTERFERENCE")


if __name__ == "__main__":
    unittest.main()
