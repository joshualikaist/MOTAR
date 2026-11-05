"""CPU fixtures for the dev08 attempt recorder (tools/sitl_dev08_attempt.py; motar-5md).

Temporary directories, a temporary Git repository and injected clocks/fault counters only; nothing is launched.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


def load(name, rel):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


A = load("sitl_dev08_attempt_t", "tools/sitl_dev08_attempt.py")
P = load("sitl_dev08_prefix_t", "tools/sitl_dev08_prefix_contract.py")
ENV_OK = {"CUDA_VISIBLE_DEVICES": "", "PYTHONNOUSERSITE": "1", "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}


def record(member, outcome="capture", k=30, status="VALID"):
    frames = 208 if (member["bars"], member["env"]) == (70, 5) else 600
    if outcome == P.CENSORED:
        k = frames - 1
    return {"phase": member["phase"], "bars": member["bars"], "env": member["env"], "arm": member["arm"],
            "recorded_frames": frames,
            "operational_status": status, "outcome": outcome, "final_frame_index": k, "observation_checks": k + 1,
            "commands_issued": k, "start_sim_time_s": 40.0, "final_sim_time_s": 40.0 + k * 0.1,
            "scheduled_elapsed_s": round(k * P.DT, 10)}


class Schedule(unittest.TestCase):
    def test_fixed_order(self):
        plan = A.schedule()
        self.assertEqual(len(plan), 42)
        self.assertEqual([(m["phase"], m["bars"], m["env"], m["arm"]) for m in plan[:2]],
                         [("smoke", 70, 0, "native"), ("smoke", 205, 0, "native")])
        main = plan[2:]
        self.assertTrue(all(m["phase"] == "main" for m in main))
        self.assertEqual({(m["bars"], m["env"], m["arm"]) for m in main},
                         {(b, e, a) for b in (70, 205) for e in range(10) for a in ("native", "candidate")})
        self.assertEqual([m["bars"] for m in main], [70] * 20 + [205] * 20)
        for i in range(0, 40, 2):
            a, b = main[i], main[i + 1]
            self.assertEqual(a["env"], b["env"])
            self.assertEqual((a["arm"], b["arm"]), ("native", "candidate") if a["env"] % 2 == 0
                             else ("candidate", "native"))
        self.assertEqual(len({m["name"] for m in plan}), 42)

    def test_environment_rules(self):
        good = {"CUDA_VISIBLE_DEVICES": "", "PYTHONNOUSERSITE": "1", "affinity": [6, 7, 12, 13], "nice": 10,
                "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}
        self.assertEqual(A.environment_problems(good), [])
        for key, bad in (("CUDA_VISIBLE_DEVICES", None), ("CUDA_VISIBLE_DEVICES", "0"), ("PYTHONNOUSERSITE", None),
                         ("affinity", [0, 6]), ("affinity", []), ("nice", 0), ("OMP_NUM_THREADS", "4"),
                         ("OMP_NUM_THREADS", None), ("MKL_NUM_THREADS", "4")):
            self.assertEqual(len(A.environment_problems(dict(good, **{key: bad}))), 1, (key, bad))


class RecorderBase(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="dev08_attempt_"))
        self.repo = self.tmp / "repo"
        self.repo.mkdir()
        git = lambda *a: subprocess.run(["git", "-C", str(self.repo), *a], check=True, capture_output=True)  # noqa
        git("init", "-q")
        (self.repo / "src.py").write_text("x = 1\n")
        git("add", "src.py")
        git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "init")
        self.ckpt = self.tmp / "f.pth"
        self.ckpt.write_bytes(b"policy-f")
        self.fsha = hashlib.sha256(b"policy-f").hexdigest()
        self.faults = [0]
        self.now = [0.0]
        self.host = (True, {"core_idle": "fixture"})
        self.env = mock.patch.dict(os.environ, ENV_OK)
        self.env.start()

    def tearDown(self):
        self.env.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def recorder(self, name="attempt01", **kw):
        opts = dict(scope={"approval": "fixture"}, root=self.repo, f_checkpoint=self.ckpt, f_sha=self.fsha,
                    sources=("src.py",), external=(), cores=frozenset(os.sched_getaffinity(0)), min_nice=os.nice(0),
                    host_check=lambda: self.host,
                    faults=lambda: self.faults[0], clock=lambda: self.now[0])
        opts.update(kw)
        return A.AttemptRecorder(self.tmp / name, **opts)


class Recorder(RecorderBase):
    def test_open_binds_and_refuses(self):
        h = self.recorder().open()
        self.assertEqual(h["sources"]["src.py"], hashlib.sha256(b"x = 1\n").hexdigest())
        self.assertEqual(h["policy_f"]["sha256"], self.fsha)
        self.assertEqual(h["git"]["dirty"], "")
        with self.assertRaises(A.AttemptError):
            self.recorder().open()                                    # exists
        refusals = {
            "no_approval": dict(scope={}),
            "wrong_f": dict(f_sha="0" * 64),
            "missing_source": dict(sources=("absent.py",)),
            "cores": dict(cores=frozenset({999})),
            "nice": dict(min_nice=os.nice(0) + 1),
        }
        for name, kw in refusals.items():
            with self.assertRaises(A.AttemptError, msg=name):
                self.recorder(name, **kw).open()
            self.assertFalse((self.tmp / name).exists(), name)
        with self.assertRaises(A.AttemptError):
            A.AttemptRecorder(ROOT / "__attempt_probe__", scope={"approval": "x"}).open()
        self.assertFalse((ROOT / "__attempt_probe__").exists())
        (self.repo / "untracked.py").write_text("y = 1\n")             # a source in no commit
        with self.assertRaises(A.AttemptError):
            self.recorder("untracked", sources=("src.py", "untracked.py")).open()
        self.assertFalse((self.tmp / "untracked").exists())
        (self.repo / "src.py").write_text("x = 2\n")                  # tracked change
        with self.assertRaises(A.AttemptError):
            self.recorder("dirty").open()
        self.assertEqual(A.source_problems(self.repo, ("src.py",)), ["source src.py differs from HEAD"])
        with mock.patch.dict(os.environ, {"CUDA_VISIBLE_DEVICES": "0"}):
            with self.assertRaises(A.AttemptError):
                self.recorder("cuda", require_clean=False).open()

    def test_complete_attempt_feeds_the_pair_summary(self):
        r = self.recorder()
        r.open()
        for m in A.schedule():
            outcome = P.CENSORED if m["env"] % 3 == 0 else ("capture" if m["arm"] == "native" else "crash")
            r.append(record(m, outcome))
        self.assertIsNone(r.status)                                    # the caller finishes, after its checks
        r.finish()
        self.assertEqual(r.status, "COMPLETE")
        mains = A.main_records(self.tmp / "attempt01")
        self.assertEqual(len(mains), 40)
        windows = {(b, e): P.frame_window(208 if (b, e) == (70, 5) else 600) for b in (70, 205) for e in range(10)}
        summary = P.summarize_complete_pairs(mains, windows)
        self.assertEqual(summary["arms"]["native"]["planned"], 20)
        with self.assertRaises(A.AttemptError):
            r.append(record(A.schedule()[0]))                         # closed

    def test_order_is_enforced(self):
        r = self.recorder()
        r.open()
        plan = A.schedule()
        with self.assertRaises(A.AttemptError):
            r.append(record(plan[2]))                                 # main before smoke
        r.append(record(plan[0]))
        with self.assertRaises(A.AttemptError):
            r.append(record(plan[0]))                                 # rerun
        with self.assertRaises(A.AttemptError):
            r.append(record(dict(plan[1], arm="candidate")))          # wrong arm
        r.append(record(plan[1]))
        self.assertEqual(r.next, 2)

    def test_failures_end_the_attempt_and_list_unrun_members(self):
        cases = {"smoke": ("SMOKE_FAILED", 0), "member": ("MEMBER_NOT_VALID", 5)}
        for name, (reason, at) in cases.items():
            r = self.recorder(name)
            r.open()
            for m in A.schedule()[:at]:
                r.append(record(m))
            r.append(record(A.schedule()[at], status="HARNESS_INVALID"))
            final = json.loads((self.tmp / name / "final.json").read_text())
            self.assertEqual((final["status"], final["reason"]), ("ABORTED", reason))
            self.assertEqual(len(final["unrun"]), 42 - at - 1)
            with self.assertRaises(A.AttemptError):
                A.main_records(self.tmp / name)
        r = self.recorder("fault")
        r.open()
        r.append(record(A.schedule()[0]))
        self.faults[0] = 1
        r.append(record(A.schedule()[1]))
        final = json.loads((self.tmp / "fault" / "final.json").read_text())
        self.assertEqual(final["status"], "VOID_HOST_FAULT")

    def test_budgets(self):
        r = self.recorder()
        r.open()
        for m in A.schedule()[:3]:
            r.append(record(m))
        self.now[0] = 2000.0
        r.check_budget()                                              # positive: inside both budgets
        self.now[0] = 2401.0
        with self.assertRaises(A.AttemptError):
            r.check_budget()
        self.assertEqual(json.loads((self.tmp / "attempt01" / "final.json").read_text())["reason"],
                         "BUDGET_DENSITY_70")
        r2 = self.recorder("overall")
        self.now[0] = 0.0
        r2.open()
        self.now[0] = 5401.0
        with self.assertRaises(A.AttemptError):
            r2.check_budget()

    def test_tampering_and_truncation_detected(self):
        r = self.recorder()
        r.open()
        for m in A.schedule()[:4]:
            r.append(record(m))
        r.finish()
        d = self.tmp / "attempt01"
        self.assertEqual(json.loads((d / "final.json").read_text())["status"], "INCOMPLETE")
        lines, _ = A.read_ledger(d)                                    # positive control
        self.assertEqual(len(lines), 4)
        text = (d / "ledger.jsonl").read_text().splitlines()
        (d / "ledger.jsonl").write_text("\n".join([text[0], text[1].replace('"capture"', '"crash"')] + text[2:]) + "\n")
        with self.assertRaises(A.AttemptError):
            A.read_ledger(d)
        (d / "ledger.jsonl").write_text("\n".join(text[:3]) + "\n")
        with self.assertRaises(A.AttemptError):
            A.read_ledger(d)


class RecordRules(RecorderBase):
    def test_invalid_valid_records_stop_the_attempt(self):
        plan = A.schedule()
        cases = {"smoke_timeout": (0, dict(outcome="timeout"), "SMOKE_FAILED"),
                 "smoke_counts": (0, dict(commands_issued=31), "SMOKE_FAILED"),
                 "main_premature_censor": (2, dict(outcome=P.CENSORED, final_frame_index=40,
                                                   observation_checks=41, commands_issued=40,
                                                   final_sim_time_s=44.0, scheduled_elapsed_s=4.0),
                                           "RECORD_INVALID")}
        for name, (at, change, reason) in cases.items():
            r = self.recorder(name)
            r.open()
            for m in plan[:at]:
                r.append(record(m))
            line = r.append(dict(record(plan[at]), **change))
            self.assertTrue(line["record_problems"], name)
            final = json.loads((self.tmp / name / "final.json").read_text())
            self.assertEqual((final["status"], final["reason"]), ("ABORTED", reason), name)
        self.assertEqual(A.record_problems(record(plan[0])), [])      # positive control

    def test_chain_root_and_schedule_names(self):
        r = self.recorder()
        r.open()
        for m in A.schedule()[:3]:
            r.append(record(m))
        r.finish()
        d = self.tmp / "attempt01"
        A.read_ledger(d)
        header = (d / "attempt.json").read_text()
        (d / "attempt.json").write_text(header.replace('"fixture"', '"forged"'))
        with self.assertRaises(A.AttemptError):                        # the chain's root is the header's hash
            A.read_ledger(d)
        (d / "attempt.json").write_text(header)
        lines = (d / "ledger.jsonl").read_text().splitlines()
        first = json.loads(lines[0])
        first["name"] = A.schedule()[1]["name"]                         # wrong member name, chain recomputed
        prev = A.sha256_file(d / "attempt.json")
        rebuilt = []
        for i, text in enumerate([json.dumps(first, sort_keys=True)] + lines[1:]):
            obj = json.loads(text)
            obj["prev_sha256"] = prev
            t = json.dumps(obj, sort_keys=True)
            prev = hashlib.sha256(t.encode()).hexdigest()
            rebuilt.append(t)
        (d / "ledger.jsonl").write_text("\n".join(rebuilt) + "\n")
        final = json.loads((d / "final.json").read_text())
        final["last_record_sha256"] = prev
        (d / "final.json").write_text(json.dumps(final))
        with self.assertRaises(A.AttemptError):
            A.read_ledger(d)

    def test_nonfinite_record_is_kept_and_stops_the_attempt(self):
        r = self.recorder("nan")
        r.open()
        line = r.append(dict(record(A.schedule()[0]), min_dist_m=float("nan")))
        self.assertEqual(line["min_dist_m"], None)
        self.assertTrue(any("non-finite" in x for x in line["record_problems"]))
        lines, final = A.read_ledger(self.tmp / "nan")
        self.assertEqual((len(lines), final["reason"]), (1, "SMOKE_FAILED"))

    def test_fault_check_failure_still_writes_final(self):
        calls = {"n": 0}

        def faults():
            calls["n"] += 1
            if calls["n"] > 1:
                raise A.AttemptError("journal gone")
            return 0
        r = self.recorder("faults", faults=faults)
        r.open()
        r.append(record(A.schedule()[0]))
        final = json.loads((self.tmp / "faults" / "final.json").read_text())
        self.assertEqual((final["reason"], final["kernel_faults_end"]), ("FAULT_CHECK_ERROR", None))
        self.assertIn("journal gone", final["fault_check_error"])

    def test_busy_runner_cores_refuse_and_stop(self):
        self.host = (False, {"core_idle": {6: 0.1}})
        with self.assertRaises(A.AttemptError):
            self.recorder("busy").open()
        self.assertFalse((self.tmp / "busy").exists())
        self.host = (True, {})
        r = self.recorder("later")
        r.open()
        r.append(record(A.schedule()[0]))
        self.host = (False, {"core_idle": {6: 0.1}})
        with self.assertRaises(A.AttemptError):
            r.check_host()
        self.assertEqual(json.loads((self.tmp / "later" / "final.json").read_text())["reason"], "HOST_NOT_IDLE")
        idle = A.core_idle(seconds=0.2)                                  # positive control: the real reader
        self.assertEqual(set(idle), set(A.CORES))
        self.assertTrue(all(0.0 <= v <= 1.0 for v in idle.values()))

    def test_signals_are_held_over_the_ledger_line_and_final(self):
        """Review r4 Q1/Q2: a SIGTERM landing inside the write is delivered after the line, its chain update and
        final.json are complete."""
        import signal as sig

        class Stop(BaseException):
            pass

        def handler(signum, frame):
            raise Stop()
        old = sig.signal(sig.SIGTERM, handler)
        try:
            r = self.recorder("held")
            r.open()
            real = A.hashlib.sha256

            def sha_then_signal(*a, **k):
                os.kill(os.getpid(), sig.SIGTERM)                      # lands inside the held region
                return real(*a, **k)
            with mock.patch.object(A.hashlib, "sha256", sha_then_signal):
                with self.assertRaises(Stop):
                    r.append(record(A.schedule()[0]))
            self.assertEqual(r.next, 1)
            lines = (self.tmp / "held" / "ledger.jsonl").read_text().splitlines()
            self.assertEqual(r.prev_sha, hashlib.sha256(lines[0].encode()).hexdigest())
            calls = {"n": 0}

            def faults_then_signal():
                calls["n"] += 1
                os.kill(os.getpid(), sig.SIGTERM)
                return 0
            r.faults = faults_then_signal
            with self.assertRaises(Stop):
                r.abort("TEST")
            final = json.loads((self.tmp / "held" / "final.json").read_text())   # written despite the signal
            self.assertEqual((final["reason"], final["recorded"]), ("TEST", 1))
            A.read_ledger(self.tmp / "held")
        finally:
            sig.signal(sig.SIGTERM, old)

    def test_signals_are_held_with_other_threads_alive(self):
        """Review r5 MJ1-r5: with another thread alive the kernel may deliver the signal to it; the hold must still
        defer the handler until final.json is complete. Repeated, since delivery varies."""
        import signal as sig
        import threading as th

        class Stop(BaseException):
            pass

        def handler(signum, frame):
            raise Stop()
        stop = th.Event()
        idle = th.Thread(target=stop.wait, daemon=True)
        idle.start()
        old = sig.signal(sig.SIGTERM, handler)
        try:
            for i in range(8):
                r = self.recorder("threads%d" % i)
                r.open()

                def faults_then_signal():
                    os.kill(os.getpid(), sig.SIGTERM)
                    time.sleep(0.01)                                   # let any thread take it
                    return 0
                r.faults = faults_then_signal
                with self.assertRaises(Stop):
                    r.abort("TEST")
                final = json.loads((self.tmp / ("threads%d" % i) / "final.json").read_text())
                self.assertEqual(final["reason"], "TEST")
            self.assertIs(sig.getsignal(sig.SIGTERM), handler)          # restored after each hold
        finally:
            sig.signal(sig.SIGTERM, old)
            stop.set()
            idle.join()

    def test_remaining_guards(self):
        """Review r2/r3 surviving attempt mutants A10, A19, A20, A23, A24, A26, A27, A29."""
        plan = A.schedule()
        base = record(plan[0])
        self.assertEqual(A.record_problems(base), [])                    # positive control
        for change in ({"final_sim_time_s": 39.0},                          # A19: before the start
                       {"scheduled_elapsed_s": 3.1},                        # A20: not k * DT
                       {"recorded_frames": 0}, {"recorded_frames": 601}):   # A29: outside 1..600
            self.assertTrue(A.record_problems(dict(base, **change)), change)
        # A10: a tracked file that is not a listed source still makes the tree dirty
        (self.repo / "notes.txt").write_text("a\n")
        subprocess.run(["git", "-C", str(self.repo), "add", "notes.txt"], check=True, capture_output=True)
        subprocess.run(["git", "-C", str(self.repo), "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "n"],
                       check=True, capture_output=True)
        (self.repo / "notes.txt").write_text("b\n")
        with self.assertRaises(A.AttemptError):
            self.recorder("dirty_other").open()
        (self.repo / "notes.txt").write_text("a\n")
        # A26: an external source with local changes or missing refuses
        ext_tree = self.tmp / "ext"
        ext_tree.mkdir()
        subprocess.run(["git", "-C", str(ext_tree), "init", "-q"], check=True, capture_output=True)
        with self.assertRaises(A.AttemptError):
            self.recorder("ext_missing", external=((str(ext_tree), "replay.py"),)).open()
        (ext_tree / "replay.py").write_text("r\n")
        with self.assertRaises(A.AttemptError):                         # untracked counts as local change
            self.recorder("ext_dirty", external=((str(ext_tree), "replay.py"),)).open()
        # A27: only the Git-tree condition fails
        inside = self.repo / "attempt_inside"
        with self.assertRaises(A.AttemptError) as cm:
            A.AttemptRecorder(inside, scope={"approval": "x"}, root=self.repo, f_checkpoint=self.ckpt,
                              f_sha=self.fsha, sources=("src.py",), external=(),
                              cores=frozenset(os.sched_getaffinity(0)), min_nice=os.nice(0),
                              host_check=lambda: (True, {}), faults=lambda: 0).open()
        self.assertIn("Git work tree", str(cm.exception))
        self.assertFalse(inside.exists())
        # A23/A24: main_records refuses a changed fault count or a record with problems
        for name, mutate in (("faults_end", lambda f, lines: f.update(kernel_faults_end=1)),
                             ("problems", None)):
            r = self.recorder("mr_" + name)
            r.open()
            for m in plan:
                r.append(record(m))
            r.finish()
            d = self.tmp / ("mr_" + name)
            if name == "faults_end":
                final = json.loads((d / "final.json").read_text())
                mutate(final, None)
                (d / "final.json").write_text(json.dumps(final))
                with self.assertRaises(A.AttemptError):
                    A.main_records(d)
            else:
                self.assertEqual(len(A.main_records(d)), 40)              # positive control
                with mock.patch.object(A, "read_ledger", lambda dd: ([{"phase": "main", "record_problems": ["x"]}],
                                                                     {"status": "COMPLETE", "reason": None,
                                                                      "kernel_faults_start": 0,
                                                                      "kernel_faults_end": 0})):
                    with self.assertRaises(A.AttemptError):
                        A.main_records(d)

    def test_unreadable_kernel_log_is_an_error(self):
        class R:
            returncode, stdout = 1, b""
        with mock.patch.object(A.subprocess, "run", lambda *a, **k: R()):
            with self.assertRaises(A.AttemptError):
                A.kernel_faults()
        self.assertIsInstance(A.kernel_faults(), int)                   # positive control on this host


if __name__ == "__main__":
    unittest.main()
