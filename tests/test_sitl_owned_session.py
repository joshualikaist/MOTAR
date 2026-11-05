"""CPU lifecycle fixtures for the dev08 owned SITL session (tools/sitl_owned_session.py; motar-5md).

Fake gzserver/spawn/px4 processes (Python one-liners) stand in for the simulator: no PX4, Gazebo, GPU or network
peer is used, and only test-chosen free ports, a temporary lock and a temporary PX4-lock path are touched. Each
rejection has a positive control. Unrelated sentinels run the same command line as the owned fakes, so a pattern
kill would hit them.
"""
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import select
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "tools/sitl_owned_session.py"
SPEC = importlib.util.spec_from_file_location("sitl_owned_session", MODULE)
S = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(S)

UNIQUE = "mfg%d" % (os.getpid() % 10 ** 8)      # a process name used only by this test process
FAKE = r'''
import os, signal, sys, time
args = dict(a.split("=", 1) if "=" in a else (a, "1") for a in sys.argv[1:])
signal.signal(signal.SIGINT, signal.SIG_IGN if "ignore_int" in args else signal.default_int_handler)
def note(key, value):
    if "notes" in args:
        with open(args["notes"], "a") as f:
            f.write("%s=%s\n" % (key, value))
def forever():
    while True:
        time.sleep(0.2)
note("cwd", os.getcwd())
if "counter" in args:
    c = args["counter"]
    n = int(open(c).read()) if os.path.exists(c) else 0
    open(c, "w").write(str(n + 1))
    print("An instance of Gazebo is not running." if n < int(args["fails"]) else "spawned", flush=True)
    sys.exit(0)
if "exit" in args:
    sys.exit(int(args["exit"]))
if "grandchild" in args:
    pid = os.fork()
    if pid == 0:
        forever()
    note("grandchild", pid)
if "gc_alloc_mb" in args:
    pid = os.fork()
    if pid == 0:
        blob = bytearray(int(args["gc_alloc_mb"]) * 1024 * 1024)
        for i in range(0, len(blob), 4096):
            blob[i] = 1
        note("gc_ready", os.getpid())
        forever()
    note("grandchild", pid)
if "gc_fork_on_int" in args:
    # a child in the same session that, on SIGINT, forks a SIGINT-ignoring process and exits
    pid = os.fork()
    if pid == 0:
        def on_int(signum, frame):
            d = os.fork()
            if d == 0:
                signal.signal(signal.SIGINT, signal.SIG_IGN)
                forever()
            note("late", d)
            os._exit(0)
        signal.signal(signal.SIGINT, on_int)
        note("child", os.getpid())
        forever()
    note("grandchild", pid)
if "gc_escape_on_int" in args:
    # a session child that, on SIGINT, starts a setsid process, waits 0.3 s and exits: only re-sampling finds it
    pid = os.fork()
    if pid == 0:
        def on_int(signum, frame):
            d = os.fork()
            if d == 0:
                os.setsid()
                signal.signal(signal.SIGINT, signal.SIG_IGN)
                forever()
            note("late", d)
            time.sleep(0.3)
            os._exit(0)
        signal.signal(signal.SIGINT, on_int)
        note("child", os.getpid())
        forever()
    note("grandchild", pid)
if "escape" in args:
    time.sleep(float(args["escape"]))
    pid = os.fork()
    if pid == 0:
        os.setsid()
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        forever()
    note("escaped", pid)
    if "then_exit" in args:
        time.sleep(float(args["then_exit"]))
        os._exit(0)
if "alloc_mb" in args:
    blob = bytearray(int(args["alloc_mb"]) * 1024 * 1024)
    for i in range(0, len(blob), 4096):
        blob[i] = 1
note("ready", 1)
forever()
'''


def free_ports():
    out = []
    for proto, kind in (("udp", socket.SOCK_DGRAM), ("tcp", socket.SOCK_STREAM)):
        s = socket.socket(socket.AF_INET, kind)
        s.bind(("0.0.0.0", 0))
        out.append((proto, s.getsockname()[1]))
        s.close()
    return tuple(out)


def wait_for(cond, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def read_notes(path):
    try:
        lines = Path(path).read_text().splitlines()
    except FileNotFoundError:
        return {}
    return dict(line.split("=", 1) for line in lines if "=" in line)


def identity(pid):
    st = S.proc_stat(pid)
    return None if st is None else (pid, st["start"])


def alive(ident):
    return S.alive(ident[0], ident[1])


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="owned_session_"))
        self.allowed = self.tmp / "allowed"
        self.allowed.mkdir()
        self.lock = self.tmp / "locks" / "instance.lock"
        self.px4_lock = self.tmp / "px4_lock-0"
        self.ports = free_ports()
        self.env = dict(os.environ, PYTHONNOUSERSITE="1")
        self.children = []
        self.strays = []

    def tearDown(self):
        for p in self.children:
            if p.poll() is None:
                p.kill()
            p.wait()
        for ident in self.strays:
            if alive(ident):
                S.signal_verified(ident[0], ident[1], signal.SIGKILL)
        for journal in self.allowed.glob("*/launches.jsonl"):           # fallback: anything a session launched
            for e in (json.loads(line) for line in journal.read_text().splitlines()):
                if S.alive(e["pid"], e["start"]):
                    S.signal_verified(e["pid"], e["start"], signal.SIGKILL)
        mark = str(self.tmp).encode()                                    # last resort: fakes carry this test's path
        for name in os.listdir("/proc"):
            if not name.isdigit() or int(name) == os.getpid():
                continue
            try:
                cmd = Path("/proc/%s/cmdline" % name).read_bytes()
            except OSError:
                continue
            if mark in cmd:
                ident = identity(int(name))
                if ident is not None:
                    S.signal_verified(ident[0], ident[1], signal.SIGKILL)
        shutil.rmtree(self.tmp, ignore_errors=True)

    def opts(self, **kw):
        opts = dict(allowed_root=self.allowed, lock_path=self.lock, ports=self.ports, foreign_names=(UNIQUE,),
                    forbidden_roots=(), px4_lock=self.px4_lock, exclusive_udp=(self.ports[0][1],), deadline_s=30.0,
                    kill_grace_s=0.5, sample_s=0.05, include_self_rss=False)
        opts.update(kw)
        return opts

    def session(self, name="ep", **kw):
        return S.OwnedSession(self.allowed / name, **self.opts(**kw))

    def notes(self, tag):
        return str(self.tmp / ("notes_" + tag))

    def fake(self, tag, *args):
        return [sys.executable, "-c", FAKE, "notes=" + self.notes(tag), *args]

    def sentinel(self, tag, new_session=True):
        p = subprocess.Popen(self.fake(tag), env=self.env, start_new_session=new_session,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append(p)
        self.assertTrue(wait_for(lambda: "ready" in read_notes(self.notes(tag))))
        return p

    def ready(self, tag, key="ready"):
        self.assertTrue(wait_for(lambda: key in read_notes(self.notes(tag))), "fake %s never wrote %s" % (tag, key))
        return read_notes(self.notes(tag))

    def stray(self, pid):
        ident = identity(int(pid))
        self.assertIsNotNone(ident)
        self.strays.append(ident)
        return ident

    def receipt(self, name="ep"):
        return json.loads((self.allowed / name / "session.json").read_text())


class MembershipRules(unittest.TestCase):
    """members() on synthetic /proc tables: which processes the session may signal."""

    @staticmethod
    def row(ppid, sid, start, state="S"):
        return {"state": state, "ppid": ppid, "pgrp": sid, "sid": sid, "start": start, "rss_pages": 1}

    def test_live_leader_session_and_descendants(self):
        t = {100: self.row(1, 100, 50), 101: self.row(100, 100, 51), 102: self.row(101, 100, 52),
             103: self.row(102, 103, 53),                      # left the session, still a descendant
             200: self.row(1, 200, 54), 201: self.row(1, 1, 55)}
        live, _ = S.members(t, {100: 50}, {})
        self.assertEqual(set(live), {100, 101, 102, 103})

    def test_reused_leader_number_owns_nothing(self):
        t = {300: self.row(1, 300, 99), 301: self.row(300, 300, 100)}          # 300 is now another process
        self.assertEqual(S.members(t, {300: 60}, {})[0], {})
        t[305] = self.row(1, 305, 61)
        self.assertEqual(set(S.members(t, {300: 60}, {305: 61})[0]), {305})   # positive: a known escaped member

    def test_gone_leader_accepts_known_members_only(self):
        t = {501: self.row(1, 500, 80)}
        self.assertEqual(S.members(t, {500: 70}, {})[0], {})
        self.assertEqual(set(S.members(t, {500: 70}, {501: 80})[0]), {501})
        self.assertEqual(S.members(t, {500: 70}, {501: 81})[0], {})           # same pid, other start: not it

    def test_zombie_leader_keeps_session_but_is_not_live(self):
        t = {400: self.row(1, 400, 70, "Z"), 401: self.row(1, 400, 71)}
        live, found = S.members(t, {400: 70}, {})
        self.assertEqual(set(live), {401})
        self.assertEqual(set(found), {400, 401})


class Lifecycle(Base):
    def test_normal_exit_cleans_owned_tree_only(self):
        s1 = self.sentinel("s1", new_session=True)
        s2 = self.sentinel("s2", new_session=False)
        with self.session() as s:
            g = s.start("gzserver", self.fake("g", "grandchild"), env=self.env, cwd=s.episode_dir)
            p = s.start("px4", self.fake("p"), env=self.env, cwd=s.rootfs)
            gc = self.stray(self.ready("g")["grandchild"])
            self.assertEqual(self.ready("p")["cwd"], str(s.rootfs))
            s3 = self.sentinel("s3", new_session=True)          # started while the session runs
            s.check(required=("gzserver", "px4"))
            journal = [json.loads(x) for x in (s.episode_dir / "launches.jsonl").read_text().splitlines()]
            self.assertEqual([(e["role"], e["pid"], e["start"]) for e in journal],
                             [("gzserver", g["pid"], g["start_ticks"]), ("px4", p["pid"], p["start_ticks"])])
        owned = {(g["pid"], g["start_ticks"]), (p["pid"], p["start_ticks"]), gc}
        for ident in owned:
            self.assertFalse(alive(ident), ident)
        for sent in (s1, s2, s3):
            self.assertIsNone(sent.poll(), "an unrelated sentinel was terminated")
        r = self.receipt()
        self.assertEqual(r["cleanup"]["reason"], "normal")
        self.assertEqual((r["cleanup"]["leaked"], r["cleanup"]["unreaped_roles"], r["cleanup"]["errors"]),
                         ([], [], []))
        signalled = {(a["pid"], a["start"]) for a in r["cleanup"]["actions"]}
        self.assertEqual(signalled, owned)
        self.assertEqual({a["signal"] for a in r["cleanup"]["actions"]}, {int(signal.SIGINT)})
        for rec in (g, p):
            with self.assertRaises(ChildProcessError):
                os.waitpid(rec["pid"], os.WNOHANG)                  # reaped by the session
        self.assertTrue(all(v is not None for v in r["exit_codes"].values()))
        self.assertEqual(r["module_sha256_at_import"], r["module_sha256_at_exit"])
        self.assertIn("setsid", r["leak_detection"])
        self.assertFalse(S.leak_marker(self.lock).exists())
        with self.session("ep2"):                                   # the lock was released
            pass

    def test_launch_sequence_retries_spawn_until_ready(self):
        counter = str(self.tmp / "spawn_counter")
        with self.session() as s:
            world = s.write_world("w", "<sdf/>")
            plan = {"world_path": world,
                    "gzserver": {"argv": self.fake("g"), "cwd": s.episode_dir, "env": self.env},
                    "spawn": {"argv": self.fake("sp", "counter=" + counter, "fails=2"), "cwd": s.episode_dir,
                              "env": self.env},
                    "px4": {"argv": self.fake("p"), "cwd": s.rootfs, "env": self.env}}
            spawned = S.launch_px4_gazebo(s, plan, spawn_attempts=5, spawn_retry_s=0.05, spawn_timeout_s=10)
            self.assertEqual(spawned, "spawn_02")
            self.assertEqual(s._order, ["gzserver", "spawn_00", "spawn_01", "spawn_02", "px4"])
            self.assertEqual(self.ready("p")["cwd"], str(s.rootfs))
        # rejections: Gazebo never ready, or the spawn fails without the phrase -> bounded failure, gzserver cleaned
        for name, spawn_args in (("never_ready", ("counter=" + counter + "b", "fails=99")), ("exit1", ("exit=1",))):
            with self.assertRaises(S.SessionError):
                with self.session(name) as s:
                    world = s.write_world("w", "<sdf/>")
                    plan = {"world_path": world,
                            "gzserver": {"argv": self.fake("g_" + name), "cwd": s.episode_dir, "env": self.env},
                            "spawn": {"argv": self.fake("sp_" + name, *spawn_args), "cwd": s.episode_dir,
                                      "env": self.env},
                            "px4": {"argv": self.fake("p_" + name), "cwd": s.rootfs, "env": self.env}}
                    S.launch_px4_gazebo(s, plan, spawn_attempts=3, spawn_retry_s=0.05, spawn_timeout_s=10)
            r = self.receipt(name)
            self.assertEqual([x["role"] for x in r["launches"]], ["gzserver", "spawn_00", "spawn_01", "spawn_02"])
            gz = r["launches"][0]
            self.assertFalse(S.alive(gz["pid"], gz["start_ticks"]))
            self.assertEqual(r["cleanup"]["leaked"], [])

    def test_launch_refuses_unwritten_world(self):
        with self.assertRaises(S.SessionError):
            with self.session() as s:
                plan = {"world_path": s.episode_dir / "world" / "w.world",
                        "gzserver": {"argv": self.fake("g"), "cwd": s.episode_dir, "env": self.env},
                        "spawn": {"argv": self.fake("sp"), "cwd": s.episode_dir, "env": self.env},
                        "px4": {"argv": self.fake("p"), "cwd": s.rootfs, "env": self.env}}
                S.launch_px4_gazebo(s, plan)
        self.assertEqual(self.receipt()["launches"], [])

    def test_sigint_ignoring_process_is_killed_after_grace(self):
        with self.session() as s:
            p = s.start("px4", self.fake("p", "ignore_int"), env=self.env, cwd=s.rootfs)
            self.ready("p")
        self.assertFalse(alive((p["pid"], p["start_ticks"])))
        sigs = [a["signal"] for a in self.receipt()["cleanup"]["actions"]]
        self.assertEqual(sigs, [int(signal.SIGINT), int(signal.SIGKILL)])

    def test_escaped_descendant_is_tracked_and_cleaned(self):
        with self.session() as s:
            p = s.start("px4", self.fake("p", "escape=0.3", "then_exit=0.4"), env=self.env, cwd=s.rootfs)
            esc = self.stray(self.ready("p", "escaped")["escaped"])
            self.assertTrue(wait_for(lambda: s._rc("px4") is not None))
            time.sleep(0.2)
            # non-vacuity: the session-id scan alone does not see it (own session, parent gone)
            self.assertNotIn(esc[0], S.members(S.snapshot(), s._leaders(), {})[0])
            self.assertIn(esc[0], S.members(S.snapshot(), s._leaders(), dict(s._known))[0])
        self.assertFalse(alive(esc))
        acts = [a for a in self.receipt()["cleanup"]["actions"] if a["pid"] == esc[0]]
        self.assertEqual([a["signal"] for a in acts], [int(signal.SIGINT), int(signal.SIGKILL)])
        self.assertEqual(self.receipt()["cleanup"]["leaked"], [])

    def test_process_forked_during_shutdown_is_found(self):
        """Review r1 P4: the leader exits on SIGINT while its session child forks a SIGINT-ignoring process.
        The leader stays unreaped until the final scan, so its session id still finds the late process."""
        with self.session() as s:
            s.start("px4", self.fake("p", "gc_fork_on_int"), env=self.env, cwd=s.rootfs)
            self.ready("p")
            self.ready("p", "child")
        late = int(read_notes(self.notes("p"))["late"])
        ident = identity(late)
        if ident is not None and S.is_live(S.proc_stat(late)):
            self.strays.append(ident)                                    # never outlives the test
        self.assertFalse(S.is_live(S.proc_stat(late)))
        r = self.receipt()
        self.assertEqual(r["cleanup"]["leaked"], [])
        self.assertIn((late, int(signal.SIGKILL)), [(a["pid"], a["signal"]) for a in r["cleanup"]["actions"]])

    def test_escape_during_cleanup_is_found_by_resampling(self):
        """Review r3 X18: a process that leaves its session during the cleanup grace is found by the grace wait's
        re-sampling (sample_s 0.05 here)."""
        with self.session(kill_grace_s=1.0) as s:
            s.start("px4", self.fake("p", "gc_escape_on_int"), env=self.env, cwd=s.rootfs)
            self.ready("p", "child")
        late = int(read_notes(self.notes("p"))["late"])
        ident = identity(late)
        if ident is not None and S.is_live(S.proc_stat(late)):
            self.strays.append(ident)
        self.assertFalse(S.is_live(S.proc_stat(late)))
        self.assertIn(late, [a["pid"] for a in self.receipt()["cleanup"]["actions"]])

    def test_exception_cleans_up_and_propagates(self):
        sent = self.sentinel("s")
        with self.assertRaises(ValueError):
            with self.session() as s:
                p = s.start("gzserver", self.fake("g"), env=self.env, cwd=s.episode_dir)
                self.ready("g")
                raise ValueError("inside the session")
        self.assertFalse(alive((p["pid"], p["start_ticks"])))
        self.assertIsNone(sent.poll())
        r = self.receipt()
        self.assertEqual(r["cleanup"]["reason"], "exception")
        self.assertIn("ValueError", r["exception"])
        with self.session("ep2"):
            pass

    def test_deadline_aborts_through_owned_cleanup(self):
        sent = self.sentinel("s")
        t0 = time.monotonic()
        with self.assertRaises(S.SessionAborted):
            with self.session(deadline_s=0.6) as s:
                p = s.start("px4", self.fake("p"), env=self.env, cwd=s.rootfs)
                s.sleep(20)
        self.assertLess(time.monotonic() - t0, 10)
        self.assertFalse(alive((p["pid"], p["start_ticks"])))
        self.assertIsNone(sent.poll())
        r = self.receipt()
        self.assertEqual((r["cleanup"]["reason"], r["watchdog"]["abort"]), ("timeout", "timeout"))
        with self.session("ep2"):
            pass

    def test_rss_cap_covers_the_tree_and_the_runner(self):
        with self.session("ok", rss_cap_bytes=2 * 1024 ** 3) as s:
            s.start("px4", self.fake("p0", "alloc_mb=96"), env=self.env, cwd=s.rootfs)
            self.ready("p0")
            s.sleep(0.4)
        self.assertGreater(self.receipt("ok")["watchdog"]["peak_rss_bytes"], 90 * 1024 ** 2)
        self.assertIsNone(self.receipt("ok")["watchdog"]["abort"])
        tiny = [shutil.which("sleep"), "30"]                       # a child of about 1 MB
        cases = {"leader": dict(argv=self.fake("p_leader", "alloc_mb=96"), cap=48 * 1024 ** 2, self_rss=False),
                 "grandchild": dict(argv=self.fake("p_gc", "gc_alloc_mb=96"), cap=48 * 1024 ** 2, self_rss=False),
                 "runner": dict(argv=tiny, cap=8 * 1024 ** 2, self_rss=True)}   # this test process > 8 MB
        for name, c in cases.items():
            with self.assertRaises(S.SessionAborted, msg=name):
                with self.session(name, rss_cap_bytes=c["cap"], include_self_rss=c["self_rss"]) as s:
                    p = s.start("px4", c["argv"], env=self.env, cwd=s.rootfs)
                    s.sleep(20)
            self.assertFalse(alive((p["pid"], p["start_ticks"])), name)
            self.assertEqual(self.receipt(name)["cleanup"]["reason"], "rss_cap", name)
        with self.session("runner_off", rss_cap_bytes=8 * 1024 ** 2, include_self_rss=False) as s:  # control
            s.start("px4", tiny, env=self.env, cwd=s.rootfs)
            s.sleep(0.4)

    def test_unexpected_exit_is_reported_without_reaping(self):
        with self.session() as s:
            s.start("gzserver", self.fake("g"), env=self.env, cwd=s.episode_dir)
            p = s.start("px4", self.fake("p", "exit=3"), env=self.env, cwd=s.rootfs)
            self.ready("g")
            s.check(required=("gzserver",))                         # positive control
            self.assertTrue(wait_for(lambda: s._rc("px4") is not None))
            with self.assertRaises(S.ProcessExited):
                s.check(required=("gzserver", "px4"))
            self.assertEqual(S.proc_stat(p["pid"])["state"], "Z")   # observed, still unreaped
        self.assertEqual(self.receipt()["exit_codes"]["px4"], 3)

    def test_start_refusals(self):
        with self.session() as s:
            s.start("px4", self.fake("p"), env=self.env, cwd=s.rootfs)
            with self.assertRaises(S.SessionError):
                s.start("px4", self.fake("p2"), env=self.env, cwd=s.rootfs)
            with self.assertRaises(S.PathRefused):
                s.start("other", self.fake("o"), env=self.env, cwd=self.tmp)
            for bad in (["/x/y/sitl_run.sh"], ["pkill", "-x", "gazebo"], ["killall", "gzserver"], [],
                        ["bash", "-c", "pkill -x gazebo"], ["sh", "-c", "true;killall gzserver"]):
                with self.assertRaises(S.SessionError, msg=bad):
                    s.start("bad", bad, env=self.env, cwd=s.episode_dir)
            s.write_world("w", "<sdf/>")
            with self.assertRaises(FileExistsError):
                s.write_world("w", "<sdf/>")
            with self.assertRaises(S.PathRefused):
                s.path("..", "escape")
        self.assertEqual([x["role"] for x in self.receipt()["launches"]], ["px4"])

    def test_identity_mismatch_is_never_signalled(self):
        sent = self.sentinel("s")
        wrong = S.proc_stat(sent.pid)["start"] + 1

        class Ghost:
            returncode = None

            def poll(self):
                return None

            def wait(self, timeout=None):
                raise AssertionError("must not wait on a refused identity")

        with self.assertRaises(S.CleanupError):
            with self.session() as s:
                s._procs["ghost"] = {"popen": Ghost(), "pid": sent.pid, "start": wrong, "rc": None,
                                     "record": {"role": "ghost", "argv": ["ghost"], "pid": sent.pid}}
                s._order.append("ghost")
        self.assertIsNone(sent.poll())
        r = self.receipt()
        self.assertNotIn(sent.pid, [a["pid"] for a in r["cleanup"]["actions"]])
        self.assertEqual(r["cleanup"]["refused_identity"][0]["pid"], sent.pid)

    def test_survivor_is_a_cleanup_error_and_blocks_the_next_session(self):
        """A process the signals cannot stop: CleanupError, leaked and unreaped reported, marker blocks entry."""
        real = S.signal_verified
        immune = {}

        def fake_signal(pid, start, sig):
            return "sent" if (pid, start) in immune.items() else real(pid, start, sig)

        with mock.patch.object(S, "signal_verified", fake_signal):
            with self.assertRaises(S.CleanupError):
                with self.session() as s:
                    p = s.start("px4", self.fake("p"), env=self.env, cwd=s.rootfs)
                    self.ready("p")
                    immune[p["pid"]] = p["start_ticks"]
        ident = (p["pid"], p["start_ticks"])
        self.strays.append(ident)
        self.assertTrue(alive(ident))
        r = self.receipt()
        self.assertEqual(r["cleanup"]["leaked"], [{"pid": ident[0], "start": ident[1]}])
        self.assertEqual(r["cleanup"]["unreaped_roles"], ["px4"])
        marker = json.loads(S.leak_marker(self.lock).read_text())
        self.assertEqual((marker["state"], marker["leaked"][0]["pid"]), ("leaked", ident[0]))
        with self.assertRaises(S.LockBusy):
            with self.session("next"):
                pass
        self.assertFalse(os.path.lexists(self.allowed / "next"))
        S.signal_verified(ident[0], ident[1], signal.SIGKILL)
        self.assertTrue(wait_for(lambda: not alive(ident)))
        with self.session("after"):                                 # positive: marker cleared once nothing lives
            pass
        self.assertFalse(S.leak_marker(self.lock).exists())
        self.assertEqual(len(list(self.lock.parent.glob(self.lock.name + ".leaked.cleared-*"))), 1)

    def test_watchdog_error_aborts_and_is_reported(self):
        with self.assertRaises(S.CleanupError):
            with self.session() as s:
                p = s.start("px4", self.fake("p"), env=self.env, cwd=s.rootfs)
                self.ready("p")
                real = S.snapshot
                calls = {"n": 0}

                def failing():
                    if threading.current_thread() is s._watchdog and calls["n"] == 0:
                        calls["n"] += 1
                        raise PermissionError("simulated /proc failure")
                    return real()
                with mock.patch.object(S, "snapshot", failing):
                    self.assertTrue(wait_for(lambda: s._abort == "watchdog_error"))
                with self.assertRaises(S.SessionAborted):
                    s.sleep(1)
        self.assertFalse(alive((p["pid"], p["start_ticks"])))
        r = self.receipt()
        self.assertIn("PermissionError", r["watchdog"]["error"])
        self.assertEqual(r["cleanup"]["leaked"], [])

    def test_dead_watchdog_refuses_work(self):
        with self.assertRaises(S.SessionAborted):
            with self.session() as s:
                s._stop.set()
                s._watchdog.join(timeout=5)
                s.sleep(0.01)

    def test_udp_contention_during_the_session_aborts(self):
        port = self.ports[0][1]
        socks = []
        with self.assertRaises(S.SessionAborted):
            with self.session() as s:
                for _ in range(2):                                   # two SO_REUSEADDR links share the port
                    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    u.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    u.bind(("0.0.0.0", port))
                    socks.append(u)
                s.sleep(5)
        for u in socks:
            u.close()
        self.assertEqual(self.receipt()["watchdog"]["abort"], "port_contention")
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)       # positive: one socket is not contention
        with self.session("one") as s:
            u.bind(("0.0.0.0", port))
            s.sleep(0.3)
        u.close()

    def test_signal_verified(self):
        p = subprocess.Popen(self.fake("v"), env=self.env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append(p)
        self.ready("v")
        start = S.proc_stat(p.pid)["start"]
        self.assertEqual(S.signal_verified(p.pid, start + 1, signal.SIGKILL), "refused_identity")
        time.sleep(0.1)
        self.assertIsNone(p.poll())
        self.assertEqual(S.signal_verified(p.pid, start, signal.SIGKILL), "sent")
        p.wait(timeout=5)
        self.assertEqual(S.signal_verified(p.pid, start, signal.SIGKILL), "gone")

    def test_reaped_pid_pinned_as_session_id_is_gone_not_an_error(self):
        """Review r1 P2: pidfd_open gives EINVAL for a reaped pid still used as a session id."""
        q = subprocess.Popen([sys.executable, "-c", "import os,time\nif os.fork()==0:\n    time.sleep(30)\n"],
                             env=self.env, start_new_session=True)
        qstart = S.proc_stat(q.pid)["start"]
        q.wait(timeout=5)
        self.assertTrue(wait_for(lambda: [p for p, st in S.snapshot().items() if st["sid"] == q.pid]))
        kids = [p for p, st in S.snapshot().items() if st["sid"] == q.pid and p != q.pid]
        self.assertEqual(len(kids), 1)
        kid = self.stray(kids[0])
        self.assertEqual(S.signal_verified(q.pid, qstart, signal.SIGKILL), "gone")
        self.assertEqual(S.signal_verified(kid[0], kid[1], signal.SIGKILL), "sent")


class SessionEdges(Base):
    def test_signal_handlers_are_restored(self):
        before = (signal.getsignal(signal.SIGTERM), signal.getsignal(signal.SIGHUP))
        with self.session() as s:
            self.assertNotEqual(signal.getsignal(signal.SIGTERM), before[0])
            s.start("px4", self.fake("p"), env=self.env, cwd=s.rootfs)
        self.assertEqual((signal.getsignal(signal.SIGTERM), signal.getsignal(signal.SIGHUP)), before)
        with self.assertRaises(ValueError):
            with self.session("ep2"):
                raise ValueError("x")
        self.assertEqual((signal.getsignal(signal.SIGTERM), signal.getsignal(signal.SIGHUP)), before)
        self.assertFalse(issubclass(S.SessionTerminated, Exception))

    def test_internal_cleanup_error_writes_the_receipt_and_marker(self):
        with self.assertRaises(S.CleanupError):
            with self.session() as s:
                p = s.start("px4", self.fake("p"), env=self.env, cwd=s.rootfs)
                self.ready("p")
                real = s._terminate
                calls = {"n": 0}

                def broken(why):
                    calls["n"] += 1
                    if calls["n"] == 1:
                        raise RuntimeError("simulated cleanup bug")
                    return real(why)
                s._terminate = broken
        ident = (p["pid"], p["start_ticks"])
        self.strays.append(ident)
        r = self.receipt()
        self.assertIn("simulated cleanup bug", r["cleanup"]["internal_error"])
        self.assertTrue(alive(ident))
        self.assertEqual(json.loads(S.leak_marker(self.lock).read_text())["state"], "leaked")
        with self.assertRaises(S.LockBusy):
            with self.session("next"):
                pass
        S.signal_verified(ident[0], ident[1], signal.SIGKILL)
        self.assertTrue(wait_for(lambda: not alive(ident)))
        with self.session("after"):
            pass

    def test_enter_failure_restores_handlers_and_lock(self):
        before = signal.getsignal(signal.SIGTERM)
        s = self.session()
        with mock.patch.object(S.threading.Thread, "start", side_effect=RuntimeError("no threads")):
            with self.assertRaises(RuntimeError):
                s.__enter__()
        self.assertEqual(signal.getsignal(signal.SIGTERM), before)
        with self.session("ep2"):                                      # the lock was released
            pass

    def test_marker_is_checked_after_contention_and_unreadable_marker_refuses(self):
        marker = S.leak_marker(self.lock)
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.write_text(json.dumps({"state": "leaked", "boot_id": S.boot_id(), "leaked": []}))
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        u.bind(("0.0.0.0", self.ports[0][1]))
        with self.assertRaises(S.Contention):                          # refused before the marker is touched
            with self.session("a"):
                pass
        u.close()
        self.assertTrue(marker.exists())
        with self.session("b"):                                         # positive: a dead marker is cleared
            pass
        marker.write_text("")
        with self.assertRaises(S.LockBusy):
            with self.session("c"):
                pass

    def test_reap_journal_reports_a_survivor(self):
        p = subprocess.Popen(self.fake("r"), env=self.env, start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append(p)
        self.ready("r")
        ep = self.allowed / "journal_only"
        ep.mkdir()
        start = S.proc_stat(p.pid)["start"]
        (ep / "launches.jsonl").write_text(json.dumps({"role": "px4", "pid": p.pid, "start": start,
                                                       "boot_id": S.boot_id(), "argv": []}) + "\n")
        with mock.patch.object(S, "signal_verified", lambda pid, st, sig: "sent"):   # signals that do nothing
            report = S.reap_journal(ep, kill_grace_s=0.2, sample_s=0.05)
        self.assertEqual(report["alive_after"], [p.pid])
        self.assertIsNone(p.poll())

    def test_udp_count_includes_ipv6(self):
        port = self.ports[0][1]
        self.assertEqual(S.udp_bound_count(port), 0)
        u = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
        u.bind(("::", port))
        try:
            self.assertEqual(S.udp_bound_count(port), 1)
        finally:
            u.close()

    def test_quoted_kill_commands_are_refused(self):
        for bad in (["python3", "-c", "import os; os.system('pkill -x gazebo')"], ["sh", "-c", '"killall" gzserver']):
            with self.assertRaises(S.SessionError, msg=bad):
                S.validate_argv(bad)
        S.validate_argv([sys.executable, "-c", FAKE])                   # positive: the fakes pass


class RunnerTermination(Base):
    RUNNER = r'''
import importlib.util, json, os, signal, subprocess, sys, time
spec = importlib.util.spec_from_file_location("s", sys.argv[1]); S = importlib.util.module_from_spec(spec)
spec.loader.exec_module(S)
opts = json.loads(sys.argv[3])
opts["ports"] = [tuple(p) for p in opts["ports"]]
mode = sys.argv[5]
argv = json.loads(sys.argv[4])
if mode == "start_signal":                       # a SIGTERM lands between Popen and the journal write
    real = subprocess.Popen
    def popen(*a, **k):
        p = real(*a, **k)
        os.kill(os.getpid(), signal.SIGTERM)
        return p
    S.subprocess.Popen = popen
episodes = 3 if mode == "multi" else 1
for i in range(episodes):
    try:                                          # an ordinary wrapper that would swallow an Exception
        with S.OwnedSession(sys.argv[2] + ("_%d" % i if mode == "multi" else ""), **opts) as s:
            s.start("px4", argv, env=dict(os.environ), cwd=s.rootfs)
            print("ready", flush=True)
            if mode == "close":
                time.sleep(0.3)
                print("closing", flush=True)
            else:
                time.sleep(60)
    except Exception as e:
        print("swallowed", type(e).__name__, flush=True)
print("finished", flush=True)
'''

    def runner(self, name, mode="sleep", child_args=(), grace=0.5):
        opts = self.opts(kill_grace_s=grace)
        opts = {k: (str(v) if isinstance(v, Path) else v) for k, v in opts.items()}
        opts["forbidden_roots"] = []
        proc = subprocess.Popen([sys.executable, "-c", self.RUNNER, str(MODULE), str(self.allowed / name),
                                 json.dumps(opts), json.dumps(self.fake(name, *child_args)), mode], env=self.env,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.children.append(proc)
        return proc

    def wait_line(self, proc, want, timeout=20):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            ready, _, _ = select.select([proc.stdout], [], [], max(0.0, end - time.monotonic()))
            if not ready:
                break
            line = proc.stdout.readline().strip()
            if line == want:
                return True
            if not line:
                break
        return False

    def journal_child(self, epdir):
        journal = json.loads((epdir / "launches.jsonl").read_text().splitlines()[0])
        return (journal["pid"], journal["start"])

    def finish(self, proc):
        out, _ = proc.communicate(timeout=30)
        return proc.returncode, out.decode()

    def test_sigterm_and_sighup_run_the_cleanup(self):
        for name, sig in (("term", signal.SIGTERM), ("hup", signal.SIGHUP)):
            proc = self.runner(name)
            self.assertTrue(self.wait_line(proc, b"ready"))
            child = self.journal_child(self.allowed / name)
            proc.send_signal(sig)
            rc, out = self.finish(proc)
            self.assertEqual(rc, -int(sig), name)                      # re-delivered to the default handler
            self.assertNotIn("finished", out)
            self.assertFalse(alive(child), name)
            r = self.receipt(name)
            self.assertEqual(r["cleanup"]["reason"], "terminated")
            self.assertEqual(r["signals"][0]["signal"], int(sig))
            self.assertEqual(r["cleanup"]["leaked"], [])

    def test_a_multi_episode_runner_stops(self):
        """Review r2 R2-M1: the request is not consumed by one episode or swallowed by `except Exception`."""
        proc = self.runner("multi", mode="multi")
        self.assertTrue(self.wait_line(proc, b"ready"))
        child = self.journal_child(self.allowed / "multi_0")
        proc.send_signal(signal.SIGTERM)
        rc, out = self.finish(proc)
        self.assertEqual(rc, -int(signal.SIGTERM))
        self.assertNotIn("swallowed", out)
        self.assertFalse(os.path.lexists(self.allowed / "multi_1"))    # no further episode started
        self.assertFalse(alive(child))
        self.assertEqual(self.receipt("multi_0")["cleanup"]["reason"], "terminated")

    def test_a_signal_during_cleanup_is_delivered_after_it(self):
        proc = self.runner("close", mode="close", child_args=("ignore_int",), grace=1.0)
        self.assertTrue(self.wait_line(proc, b"ready"))
        child = self.journal_child(self.allowed / "close")
        self.assertTrue(self.wait_line(proc, b"closing"))
        time.sleep(0.4)                                                # inside the 1 s SIGINT grace
        proc.send_signal(signal.SIGTERM)
        rc, out = self.finish(proc)
        self.assertEqual(rc, -int(signal.SIGTERM))
        r = self.receipt("close")
        self.assertTrue(r["signals"] and r["signals"][0]["during_cleanup"])
        self.assertEqual(r["cleanup"]["leaked"], [])
        self.assertIn(int(signal.SIGKILL), [a["signal"] for a in r["cleanup"]["actions"]])   # cleanup completed
        self.assertFalse(alive(child))

    def test_a_signal_inside_start_is_deferred_until_recorded(self):
        proc = self.runner("startsig", mode="start_signal")
        rc, out = self.finish(proc)
        self.assertEqual(rc, -int(signal.SIGTERM))
        child = self.journal_child(self.allowed / "startsig")             # journalled despite the signal
        self.assertFalse(alive(child))
        r = self.receipt("startsig")
        self.assertEqual([x["role"] for x in r["launches"]], ["px4"])
        self.assertTrue(r["signals"][0]["deferred"])
        self.assertEqual(r["cleanup"]["leaked"], [])

    def test_sigkill_leaves_a_journal_that_reaps_by_identity(self):
        sent = self.sentinel("s")
        for name, args in (("kill", ()), ("kill_ignore", ("ignore_int",)), ("kill_fork", ("gc_fork_on_int",)),
                           ("kill_escape", ("gc_escape_on_int",))):
            proc = self.runner(name, child_args=args)
            self.assertTrue(self.wait_line(proc, b"ready"))
            self.ready(name)
            child = self.journal_child(self.allowed / name)
            proc.kill()
            proc.wait(timeout=10)
            proc.stdout.close()
            proc.stderr.close()
            self.strays.append(child)
            self.assertTrue(alive(child))                              # the runner died; its launch did not
            self.assertFalse((self.allowed / name / "session.json").exists())
            with self.assertRaises(S.LockBusy):                         # the open marker names the journal
                with self.session(name + "_next"):
                    pass
            report = S.reap_journal(self.allowed / name, kill_grace_s=0.5, sample_s=0.05)
            self.assertEqual(report["alive_at_start"][0], child[0])
            self.assertEqual(report["alive_after"], [], name)
            self.assertFalse(alive(child))
            if name == "kill_ignore":
                self.assertIn(int(signal.SIGKILL), [a["signal"] for a in report["actions"]])
            if name == "kill_escape":                                # review r3 X20: found by the wait's re-scan
                late = int(read_notes(self.notes(name))["late"])
                self.assertFalse(S.is_live(S.proc_stat(late)))
            if name == "kill_fork":
                late = int(read_notes(self.notes(name))["late"])         # forked on SIGINT, then found
                self.assertIn(late, [a["pid"] for a in report["actions"]])
                self.assertFalse(S.is_live(S.proc_stat(late)))
            with self.session(name + "_after"):                        # positive: marker cleared
                pass
        self.assertIsNone(sent.poll())


class Preconditions(Base):
    def assert_refused(self, exc, name="ep", **kw):
        with self.assertRaises(exc):
            with self.session(name, **kw):
                self.fail("entered a refused session")
        self.assertFalse(os.path.lexists(self.allowed / name))

    def test_existing_or_misplaced_directory_refused(self):
        (self.allowed / "ep").mkdir()
        with self.assertRaises(S.PathRefused):
            with self.session("ep"):
                pass
        self.assertEqual(list((self.allowed / "ep").iterdir()), [])
        for bad in (Path("relative/ep"), self.tmp / "outside" / "ep", self.allowed / "missing" / "ep"):
            with self.assertRaises(S.PathRefused):
                with S.OwnedSession(bad, **self.opts()):
                    pass
            self.assertFalse(os.path.lexists(bad))
        with self.session("fine"):                                  # positive control
            pass

    def test_git_tree_and_forbidden_root_refused(self):
        repo = self.allowed / "repo"
        (repo / ".git").mkdir(parents=True)                         # a temporary work tree, not the real one
        with self.assertRaises(S.PathRefused):
            with S.OwnedSession(repo / "ep", **self.opts()):
                pass
        self.assertFalse(os.path.lexists(repo / "ep"))
        (self.allowed / "px4").mkdir()
        with self.assertRaises(S.PathRefused):
            with S.OwnedSession(self.allowed / "px4" / "ep", **self.opts(forbidden_roots=(self.allowed / "px4",))):
                pass
        self.assertFalse(os.path.lexists(self.allowed / "px4" / "ep"))
        self.assertIsNotNone(S.git_tree_of(ROOT / "tools"))        # the check sees this repository

    def test_lock_contention(self):
        with self.session("a"):
            self.assert_refused(S.LockBusy, "b")
        holder = subprocess.Popen(
            [sys.executable, "-c", "import fcntl,os,sys,time\nfd=os.open(sys.argv[1],os.O_RDWR|os.O_CREAT)\n"
             "end=time.time()+10\nwhile True:\n    try:\n        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB);break\n"
             "    except BlockingIOError:\n        assert time.time()<end; time.sleep(0.05)\n"
             "print('held',flush=True);time.sleep(30)", str(self.lock)],
            stdout=subprocess.PIPE, env=self.env)
        self.children.append(holder)
        ready, _, _ = select.select([holder.stdout], [], [], 15)
        self.assertTrue(ready, "holder never got the lock")
        self.assertEqual(holder.stdout.readline().strip(), b"held")
        self.assert_refused(S.LockBusy, "c")
        holder.kill()
        holder.wait()
        holder.stdout.close()
        with self.session("d"):                                     # positive control
            pass

    def test_busy_ports_refused(self):
        (udp_port, tcp_port) = (self.ports[0][1], self.ports[1][1])
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        u.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)    # as pymavlink's udpin
        u.bind(("127.0.0.1", udp_port))
        self.assert_refused(S.Contention, "a")
        u.close()
        t = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        t.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        t.bind(("0.0.0.0", tcp_port))
        t.listen(1)
        self.assert_refused(S.Contention, "b")
        # positive control: a TIME_WAIT connection behind a SO_REUSEADDR listener does not count
        c = socket.create_connection(("127.0.0.1", tcp_port))
        a, _ = t.accept()
        a.close()                                                   # server side closes first -> TIME_WAIT on it
        time.sleep(0.05)
        c.close()
        t.close()
        self.assertEqual(S.busy_ports([("tcp", tcp_port)]), [])
        with self.session("c"):
            pass

    def test_foreign_simulator_process_refused(self):
        exe = self.tmp / UNIQUE
        os.symlink(shutil.which("sleep"), exe)
        p = subprocess.Popen([str(exe), "30"])
        self.children.append(p)
        self.assertTrue(wait_for(lambda: Path("/proc/%d/comm" % p.pid).read_text().strip() == UNIQUE))
        self.assert_refused(S.Contention, "a")
        p.kill()
        p.wait()
        with self.session("b"):                                     # positive control
            pass

    def test_px4_instance_lock_refused(self):
        self.px4_lock.write_text("")
        self.assertIsNone(S.fcntl_lock_holder(self.px4_lock))
        holder = subprocess.Popen(
            [sys.executable, "-c", "import fcntl,os,sys,time\nfd=os.open(sys.argv[1],os.O_RDWR)\n"
             "fcntl.lockf(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)\nprint('held',flush=True);time.sleep(30)",
             str(self.px4_lock)], stdout=subprocess.PIPE, env=self.env)
        self.children.append(holder)
        ready, _, _ = select.select([holder.stdout], [], [], 15)
        self.assertTrue(ready)
        self.assertEqual(holder.stdout.readline().strip(), b"held")
        self.assertEqual(S.fcntl_lock_holder(self.px4_lock), holder.pid)
        self.assert_refused(S.Contention, "a")
        holder.kill()
        holder.wait()
        holder.stdout.close()
        with self.session("b"):                                     # positive control
            pass


class LaunchPlan(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="owned_plan_"))
        self.px4 = self.tmp / "PX4"
        self.build = self.px4 / "build/px4_sitl_default"
        self.models = self.px4 / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
        files = {self.build / "bin/px4": "px4", self.build / "etc/init.d-posix/rcS": "rcS-installed",
                 self.build / "etc/init.d-posix/px4-rc.mavlink": "mavlink",
                 self.build / "etc/init.d-posix/px4-rc.simulator": "simulator",
                 self.build / "etc/init.d/rc.mc_defaults": "mc",
                 self.build / "etc/init.d-posix/airframes" / S.AIRFRAME: "airframe-installed",
                 self.build / "parameters.json": "{}",
                 self.px4 / "ROMFS/px4fmu_common/init.d-posix/rcS": "rcS-source",
                 self.px4 / "ROMFS/px4fmu_common/init.d-posix/airframes" / S.AIRFRAME: "airframe-source",
                 self.models / "navrl_quad/navrl_quad.sdf":
                     "<sdf><include><uri>model://lidar</uri></include><mesh><uri>model://iris/meshes/a.stl</uri>"
                     "</mesh></sdf>",
                 self.models / "lidar/model.sdf": "<sdf><uri>model://iris/meshes/b.dae</uri></sdf>",
                 self.models / "lidar/model.config": "cfg",
                 self.models / "iris/meshes/a.stl": "a", self.models / "iris/meshes/b.dae": "b",
                 self.build / "build_gazebo-classic/libgazebo_a.so": "a",
                 self.build / "build_gazebo-classic/libgazebo_b.so": "b",
                 self.build / "build_gazebo-classic/OpticalFlow/libOpticalFlow.so": "of",
                 self.tmp / "bin/gzserver": "#!/bin/sh\n", self.tmp / "bin/gz": "#!/bin/sh\n"}
        for path, text in files.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
        for exe in ("gzserver", "gz"):
            (self.tmp / "bin" / exe).chmod(0o755)
        self.ep = self.tmp / "ep"
        self.base = {"PATH": str(self.tmp / "bin")}

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def plan(self, base=None):
        return S.direct_launch_plan(self.ep, "motar_dev08", self.base if base is None else base,
                                    px4_root=self.px4, build=self.build)

    def test_plan_reproduces_sitl_run_launches_without_its_hazards(self):
        p = self.plan()
        sdf = self.models / "navrl_quad/navrl_quad.sdf"
        self.assertEqual(p["gzserver"]["argv"], ["gzserver", str(self.ep / "world/motar_dev08.world")])
        self.assertEqual(p["spawn"]["argv"], ["gz", "model", "--verbose", "--spawn-file=%s" % sdf,
                                              "--model-name=navrl_quad", "-x", "1.01", "-y", "0.98", "-z", "0.83"])
        self.assertEqual(p["px4"]["argv"], [str(self.build / "bin/px4"), "-d", str(self.build / "etc")])
        self.assertEqual((Path(p["px4"]["cwd"]), Path(p["gzserver"]["cwd"]), Path(p["spawn"]["cwd"])),
                         (self.ep / "rootfs", self.ep, self.ep))
        env = p["env"]
        self.assertEqual({k: env[k] for k in ("PX4_SIM_MODEL", "PX4_SIM_WORLD", "HEADLESS", "GAZEBO_LOG_PATH",
                                              "CUDA_VISIBLE_DEVICES", "GAZEBO_PLUGIN_PATH", "GAZEBO_MODEL_PATH",
                                              "LD_LIBRARY_PATH")},
                         {"PX4_SIM_MODEL": "gazebo-classic_navrl_quad", "PX4_SIM_WORLD": "motar_dev08",
                          "HEADLESS": "1", "GAZEBO_LOG_PATH": str(self.ep / "gazebo_log"), "CUDA_VISIBLE_DEVICES": "",
                          "GAZEBO_PLUGIN_PATH": ":" + str(self.build / "build_gazebo-classic"),
                          "GAZEBO_MODEL_PATH": ":" + str(self.models),
                          "LD_LIBRARY_PATH": ":" + str(self.build / "build_gazebo-classic")})
        self.assertNotIn("NO_PXH", env)
        joined = " ".join(" ".join(p[r]["argv"]) for r in ("gzserver", "spawn", "px4"))
        for word in ("sitl_run.sh", "pkill", "killall", "gzclient"):
            self.assertNotIn(word, joined)
        q = self.plan(dict(self.base, GAZEBO_PLUGIN_PATH="/opt/a", GAZEBO_MODEL_PATH="/nonexistent"))
        self.assertEqual(q["env"]["GAZEBO_PLUGIN_PATH"], "/opt/a:" + str(self.build / "build_gazebo-classic"))
        self.assertEqual(q["sdf_path"], sdf)
        other = self.tmp / "other_models/navrl_quad"
        other.mkdir(parents=True)
        (other / "navrl_quad.sdf").write_text("<sdf/>")
        with self.assertRaises(S.SessionError):                     # a shadowing SDF is not the PX4 tree's
            self.plan(dict(self.base, GAZEBO_MODEL_PATH=str(other.parent)))

    def test_validate_plan_rejections(self):
        S.validate_plan(self.plan(), self.ep, self.build)              # positive control
        mutants = {
            "shared_rootfs": lambda p: p["px4"].update(cwd=self.build / "rootfs"),
            "historical_launcher": lambda p: p["gzserver"].update(argv=["/x/Tools/simulation/sitl_run.sh"]),
            "global_kill": lambda p: p["spawn"].update(argv=["pkill", "-x", "gazebo"]),
            "shell_kill": lambda p: p["spawn"].update(argv=["bash", "-c", "pkill -x gazebo"]),
            "world_outside": lambda p: p.update(world_path=self.tmp / "w.world"),
            "cwd_outside": lambda p: p["spawn"].update(cwd=self.tmp),
        }
        for name, mutate in mutants.items():
            p = self.plan()
            mutate(p)
            with self.assertRaises(S.SessionError, msg=name):
                S.validate_plan(p, self.ep, self.build)

    def test_model_resolution_and_missing_model(self):
        with self.assertRaises(S.SessionError):
            S.model_sdf("/nonexistent:/also/missing")
        self.assertEqual(S.model_sdf(":" + str(self.models)), self.models / "navrl_quad/navrl_quad.sdf")

    def test_runtime_binding(self):
        p = self.plan()
        p["world_path"].parent.mkdir(parents=True)
        p["world_path"].write_text("<world/>")
        b = S.runtime_binding(p, px4_root=self.px4, build=self.build)
        f = b["files"]
        self.assertEqual(f["px4_binary"]["sha256"], hashlib.sha256(b"px4").hexdigest())
        self.assertEqual(f["installed_airframe"]["sha256"], hashlib.sha256(b"airframe-installed").hexdigest())
        self.assertEqual(f["source_airframe"]["sha256"], hashlib.sha256(b"airframe-source").hexdigest())
        self.assertEqual(b["plugins"], 3)                            # recursive: OpticalFlow/ included
        self.assertIn("plugin:OpticalFlow/libOpticalFlow.so", f)
        self.assertEqual({k for k in f if k.startswith("installed_px4-rc")},
                         {"installed_px4-rc.mavlink", "installed_px4-rc.simulator"})
        self.assertEqual({k for k in f if k.startswith("model_resource:")},
                         {"model_resource:lidar/model.sdf", "model_resource:lidar/model.config",
                          "model_resource:iris/meshes/a.stl", "model_resource:iris/meshes/b.dae"})
        self.assertIn("parameters_json", f)
        self.assertEqual(f["gzserver_executable"]["path"], str(self.tmp / "bin/gzserver"))
        self.assertEqual(b["env"]["PX4_SIM_MODEL"], "gazebo-classic_navrl_quad")
        (self.models / "iris/meshes/b.dae").unlink()                 # a resource named by an included model
        with self.assertRaises(S.SessionError):
            S.runtime_binding(p, px4_root=self.px4, build=self.build)

    def test_spawn_pose_matches_historical_sources(self):
        tree = ast.parse((ROOT / "tools/sitl_s3b_closed_loop.py").read_text())
        spawn = [ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == "SPAWN" for t in n.targets)]
        self.assertEqual(spawn, [(1.01, 0.98)])
        self.assertEqual((float(S.SPAWN_ARGS[1]), float(S.SPAWN_ARGS[3])), spawn[0])
        launcher = S.PX4_ROOT / "Tools/simulation/gazebo-classic/sitl_run.sh"
        if not launcher.is_file():
            self.skipTest("PX4 tree not on this host")
        text = launcher.read_text()
        self.assertIn("-x 1.01 -y 0.98 -z 0.83", text)
        self.assertIn("pkill -x gazebo", text)                      # why it is not used
        self.assertIn('rootfs="$build_path/rootfs"', text)


def kill_calls(source):
    """Calls that signal by pid or pattern: os.kill/killpg and pkill/killall subprocess command lines."""
    found = []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        f = node.func
        if isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name) and f.value.id == "os" \
                and f.attr in ("kill", "killpg"):
            found.append("os." + f.attr)
        for arg in node.args:
            if isinstance(arg, (ast.List, ast.Tuple)) and arg.elts and isinstance(arg.elts[0], ast.Constant) \
                    and arg.elts[0].value in ("pkill", "killall"):
                found.append(arg.elts[0].value)
    return found


class SourceContract(unittest.TestCase):
    def test_no_global_or_pid_only_kill(self):
        self.assertEqual(kill_calls(MODULE.read_text()), [])
        legacy = kill_calls((ROOT / "tools/sitl_s3b_closed_loop.py").read_text())   # non-vacuity
        self.assertIn("os.killpg", legacy)
        self.assertIn("pkill", legacy)

    def test_import_starts_nothing_and_claims_nothing(self):
        code = ("import importlib.util,os,sys,threading\n"
                "spec=importlib.util.spec_from_file_location('m',sys.argv[1]);m=importlib.util.module_from_spec(spec)\n"
                "spec.loader.exec_module(m)\n"
                "def ppid(d):\n"
                "    try:\n        return open('/proc/%s/stat'%d).read().rsplit(')',1)[1].split()[1]\n"
                "    except OSError:\n        return None\n"
                "kids=[d for d in os.listdir('/proc') if d.isdigit() and ppid(d)==str(os.getpid())]\n"
                "print(threading.active_count(), len(kids))")
        out = subprocess.run([sys.executable, "-c", code, str(MODULE)], capture_output=True, text=True, timeout=60,
                             env=dict(os.environ, PYTHONNOUSERSITE="1"))
        self.assertEqual(out.stdout.split(), ["1", "0"], out.stderr[-500:])
        self.assertEqual({k: S.STATUS[k] for k in ("live_verified", "execution_supported", "launch_ready")},
                         {"live_verified": False, "execution_supported": False, "launch_ready": False})

    def test_historical_runner_and_bridge_unchanged(self):
        spec = importlib.util.spec_from_file_location("dev08_cfg", ROOT / "tools/sitl_s3b_dev08.py")
        cfg = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cfg)
        for rel in (cfg.RUNNER, cfg.BRIDGE):
            self.assertEqual(hashlib.sha256((ROOT / rel).read_bytes()).hexdigest(), cfg.PINS[rel], rel)


if __name__ == "__main__":
    unittest.main()
