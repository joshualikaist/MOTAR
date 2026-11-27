"""Owned PX4 SITL + Gazebo Classic session for dev08 (amendment1 runtime-stack row "Owned session"; motar-5md).

Library only: importing it starts nothing. The session launches its own processes and signals nothing else.

- **No historical launcher.** tools/sitl_s3b_closed_loop.py runs PX4's sitl_run.sh. That script calls
  `pkill -x gazebo` and works in the shared build/px4_sitl_default/rootfs, whose parameters.bson rcS imports at boot,
  so parameters set in one episode persisted into the next. Here every episode gets a new external directory with
  its own empty rootfs, world, logs and Gazebo log path. The three launches sitl_run.sh makes (gzserver, the
  `gz model` spawn, px4) are made directly with the same arguments.
  - Stated differences:
    - gzserver and the spawn run in the episode directory, not in simulator_mavlink/;
    - the spawn is accepted only on exit 0 without Gazebo's not-ready phrase, and is retried a bounded number of
      times;
    - no gzclient; NO_PXH is not exported (only sitl_run.sh read it).
- **Exclusive instance.** One flock for PX4 instance 0. The session refuses when:
  - any of the instance's ports is bound;
  - PX4's own instance lock (/tmp/px4_lock-0, fcntl) is held;
  - a process with a simulator's name runs, whoever owns it;
  - a previous session left a leak marker naming a live process.
  While the session runs, the watchdog aborts if more than one socket is bound to UDP 14540.
- **Owned-only cleanup.** Each launch starts a new session.
  - A process is owned when it is:
    - a launched process with the recorded pid and start time;
    - a member of a launched process's session;
    - a descendant seen by the 0.25 s watchdog with the same pid and start time.
  - Launched processes are not reaped before the final scan. Their exit is observed with waitid(WNOWAIT), so each
    session id stays reserved and the session scan stays valid to the end.
  - Signals go through a pidfd opened before the start time is re-checked, so a reused pid is never signalled.
  - Sequence: SIGINT, then SIGKILL after the grace, then a final scan, then reaping.
  - A survivor, an identity refusal or a signalling error is a CleanupError and leaves a leak marker that blocks
    the next session.
  - No pkill, killall or killpg. os.kill is used only through Popen.kill on an unreaped child of this process
    whose identity check failed at start-up.
  - Detection limit: a descendant that leaves its session and exits its parent within one sampling period is not
    seen. The cleanup's grace wait re-samples at the same period (receipt key leak_detection).
- **Runner termination.** While a session is open in the main thread, SIGTERM and SIGHUP raise SessionTerminated
  (a BaseException, so `except Exception` wrappers do not swallow it), and the same cleanup runs. `timeout` sends
  SIGTERM. A signal that arrives inside start() or during cleanup is deferred, not dropped. After the receipt and
  the lock release, the first signal is re-delivered to the handler that was there before, so a multi-episode
  runner stops too.
  - Every launch is journalled (launches.jsonl) before start() returns.
  - A leak marker in state "open" names the journal while the session runs, so a session killed before its
    cleanup still blocks the next one while any journalled launch is alive. Members that outlive their launch
    are not in the journal, and a missing journal file clears the marker.
  - reap_journal() cleans up by identity after a SIGKILL of the runner.
- **Watchdog.** A wall-clock deadline (240 s per episode) and a sampled aggregate RSS cap (12 GiB over the runner
  and the owned tree; not a kernel hard cap). An internal watchdog error aborts too, and check()/wait()/sleep()
  refuse once the watchdog is gone. A watchdog abort kills the owned processes but cannot interrupt a main thread
  blocked in a system call, so callers use bounded waits.

Not in this module (later runtime-stack rows): parameter echo/readback, telemetry/clock checks, the policy step,
the attempt ledger. NOT_VERIFIED live: no PX4 or Gazebo process has been started with it. The direct-launch
arguments, the fresh-rootfs boot and the Gazebo log path need a separately approved smoke.
"""
import ctypes
import errno
import fcntl
import hashlib
import json
import os
import pathlib
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

STATUS = {"component": "owned_session", "contract": "amendment1 runtime-stack: Owned session",
          "live_verified": False, "execution_supported": False, "launch_ready": False}

PX4_ROOT = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot")
BUILD = PX4_ROOT / "build/px4_sitl_default"
MODEL = "navrl_quad"
AIRFRAME = "10090_gazebo-classic_navrl_quad"
SPAWN_ARGS = ("-x", "1.01", "-y", "0.98", "-z", "0.83")   # sitl_run.sh's spawn pose, as literal strings
NOT_READY = "An instance of Gazebo is not running."    # sitl_run.sh retries the spawn while gz prints this
ALLOWED_ROOT = pathlib.Path.home() / "motar_tmp"
LOCK_PATH = ALLOWED_ROOT / "locks/px4_sitl_instance0.lock"
PX4_INSTANCE_LOCK = pathlib.Path("/tmp/px4_lock-0")      # PX4's own fcntl lock (platforms/posix/.../main.cpp)
# PX4 instance 0 (px4-rc.mavlink, px4-rc.simulator) and the Gazebo master
PORTS = (("udp", 14540), ("udp", 14580), ("udp", 18570), ("udp", 14280), ("udp", 13030), ("tcp", 4560),
         ("tcp", 11345))
EXCLUSIVE_UDP = (14540,)
FOREIGN_NAMES = ("gzserver", "gzclient", "gazebo", "px4")
FORBIDDEN_ARGV = ("sitl_run.sh", "pkill", "killall", "gzclient")
SUBDIRS = ("rootfs", "world", "logs", "gazebo_log")
DEADLINE_S = 240.0
KILL_GRACE_S = 20.0
RSS_CAP_BYTES = 12 * 1024 ** 3
SAMPLE_S = 0.25
PAGE = os.sysconf("SC_PAGE_SIZE")
SYS_PIDFD_SEND_SIGNAL, SYS_PIDFD_OPEN = 424, 434   # same numbers on every Linux architecture (5.3+)
LEAK_DETECTION = "launched identities + session ids of unreaped launched processes + descendants sampled every " \
                 "period (watchdog, and the cleanup's grace wait); a setsid-and-orphan within one period is not seen"


def _sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


MODULE_SHA256 = _sha(__file__)


class SessionError(RuntimeError):
    """Refused or failed; nothing outside the session was signalled."""


class PathRefused(SessionError):
    pass


class LockBusy(SessionError):
    pass


class Contention(SessionError):
    pass


class SessionAborted(SessionError):
    pass


class SessionTerminated(BaseException):
    """The runner received SIGTERM or SIGHUP while a session was open (not an Exception: wrappers must not
    swallow a termination request)."""


class ProcessExited(SessionError):
    pass


class CleanupError(SessionError):
    pass


# ---------------------------------------------------------------- process identity

def proc_stat(pid):
    """state, ppid, pgrp, sid, start (clock ticks since boot) and rss_pages from /proc/<pid>/stat; None if gone."""
    try:
        raw = pathlib.Path("/proc/%d/stat" % pid).read_bytes()
    except (FileNotFoundError, ProcessLookupError):
        return None
    rest = raw[raw.rindex(b")") + 2:].split()
    return {"state": rest[0].decode(), "ppid": int(rest[1]), "pgrp": int(rest[2]), "sid": int(rest[3]),
            "start": int(rest[19]), "rss_pages": int(rest[21])}


def snapshot():
    table = {}
    for name in os.listdir("/proc"):
        if name.isdigit():
            st = proc_stat(int(name))
            if st is not None:
                table[int(name)] = st
    return table


def is_live(st):
    return st is not None and st["state"] not in ("Z", "X", "x")


def alive(pid, start):
    st = proc_stat(pid)
    return is_live(st) and st["start"] == start


def boot_id():
    return pathlib.Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def members(table, leaders, known):
    """The session's processes in a /proc table, as ({pid: start} live, {pid: start} including zombies).

    leaders: {pid: start} of every launched process. known: {pid: start} seen as members in earlier snapshots.
    Linux keeps a pid number allocated while any process uses it as a session id, so while a leader (alive or an
    unreaped zombie) still has its recorded start, every process with that session id is its member. Once the
    leader is gone or its number names another process, only known members are accepted. Descendants of members
    are members; a known pid with the same start is a member even after it left the session (setsid).
    """
    found = {}
    for pid, start in leaders.items():
        st = table.get(pid)
        if st is not None and st["start"] == start:
            found[pid] = start
            for p, s in table.items():
                if s["sid"] == pid:
                    found[p] = s["start"]
    for pid, start in known.items():
        st = table.get(pid)
        if st is not None and st["start"] == start:
            found[pid] = start
    children = {}
    for p, s in table.items():
        children.setdefault(s["ppid"], []).append(p)
    queue = list(found)
    while queue:
        for c in children.get(queue.pop(), ()):
            if c not in found:
                found[c] = table[c]["start"]
                queue.append(c)
    live = {p: s for p, s in found.items() if is_live(table.get(p))}
    return live, found


_libc = ctypes.CDLL(None, use_errno=True)
_libc.syscall.restype = ctypes.c_long


def _pidfd_open(pid):
    if hasattr(os, "pidfd_open"):
        return os.pidfd_open(pid)
    fd = _libc.syscall(SYS_PIDFD_OPEN, ctypes.c_int(pid), ctypes.c_uint(0))
    if fd < 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))
    return int(fd)


def _pidfd_send(fd, sig):
    if hasattr(signal, "pidfd_send_signal"):
        signal.pidfd_send_signal(fd, sig)
        return
    r = _libc.syscall(SYS_PIDFD_SEND_SIGNAL, ctypes.c_int(fd), ctypes.c_int(sig), None, ctypes.c_uint(0))
    if r < 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))


def signal_verified(pid, start, sig):
    """Signal pid only if it is still the process with this start time.
    Returns 'sent', 'gone', 'refused_identity' or 'error:<errno>'; never raises for an OS error."""
    try:
        fd = _pidfd_open(pid)
    except OSError as e:
        # ESRCH: gone. EINVAL also for a reaped pid still used as another process's session/group id (kernel 5.15).
        st = proc_stat(pid)
        if st is None or not is_live(st):
            return "gone"
        if st["start"] != start:
            return "refused_identity"
        return "error:%d" % e.errno
    try:
        st = proc_stat(pid)                      # the fd pins one process; the start time says which
        if st is None:
            return "gone"
        if st["start"] != start:
            return "refused_identity"
        try:
            _pidfd_send(fd, sig)
        except ProcessLookupError:
            return "gone"
        except OSError as e:
            return "error:%d" % e.errno
        return "sent"
    finally:
        os.close(fd)


def exit_code(pid):
    """The exit status of an exited, still unreaped child (it stays a zombie), or None while it runs."""
    info = os.waitid(os.P_PID, pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
    if info is None or info.si_pid == 0:
        return None
    return info.si_status if info.si_code == os.CLD_EXITED else -info.si_status


# ---------------------------------------------------------------- preconditions

def _inside(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def git_tree_of(path):
    for p in (path, *path.parents):
        if os.path.lexists(p / ".git"):
            return p
    return None


def check_episode_dir(episode_dir, allowed_root, forbidden_roots):
    """The new episode directory: absolute, absent, parent present, under allowed_root, outside every Git work tree
    and every forbidden root (the PX4 tree)."""
    given = pathlib.Path(episode_dir)
    if not given.is_absolute():
        raise PathRefused("episode directory must be absolute: %s" % given)
    if os.path.lexists(given):
        raise PathRefused("episode directory exists: %s" % given)
    try:
        parent = given.parent.resolve(strict=True)
    except (FileNotFoundError, NotADirectoryError):
        raise PathRefused("parent directory missing: %s" % given.parent) from None
    if not parent.is_dir():
        raise PathRefused("parent is not a directory: %s" % parent)
    path = parent / given.name
    if os.path.lexists(path):
        raise PathRefused("episode directory exists: %s" % path)
    if not _inside(path, pathlib.Path(allowed_root).resolve()):
        raise PathRefused("episode directory outside %s: %s" % (allowed_root, path))
    for root in forbidden_roots:
        if _inside(path, pathlib.Path(root).resolve()):
            raise PathRefused("episode directory inside forbidden root %s: %s" % (root, path))
    tree = git_tree_of(path)
    if tree is not None:
        raise PathRefused("episode directory inside the Git work tree %s" % tree)
    return path


def acquire_lock(lock_path, holder):
    lock_path = pathlib.Path(lock_path)
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(str(lock_path), os.O_RDWR | os.O_CREAT, 0o644)      # not inheritable: children never hold it
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        raise LockBusy("instance lock held: %s" % lock_path) from None
    os.ftruncate(fd, 0)
    os.write(fd, (json.dumps(holder) + "\n").encode())
    return fd


def leak_marker(lock_path):
    p = pathlib.Path(lock_path)
    return p.with_name(p.name + ".leaked")


def fcntl_lock_holder(path):
    """The pid holding a POSIX (fcntl) write lock on path, or None (F_GETLK; nothing is locked or created)."""
    try:
        fd = os.open(str(path), os.O_RDONLY)
    except FileNotFoundError:
        return None
    try:
        q = struct.pack("hhxxxxqqixxxx", fcntl.F_WRLCK, os.SEEK_SET, 0, 0, 0)
        r = fcntl.fcntl(fd, fcntl.F_GETLK, q)
        l_type, _, _, _, l_pid = struct.unpack("hhxxxxqqixxxx", r)
        return None if l_type == fcntl.F_UNLCK else l_pid
    finally:
        os.close(fd)


def foreign_processes(names):
    """Running processes (any user) whose name is a simulator's: the session refuses to start beside them."""
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            comm = pathlib.Path("/proc/%s/comm" % name).read_text().strip()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if comm in names and is_live(proc_stat(int(name))):
            out.append({"pid": int(name), "comm": comm})
    return out


def busy_ports(ports):
    """Ports that cannot be bound now.
    UDP is probed without SO_REUSEADDR, so any bound socket conflicts, including pymavlink's udpin, which sets it.
    TCP is probed with SO_REUSEADDR. A bound or listening socket then conflicts, and so does a TIME_WAIT connection
    left by a listener that did not set SO_REUSEADDR; one that did set it does not count."""
    busy = []
    for proto, port in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM if proto == "udp" else socket.SOCK_STREAM)
        try:
            if proto == "tcp":
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", port))
        except OSError as e:
            busy.append({"proto": proto, "port": port, "errno": e.errno})
        finally:
            s.close()
    return busy


def udp_bound_count(port):
    """Sockets bound to a local UDP port (IPv4 and IPv6), from /proc/net/udp{,6}."""
    n = 0
    for table in ("/proc/net/udp", "/proc/net/udp6"):
        try:
            lines = pathlib.Path(table).read_text().splitlines()[1:]
        except FileNotFoundError:
            continue
        for line in lines:
            local = line.split()[1]
            if int(local.rsplit(":", 1)[1], 16) == port:
                n += 1
    return n


_TOKENS = re.compile(r"[\s;|&()`$'\"]+")


def validate_argv(argv):
    if not argv or not all(isinstance(a, str) for a in argv):
        raise SessionError("argv must be a non-empty list of strings")
    for a in argv:
        for tok in [a] + _TOKENS.split(a):
            if tok and os.path.basename(tok) in FORBIDDEN_ARGV:
                raise SessionError("forbidden launch element: %s" % a)


def write_new(path, text):
    with open(path, "x") as f:
        f.write(text)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


# ---------------------------------------------------------------- the session

class OwnedSession:
    """with OwnedSession(episode_dir) as s: s.start(role, argv, env=..., cwd=...); ...

    Exiting (normally, by exception, after a watchdog abort or on SIGTERM/SIGHUP) runs the owned-only cleanup,
    releases the lock and writes episode_dir/session.json. A watchdog abort raises SessionAborted; an owned
    survivor, an identity refusal, a signalling error or an internal cleanup error raises CleanupError (chained to
    any exception in flight) and leaves a leak marker beside the lock.
    """

    def __init__(self, episode_dir, *, allowed_root=ALLOWED_ROOT, lock_path=LOCK_PATH, ports=PORTS,
                 foreign_names=FOREIGN_NAMES, forbidden_roots=(PX4_ROOT,), px4_lock=PX4_INSTANCE_LOCK,
                 exclusive_udp=EXCLUSIVE_UDP, deadline_s=DEADLINE_S, kill_grace_s=KILL_GRACE_S,
                 rss_cap_bytes=RSS_CAP_BYTES, sample_s=SAMPLE_S, include_self_rss=True, handle_signals=True):
        self.episode_dir_given = episode_dir
        self.allowed_root = pathlib.Path(allowed_root)
        self.lock_path = pathlib.Path(lock_path)
        self.ports = tuple(ports)
        self.foreign_names = tuple(foreign_names)
        self.forbidden_roots = tuple(forbidden_roots)
        self.px4_lock = pathlib.Path(px4_lock) if px4_lock else None
        self.exclusive_udp = tuple(exclusive_udp)
        self.deadline_s = float(deadline_s)
        self.kill_grace_s = float(kill_grace_s)
        self.rss_cap_bytes = int(rss_cap_bytes)
        self.sample_s = float(sample_s)
        self.include_self_rss = bool(include_self_rss)
        self.handle_signals = bool(handle_signals)
        self.episode_dir = None
        self._procs = {}            # role -> {"popen", "pid", "start", "record", "rc"}
        self._order = []
        self._known = {}
        self._lock = threading.RLock()
        self._term_lock = threading.Lock()
        self._stop = threading.Event()
        self._abort = None
        self._watchdog = None
        self._watch_error = None
        self._lock_fd = None
        self._closed = False
        self._closing = False
        self._actions = []
        self._refused = []
        self._errors = []
        self._signals = []
        self._first_signal = None
        self._critical = False
        self._saved_handlers = {}
        self._samples = 0
        self._peak_rss = 0
        self._t0 = None
        self._wall0 = None
        self._journal = None

    # -- paths
    def path(self, *parts):
        p = self.episode_dir.joinpath(*parts)
        if not _inside(p.resolve(), self.episode_dir):
            raise PathRefused("outside the episode directory: %s" % p)
        return p

    @property
    def rootfs(self):
        return self.episode_dir / "rootfs"

    def write_world(self, name, text):
        p = self.path("world", name + ".world")
        write_new(p, text)
        return p

    # -- lifecycle
    def __enter__(self):
        if proc_stat(1) is None:
            raise SessionError("/proc is not readable (hidepid?): ownership cannot be verified")
        path = check_episode_dir(self.episode_dir_given, self.allowed_root, self.forbidden_roots)
        holder = {"pid": os.getpid(), "start": proc_stat(os.getpid())["start"], "episode_dir": str(path),
                  "wall": time.time()}
        self._lock_fd = acquire_lock(self.lock_path, holder)
        try:
            foreign = foreign_processes(self.foreign_names)
            if foreign:
                raise Contention("simulator processes already running: %s" % foreign[:5])
            busy = busy_ports(self.ports)
            if busy:
                raise Contention("ports in use: %s" % busy)
            if self.px4_lock is not None:
                holder_pid = fcntl_lock_holder(self.px4_lock)
                if holder_pid is not None:
                    raise Contention("PX4 instance lock %s held by pid %d" % (self.px4_lock, holder_pid))
            marker = leak_marker(self.lock_path)
            if marker.exists():
                try:
                    entries = json.loads(marker.read_text())
                except ValueError:
                    raise LockBusy("unreadable leak marker %s: inspect it before the next session" % marker) from None
                ids = list(entries.get("leaked", []))
                journal = entries.get("journal")
                if journal and pathlib.Path(journal).is_file():          # a session that never reached cleanup
                    ids += [json.loads(x) for x in pathlib.Path(journal).read_text().splitlines()]
                if entries.get("boot_id") == boot_id() and any(alive(e["pid"], e["start"]) for e in ids):
                    raise LockBusy("a previous session left live processes: %s" % marker)
                os.replace(str(marker), str(marker.with_name(marker.name + ".cleared-%d" % time.time_ns())))
            path.mkdir(exist_ok=False)
            for sub in SUBDIRS:
                (path / sub).mkdir()
            self._journal = open(path / "launches.jsonl", "x")
            write_new(marker, json.dumps({"state": "open", "boot_id": boot_id(), "episode_dir": str(path),
                                          "journal": str(path / "launches.jsonl"), "leaked": []}) + "\n")
            self.episode_dir = path
            self._t0 = time.monotonic()
            self._wall0 = time.time()
            if self.handle_signals and threading.current_thread() is threading.main_thread():
                for sig in (signal.SIGTERM, signal.SIGHUP):
                    self._saved_handlers[sig] = signal.signal(sig, self._on_signal)
            self._watchdog = threading.Thread(target=self._watch, name="owned-session-watchdog", daemon=True)
            self._watchdog.start()
        except BaseException:
            for sig, h in self._saved_handlers.items():
                signal.signal(sig, h)
            self._saved_handlers = {}
            if self._journal is not None:
                self._journal.close()
            os.close(self._lock_fd)
            self._lock_fd = None
            raise
        return self

    def _on_signal(self, signum, frame):
        self._signals.append({"signal": signum, "t_s": round(time.monotonic() - self._t0, 3),
                              "during_cleanup": self._closing, "deferred": self._closing or self._critical})
        if self._first_signal is None:
            self._first_signal = signum
        if not (self._closing or self._critical):
            raise SessionTerminated("runner received signal %d" % signum)

    def start(self, role, argv, *, env, cwd):
        with self._lock:
            if self._closed or self._abort:
                raise SessionAborted("session is closing: %s" % (self._abort or "closed"))
            self._require_watchdog()
            if role in self._procs:
                raise SessionError("duplicate role: %s" % role)
            argv = list(argv)
            validate_argv(argv)
            cwd = pathlib.Path(cwd).resolve()
            if not _inside(cwd, self.episode_dir):
                raise PathRefused("cwd outside the episode directory: %s" % cwd)
            log_path = self.path("logs", role + ".log")
            log = open(log_path, "xb")
            self._critical = True                  # a signal from here to the journal write is deferred
            try:
                return self._start_recorded(role, argv, env, cwd, log, log_path)
            finally:
                self._critical = False
                if self._first_signal is not None and not self._closing:
                    raise SessionTerminated("runner received signal %d" % self._first_signal)

    def _start_recorded(self, role, argv, env, cwd, log, log_path):
        with self._lock:
            try:
                p = subprocess.Popen(argv, cwd=str(cwd), env=dict(env), stdin=subprocess.DEVNULL, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
            finally:
                log.close()
            st = proc_stat(p.pid)                # unreaped, so readable even if it has already exited
            if st is None or st["sid"] != p.pid or st["ppid"] != os.getpid():
                p.kill()
                p.wait()
                raise SessionError("launched process is not a new session of this process: %s" % st)
            record = {"role": role, "argv": argv, "cwd": str(cwd), "pid": p.pid, "start_ticks": st["start"],
                      "log": str(log_path), "t_s": round(time.monotonic() - self._t0, 3)}
            self._procs[role] = {"popen": p, "pid": p.pid, "start": st["start"], "record": record, "rc": None}
            self._order.append(role)
            self._known[p.pid] = st["start"]
            self._journal.write(json.dumps({"role": role, "pid": p.pid, "start": st["start"], "boot_id": boot_id(),
                                            "argv": argv}) + "\n")
            self._journal.flush()
            os.fsync(self._journal.fileno())
            return record

    def _rc(self, role):
        v = self._procs[role]
        if v["rc"] is None and v["popen"].returncode is None:
            v["rc"] = exit_code(v["pid"])
        return v["rc"] if v["popen"].returncode is None else v["popen"].returncode

    def _require_watchdog(self):
        if self._watchdog is None or not self._watchdog.is_alive():
            if not self._abort:
                raise SessionAborted("watchdog is not running (%s)" % (self._watch_error or "stopped"))

    def check(self, required=()):
        """Raise SessionAborted after a watchdog abort or without a watchdog, ProcessExited if a required role has
        exited. Exited launches are not reaped here."""
        if self._abort:
            raise SessionAborted(self._abort)
        self._require_watchdog()
        for role in required:
            rc = self._rc(role)
            if rc is not None:
                raise ProcessExited("%s exited with %s" % (role, rc))

    def wait(self, role, timeout):
        """Wait for a role to exit (abort-aware); its return code, or SessionError on timeout."""
        end = time.monotonic() + timeout
        while True:
            rc = self._rc(role)
            if rc is not None:
                return rc
            if self._abort:
                raise SessionAborted(self._abort)
            self._require_watchdog()
            if time.monotonic() > end:
                raise SessionError("%s still running after %.1f s" % (role, timeout))
            time.sleep(0.02)

    def sleep(self, seconds):
        if self._abort:
            raise SessionAborted(self._abort)
        self._require_watchdog()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._abort:
                raise SessionAborted(self._abort)
            time.sleep(min(0.02, max(0.0, end - time.monotonic())))

    def log_text(self, role):
        return pathlib.Path(self._procs[role]["record"]["log"]).read_text(errors="replace")

    def pids(self):
        return {role: (v["pid"], v["start"]) for role, v in self._procs.items()}

    # -- ownership
    def _leaders(self):
        with self._lock:
            return {v["pid"]: v["start"] for v in self._procs.values() if v["popen"].returncode is None}

    def _members(self):
        live, found = members(snapshot(), self._leaders(), dict(self._known))
        with self._lock:
            self._known.update(found)
        return live

    def _abort_with(self, reason):
        with self._lock:
            if self._abort is None:
                self._abort = reason
        self._terminate(reason)

    def _watch(self):
        own = os.getpid()
        while not self._stop.wait(self.sample_s):
            try:
                table = snapshot()
                live, found = members(table, self._leaders(), dict(self._known))
                with self._lock:
                    self._known.update(found)
                rss = sum(table[p]["rss_pages"] for p in live if p in table) * PAGE
                if self.include_self_rss and own in table:
                    rss += table[own]["rss_pages"] * PAGE
                self._samples += 1
                self._peak_rss = max(self._peak_rss, rss)
                reason = None
                if time.monotonic() - self._t0 > self.deadline_s:
                    reason = "timeout"
                elif rss > self.rss_cap_bytes:
                    reason = "rss_cap"
                elif any(udp_bound_count(p) > 1 for p in self.exclusive_udp):
                    reason = "port_contention"
            except Exception as e:                            # noqa: BLE001 - recorded, then the session aborts
                self._watch_error = "%s: %s" % (e.__class__.__name__, e)
                reason = "watchdog_error"
            if reason:
                try:
                    self._abort_with(reason)
                except Exception as e:                        # noqa: BLE001
                    self._errors.append("watchdog cleanup: %s: %s" % (e.__class__.__name__, e))
                return

    def _wait_gone(self, targets, timeout):
        """Wait for targets to go; re-sample the owned tree every period meanwhile (escapes during cleanup)."""
        end = time.monotonic() + timeout
        next_scan = time.monotonic() + self.sample_s
        while True:
            left = {p: s for p, s in targets.items() if alive(p, s)}
            if not left or time.monotonic() > end:
                return left
            if time.monotonic() >= next_scan:
                self._members()
                next_scan += self.sample_s
            time.sleep(0.02)

    def _send(self, targets, sig, why):
        order = {v["pid"]: i for i, v in enumerate(self._procs[r] for r in self._order)}
        for pid in sorted(targets, key=lambda p: (-order.get(p, -1), p)):
            try:
                result = signal_verified(pid, targets[pid], sig)
            except Exception as e:                            # noqa: BLE001 - recorded as a cleanup error
                result = "error:%s" % e.__class__.__name__
            self._actions.append({"pid": pid, "start": targets[pid], "signal": int(sig), "result": result,
                                  "why": why, "t_s": round(time.monotonic() - self._t0, 3)})
            if result == "refused_identity":
                self._refused.append({"pid": pid, "start": targets[pid]})
            elif result.startswith("error:"):
                self._errors.append("signal %d to %d: %s" % (sig, pid, result))

    def _terminate(self, why):
        """Owned-only cleanup. Launched processes stay unreaped until the final scan; returns the live leftovers."""
        with self._term_lock:
            with self._lock:
                self._closed = True
            for v in self._procs.values():                       # an unreaped child must still be itself
                if v["popen"].returncode is None:
                    st = proc_stat(v["pid"])
                    if st is not None and st["start"] != v["start"]:
                        self._refused.append({"pid": v["pid"], "start": v["start"], "role": v["record"]["role"]})
            refused = {(r["pid"], r["start"]) for r in self._refused}
            targets = {p: s for p, s in self._members().items() if (p, s) not in refused}
            if targets:
                self._send(targets, signal.SIGINT, why)
                left = self._wait_gone(targets, self.kill_grace_s)
                more = {p: s for p, s in self._members().items() if (p, s) not in refused and p not in targets}
                left.update(more)
                if left:
                    self._send(left, signal.SIGKILL, why + ":after_grace")
                    self._wait_gone(left, 5.0)
            final = {p: s for p, s in self._members().items() if (p, s) not in refused}
            if final:                                            # forked during shutdown: one more round
                self._send(final, signal.SIGKILL, why + ":final_scan")
                self._wait_gone(final, 5.0)
                final = {p: s for p, s in self._members().items() if (p, s) not in refused}
            if not final:                                        # only now: reap the launched processes
                for v in self._procs.values():
                    if v["popen"].returncode is None and not any(r["pid"] == v["pid"] for r in self._refused):
                        try:
                            v["popen"].wait(timeout=5.0)
                        except subprocess.TimeoutExpired:
                            pass
            return final

    def __exit__(self, exc_type, exc, tb):
        self._closing = True
        if self._saved_handlers and threading.current_thread() is threading.main_thread():
            # Ctrl-C during the cleanup is deferred too, then re-delivered (review r3 R3-m1)
            self._saved_handlers[signal.SIGINT] = signal.signal(signal.SIGINT, self._on_signal)
        self._stop.set()
        leaked, internal = {}, None
        try:
            if self._watchdog is not None:
                self._watchdog.join(timeout=self.kill_grace_s + 15.0)
            reason = self._abort or ("terminated" if isinstance(exc, SessionTerminated)
                                     else ("exception" if exc_type is not None else "normal"))
            leaked = self._terminate(reason)
        except BaseException as e:                               # noqa: BLE001 - recorded; the receipt is written
            internal = "%s: %s" % (e.__class__.__name__, e)
            reason = self._abort or "cleanup_error"
            try:
                leaked = members(snapshot(), self._leaders(), dict(self._known))[0]
            except Exception:                                    # noqa: BLE001
                leaked = {}
        finally:
            for sig, h in self._saved_handlers.items():
                signal.signal(sig, h)
            self._closing = False
        exits = {r: v["popen"].returncode for r, v in self._procs.items()}
        unreaped = [r for r, rc in exits.items() if rc is None]
        problems = []
        if leaked:
            problems.append("owned processes survived cleanup: %s" % sorted(leaked))
        if self._refused:
            problems.append("identity check refused: %s" % self._refused)
        if unreaped:
            problems.append("launched processes not reaped: %s" % unreaped)
        if self._errors:
            problems.append("cleanup errors: %s" % self._errors)
        if internal:
            problems.append("internal cleanup error: %s" % internal)
        if self._watch_error:
            problems.append("watchdog error: %s" % self._watch_error)
        marker = leak_marker(self.lock_path)
        if problems:                                             # block the next session while these live
            ids = [{"pid": p, "start": s} for p, s in sorted(leaked.items())]
            ids += [{"pid": v["pid"], "start": v["start"]} for r, v in self._procs.items() if r in unreaped]
            tmp = marker.with_name(marker.name + ".tmp-%d" % os.getpid())
            tmp.write_text(json.dumps({"state": "leaked", "boot_id": boot_id(), "episode_dir": str(self.episode_dir),
                                       "journal": str(self.episode_dir / "launches.jsonl"), "leaked": ids,
                                       "problems": problems}) + "\n")
            os.replace(str(tmp), str(marker))
        elif marker.exists():
            os.replace(str(marker), str(marker.with_name(marker.name + ".closed-%d" % time.time_ns())))
        os.close(self._lock_fd)
        self._lock_fd = None
        self._journal.close()
        receipt = {
            "status": STATUS, "module_sha256_at_import": MODULE_SHA256, "module_sha256_at_exit": _sha(__file__),
            "python": sys.version.split()[0], "boot_id": boot_id(), "episode_dir": str(self.episode_dir),
            "lock_path": str(self.lock_path), "ports_checked": [list(p) for p in self.ports],
            "foreign_names": list(self.foreign_names), "px4_instance_lock": str(self.px4_lock),
            "exclusive_udp": list(self.exclusive_udp), "leak_detection": LEAK_DETECTION,
            "limits": {"deadline_s": self.deadline_s, "kill_grace_s": self.kill_grace_s,
                       "rss_cap_bytes": self.rss_cap_bytes, "sample_s": self.sample_s,
                       "include_self_rss": self.include_self_rss},
            "launches": [self._procs[r]["record"] for r in self._order], "exit_codes": exits,
            "cleanup": {"reason": reason, "actions": self._actions, "refused_identity": self._refused,
                        "leaked": [{"pid": p, "start": s} for p, s in sorted(leaked.items())],
                        "unreaped_roles": unreaped, "errors": self._errors, "internal_error": internal},
            "watchdog": {"samples": self._samples, "peak_rss_bytes": self._peak_rss, "abort": self._abort,
                         "error": self._watch_error},
            "signals": self._signals, "known_members": len(self._known), "wall_start": self._wall0,
            "wall_end": time.time(), "exception": None if exc_type is None else "%s: %s" % (exc_type.__name__, exc),
        }
        write_new(self.episode_dir / "session.json", json.dumps(receipt, indent=1, default=str) + "\n")
        if self._first_signal is not None:
            # re-deliver the termination request to the handler that was there before (default: the process ends)
            signal.raise_signal(self._first_signal)
        if problems:
            raise CleanupError("; ".join(problems)) from exc
        if self._abort:
            raise SessionAborted(self._abort) from exc
        return False


def reap_journal(episode_dir, *, kill_grace_s=KILL_GRACE_S, sample_s=SAMPLE_S):
    """After the runner was killed (SIGKILL, OOM): stop the journalled launches of one episode that are still the
    same processes (pid, start time, boot id), with their session members and descendants. Without the runner the
    launches are no longer anyone's unreaped children, so a launch's session id is trusted only while it lives:
    the other members are stopped first (SIGINT, re-sampling during the grace, then SIGKILL) and the launches
    last. A process forked by a launch while the launch itself is shutting down can still escape (reported in
    alive_after when it is seen). Returns a report; never signals anything not identified through the journal."""
    entries = [json.loads(line) for line in (pathlib.Path(episode_dir) / "launches.jsonl").read_text().splitlines()]
    this_boot = boot_id()
    leaders = {e["pid"]: e["start"] for e in entries if e["boot_id"] == this_boot and alive(e["pid"], e["start"])}
    known, actions = {}, []

    def scan():
        live, found = members(snapshot(), {p: s for p, s in leaders.items() if alive(p, s)}, known)
        known.update(found)
        return live

    def stop(targets, sig):
        for p, s in sorted(targets.items()):
            actions.append({"pid": p, "signal": int(sig), "result": signal_verified(p, s, sig)})

    def wait(targets, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and any(alive(p, s) for p, s in targets.items()):
            time.sleep(sample_s)
            targets.update({p: s for p, s in scan().items() if p not in leaders})
        return {p: s for p, s in targets.items() if alive(p, s)}

    first = scan()
    for phase in ("members", "launches"):
        targets = {p: s for p, s in scan().items() if (p in leaders) == (phase == "launches")}
        if phase == "launches":
            targets.update({p: s for p, s in scan().items() if p not in leaders})
        if not targets:
            continue
        stop(targets, signal.SIGINT)
        left = wait(dict(targets), kill_grace_s)
        if left:
            stop(left, signal.SIGKILL)
            wait(dict(left), 5.0)
    time.sleep(0.2)
    after = {p: s for p, s in known.items() if alive(p, s)}
    after.update(scan())
    return {"journalled": len(entries), "alive_at_start": sorted(first), "actions": actions,
            "alive_after": sorted(after)}


# ---------------------------------------------------------------- the PX4 / Gazebo Classic launch

def _append(env, key, value):
    env[key] = env.get(key, "") + ":" + value               # as setup_gazebo.bash: export K=$K:value


def model_sdf(model_path, model=MODEL):
    for d in model_path.split(":"):
        if d and (pathlib.Path(d) / model / (model + ".sdf")).is_file():
            return pathlib.Path(d) / model / (model + ".sdf")
    raise SessionError("model %s not found in GAZEBO_MODEL_PATH" % model)


def direct_launch_plan(episode_dir, world_name, base_env, *, px4_root=PX4_ROOT, build=BUILD, model=MODEL,
                       gzserver="gzserver", gz="gz"):
    """The three launches sitl_run.sh makes (HEADLESS, NO_PXH, no debugger), with this episode's world, rootfs and
    Gazebo log path. Pure: starts nothing; the world file need not exist yet. The model SDF must be the PX4 tree's
    (the first match on GAZEBO_MODEL_PATH, as sitl_run.sh, then checked)."""
    episode_dir = pathlib.Path(episode_dir)
    px4_root, build = pathlib.Path(px4_root), pathlib.Path(build)
    models = px4_root / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
    world_path = episode_dir / "world" / (world_name + ".world")
    env = dict(base_env)
    _append(env, "GAZEBO_PLUGIN_PATH", str(build / "build_gazebo-classic"))
    _append(env, "GAZEBO_MODEL_PATH", str(models))
    _append(env, "LD_LIBRARY_PATH", str(build / "build_gazebo-classic"))
    env.update({"PX4_SIM_MODEL": "gazebo-classic_" + model, "PX4_SIM_WORLD": world_name, "HEADLESS": "1",
                "GAZEBO_LOG_PATH": str(episode_dir / "gazebo_log"), "CUDA_VISIBLE_DEVICES": ""})
    sdf = model_sdf(env["GAZEBO_MODEL_PATH"], model)
    if sdf != models / model / (model + ".sdf"):
        raise SessionError("model SDF %s is not the PX4 tree's" % sdf)
    plan = {
        "world_path": world_path, "sdf_path": sdf, "rootfs": episode_dir / "rootfs", "env": env,
        "gzserver": {"argv": [gzserver, str(world_path)], "cwd": episode_dir, "env": env},
        "spawn": {"argv": [gz, "model", "--verbose", "--spawn-file=%s" % sdf, "--model-name=%s" % model,
                           *SPAWN_ARGS], "cwd": episode_dir, "env": env},
        "px4": {"argv": [str(build / "bin/px4"), "-d", str(build / "etc")], "cwd": episode_dir / "rootfs",
                "env": env},
    }
    validate_plan(plan, episode_dir, build)
    return plan


def validate_plan(plan, episode_dir, build=BUILD):
    episode_dir = pathlib.Path(episode_dir)
    for role in ("gzserver", "spawn", "px4"):
        validate_argv(plan[role]["argv"])
        if not _inside(pathlib.Path(plan[role]["cwd"]), episode_dir):
            raise SessionError("%s cwd outside the episode directory" % role)
    if pathlib.Path(plan["px4"]["cwd"]) != episode_dir / "rootfs":
        raise SessionError("px4 must run in the episode's own rootfs")
    if _inside(pathlib.Path(plan["px4"]["cwd"]), pathlib.Path(build)):
        raise SessionError("px4 rootfs inside the shared build tree")
    if not _inside(pathlib.Path(plan["world_path"]), episode_dir):
        raise SessionError("world outside the episode directory")


def launch_px4_gazebo(session, plan, *, spawn_attempts=30, spawn_retry_s=1.0, spawn_timeout_s=60.0):
    """gzserver, then the model spawn (bounded retry while Gazebo is not ready), then px4 in the fresh rootfs.
    A spawn counts only when it exits 0 without Gazebo's not-ready phrase (sitl_run.sh checked the phrase only)."""
    validate_plan(plan, session.episode_dir)
    if not pathlib.Path(plan["world_path"]).is_file():
        raise SessionError("world file not written: %s" % plan["world_path"])
    session.start("gzserver", **plan["gzserver"])
    spawned = None
    for i in range(spawn_attempts):
        session.check(required=("gzserver",))
        role = "spawn_%02d" % i
        session.start(role, **plan["spawn"])
        rc = session.wait(role, spawn_timeout_s)
        if rc == 0 and NOT_READY not in session.log_text(role):
            spawned = role
            break
        session.sleep(spawn_retry_s)
    if spawned is None:
        raise SessionError("model spawn did not succeed in %d attempts" % spawn_attempts)
    session.start("px4", **plan["px4"])
    session.check(required=("gzserver", "px4"))
    return spawned


def _model_resources(sdf_text, model_path, seen=None):
    """{key: file} for every model:// resource an SDF names, resolved on GAZEBO_MODEL_PATH. A directory (an included
    model) contributes all its files, and the SDFs inside are followed in turn."""
    seen = set() if seen is None else seen
    out = {}
    for uri in sorted(set(re.findall(r"model://([^<\s\"']+)", sdf_text)) - seen):
        seen.add(uri)
        for d in model_path.split(":"):
            p = pathlib.Path(d) / uri if d else None
            if p is not None and (p.is_file() or p.is_dir()):
                break
        else:
            raise SessionError("model resource not found: model://%s" % uri)
        if p.is_file():
            out[uri] = p
            continue
        for f in sorted(x for x in p.rglob("*") if x.is_file()):
            out["%s/%s" % (uri, f.relative_to(p))] = f
            if f.suffix == ".sdf":
                out.update(_model_resources(f.read_text(errors="replace"), model_path, seen))
    return out


def runtime_binding(plan, *, px4_root=PX4_ROOT, build=BUILD, airframe=AIRFRAME):
    """Path, size and SHA-256 of what the launch runs and reads at boot: binaries; installed rcS, px4-rc.*,
    rc.mc_defaults and airframe; source rcS and airframe; parameters.json; the model SDF and every model://
    resource it names; the world; every Gazebo plugin (recursive). Plus the launch environment's simulator keys.
    Missing file -> SessionError."""
    px4_root, build = pathlib.Path(px4_root), pathlib.Path(build)
    posix = build / "etc/init.d-posix"
    files = {"px4_binary": build / "bin/px4", "installed_rcS": posix / "rcS",
             "installed_airframe": posix / "airframes" / airframe,
             "installed_rc_mc_defaults": build / "etc/init.d/rc.mc_defaults",
             "source_rcS": px4_root / "ROMFS/px4fmu_common/init.d-posix/rcS",
             "source_airframe": px4_root / "ROMFS/px4fmu_common/init.d-posix/airframes" / airframe,
             "parameters_json": build / "parameters.json",
             "model_sdf": pathlib.Path(plan["sdf_path"]), "world": pathlib.Path(plan["world_path"])}
    for rc in sorted(posix.glob("px4-rc.*")):
        files["installed_" + rc.name] = rc
    for role in ("gzserver", "spawn"):
        exe = shutil.which(plan[role]["argv"][0], path=plan["env"].get("PATH", os.environ.get("PATH")))
        if exe is None:
            raise SessionError("executable not found: %s" % plan[role]["argv"][0])
        files[role + "_executable"] = pathlib.Path(os.path.realpath(exe))
    for so in sorted((build / "build_gazebo-classic").rglob("*.so")):
        files["plugin:" + str(so.relative_to(build / "build_gazebo-classic"))] = so
    sdf_path = pathlib.Path(plan["sdf_path"])
    if sdf_path.is_file():
        for uri, p in _model_resources(sdf_path.read_text(), plan["env"].get("GAZEBO_MODEL_PATH", "")).items():
            files["model_resource:" + uri] = p
    out = {}
    for key, p in files.items():
        if not p.is_file():
            raise SessionError("runtime file missing: %s %s" % (key, p))
        out[key] = {"path": str(p), "bytes": p.stat().st_size, "sha256": sha256_file(p)}
    env = {k: v for k, v in plan["env"].items()
           if k.startswith(("GAZEBO_", "PX4_")) or k in ("LD_LIBRARY_PATH", "CUDA_VISIBLE_DEVICES", "HEADLESS")}
    return {"files": out, "env": env, "plugins": sum(1 for k in out if k.startswith("plugin:"))}


if __name__ == "__main__":
    print(json.dumps(STATUS))
