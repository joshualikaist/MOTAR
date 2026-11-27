"""Bound aggregate process-tree RSS and elapsed time without delegated cgroups."""
import argparse
import ctypes
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def snapshot():
    rows = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            rows[int(path.parent.name)] = {'state': fields[0], 'ppid': int(fields[1]), 'start': int(fields[19]),
                                           'rss': int(fields[21]) * os.sysconf('SC_PAGE_SIZE')}
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
            continue
    return rows


def descendants(root, rows):
    found = {root}
    while True:
        more = {pid for pid, row in rows.items() if row['ppid'] in found}
        if more.issubset(found):
            return found
        found |= more


def owned(rows):
    return {pid for pid in descendants(os.getpid(), rows) - {os.getpid()}
            if rows.get(pid, {}).get('state') != 'Z'}


def stop():
    # As a subreaper we retain orphaned descendants, including processes forked in TERM handlers.
    for sig, duration in ((signal.SIGTERM, 1.0), (signal.SIGKILL, 3.0)):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rows = snapshot()
            live = owned(rows)
            if not live:
                return True
            for pid in live:
                try:
                    fields = Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()
                    if int(fields[19]) == rows[pid]['start']:
                        os.kill(pid, sig)
                except (FileNotFoundError, PermissionError, ProcessLookupError):
                    pass
            time.sleep(0.1)
    return not owned(snapshot())


def run(argv, rss_limit, timeout, interval=0.25):
    started = datetime.datetime.now().astimezone().isoformat()
    clock = time.monotonic()
    # Linux PR_SET_CHILD_SUBREAPER. Failure is fatal before launching an uncontained job.
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(36, 1, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'PR_SET_CHILD_SUBREAPER failed')
    child = subprocess.Popen(argv, start_new_session=True)
    known, peak, samples, reason = {}, 0, 0, None
    try:
        while True:
            rows = snapshot()
            for pid in descendants(os.getpid(), rows) - {os.getpid()}:
                if pid in rows:
                    known[pid] = rows[pid]['start']
            live = {pid for pid, start in known.items() if rows.get(pid, {}).get('start') == start
                    and rows[pid]['state'] != 'Z'}
            rss = sum(rows[pid]['rss'] for pid in live)
            peak = max(peak, rss)
            samples += 1
            if rss > rss_limit:
                reason = 'RSS_LIMIT'
                break
            if time.monotonic() - clock > timeout:
                reason = 'TIMEOUT'
                break
            code = child.poll()
            if code is not None:
                others = live - {child.pid}
                if others:
                    reason = 'LINGERING_DESCENDANTS'
                break
            time.sleep(interval)
    except BaseException:
        reason = 'MONITOR_EXCEPTION'
        raise
    finally:
        # Always rescan: a successful parent can fork after the last observation and exit.
        if owned(snapshot()):
            if reason is None:
                reason = 'LINGERING_DESCENDANTS'
            if not stop():
                reason = 'CLEANUP_FAILED'
        code = child.wait(timeout=10)
        for pid in known:
            if pid != child.pid:
                try:
                    os.waitpid(pid, os.WNOHANG)
                except (ChildProcessError, ProcessLookupError):
                    pass
    remaining = len(owned(snapshot()))
    if remaining:
        reason = 'CLEANUP_FAILED'
    return {'started': started, 'finished': datetime.datetime.now().astimezone().isoformat(),
            'argv': argv, 'exit_code': code, 'stop_reason': reason, 'peak_aggregate_rss_bytes': peak,
            'rss_limit_bytes': rss_limit, 'timeout_s': timeout, 'poll_interval_s': interval,
            'samples': samples, 'processes_observed': len(known),
            'limit_kind': 'aggregate process-tree RSS watchdog; shared pages count per process',
            'kernel_cgroup_limit_claimed': False, 'subreaper_enabled': True,
            'live_descendants_after_cleanup': remaining}


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--rss-limit-mib', type=int, required=True)
    ap.add_argument('--timeout-s', type=float, required=True)
    ap.add_argument('--report', type=Path, required=True)
    ap.add_argument('argv', nargs=argparse.REMAINDER)
    args = ap.parse_args()
    argv = args.argv[1:] if args.argv[:1] == ['--'] else args.argv
    if args.rss_limit_mib <= 0 or args.timeout_s <= 0 or not argv or args.report.exists():
        raise SystemExit('invalid limits/command or output already exists')
    try:
        result = run(argv, args.rss_limit_mib * 1024 ** 2, args.timeout_s)
    except Exception as exc:
        result = {'argv': argv, 'exit_code': None, 'stop_reason': 'LAUNCH_OR_MONITOR_ERROR',
                  'error': str(exc), 'finished': datetime.datetime.now().astimezone().isoformat()}
    with args.report.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    raise SystemExit(0 if result['exit_code'] == 0 and result['stop_reason'] is None
                     and result['live_descendants_after_cleanup'] == 0 else 1)
