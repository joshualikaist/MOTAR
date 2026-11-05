"""Provider-neutral clock/handoff; no scheduler, quota inspector, run authorization or process killer."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
import fcntl
import hashlib
import json
from pathlib import Path
import os
import subprocess
import tempfile
import uuid


def now():
    return datetime.now(timezone.utc)


def timestamp(value):
    result = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if result.tzinfo is None:
        raise ValueError('timestamps must include a timezone')
    return result.astimezone(timezone.utc)


def schedule(started, deadline=None):
    end = min(started + timedelta(hours=5), deadline) if deadline else started + timedelta(hours=5)
    if end <= started:
        raise ValueError('deadline has already passed; close out instead of starting a fresh clock')
    return {'started_at': started.isoformat(), 'deadline': end.isoformat(),
            'wrap_at': (end - timedelta(hours=1)).isoformat(),
            'finish_by': (end - timedelta(minutes=10)).isoformat(),
            'deadline_source': 'explicit' if deadline else 'five_hour_fallback_not_verified_quota'}


def phase(record, clock):
    if record.get('closed_at') or clock >= timestamp(record['finish_by']):
        return 'STOP_AND_HANDOFF'
    if clock >= timestamp(record['wrap_at']):
        return 'WRAP_UP'
    return 'WORK'


def fits(record, clock, minutes, verification):
    if minutes < 0 or verification < 0:
        raise ValueError('durations must be nonnegative')
    return phase(record, clock) == 'WORK' and clock + timedelta(minutes=minutes + verification) <= timestamp(record['finish_by'])


def tighten(record, deadline=None, started=None):
    """New evidence may shorten an existing allowance, never extend/reopen it."""
    result = dict(record)
    beginning = min(timestamp(record['started_at']), started) if started else timestamp(record['started_at'])
    end = min(timestamp(record['deadline']), beginning + timedelta(hours=5))
    if deadline:
        end = min(end, deadline)
    if beginning != timestamp(record['started_at']) or end != timestamp(record['deadline']):
        result.update(started_at=beginning.isoformat(), deadline=end.isoformat(),
                      wrap_at=(end - timedelta(hours=1)).isoformat(),
                      finish_by=(end - timedelta(minutes=10)).isoformat(),
                      deadline_source='tightened_existing_clock')
    return result


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, prefix='.session-', delete=False) as stream:
            name = stream.name
            json.dump(value, stream, indent=2, ensure_ascii=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if name and Path(name).exists():
            Path(name).unlink()


@contextmanager
def lock(directory):
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / '.lock').open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def location(root, state_root):
    common = Path(git(root, 'rev-parse', '--git-common-dir'))
    if not common.is_absolute():
        common = root / common
    digest = lambda p: hashlib.sha256(str(p.resolve()).encode()).hexdigest()[:16]
    return state_root / digest(common) / digest(root)


def handoff_valid(value):
    required = {'completed', 'remaining', 'next_action', 'running_processes', 'pending_approvals', 'validation'}
    if not isinstance(value, dict) or not required.issubset(value):
        raise ValueError('handoff requires: ' + ', '.join(sorted(required)))
    if not isinstance(value['next_action'], str) or not value['next_action'].strip():
        raise ValueError('one concrete next_action is required')
    for field in required - {'next_action'}:
        if not isinstance(value[field], list):
            raise ValueError(field + ' must be a list; use [] when empty')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worktree', type=Path, default=Path.cwd())
    parser.add_argument('--state-root', type=Path, default=Path.home() / 'motar_tmp/agent_sessions')
    sub = parser.add_subparsers(dest='command', required=True)
    start = sub.add_parser('start')
    start.add_argument('--agent', required=True)
    start.add_argument('--started-at', help='known start time in ISO8601; never use chat creation as quota-window start')
    start.add_argument('--deadline', help='known quota/user deadline in ISO8601, including timezone')
    start.add_argument('--new-window', action='store_true', help='explicitly start a new account/window after the old session closed')
    sub.add_parser('status')
    check = sub.add_parser('check')
    check.add_argument('--minutes', type=float, required=True)
    check.add_argument('--verification-minutes', type=float, default=20)
    close = sub.add_parser('close')
    close.add_argument('--handoff', type=Path, required=True)
    args = parser.parse_args()
    root = Path(git(args.worktree.resolve(), 'rev-parse', '--show-toplevel')).resolve()
    directory = location(root, args.state_root.expanduser().resolve())
    state = directory / 'session.json'
    with lock(directory):
        record = json.loads(state.read_text()) if state.exists() else None
        clock = now()
        if args.command == 'start':
            declared_start = timestamp(args.started_at) if args.started_at else None
            if declared_start and declared_start > clock:
                raise ValueError('session start cannot be in the future')
            if record and not args.new_window:
                record = tighten(record, timestamp(args.deadline) if args.deadline else None, declared_start)
                atomic_json(state, record)
            else:
                if record and not record.get('closed_at'):
                    raise ValueError('an existing session is not handed off; preserve/close it before a new window')
                previous = record
                started = declared_start or clock
                record = dict(schedule(started, timestamp(args.deadline) if args.deadline else None),
                              session_id=str(uuid.uuid4()), agent=args.agent, worktree=str(root),
                              base_sha=git(root, 'rev-parse', 'HEAD'), predecessor=previous.get('session_id') if previous else None)
                atomic_json(state, record)
        elif record is None:
            raise ValueError('no session clock; start it with the known deadline before working')
        if args.command == 'close':
            handoff_path = directory / ('handoff-' + record['session_id'] + '.json')
            if handoff_path.exists():
                value = json.loads(handoff_path.read_text())
                if value.get('session_id') != record['session_id']:
                    raise ValueError('existing handoff has a different owner; preserve it')
                handoff_valid(value)
            else:
                if record.get('closed_at'):
                    raise ValueError('closed session is missing its immutable handoff; inspect records')
                value = json.loads(args.handoff.read_text())
                handoff_valid(value)
                value.update(session_id=record['session_id'], completed_at=clock.isoformat(), worktree=str(root),
                             branch=git(root, 'branch', '--show-current'), base_sha=record['base_sha'],
                             head=git(root, 'rev-parse', 'HEAD'), uncommitted=git(root, 'status', '--porcelain'),
                             clock={k: record[k] for k in ('started_at', 'deadline', 'wrap_at', 'finish_by', 'deadline_source')})
                atomic_json(handoff_path, value)
            atomic_json(directory / 'latest_handoff.json', value)
            with lock(args.state_root.expanduser().resolve()):
                latest = args.state_root.expanduser().resolve() / 'latest_handoff.json'
                old = json.loads(latest.read_text()) if latest.exists() else None
                if not old or timestamp(old['completed_at']) <= timestamp(value['completed_at']):
                    atomic_json(latest, value)
            record.update(closed_at=value['completed_at'], handoff=str(handoff_path))
            atomic_json(state, record)
        result = dict(record, phase=phase(record, clock), state_file=str(state),
                      latest_handoff=str(directory / 'latest_handoff.json'))
        if args.command == 'check':
            result['fits_time_budget'] = fits(record, clock, args.minutes, args.verification_minutes)
            result['approval_scope'] = 'time check only; existing run/commit/host approvals still apply'
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 2 if args.command == 'check' and not result['fits_time_budget'] else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, subprocess.SubprocessError) as exc:
        raise SystemExit('session clock: ' + str(exc))
