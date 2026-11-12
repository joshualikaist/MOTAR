"""Provider-neutral usage/handoff record; no quota inspector or run authorization.

Usage is an attributed observation, never inferred from elapsed time. Historical
wall-clock records/handoffs remain evidence; only an explicitly supplied current
user deadline may impose a wall-clock limit on a usage-based session.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
import fcntl
import hashlib
import json
import math
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


POLICY = 'observed_usage_v1'
# Preserve the previous closeout fractions (1/5 and 10/300) as policy defaults.
# These are thresholds, not measured account usage or a five-hour allowance.
WRAP_REMAINING_PERCENT = 20.0
STOP_REMAINING_PERCENT = 100.0 / 30.0


def schedule(started, deadline=None):
    if deadline and deadline <= started:
        raise ValueError('explicit user deadline has already passed')
    return {'policy': POLICY, 'started_at': started.isoformat(),
            'deadline': deadline.isoformat() if deadline else None,
            'deadline_source': 'explicit_user' if deadline else 'none',
            'usage': {'remaining_percent': None, 'source': None, 'observed_at': None},
            'wrap_remaining_percent': WRAP_REMAINING_PERCENT,
            'stop_remaining_percent': STOP_REMAINING_PERCENT}


def usage_record(record):
    """Migrate the active state without rewriting immutable historical handoffs."""
    if record.get('policy') == POLICY:
        return dict(record)
    result = dict(record)
    result['legacy_clock'] = {k: result.pop(k) for k in
                              ('deadline', 'wrap_at', 'finish_by', 'deadline_source') if k in result}
    result.update(policy=POLICY, deadline=None, deadline_source='none',
                  usage={'remaining_percent': None, 'source': None, 'observed_at': None},
                  wrap_remaining_percent=WRAP_REMAINING_PERCENT,
                  stop_remaining_percent=STOP_REMAINING_PERCENT)
    return result


def phase(record, clock):
    record = usage_record(record)
    if record.get('closed_at') or (record.get('deadline') and clock >= timestamp(record['deadline'])):
        return 'STOP_AND_HANDOFF'
    usage = record['usage']
    remaining = usage.get('remaining_percent')
    if usage.get('exhaustion_warning') or (remaining is not None and remaining <= record['stop_remaining_percent']):
        return 'STOP_AND_HANDOFF'
    if usage.get('closeout_required') or (remaining is not None and remaining <= record['wrap_remaining_percent']):
        return 'WRAP_UP'
    return 'USAGE_UNKNOWN' if remaining is None else 'WORK'


def fits(record, clock, minutes=0, verification=0, usage_percent=None, verification_percent=0):
    """None means no measured allowance/estimate; it must not be labelled PASS."""
    record = usage_record(record)
    if any(not math.isfinite(x) or x < 0 for x in (minutes, verification)):
        raise ValueError('durations must be finite and nonnegative')
    if any(not math.isfinite(x) or x < 0 for x in (verification_percent,)) or (
            usage_percent is not None and (not math.isfinite(usage_percent) or usage_percent < 0)):
        raise ValueError('usage estimates must be finite and nonnegative')
    if phase(record, clock) in ('WRAP_UP', 'STOP_AND_HANDOFF'):
        return False
    if record.get('deadline') and clock + timedelta(minutes=minutes + verification) > timestamp(record['deadline']):
        return False
    if usage_percent is None or record['usage'].get('remaining_percent') is None:
        return None
    remaining = record['usage']['remaining_percent']
    return (usage_percent <= remaining - record['wrap_remaining_percent']
            and usage_percent + verification_percent <= remaining - record['stop_remaining_percent'])


def tighten(record, deadline=None, started=None):
    """Only explicit current deadlines tighten; elapsed time never creates quota."""
    result = usage_record(record)
    beginning = min(timestamp(record['started_at']), started) if started else timestamp(record['started_at'])
    result['started_at'] = beginning.isoformat()
    if deadline:
        end = min(timestamp(result['deadline']), deadline) if result.get('deadline') else deadline
        result.update(deadline=end.isoformat(), deadline_source='explicit_user')
    return result


def observe_usage(record, remaining, source, observed_at, *, exhaustion_warning=False, closeout_required=False):
    result = usage_record(record)
    if result.get('closed_at'):
        raise ValueError('session is closed; preserve its handoff and start a successor session')
    if not source or not source.strip():
        raise ValueError('usage observations require an attributed source')
    if remaining is not None and (not math.isfinite(remaining) or not 0 <= remaining <= 100):
        raise ValueError('remaining percent must be finite and between 0 and 100')
    previous = result['usage']
    if previous.get('observed_at') and observed_at < timestamp(previous['observed_at']):
        raise ValueError('usage observations cannot regress in time')
    if remaining is not None and previous.get('remaining_percent') is not None and remaining > previous['remaining_percent']:
        raise ValueError('remaining usage increased; record an explicitly confirmed new quota window after handoff')
    if remaining is None and not (exhaustion_warning or closeout_required):
        raise ValueError('supply remaining percent or an observed closeout/exhaustion warning')
    # A later missing percentage or weaker warning cannot erase an earlier stop.
    result['usage'] = {'remaining_percent': previous.get('remaining_percent') if remaining is None else remaining,
                       'source': source.strip(), 'observed_at': observed_at.isoformat(),
                       'remaining_source': previous.get('remaining_source', previous.get('source')) if remaining is None else source.strip(),
                       'remaining_observed_at': previous.get('remaining_observed_at', previous.get('observed_at')) if remaining is None else observed_at.isoformat(),
                       'exhaustion_warning': exhaustion_warning or previous.get('exhaustion_warning', False),
                       'closeout_required': closeout_required or previous.get('closeout_required', False)}
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
    start.add_argument('--started-at', help='activity timestamp only; never a quota-window estimate')
    start.add_argument('--deadline', help='explicit current user wall-clock deadline in ISO8601, including timezone')
    start.add_argument('--new-session', action='store_true', help='successor activity after handoff, retaining last observed usage')
    start.add_argument('--new-window', action='store_true', help='explicitly confirmed new quota window after handoff; clears old usage observations')
    sub.add_parser('status')
    usage = sub.add_parser('usage', help='record a provider-visible or user-reported usage observation')
    usage.add_argument('--remaining-percent', type=float)
    usage.add_argument('--source', required=True)
    usage.add_argument('--observed-at', help='actual observation time in ISO8601; defaults to now')
    usage.add_argument('--closeout-required', action='store_true')
    usage.add_argument('--exhaustion-warning', action='store_true')
    check = sub.add_parser('check')
    check.add_argument('--minutes', type=float, default=0, help='legacy duration estimate, only for an explicit user wall deadline')
    check.add_argument('--verification-minutes', type=float, default=0)
    check.add_argument('--usage-percent', type=float, help='estimated main-work quota cost, excluding verification/closeout')
    check.add_argument('--verification-percent', type=float, default=0,
                       help='estimated quota cost of verification, review, records and other closeout work')
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
            if record and not (args.new_window or args.new_session):
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
                if previous and not args.new_window:
                    record['usage'] = usage_record(previous)['usage']
                record['usage_window_id'] = (previous.get('usage_window_id', previous['session_id'])
                                             if previous and not args.new_window else record['session_id'])
                atomic_json(state, record)
        elif record is None:
            raise ValueError('no session record; start one with usage unknown or an attributed observation')
        if args.command == 'usage':
            observed_at = timestamp(args.observed_at) if args.observed_at else clock
            if observed_at > clock:
                raise ValueError('usage observation cannot be in the future')
            record = observe_usage(record, args.remaining_percent, args.source, observed_at,
                                   exhaustion_warning=args.exhaustion_warning, closeout_required=args.closeout_required)
            atomic_json(state, record)
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
                             session_budget={k: usage_record(record)[k] for k in
                                             ('policy', 'started_at', 'deadline', 'deadline_source', 'usage',
                                              'wrap_remaining_percent', 'stop_remaining_percent')})
                atomic_json(handoff_path, value)
            atomic_json(directory / 'latest_handoff.json', value)
            with lock(args.state_root.expanduser().resolve()):
                latest = args.state_root.expanduser().resolve() / 'latest_handoff.json'
                old = json.loads(latest.read_text()) if latest.exists() else None
                if not old or timestamp(old['completed_at']) <= timestamp(value['completed_at']):
                    atomic_json(latest, value)
            record.update(closed_at=value['completed_at'], handoff=str(handoff_path))
            atomic_json(state, record)
        result = dict(usage_record(record), phase=phase(record, clock), state_file=str(state),
                      latest_handoff=str(directory / 'latest_handoff.json'),
                      usage_observation_status='LAST_OBSERVED_NOT_LIVE' if usage_record(record)['usage'].get('observed_at') else 'UNKNOWN',
                      quota_inspection_available=False)
        if args.command == 'check':
            result['fits_usage_budget'] = fits(record, clock, args.minutes, args.verification_minutes,
                                              args.usage_percent, args.verification_percent)
            result['budget_verdict'] = ('UNKNOWN' if result['fits_usage_budget'] is None else
                                        'ESTIMATE_WITHIN_LAST_OBSERVATION' if result['fits_usage_budget'] else 'DO_NOT_START')
            result['approval_scope'] = 'usage check only; unknown is not PASS; existing run/commit/host approvals still apply'
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 2 if args.command == 'check' and result['fits_usage_budget'] is False else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, subprocess.SubprocessError) as exc:
        raise SystemExit('session record: ' + str(exc))
