"""Session-boundary and handoff guards; pure CPU, no provider credentials or runtime jobs."""
import importlib.util
from datetime import datetime, timedelta, timezone
from pathlib import Path
import tempfile
import unittest


SPEC = importlib.util.spec_from_file_location('agent_session', Path(__file__).resolve().parents[1] / 'tools/agent_session.py')
S = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(S)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.start = datetime(2026, 11, 5, 0, tzinfo=timezone.utc)
        self.record = S.schedule(self.start)

    def test_work_wrap_and_stop_boundaries(self):
        self.assertEqual(S.phase(self.record, self.start + timedelta(hours=3, minutes=59)), 'WORK')
        self.assertEqual(S.phase(self.record, self.start + timedelta(hours=4)), 'WRAP_UP')
        self.assertEqual(S.phase(self.record, self.start + timedelta(hours=4, minutes=50)), 'STOP_AND_HANDOFF')

    def test_explicit_earlier_deadline_wins(self):
        r = S.schedule(self.start, self.start + timedelta(hours=2))
        self.assertEqual(S.timestamp(r['finish_by']), self.start + timedelta(hours=1, minutes=50))
        self.assertEqual(r['deadline_source'], 'explicit')

    def test_later_deadline_does_not_extend_five_hours(self):
        r = S.schedule(self.start, self.start + timedelta(hours=8))
        self.assertEqual(S.timestamp(r['deadline']), self.start + timedelta(hours=5))

    def test_fit_positive_and_overrun_negative(self):
        clock = self.start + timedelta(hours=3)
        self.assertTrue(S.fits(self.record, clock, 20, 20))
        self.assertFalse(S.fits(self.record, clock, 100, 20))
        self.assertFalse(S.fits(self.record, self.start + timedelta(hours=4), 1, 1))

    def test_closed_session_cannot_admit_work(self):
        self.record['closed_at'] = self.start.isoformat()
        self.assertFalse(S.fits(self.record, self.start, 1, 1))

    def test_existing_deadline_tightens_but_never_extends_or_reopens(self):
        shorter = S.tighten(self.record, self.start + timedelta(hours=2))
        self.assertEqual(S.timestamp(shorter['deadline']), self.start + timedelta(hours=2))
        self.assertEqual(S.tighten(shorter, self.start + timedelta(hours=9)), shorter)
        shorter['closed_at'] = self.start.isoformat()
        self.assertIn('closed_at', S.tighten(shorter, self.start + timedelta(hours=1)))
        expired = S.tighten(self.record, self.start - timedelta(minutes=1))
        self.assertEqual(S.phase(expired, self.start), 'STOP_AND_HANDOFF')

    def test_timezone_and_past_deadline_rejected(self):
        with self.assertRaises(ValueError):
            S.timestamp('2026-11-05T00:00:00')
        with self.assertRaises(ValueError):
            S.schedule(self.start, self.start - timedelta(minutes=1))

    def test_handoff_positive_and_missing_negative(self):
        value = {k: [] for k in ['completed', 'remaining', 'running_processes', 'pending_approvals', 'validation']}
        value['next_action'] = 'Read WORKLOG and prepare the next approved unit.'
        S.handoff_valid(value)
        with self.assertRaises(ValueError):
            S.handoff_valid({})
        value['next_action'] = ' '
        with self.assertRaises(ValueError):
            S.handoff_valid(value)

    def test_atomic_record_can_be_read_back(self):
        import json
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'session.json'
            S.atomic_json(p, self.record)
            self.assertEqual(json.loads(p.read_text()), self.record)
            self.assertEqual(list(Path(directory).iterdir()), [p])

    def test_cli_reconnect_and_subdirectory_preserve_clock(self):
        import json
        import subprocess
        import sys
        script = Path(S.__file__).resolve()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'repo'
            root.mkdir()
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            subprocess.run(['git', '-C', str(root), '-c', 'user.name=Session Test', '-c', 'user.email=test@invalid',
                            '-c', 'commit.gpgsign=false', 'commit', '--allow-empty', '-qm', 'fixture'], check=True)
            nested = root / 'nested'
            nested.mkdir()
            state = Path(directory) / 'state'
            def call(worktree):
                return json.loads(subprocess.check_output([sys.executable, '-B', str(script), '--worktree',
                                                          str(worktree), '--state-root', str(state), 'start',
                                                          '--agent', 'test'], text=True))
            first, second = call(root), call(nested)
            self.assertEqual(first['session_id'], second['session_id'])
            self.assertEqual(first['started_at'], second['started_at'])
            self.assertEqual(first['deadline'], second['deadline'])
            summary = Path(directory) / 'handoff.json'
            summary.write_text(json.dumps(dict(completed=[], remaining=[], running_processes=[], pending_approvals=[],
                                              validation=[], next_action='Read WORKLOG.')))
            argv = [sys.executable, '-B', str(script), '--worktree', str(root), '--state-root', str(state)]
            closed = json.loads(subprocess.check_output(argv + ['close', '--handoff', str(summary)], text=True))
            handoff = Path(closed['handoff'])
            original = handoff.read_bytes()
            # Simulate interruption after the immutable handoff write but before closed_at was persisted.
            session_path = Path(closed['state_file'])
            interrupted = json.loads(session_path.read_text())
            interrupted.pop('closed_at')
            interrupted.pop('handoff')
            session_path.write_text(json.dumps(interrupted))
            summary.unlink()
            recovered = json.loads(subprocess.check_output(argv + ['close', '--handoff', str(summary)], text=True))
            self.assertEqual(recovered['phase'], 'STOP_AND_HANDOFF')
            self.assertEqual(handoff.read_bytes(), original)
            again = json.loads(subprocess.check_output(argv + ['close', '--handoff', str(summary)], text=True))
            self.assertEqual(again['closed_at'], recovered['closed_at'])


if __name__ == '__main__':
    unittest.main()
