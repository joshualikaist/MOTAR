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

    def observed(self, remaining, **kwargs):
        return S.observe_usage(self.record, remaining, 'user-reported account usage', self.start, **kwargs)

    def test_unknown_usage_never_becomes_a_wall_clock_stop(self):
        self.assertIsNone(self.record['deadline'])
        self.assertEqual(S.phase(self.record, self.start + timedelta(days=300)), 'USAGE_UNKNOWN')
        self.assertIsNone(S.fits(self.record, self.start, 500, 20))
        self.assertIsNone(S.fits(self.record, self.start, usage_percent=5))

    def test_observed_usage_work_wrap_and_stop_boundaries(self):
        self.assertEqual(S.phase(self.observed(20.1), self.start), 'WORK')
        self.assertEqual(S.phase(self.observed(20), self.start), 'WRAP_UP')
        self.assertEqual(S.phase(self.observed(100 / 30), self.start), 'STOP_AND_HANDOFF')
        self.assertEqual(S.phase(self.observed(0), self.start), 'STOP_AND_HANDOFF')
        # Elapsed time does not pretend to consume an observed balance.
        self.assertEqual(S.phase(self.observed(60), self.start + timedelta(days=300)), 'WORK')

    def test_explicit_current_user_deadline_wins(self):
        r = S.schedule(self.start, self.start + timedelta(hours=2))
        self.assertEqual(S.timestamp(r['deadline']), self.start + timedelta(hours=2))
        self.assertEqual(S.phase(r, self.start + timedelta(hours=2)), 'STOP_AND_HANDOFF')
        self.assertEqual(r['deadline_source'], 'explicit_user')

    def test_no_automatic_five_hour_limit_on_explicit_deadline(self):
        r = S.schedule(self.start, self.start + timedelta(hours=8))
        self.assertEqual(S.timestamp(r['deadline']), self.start + timedelta(hours=8))

    def test_fit_positive_overrun_and_unknown(self):
        r = self.observed(60)
        self.assertTrue(S.fits(r, self.start, usage_percent=20, verification_percent=20))
        self.assertFalse(S.fits(r, self.start, usage_percent=50, verification_percent=20))
        self.assertIsNone(S.fits(r, self.start, 20, 20))
        self.assertFalse(S.fits(self.observed(20), self.start, usage_percent=1))
        r = S.tighten(r, self.start + timedelta(minutes=20))
        self.assertFalse(S.fits(r, self.start, 20, 1, usage_percent=1))
        for bad in (-1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                S.fits(r, self.start, usage_percent=bad)
            with self.assertRaises(ValueError):
                S.fits(r, self.start, minutes=bad)

    def test_main_work_preserves_closeout_fraction_and_total_preserves_stop_fraction(self):
        r = self.observed(30)
        self.assertFalse(S.fits(r, self.start, usage_percent=25, verification_percent=0))
        self.assertTrue(S.fits(r, self.start, usage_percent=10, verification_percent=16))
        self.assertFalse(S.fits(r, self.start, usage_percent=10, verification_percent=17))
        self.assertFalse(S.fits(r, self.start, usage_percent=10.1, verification_percent=0))

    def test_closed_session_cannot_admit_work(self):
        self.record['closed_at'] = self.start.isoformat()
        self.assertFalse(S.fits(self.record, self.start, usage_percent=1))
        with self.assertRaises(ValueError):
            S.observe_usage(self.record, 70, 'user', self.start)

    def test_existing_explicit_deadline_tightens_but_never_extends(self):
        shorter = S.tighten(self.record, self.start + timedelta(hours=2))
        self.assertEqual(S.timestamp(shorter['deadline']), self.start + timedelta(hours=2))
        self.assertEqual(S.tighten(shorter, self.start + timedelta(hours=9)), shorter)
        shorter['closed_at'] = self.start.isoformat()
        self.assertIn('closed_at', S.tighten(shorter, self.start + timedelta(hours=1)))
        expired = S.tighten(self.record, self.start - timedelta(minutes=1))
        self.assertEqual(S.phase(expired, self.start), 'STOP_AND_HANDOFF')

    def test_legacy_inferred_clock_becomes_historical_metadata(self):
        legacy = dict(started_at=self.start.isoformat(),
                      deadline=(self.start + timedelta(hours=5)).isoformat(),
                      wrap_at=(self.start + timedelta(hours=4)).isoformat(),
                      finish_by=(self.start + timedelta(hours=4, minutes=50)).isoformat(),
                      deadline_source='five_hour_fallback_not_verified_quota')
        original = dict(legacy)
        migrated = S.usage_record(legacy)
        self.assertIsNone(migrated['deadline'])
        self.assertEqual(migrated['legacy_clock']['deadline'], legacy['deadline'])
        self.assertEqual(legacy, original)
        self.assertEqual(S.phase(migrated, self.start + timedelta(days=300)), 'USAGE_UNKNOWN')
        self.assertEqual(S.usage_record(migrated), migrated)

    def test_usage_observations_require_valid_source_and_monotonic_snapshot(self):
        for bad in (-1, 101, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                self.observed(bad)
        with self.assertRaises(ValueError):
            S.observe_usage(self.record, 60, ' ', self.start)
        with self.assertRaises(ValueError):
            self.observed(None)
        r = self.observed(60)
        with self.assertRaises(ValueError):
            S.observe_usage(r, 61, 'user', self.start)
        with self.assertRaises(ValueError):
            S.observe_usage(r, 59, 'user', self.start - timedelta(seconds=1))
        newer = S.observe_usage(r, 59, 'provider display', self.start + timedelta(seconds=1))
        self.assertEqual(newer['usage']['remaining_percent'], 59)
        self.assertEqual(r['usage']['remaining_percent'], 60)

    def test_warning_preserves_old_percentage_attribution_and_cannot_reopen(self):
        r = self.observed(60)
        later = self.start + timedelta(hours=2)
        warned = S.observe_usage(r, None, 'provider near-limit warning', later, exhaustion_warning=True)
        self.assertEqual(S.phase(warned, later), 'STOP_AND_HANDOFF')
        self.assertEqual(warned['usage']['remaining_observed_at'], self.start.isoformat())
        self.assertEqual(warned['usage']['remaining_source'], 'user-reported account usage')
        weaker = S.observe_usage(warned, 59, 'user snapshot', later)
        self.assertEqual(S.phase(weaker, later), 'STOP_AND_HANDOFF')
        wrap = self.observed(None, closeout_required=True)
        self.assertEqual(S.phase(wrap, self.start), 'WRAP_UP')

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

    def test_cli_reconnect_and_subdirectory_preserve_session(self):
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
            self.assertIsNone(second['deadline'])
            self.assertEqual(second['phase'], 'USAGE_UNKNOWN')
            self.assertFalse(second['quota_inspection_available'])
            self.assertEqual(second['usage_observation_status'], 'UNKNOWN')
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

    def test_cli_successor_retains_usage_and_explicit_new_window_resets_it(self):
        import json
        import subprocess
        import sys
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'repo'
            root.mkdir()
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            subprocess.run(['git', '-C', str(root), '-c', 'user.name=Session Test', '-c', 'user.email=test@invalid',
                            '-c', 'commit.gpgsign=false', 'commit', '--allow-empty', '-qm', 'fixture'], check=True)
            argv = [sys.executable, '-B', str(Path(S.__file__).resolve()), '--worktree', str(root),
                    '--state-root', str(Path(directory) / 'state')]
            def call(*args):
                return json.loads(subprocess.check_output(argv + list(args), text=True))
            call('start', '--agent', 'test')
            usage = call('usage', '--remaining-percent', '19', '--source', 'user account display')
            self.assertEqual(usage['phase'], 'WRAP_UP')
            self.assertEqual(usage['usage_observation_status'], 'LAST_OBSERVED_NOT_LIVE')
            summary = Path(directory) / 'handoff.json'
            summary.write_text(json.dumps(dict(completed=[], remaining=[], running_processes=[], pending_approvals=[],
                                              validation=[], next_action='Read WORKLOG.')))
            closed = call('close', '--handoff', str(summary))
            original = Path(closed['handoff']).read_bytes()
            successor = call('start', '--agent', 'test', '--new-session')
            self.assertEqual(successor['usage'], usage['usage'])
            self.assertEqual(successor['usage_window_id'], usage['usage_window_id'])
            self.assertEqual(successor['phase'], 'WRAP_UP')
            self.assertEqual(Path(closed['handoff']).read_bytes(), original)
            call('close', '--handoff', str(summary))
            reset = call('start', '--agent', 'test', '--new-window')
            self.assertEqual(reset['phase'], 'USAGE_UNKNOWN')
            self.assertIsNone(reset['usage']['remaining_percent'])
            self.assertNotEqual(reset['usage_window_id'], successor['usage_window_id'])
            unknown = call('check', '--minutes', '500')
            self.assertIsNone(unknown['fits_usage_budget'])
            self.assertEqual(unknown['budget_verdict'], 'UNKNOWN')



if __name__ == '__main__':
    unittest.main()
