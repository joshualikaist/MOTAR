"""Publication-boundary controls using ROS-shaped messages, no ROS graph."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import camera_ros2_adapter as A
from camera_ros2_session import RosPolicySession, SessionFault


def header(frame, t):
    ns = round(t*1e9)
    return NS(frame_id=frame, stamp=NS(sec=ns//1000000000, nanosec=ns%1000000000))


def messages(t=3.):
    rgb = np.zeros((2, 2, 3), np.uint8)
    z = np.full((2, 2), 2000, '<u2')
    return dict(rgb=NS(header=header('camera_optical', t), width=2, height=2, step=6,
                       encoding='rgb8', is_bigendian=0, data=rgb.tobytes()),
        depth=NS(header=header('camera_optical', t), width=2, height=2, step=4,
                 encoding='16UC1', is_bigendian=0, data=z.tobytes()),
        camera_info=NS(header=header('camera_optical', t), width=2, height=2, binning_x=0, binning_y=0,
            roi=NS(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False),
            k=[20.,0,.5,0,21.,.5,0,0,1.], p=[20.,0,.5,0,0,21.,.5,0,0,0,1.,0],
            r=np.eye(3).ravel().tolist(), d=[]),
        odometry=NS(header=header('odom', t), child_frame_id='base_link',
            pose=NS(pose=NS(position=NS(x=0., y=0., z=1.), orientation=NS(x=0., y=0., z=0., w=1.))),
            twist=NS(twist=NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.)))))


def binding(path):
    path.write_text('CPU fixture calibration only, not real hardware evidence')
    proof = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    b = A.unconfigured_template()
    b.update(status='CONFIGURED', frames=dict(world='odom', body='base_link', optical='camera_optical'))
    b['geometry'] = dict(mount_flu_m=[.1,0,.03], optical_to_body_rotation=A.CAMERA_FLU_TO_OPTICAL.T.tolist(), provenance=proof)
    b['producer'] = dict(rectified=True, depth_aligned_to_rgb=True, provenance=proof)
    b['clock']['source'] = 'fixture shared ROS clock'
    return b


class Runtime:
    def __init__(self): self.calls, self.resets, self.failed = [], 0, False
    def reset(self): self.resets += 1; self.failed = False
    def invalidate(self): self.failed = True
    def step(self, f, s, *, now_s):
        self.calls.append((f, s, now_s))
        return dict(t=now_s, velocity_world_enu_mps=[.2,.1,0.], yaw_rate_rps=.3)


class RosSessionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.runtime = Runtime()
        self.session = RosPolicySession(self.runtime, binding(Path(self.tmp.name)/'fixture.txt'))

    def offer(self, t=3., receipt=100., msgs=None):
        for k, m in (msgs or messages(t)).items():
            self.session.offer(k, m, received_monotonic_s=receipt)

    def tick(self, t=3., wall=100.):
        item = self.session.prepare(now_ros_s=t, now_monotonic_s=wall)
        self.assertIsNotNone(item)
        result = self.session.infer(item)
        return self.session.finish(item, result, now_ros_s=t, now_monotonic_s=wall)

    def test_three_positive_proposals_keep_capture_stamp_and_no_lidar(self):
        for i in range(3):
            t, wall = 3.+i*.1, 100.+i*.1
            self.offer(t, wall)
            proposal = self.tick(t, wall)
            self.assertEqual(proposal['capture_stamp_s'], t)
            self.assertEqual(proposal['frame_id'], 'odom')
            self.assertFalse(proposal['command_sent_to_vehicle'])
            self.assertEqual(proposal['velocity_enu_mps'], [.2,.1,0.])
        self.assertEqual(len(self.runtime.calls), 3)
        self.assertTrue(all('ranges' not in f and set(s)=={'pos','vel','quat_xyzw','yaw_rate_rps','t'}
                            for f,s,_ in self.runtime.calls))

    def test_no_fresh_bundle_never_runs_policy_and_old_messages_do_not_renew_receipt(self):
        self.assertIsNone(self.session.prepare(now_ros_s=3., now_monotonic_s=100.))
        self.offer()
        self.tick()
        original = self.session._queues['rgb'][-1][1]
        self.assertFalse(self.session.offer('rgb', messages()['rgb'], received_monotonic_s=100.1))
        self.assertEqual(self.session._queues['rgb'][-1][1], original)
        with self.assertRaisesRegex(SessionFault, 'stale'):
            self.session.watchdog(now_ros_s=3.2, now_monotonic_s=100.2)
        self.assertEqual(len(self.runtime.calls), 1)

    def test_skew_and_wrong_frame_cannot_publish(self):
        msgs = messages()
        msgs['depth'].header.stamp.nanosec = 30000000
        self.offer(msgs=msgs)
        self.assertIsNone(self.session.prepare(now_ros_s=3.03, now_monotonic_s=100.))
        self.session.reset()
        msgs = messages()
        msgs['odometry'].header.frame_id = 'map_ned'
        self.offer(msgs=msgs)
        item = self.session.prepare(now_ros_s=3., now_monotonic_s=100.)
        with self.assertRaises(ValueError): self.session.infer(item)
        self.session.fail_inference('wrong frame')
        self.assertTrue(self.runtime.failed)
        self.assertEqual(len(self.runtime.calls), 0)

    def test_late_inference_clock_pause_and_clock_jump_latch(self):
        self.offer()
        item = self.session.prepare(now_ros_s=3., now_monotonic_s=100.)
        result = self.session.infer(item)
        with self.assertRaisesRegex(SessionFault, 'budget'):
            self.session.finish(item, result, now_ros_s=3.16, now_monotonic_s=100.16)
        self.assertTrue(self.runtime.failed)
        self.session.reset()
        self.offer()
        self.tick()
        with self.assertRaisesRegex(SessionFault, 'paused'):
            self.session.watchdog(now_ros_s=3., now_monotonic_s=100.16)
        self.session.reset()
        self.offer()
        self.tick()
        with self.assertRaisesRegex(SessionFault, 'backwards'):
            self.session.watchdog(now_ros_s=2., now_monotonic_s=100.01)

    def test_revocation_during_worker_discards_result_and_requires_explicit_reset(self):
        self.offer()
        item = self.session.prepare(now_ros_s=3., now_monotonic_s=100.)
        self.session.invalidate('operator stop')
        self.assertFalse(self.runtime.failed)  # Do not race mutation with the worker.
        with self.assertRaisesRegex(SessionFault, 'running'):
            self.session.reset()
        result = self.session.infer(item)
        with self.assertRaises(SessionFault):
            self.session.finish(item, result, now_ros_s=3.01, now_monotonic_s=100.01)
        self.assertTrue(self.runtime.failed)
        self.session.reset()
        self.offer(5., 101.)
        self.tick(5., 101.)
        self.assertFalse(self.runtime.failed)

    def test_calibration_change_revokes_and_messages_are_copied(self):
        msgs = messages()
        self.offer(msgs=msgs)
        msgs['rgb'].data = b'bad'
        self.tick()  # Snapshot owns the original buffer.
        changed = messages(3.1)['camera_info']
        changed.p[0] += 1.
        with self.assertRaisesRegex(SessionFault, 'calibration changed'):
            self.session.offer('camera_info', changed, received_monotonic_s=100.1)
        self.assertTrue(self.runtime.failed)

    def test_zero_camera_info_stamp_cannot_enter_initial_small_clock_bundle(self):
        msgs = messages(.01)
        msgs['camera_info'].header.stamp = NS(sec=0, nanosec=0)
        with self.assertRaisesRegex(SessionFault, 'nonzero'):
            self.offer(.01, 100., msgs)
        self.assertEqual(self.runtime.calls, [])
        self.assertTrue(self.runtime.failed)
        self.session.reset()
        self.offer(.01, 100.)
        self.assertGreater(self.tick(.01, 100.)['capture_stamp_s'], 0)
        self.assertEqual(len(self.runtime.calls), 1)

    def test_out_of_range_actor_output_and_future_time_never_publish(self):
        self.offer()
        item = self.session.prepare(now_ros_s=3., now_monotonic_s=100.)
        with self.assertRaisesRegex(SessionFault, 'command'):
            self.session.finish(item, dict(velocity_world_enu_mps=[100.,0,0], yaw_rate_rps=0),
                                now_ros_s=3., now_monotonic_s=100.)
        self.session.reset()
        self.offer(30., 100.)
        self.assertIsNone(self.session.prepare(now_ros_s=3., now_monotonic_s=100.))
        self.assertEqual(len(self.runtime.calls), 0)


class RosNodeMetadata(unittest.TestCase):
    def test_unconfigured_check_is_ros_and_torch_free_in_a_fresh_process(self):
        code = """
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[2])/'tools'))
from camera_ros2_node import main
result=main(config_path=Path(sys.argv[1]),source_root=Path(sys.argv[2]),check_only=True)
assert result==2
assert 'rclpy' not in sys.modules and 'torch' not in sys.modules and 'aerial_gym' not in sys.modules
"""
        p = subprocess.run([sys.executable,'-B','-c',code,
             str(ROOT/'ros2/motar_camera_policy/config/unconfigured.json'),str(ROOT)],
             capture_output=True,text=True,timeout=20)
        self.assertEqual(p.returncode,0,p.stdout+p.stderr)
        self.assertEqual(json.loads(p.stdout)['ros_graph'],'NOT_STARTED')


if __name__ == '__main__': unittest.main()
