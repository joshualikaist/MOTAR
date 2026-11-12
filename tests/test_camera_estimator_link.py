import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from camera_estimator_link import EstimatorLink, EstimatorLinkError
from sitl_camera import TM_POS_Z_VEL_XY_YAWRATE, CameraControlError


class Message(SimpleNamespace):
    def get_srcSystem(self): return getattr(self, 'sysid', 1)
    def get_srcComponent(self): return getattr(self, 'compid', 1)
    def get_type(self): return self.kind


class EstimatorReceipts(unittest.TestCase):
    def interfaces(self, publish=False):
        now, sends = [10.], []
        connection = SimpleNamespace(mav=SimpleNamespace(set_position_target_local_ned_send=lambda *a: sends.append(a)))
        link = EstimatorLink(connection, system_id=1, component_id=1, publish=publish, clock=lambda: now[0])
        pos = Message(kind='LOCAL_POSITION_NED', time_boot_ms=1000, x=0., y=0., z=-1., vx=0., vy=0., vz=0.)
        att = Message(kind='ATTITUDE', time_boot_ms=1000, roll=0., pitch=0., yaw=0., rollspeed=0., pitchspeed=0., yawspeed=0.)
        hb = Message(kind='HEARTBEAT', base_mode=128, custom_mode=6<<16)
        for m in (pos, att, hb): self.assertTrue(link.accept(m))
        return link, now, sends, pos, att, hb

    def test_snapshot_copies_receipts_and_passive_proposal_sends_nothing(self):
        link, now, sends, pos, _, _ = self.interfaces()
        snap = link.snapshot()
        self.assertEqual(set(snap), {'pos','att','hb','pos_wall_t','att_wall_t','hb_wall_t'})
        snap['pos'] = None
        self.assertIsNotNone(link.snapshot()['pos'])
        proposal = link.setpoint(TM_POS_Z_VEL_XY_YAWRATE, z=-1., vx=1.)
        self.assertFalse(proposal['published'])
        self.assertEqual(sends, [])
        self.assertEqual(link.snapshot()['pos_wall_t'], now[0])

    def test_duplicate_does_not_refresh_position_receipt_and_stale_refuses_publication(self):
        link, now, sends, pos, _, hb = self.interfaces(publish=True)
        link.setpoint(TM_POS_Z_VEL_XY_YAWRATE, z=-1., vx=1.)
        self.assertEqual(len(sends), 1)
        now[0] += .2
        self.assertFalse(link.accept(pos))
        link.accept(hb)
        self.assertEqual(link.snapshot()['pos_wall_t'], 10.)
        with self.assertRaises(CameraControlError):
            link.setpoint(TM_POS_Z_VEL_XY_YAWRATE, z=-1., vx=1.)
        self.assertEqual(len(sends), 1)

    def test_wrong_sources_and_hil_never_refresh_data(self):
        link, now, sends, pos, _, hb = self.interfaces()
        pos.sysid, hb.compid = 2, 2
        now[0] += .1
        self.assertFalse(link.accept(pos))
        self.assertFalse(link.accept(hb))
        self.assertFalse(link.accept(Message(kind='HIL_STATE_QUATERNION')))
        self.assertEqual(link.snapshot()['pos_wall_t'], 10.)

    def test_reboot_or_nonfinite_data_poison_the_session(self):
        for bad in ('reboot', 'nan'):
            with self.subTest(bad=bad):
                link, _, sends, pos, _, _ = self.interfaces(publish=True)
                if bad == 'reboot': pos.time_boot_ms = 900
                else: pos.x = float('nan')
                with self.assertRaises(EstimatorLinkError): link.accept(pos)
                with self.assertRaises(EstimatorLinkError): link.snapshot()
                self.assertEqual(sends, [])

    def test_explicit_publication_fields_and_mode_loss(self):
        link, _, sends, _, _, hb = self.interfaces(publish=True)
        link.setpoint(TM_POS_Z_VEL_XY_YAWRATE, z=-1., vx=2., vy=1., yaw_rate=-.3)
        self.assertEqual(sends[0][1:4], (1, 1, 1))
        self.assertEqual(sends[0][-1], -.3)
        hb.custom_mode=0
        link.accept(hb)
        with self.assertRaises(CameraControlError):
            link.setpoint(TM_POS_Z_VEL_XY_YAWRATE, z=-1., vx=1.)
        self.assertEqual(len(sends), 1)


if __name__ == '__main__':
    unittest.main()
