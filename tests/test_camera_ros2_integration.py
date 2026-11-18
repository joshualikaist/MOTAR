"""Actual frozen CAM CPU policy through ROS-shaped messages and PX4 payload gates.

Synthetic fixtures exercise source composition, not real sensors, ROS delivery,
latency, localization or flight. Each seed uses a fresh isolated interpreter.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = r'''
import json,math,sys,tempfile
from pathlib import Path
import numpy as np
from camera_policy_contract import build_contract,estimator_height_assertion
from camera_policy_runtime import CameraPolicyRuntime
from camera_ros2_session import RosPolicySession
from camera_ros2_px4 import Px4MessageBinding,PX4_VERSION,PX4_MESSAGE_SHA256
from camera_ros2_px4_node import BridgeSession,unconfigured_template
from test_camera_ros2_session import binding,messages
from test_camera_ros2_px4 import snapshot
# Repair r1 (F6): estimator-height assertion as a CPU fixture; (F8): publication needs a fence or a waiver.
evidence_dir=tempfile.TemporaryDirectory()
evidence=Path(evidence_dir.name)/'estimator_height_fixture.txt'
evidence.write_text('CPU test fixture: estimator z is the synthetic height above the floor. Not evidence.')
c=build_contract(int(sys.argv[1]),estimator_height=estimator_height_assertion(evidence))
runtime=CameraPolicyRuntime(c)
cfg=unconfigured_template()
cfg.update(status='CONFIGURED',enable_offboard_publication=True,frames=dict(world='odom',body='base_link'),
           geofence_waiver=dict(reason='CPU software composition fixture; geofence covered by test_camera_runtime_fixes_r2'))
bridge=BridgeSession(cfg,Px4MessageBinding(PX4_VERSION,PX4_MESSAGE_SHA256,'synthetic software composition only'))
pairs=[]
with tempfile.TemporaryDirectory() as tmp:
 session=RosPolicySession(runtime,binding(Path(tmp)/'CPU_FIXTURE_NOT_HARDWARE.txt'))
 for i in range(14):
  now,wall=100.+i*.1,20.+i*.1
  telemetry=snapshot(now,wall,nav=2 if i<11 else 14,z=-1.5)
  od=telemetry['odometry']
  od.update(q=[math.sqrt(.5),0.,0.,math.sqrt(.5)],position=[0.,0.,-1.5],velocity=[0.,0.,0.],angular_velocity=[0.,0.,0.])
  # Shared synthetic capture clock; do not substitute publication now in production.
  od['timestamp_sample']=telemetry['local_position']['timestamp_sample']=round(now*1e6)
  for kind in ('odometry','local_position','status'):bridge.offer(kind,telemetry[kind],receipt=wall)
  state,pair=bridge.poll(now=now,wall=wall)
  assert state is not None and bridge.fault is None
  if i==0:pairs.append(bridge.prime(now=now,wall=wall))
  elif pair is not None:pairs.append(pair)
  m=messages(now)
  rgb=np.zeros((90,160,3),np.uint8);rgb[40:50,75:85]=[255,0,0]
  depth=np.full((90,160),2000,dtype='<u2')
  m['rgb'].width=160;m['rgb'].height=90;m['rgb'].step=480;m['rgb'].data=rgb.tobytes()
  m['depth'].width=160;m['depth'].height=90;m['depth'].step=320;m['depth'].data=depth.tobytes()
  k=c['camera'];info=m['camera_info'];info.width=160;info.height=90
  info.k=[k['fx'],0.,k['cx'],0.,k['fy'],k['cy'],0.,0.,1.]
  info.p=[k['fx'],0.,k['cx'],0.,0.,k['fy'],k['cy'],0.,0.,0.,1.,0.]
  pose=m['odometry'].pose.pose
  pose.position.x,pose.position.y,pose.position.z=state['pos']
  pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=state['quat_xyzw']
  for kind,message in m.items():session.offer(kind,message,received_monotonic_s=wall)
  item=session.prepare(now_ros_s=now,now_monotonic_s=wall)
  assert item is not None
  result=session.infer(item)
  assert len(result['observation'])==898 and not result['lidar_visible'] and not result['command_sent']
  proposal=session.finish(item,result,now_ros_s=now,now_monotonic_s=wall)
  bridge.offer_command(proposal,receipt=wall)
 assert bridge.gate.state=='ACTIVE' and len(pairs)==14
 assert session.sequence==14 and all(np.isfinite(pair['trajectory_setpoint']['velocity'][:2]).all() for pair in pairs)
 bridge.revoke('synthetic test end')
 assert bridge.poll(now=101.4,wall=21.4)==(None,None)
 assert 'rclpy' not in sys.modules and 'px4_msgs' not in sys.modules
 assert 'warp' not in sys.modules and 'isaacgym' not in sys.modules
 print(json.dumps(dict(seed=c['seed'],frames=14,observation_dim=898,payload_pairs=len(pairs),
   commands_sent=0,ros_graph='NOT_STARTED',hardware='NOT_TESTED')))
'''


class ActualPolicyRosComposition(unittest.TestCase):
    def test_both_bound_seeds_execute_ros_message_path_without_lidar_or_vehicle_io(self):
        for seed in (1,2):
            with self.subTest(seed=seed):
                env=dict(os.environ,CUDA_VISIBLE_DEVICES='',PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',
                         MKL_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',
                         PYTHONPATH=os.pathsep.join([str(ROOT/'tools'),str(ROOT/'tests'),str(ROOT)]))
                result=subprocess.run([sys.executable,'-B','-c',SCRIPT,str(seed)],cwd=ROOT,env=env,
                    stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=90)
                self.assertEqual(result.returncode,0,result.stdout[-2000:]+result.stderr[-3000:])
                value=json.loads(result.stdout.strip().splitlines()[-1])
                self.assertEqual(value['seed'],seed)
                self.assertEqual(value['frames'],14)
                self.assertEqual(value['payload_pairs'],14)
                self.assertEqual(value['commands_sent'],0)


if __name__=='__main__':unittest.main()
