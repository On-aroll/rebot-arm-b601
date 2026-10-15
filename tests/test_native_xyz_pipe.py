"""Existing task API -> real pipe worker -> virtual drives -> real model FK.

No real CAN or live web service is used. All state/reports live in a temp root.
"""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from rebot_core import Robot
from rebot_point import PointPlanner
from rebot_motor_power import MotorPowerAPI,CONFIRMATIONS
from rebot_mit_hardware import Envelope
from rebot_xyz_trial import configure_motion,assess_motion
from test_motor_power_offline import FakeStore


class NativeXYZPipe(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.robot=Robot()

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        marker=self.root/'.venv/bin/python';marker.parent.mkdir(parents=True);marker.touch()
        self.processes=[];self.observed=[];self.store=FakeStore();self.before=copy.deepcopy(self.store.state)
        self.start,planner,preview=self.make_preview()
        plan=planner.plans.pop(preview['plan_id'])
        base=json.loads((ROOT/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        raw,config=configure_motion(base,plan['start'],plan['target'],dict(speed_rad_s=.02,timeout_mode='auto',feedforward_mode='native-csp-xyz-v1'))
        self.assertTrue(assess_motion(self.robot,plan['start'],plan['target'],config)['execution_ready'])
        self.trusted=dict(id=preview['plan_id'],expires=time.monotonic()+60,config=raw,revision=self.store.state['revision'],
            expected_start=plan['start'].tolist(),target=plan['target'].tolist())
        self.api=MotorPowerAPI(self.store,8765,self.root,list(zip(self.robot.lower,self.robot.upper)),lambda q:None,
            point_plan=lambda key:self.take(key),popen=self.spawn_virtual)
        self.addCleanup(self.cleanup)
        self.headers={'Host':'127.0.0.1:8765','Origin':'http://127.0.0.1:8765','Sec-Fetch-Site':'same-origin'}
        token=self.api.request('/hardware-point/session',self.headers,'127.0.0.1',{})['token']
        self.headers['X-Motor-Session']=token;self.sequence=0

    def make_preview(self):
        record=json.loads((ROOT/'docs/validation/official-mit-2026-10-15/native-xyz-web-preflight.json').read_text())
        start=np.deg2rad(record['preview']['start_deg']);planner=PointPlanner(self.robot)
        preview=planner.preview(dict(execution_source='hardware-preview',xyz_mm=record['target_xyz_mm'],speed_rad_s=.02),start=start)
        return start,planner,preview

    def take(self,key):
        if self.trusted is None or key!=self.trusted['id']:raise ValueError('consumed/unknown preview')
        plan=self.trusted;self.trusted=None;return plan

    def spawn_virtual(self,argv,**kwargs):
        self.assertEqual(argv,[str(self.root/'.venv/bin/python'),'-u',str(self.root/'scripts/rebot_power_worker.py')])
        kwargs.update(cwd=str(ROOT),env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(ROOT/'scripts'),str(ROOT/'tests')])))
        process=subprocess.Popen([str(ROOT/'.venv/bin/python'),'-u',str(ROOT/'tests/native_xyz_pipe_fixture.py')],**kwargs)
        self.processes.append(process);return process

    def cleanup(self):
        self.api.close()
        for process in self.processes:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill();process.wait() # Only this explicitly substituted virtual fixture.
            process.stdout.close()

    def action(self,action,**body):
        return self.api.request('/hardware-point/action',self.headers,'127.0.0.1',dict(action=action,**body))

    def run_trial(self,*,cancel=None):
        self.action('execute',plan_id=self.trusted['id'],confirmed={k:True for k in CONFIRMATIONS})
        deadline=time.monotonic()+10;cancelled=False
        while time.monotonic()<deadline:
            state=self.api.status();self.observed.append(state)
            if not state['busy']:break
            if cancel and state.get('frames',0)>0 and not cancelled:
                if cancel=='stop':self.action('stop')
                else:self.api.process.stdin.close()
                cancelled=True
            elif not cancelled:
                self.sequence+=1
                try:self.action('heartbeat',sequence=self.sequence,focused=True)
                except ValueError:
                    # Child exit can precede the reader publishing its final
                    # snapshot. Only propagate failure while that child lives;
                    # terminal report loading below still proves completion.
                    process=self.api.process
                    if self.api.status()['busy'] and process is not None and process.poll() is None:raise
            time.sleep(.05)
        else:self.fail('virtual pipe worker did not finish')
        # Completion is delivered before the reader finishes writing the report.
        deadline=time.monotonic()+2
        directory=self.root/'outputs/mit-p5/runtime'
        while time.monotonic()<deadline:
            files=list(directory.glob('*.json'))
            if files:
                try:return json.loads(files[0].read_text())
                except json.JSONDecodeError:pass
            time.sleep(.01)
        self.fail('terminal worker report missing')

    def check_cleanup(self,result):
        self.assertTrue(result['virtual_sdk']);self.assertFalse(result['busy'])
        self.assertTrue(result['restoration']['ok'],result['reason'])
        self.assertEqual(result['stop']['sdk_status_ack_axes'],list(range(1,7)))
        self.assertTrue(result['stop']['stationary_sample'])
        self.assertEqual(self.store.state,self.before)
        calls=result['virtual_sdk_calls']
        self.assertFalse(any(c[0]=='mit' or c[0]=='write' and c[2] not in (0x7005,0x700B,0x7018,0x7017,0x7016) for c in calls))
        self.assertTrue(all(c[1] in range(1,7) for c in calls if c[0] in ('enable','disable','write')))
        for state in self.observed:
            for motor in state.get('motors',{}).values():
                self.assertGreater(motor['positionUpdatedAt'],state['observed_at_ms']-600)

    def test_real_worker_reports_fk_arrival_and_live_six_axis_feedback(self):
        result=self.run_trial();self.assertEqual(result['state'],'ARRIVED',result['reason'])
        self.check_cleanup(result)
        self.assertLessEqual(result['tcp_error_mm'],1.)
        self.assertGreater(len([s for s in self.observed if len(s.get('motors',{}))==6]),2)
        self.assertTrue(any(s.get('control',{}).get('mode')=='native-csp-xyz-v1' for s in self.observed))
        self.assertTrue(result['motion_arrived_before_stop']);self.assertFalse(result['physical_acceptance'])

    def test_wrist_rotation_reaches_j6_even_when_tcp_position_does_not_change(self):
        # Test-only joint intent exercises the existing executor below the XYZ
        # planner. It does not introduce an orientation UI or hardware permit.
        target=self.trusted['target'];target[5]+=np.deg2rad(.4)
        before_target=np.asarray(target).copy();before_target[5]-=np.deg2rad(.4)
        np.testing.assert_allclose(self.robot.fk(target).translation,
                                   self.robot.fk(before_target).translation,atol=1e-12)
        self.assertGreater(np.linalg.norm(self.robot.fk(target).rotation-
                                         self.robot.fk(before_target).rotation),.009)
        self.assertTrue(assess_motion(self.robot,self.start,np.asarray(target),
            Envelope.parse(self.trusted['config']))['execution_ready'])
        result=self.run_trial()
        self.assertEqual(result['state'],'ARRIVED',result['reason']);self.check_cleanup(result)
        self.assertAlmostEqual(result['arrival_feedback']['6']['q'],target[5])
        self.assertGreater(abs(result['arrival_feedback']['6']['q']-self.start[5]),np.deg2rad(.39))
        writes=[c for c in result['virtual_sdk_calls'] if c[0]=='write' and c[1]==6 and c[2]==0x7016]
        self.assertAlmostEqual(writes[0][3],self.start[5])
        self.assertTrue(any(abs(c[3]-target[5])<1e-9 for c in writes))

    def check_cancel(self,cancel):
        result=self.run_trial(cancel=cancel)
        self.assertEqual(result['state'],'CANCELLED',result['reason']);self.check_cleanup(result)
        self.assertFalse(result['motion_arrived_before_stop'])

    def test_api_stop_cancels_running_virtual_worker_and_restores(self):
        self.check_cancel('stop')

    def test_parent_eof_cancels_running_virtual_worker_and_restores(self):
        self.check_cancel('eof')


class NativeXYZHoldPipe(NativeXYZPipe):
    """Same production pipeline with the recorded lift and raw held orientation."""
    def make_preview(self):
        proposal=json.loads((ROOT/'docs/validation/official-mit-2026-10-15/native-xyz-hold-proposal.json').read_text())
        start=np.deg2rad(proposal['assessment']['start_deg'])
        planner=PointPlanner(self.robot)
        preview=planner.preview(dict(execution_source='hardware-preview',xyz_mm=proposal['target_xyz_mm'],
            speed_rad_s=.02,orientation_mode='hold'),start=start)
        self.assertLess(preview['position_error_mm'],.001)
        np.testing.assert_allclose(self.robot.fk(np.deg2rad(preview['q_deg'])).rotation,
                                   self.robot.fk(start).rotation,atol=1e-6)
        return start,planner,preview


if __name__=='__main__':unittest.main()
