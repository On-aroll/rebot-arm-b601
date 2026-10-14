"""Offline real-MuJoCo synchronization checks; never imports a CAN driver."""
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_core import Robot, HOME
from rebot_pose_sync import synchronize_pose, feedback_pose

class PoseSync(unittest.TestCase):
    def setUp(self):
        self.robot=Robot()
        self.q=HOME+np.array([.03,.02,-.01,.01,0.,.01])
        self.mapping=dict(version=1,axes=[dict(motorId=i,direction=1,offsetDeg=0) for i in range(1,7)])
        self.motors={str(i):dict(id=i,model='rs-06' if i<=3 else 'rs-00',mechPosRad=float(self.q[i-1]),positionUpdatedAt=100000.,error=None) for i in range(1,7)}

    def sync(self): return synchronize_pose(self.robot,self.motors,self.mapping,now=101.)

    def test_q_ctrl_fk_signs_and_repeat(self):
        result=self.sync()
        self.assertFalse(result['hardware_motion']); self.assertFalse(result['hardware_start_authority'])
        np.testing.assert_allclose(self.robot.q,self.q,atol=1e-12)
        np.testing.assert_allclose(self.robot.d.ctrl[self.robot.act_ids],-self.q,atol=1e-12)
        np.testing.assert_allclose(self.robot.d.xpos[self.robot.m.body('gripper_end').id],np.array(result['xyz_mm'])/1000,atol=1e-3)
        previous=self.robot.d.qpos.copy(); target=self.robot.target.copy()
        self.sync(); np.testing.assert_array_equal(previous,self.robot.d.qpos); np.testing.assert_array_equal(target,self.robot.target)
        for _ in range(10): self.robot.step()
        self.assertLess(float(np.max(np.abs(self.robot.q-self.q))),.002)

    def test_failure_is_atomic(self):
        before=self.robot.d.qpos.copy(); ctrl=self.robot.d.ctrl.copy(); target=self.robot.target.copy()
        for invalid in ('old','future','nan','range','id','model','mapping','missing','bool'):
            with self.subTest(invalid=invalid):
                motors=copy.deepcopy(self.motors); mapping=copy.deepcopy(self.mapping)
                if invalid=='old': motors['1']['positionUpdatedAt']=1
                if invalid=='future': motors['1']['positionUpdatedAt']=102000
                if invalid=='nan': motors['1']['mechPosRad']=float('nan')
                if invalid=='range': motors['1']['mechPosRad']=10.
                if invalid=='id': motors['1']['id']=2
                if invalid=='model': motors['1']['model']='rs-00'
                if invalid=='mapping': mapping['axes'][0]['direction']=-1
                if invalid=='missing': del motors['6']
                if invalid=='bool': motors['1']['mechPosRad']=True
                with self.assertRaises(ValueError): synchronize_pose(self.robot,motors,mapping,now=101)
                np.testing.assert_array_equal(before,self.robot.d.qpos); np.testing.assert_array_equal(ctrl,self.robot.d.ctrl); np.testing.assert_array_equal(target,self.robot.target)

    def test_collision_and_preserve_gripper(self):
        before=self.robot.d.qpos.copy()
        with patch.object(self.robot,'contacts',return_value=['collision']):
            with self.assertRaisesRegex(ValueError,'碰撞'): self.sync()
        np.testing.assert_array_equal(before,self.robot.d.qpos)
        self.robot.command(HOME,.02)
        before=self.robot.d.qpos[self.robot.finger_qadr].copy(); ctrl=self.robot.d.ctrl[self.robot.finger_acts].copy()
        self.sync()
        np.testing.assert_array_equal(before,self.robot.d.qpos[self.robot.finger_qadr]); np.testing.assert_array_equal(ctrl,self.robot.d.ctrl[self.robot.finger_acts])

    def test_preview_retains_subdegree_boundary_reading_without_syncing_or_changing_limits(self):
        raw=float(self.robot.lower[1]-np.deg2rad(.39))
        self.motors['2']['mechPosRad']=raw
        before=self.robot.d.qpos.copy(); lower=self.robot.lower.copy()
        q,_=feedback_pose(self.robot,self.motors,self.mapping,now=101,start_tolerance=np.deg2rad(1))
        self.assertEqual(q[1],raw)
        np.testing.assert_array_equal(before,self.robot.d.qpos)
        np.testing.assert_array_equal(lower,self.robot.lower)
        with self.assertRaises(ValueError): self.sync()
        self.motors['2']['mechPosRad']=float(self.robot.lower[1]-np.deg2rad(1.01))
        with self.assertRaises(ValueError): feedback_pose(self.robot,self.motors,self.mapping,now=101,start_tolerance=np.deg2rad(1))

if __name__=='__main__': unittest.main(verbosity=2)
