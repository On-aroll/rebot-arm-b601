"""Production API/worker/FK with extended profile and virtual SDK only."""
import unittest
import numpy as np

import test_native_xyz_pipe as base
from rebot_native_xyz import PATH_MODE
from rebot_point import PointPlanner
from rebot_xyz_trial import configure_motion,assess_motion


class NativeXYZPathPipe(unittest.TestCase):
    @classmethod
    def setUpClass(cls):base.NativeXYZPipe.setUpClass()

    def setUp(self):
        self.case=base.NativeXYZPipe();self.addCleanup(self.case.doCleanups);self.case.setUp()
        c=self.case;robot=c.robot;goal=robot.fk(c.start).translation+np.array([0.,0.,.005])
        planner=PointPlanner(robot)
        preview=planner.preview(dict(execution_source='hardware-preview',xyz_mm=(goal*1000).tolist(),
            speed_rad_s=.02,orientation_mode='hold'),start=c.start)
        target=np.deg2rad(preview['q_deg']);raw,config=configure_motion(c.trusted['config'],c.start,target,
            dict(speed_rad_s=.02,timeout_mode='auto',feedforward_mode=PATH_MODE))
        assessment=assess_motion(robot,c.start,target,config)
        self.assertTrue(assessment['execution_ready'],assessment)
        self.assertGreater(assessment['max_axis_delta_deg'],.5)
        self.assertLessEqual(assessment['max_axis_delta_deg'],5.)
        c.trusted.update(config=raw,target=target.tolist())

    def test_five_mm_real_model_target_arrives_with_correct_profile_and_restoration(self):
        c=self.case;result=c.run_trial();self.assertEqual(result['state'],'ARRIVED',result['reason']);c.check_cleanup(result)
        self.assertEqual(result['control']['mode'],PATH_MODE)
        self.assertEqual(result['control']['task_timeout_s'],30.)
        self.assertLessEqual(result['tcp_error_mm'],1.)
        self.assertTrue(result['motion_arrived_before_stop']);self.assertFalse(result['physical_acceptance'])

    def test_stop_preempts_extended_profile_and_restores_without_retry(self):
        self.case.check_cancel('stop')

    def test_parent_eof_preempts_extended_profile_and_restores(self):
        self.case.check_cancel('eof')


if __name__=='__main__':unittest.main()
