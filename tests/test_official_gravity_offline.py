"""Check Seeed's pure gravity/ramp helpers using the explicit RS model, no CAN."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
import pinocchio as pin

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_core import URDF
from reBotArm_control_py.controllers import GravityCompensation
from reBotArm_control_py.dynamics import compute_generalized_gravity


class NoHardware:
    def __getattr__(self,name):
        raise AssertionError(f'Offline gravity verification touched hardware: {name}')


class OfficialGravityOffline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=pin.buildModelFromUrdf(str(URDF))
        cls.q=np.deg2rad([61.96,-.39,.1,.56,-.91,4.46])

    def test_pure_controller_uses_rs_model_and_keeps_explicit_gains_without_hardware(self):
        controller=GravityCompensation(NoHardware(),kp=3,kd=1,hold_kp=3,hold_kd=1,
            joint_direction=1,tau_scale=1,transition_duration=2)
        with patch('reBotArm_control_py.kinematics.load_robot_model',return_value=self.model) as load:
            controller._resolve_params(6)
        load.assert_called_once_with()
        self.assertIs(controller._gc_model,self.model)
        np.testing.assert_array_equal(controller._compliant_kp,np.full(6,3.))
        np.testing.assert_array_equal(controller._compliant_kd,np.ones(6))
        expected=compute_generalized_gravity(self.model,self.q)[:6]
        np.testing.assert_allclose(controller._gravity_comp_torque(self.q),expected,atol=1e-12)
        self.assertFalse(controller._running)

    def test_gravity_sign_matches_potential_energy_gradient_with_rs_coordinates(self):
        g=compute_generalized_gravity(self.model,self.q)[:6]
        full=np.zeros(self.model.nq);full[:6]=self.q
        gradient=[]
        for i in range(6):
            plus=full.copy();minus=full.copy();plus[i]+=1e-6;minus[i]-=1e-6
            gradient.append((pin.computePotentialEnergy(self.model,self.model.createData(),plus)
                             -pin.computePotentialEnergy(self.model,self.model.createData(),minus))/2e-6)
        np.testing.assert_allclose(g,gradient,rtol=1e-5,atol=1e-7)
        self.assertGreater(g[1],1.)
        self.assertGreater(g[2],6.)
        self.assertGreater(g[3],1.)

    def test_official_transition_is_bounded_monotonic_and_has_zero_endpoint_slope(self):
        controller=GravityCompensation(NoHardware(),transition_duration=2)
        samples=[controller._transition_blend(t) for t in np.linspace(-1,3,81)]
        self.assertEqual(samples[0],0.)
        self.assertEqual(samples[-1],1.)
        self.assertTrue(all(0<=a<=b<=1 for a,b in zip(samples,samples[1:])))
        self.assertLess(controller._transition_blend(1e-5)/1e-5,1e-4)
        self.assertLess((1-controller._transition_blend(2-1e-5))/1e-5,1e-4)


if __name__=='__main__': unittest.main()
