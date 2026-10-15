"""Fixed J6 through the actual API/private worker and virtual SDK only."""
import math
import unittest
import test_native_xyz_pipe as xyz_fixture


class NativeJ6Pipe(unittest.TestCase):
    setUpClass=xyz_fixture.NativeXYZPipe.__dict__['setUpClass']
    make_preview=xyz_fixture.NativeXYZPipe.make_preview
    take=xyz_fixture.NativeXYZPipe.take
    spawn_virtual=xyz_fixture.NativeXYZPipe.spawn_virtual
    cleanup=xyz_fixture.NativeXYZPipe.cleanup
    action=xyz_fixture.NativeXYZPipe.action

    def setUp(self):
        xyz_fixture.NativeXYZPipe.setUp(self)
        result=self.action('prepare-csp-j6')
        self.assertEqual(result['status']['state'],'READY',result)
        self.trusted=dict(self.api.plan)

    def run_trial(self,*,cancel=None):
        return xyz_fixture.NativeXYZPipe.run_trial(self,cancel=cancel,action='csp-j6-trial')

    def check_cleanup(self,result):
        self.assertTrue(result['virtual_sdk']);self.assertFalse(result['busy'])
        self.assertTrue(result['restoration']['ok'],result['reason'])
        self.assertEqual(result['stop']['sdk_status_ack_axes'],[6])
        self.assertTrue(result['stop']['stationary_sample'])
        self.assertEqual(self.store.state,self.before)
        calls=result['virtual_sdk_calls']
        self.assertTrue(all(c[1]==6 for c in calls if c[0] in ('enable','disable','write')))
        self.assertFalse(any(c[0]=='mit' or c[0]=='write' and c[2] not in (0x7005,0x700B,0x7018,0x7017,0x7016) for c in calls))

    def test_actual_worker_j6_arrives_without_touching_other_drives(self):
        result=self.run_trial();self.assertEqual(result['state'],'ARRIVED',result['reason'])
        self.assertAlmostEqual(result['arrival_feedback']['6']['q'],math.radians(.5))
        self.assertEqual(result['control']['axis'],6);self.check_cleanup(result)

    def test_api_stop_preempts_j6_and_restores(self):
        result=self.run_trial(cancel='stop');self.assertEqual(result['state'],'CANCELLED',result['reason'])
        self.check_cleanup(result)

    def test_parent_pipe_eof_preempts_j6_and_restores(self):
        result=self.run_trial(cancel='eof');self.assertEqual(result['state'],'CANCELLED',result['reason'])
        self.check_cleanup(result)


if __name__=='__main__':unittest.main()
