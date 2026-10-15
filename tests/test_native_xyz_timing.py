"""Replay historical query latency with virtual drives; no real CAN.

The trace contains aggregate position/velocity/mode query duration, not separate
query timing. Splitting it into thirds is a scheduling approximation. Motor
response remains instantaneous; this cannot establish real arrival time.
"""
import json
from pathlib import Path
import statistics
import unittest

import test_native_xyz_offline as offline

ROOT=Path(__file__).resolve().parents[1]


class NativeXYZTiming(unittest.TestCase):
    def test_historical_query_latency_keeps_startup_and_cycle_within_existing_guards(self):
        record=json.loads((ROOT/'docs/validation/official-mit-2026-10-15/native-csp-15s.json').read_text())
        samples=[sample for frame in record['command_log']['frames'] for sample in frame['feedback'].values()]
        position=[(s['received_at']-s['requested_at'])/3 for s in samples]
        current=[(s['diagnostics_received_at']-s['diagnostics_requested_at'])/2
            for s in samples if s.get('diagnostics_requested_at') is not None]
        for label,select in (('median',statistics.median),('observed_aggregate_maxima',max)):
            with self.subTest(latency=label):
                case=offline.NativeXYZOffline();case.setUp()
                pos_delay=select(position);iq_delay=select(current)
                register=case.sdk.add_robstride_motor
                def delayed_register(*args):
                    motor=register(*args)
                    for name in ('robstride_get_param_f32_host_id','robstride_get_param_i8',
                                 'robstride_write_param_f32','enable','disable'):
                        method=getattr(motor,name)
                        def delayed(*args,method=method,name=name,**kwargs):
                            before=case.clock()
                            try:return method(*args,**kwargs)
                            finally:
                                # Reads come from the historical trace. Write
                                # and power ACK costs below are synthetic.
                                cost=((iq_delay if args and args[0]==0x701A else pos_delay)
                                    if name.startswith('robstride_get') else
                                    .0001 if name=='robstride_write_param_f32' else .0043)
                                case.clock.t+=max(0.,cost-(case.clock()-before))
                                case.beat()
                        setattr(motor,name,delayed)
                    return motor
                case.sdk.add_robstride_motor=delayed_register
                case.run_trial()
                self.assertEqual(case.executor.state,'ARRIVED',case.executor.reason)
                self.assertTrue(case.executor.restoration['ok'])
                self.assertLess(case.executor.timing['stop_requested_at'],case.executor.deadline)
                self.assertEqual([c[1] for c in case.calls('enable')],list(offline.AXES))
                self.assertFalse(case.calls('mit'))


if __name__=='__main__':unittest.main()
