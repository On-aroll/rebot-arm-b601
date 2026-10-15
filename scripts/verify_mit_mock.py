#!/usr/bin/env python3
"""Run the B1 virtual-only acceptance suite and write reproducible evidence.

No CAN, network, SDK, live software ledger or frontend connection is involved.
The synthetic fixture is intentionally fixed; this is not a robot command CLI.
"""
import argparse
import csv
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import unittest

from rebot_mit_mock import MockConfig, MockExecutor, MockMotorTransport, VirtualClock
ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/mit_mock_b1')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    spec = importlib.util.spec_from_file_location('b1_tests', ROOT/'tests/test_mit_mock.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    suite = unittest.defaultTestLoader.loadTestsFromModule(module)
    ids = [case.id() for group in suite for case in group]
    with (args.output/'mock-tests.txt').open('w') as stream:
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    bad = {getattr(test, 'test_case', test).id(): trace for test, trace in result.failures+result.errors}
    config = MockConfig.parse(json.loads(module.FIXTURE.read_text()))
    clock = VirtualClock()
    bus = MockMotorTransport(clock, config.device, module.START)
    executor = MockExecutor(config, bus, clock)
    plan = executor.prepare(module.WAYPOINTS, 'b1-demonstration-v1')
    executor.start(plan, 'b1-demonstration-v1')
    peaks = [[0.]*6 for _ in range(3)]
    with (args.output/'synthetic-trace.csv').open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(['source', 'time_s', 'sequence']+[f'{field}_J{i}' for field in ('q_rad','qd_rad_s','a_rad_s2','jerk_rad_s3') for i in range(1,7)])
        for _ in range(int(config.policy.task_timeout/config.policy.period)+2):
            if executor.state != 'RUNNING_MOCK':
                break
            frame = bus.frames[-1]
            q, v, a, j = plan.trajectory.sample(frame.issued_at)
            for order, values in enumerate((v,a,j)):
                peaks[order] = [max(old,abs(value)) for old,value in zip(peaks[order],values)]
            writer.writerow(['synthetic-test-fixture', frame.issued_at, frame.sequence, *q, *v, *a, *j])
            clock.advance(config.policy.period)
            executor.tick()
    report = {
        'stage': 'B1', 'source': 'synthetic-test-fixture', 'approval_scope': 'mock-only',
        'passed': result.wasSuccessful() and executor.state == 'COMPLETED',
        'tests_run': result.testsRun, 'failures': len(result.failures), 'errors': len(result.errors),
        'cases': [{'id': name, 'passed': name not in bad, **({'failure':bad[name]} if name in bad else {})} for name in ids],
        'config_sha256': config.fingerprint, 'fixture': 'tests/fixtures/mit_mock_only.json',
        'real_hardware_operations_in_this_runner': 0,
        'no_claim_about_external_process_CAN_traffic': True,
        'configuration_backup_status': 'software-only; actual MIT gains unknown and not backed up',
        'trajectory': {'duration_s':plan.trajectory.duration, 'segment_durations_s':plan.trajectory.durations,
                       'sampled_max_abs_velocity':peaks[0], 'sampled_max_abs_acceleration':peaks[1], 'sampled_max_abs_jerk':peaks[2],
                       'axis_limits':[asdict(a) for a in config.axes]},
        'execution': executor.status(), 'first_frame': asdict(bus.frames[0]), 'last_frame':asdict(bus.frames[-1]),
        'limitations': ['Ideal follower, no inertia/gravity/contact/drive encoding',
                       'Virtual scheduler, not wall-clock realtime or physical stop proof',
                       'Rest at each waypoint; moving-start/streamed replanning/J7 unsupported',
                       'No physical transport, no CAN ownership or independent watchdog implemented']}
    (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(passed=report['passed'],tests_run=result.testsRun,state=executor.state,
                          virtual_frames=len(bus.frames),real_hardware_operations=0,report=str(args.output/'report.json')),ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
