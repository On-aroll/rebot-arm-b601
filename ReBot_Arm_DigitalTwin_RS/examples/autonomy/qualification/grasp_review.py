"""Replay a frozen production GraspMonitor profile against annotated recordings."""
from rebotarmcontroller.twin_grasp import GraspMonitor, validate_profile

from .common import identifier, number, require_schema


def analyze(data):
    require_schema(data, 'b601-grasp-evaluation/1.0')
    recording = data['recording'];require_schema(recording, 'b601-j7-recording/1.0')
    profile = validate_profile(data['profile'])
    trials = recording['trials'];truth = data['truth']
    if not isinstance(trials, list) or not 1 <= len(trials) <= 100 or not isinstance(truth, dict):
        raise ValueError('need 1–100 trials and explicit independent truth labels')
    ids = [identifier(t['id'], 'trial id') for t in trials]
    if len(ids) != len(set(ids)) or set(ids) != set(truth):
        raise ValueError('unique recording ids must exactly match truth annotation ids')
    results = []
    for trial in trials:
        key = trial['id'];annotation = truth[key];label = annotation['label']
        if label not in ('object', 'empty', 'fixture', 'free', 'slip'):
            raise ValueError('truth label must be object, empty, fixture, free or slip')
        origin = trial['source']
        if origin not in ('hardware', 'fake', 'synthetic'):raise ValueError('explicit trial source required')
        samples = trial['samples']
        if not isinstance(samples, list) or not 3 <= len(samples) <= 5000:
            raise ValueError('trial needs 3–5000 samples including pre-contact closure')
        start = number(samples[0]['position'], 'initial position', 0, 5)
        began = number(samples[0]['sample_at'], 'initial sample_at', 0)
        monitor = GraspMonitor(profile, start, began, key)
        last_stamp = last_receive = -1.;first_holding = failed_at = None
        event = number(annotation['event_at'], 'slip event_at', began) if label == 'slip' else None
        for sample in samples:
            stamp = number(sample['sample_at'], 'sample_at', 0)
            received = number(sample['received_ms'], 'received_ms', 0)
            if stamp <= last_stamp or received < last_receive or last_stamp >= 0 and stamp - last_stamp > .6:
                raise ValueError('duplicate, reversed or interrupted sample clock')
            last_stamp, last_receive = stamp, received
            # Saved browser wall time and driver monotonic time have no absolute
            # offset. Replay driver intervals, never invent a freshness estimate.
            monitor.update(position=number(sample['position'], 'position', 0, 5),
                velocity=number(sample['velocity'], 'velocity', -10, 10),
                current=number(sample['current_a'], 'current_a', -50, 50),
                stamp=stamp, now=stamp, reference=number(sample['reference'], 'reference', 0, 5))
            if monitor.status == 'holding' and first_holding is None:first_holding = stamp
            if monitor.status == 'failed' and failed_at is None:failed_at = stamp
        if label == 'slip' and not began <= event <= last_stamp:
            raise ValueError('slip event must lie within the recording')
        lost = bool(first_holding is not None and failed_at is not None and
                    '保持证据丢失' in monitor.reason and first_holding <= event <= failed_at) if label == 'slip' else None
        results.append(dict(id=key, source=origin, truth=label, final_status=monitor.status,
            first_holding_at=first_holding, reason=monitor.reason,
            false_object_acceptance=label in ('empty', 'fixture', 'free') and first_holding is not None,
            missed_object=label == 'object' and monitor.status != 'holding',
            slip_detected=lost, slip_detection_delay_s=failed_at - event if lost else None))
    def metrics(rows):
        return dict(trials=len(rows), false_object_acceptances=sum(r['false_object_acceptance'] for r in rows),
                    missed_objects=sum(r['missed_object'] for r in rows),
                    slip_trials=sum(r['truth'] == 'slip' for r in rows),
                    slips_detected=sum(r['slip_detected'] is True for r in rows))
    return dict(schema_version='b601-grasp-evaluation-review/1.0', hardware_execution_allowed=False,
        physical_validation_passed=False, object_verified=False, calibration_ready=False,
        profile=profile, trials=results, hardware_metrics=metrics([r for r in results if r['source'] == 'hardware']),
        nonhardware_metrics=metrics([r for r in results if r['source'] != 'hardware']),
        limitations=['replays a frozen profile; never chooses or installs current thresholds',
                     'partial stationary snippets are insufficient; include pre-contact closure',
                     'browser/driver clock offset unknown: absolute receive freshness not validated',
                     'contact with fixture may satisfy motor evidence; object identity remains unverified'])
