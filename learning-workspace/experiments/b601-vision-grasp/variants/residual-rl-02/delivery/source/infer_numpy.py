"""The deterministic SAC residual MLP, using only NumPy at inference time."""
import argparse
import json
from pathlib import Path
import time
import numpy as np


def predict(observation, weights):
    hidden = np.maximum(observation @ weights['w0'].T + weights['b0'], 0.)
    hidden = np.maximum(hidden @ weights['w1'].T + weights['b1'], 0.)
    return np.tanh(hidden @ weights['wm'].T + weights['bm'])


def resident_bytes():
    import os
    if os.name != 'nt':
        return int(Path('/proc/self/statm').read_text().split()[1]) * os.sysconf('SC_PAGE_SIZE')
    import ctypes
    from ctypes import wintypes
    class Counters(ctypes.Structure):
        _fields_ = [('cb', wintypes.DWORD), ('faults', wintypes.DWORD)] + [
            (name, ctypes.c_size_t) for name in ('peak_working', 'working', 'peak_paged',
            'paged', 'peak_nonpaged', 'nonpaged', 'pagefile', 'peak_pagefile')]
    counters = Counters()
    counters.cb = ctypes.sizeof(counters)
    kernel, psapi = ctypes.WinDLL('kernel32'), ctypes.WinDLL('psapi')
    kernel.GetCurrentProcess.restype = wintypes.HANDLE
    psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(Counters), wintypes.DWORD]
    if not psapi.GetProcessMemoryInfo(kernel.GetCurrentProcess(), ctypes.byref(counters), counters.cb):
        raise ctypes.WinError()
    return counters.working


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--observation', type=Path)
    parser.add_argument('--benchmark', action='store_true')
    args = parser.parse_args()
    config = json.loads((args.directory / 'inference_config.json').read_text())
    observation = np.load(args.observation or args.directory / 'example_observation.npz')['observation'].astype('float32')
    if observation.shape != (39,) or not np.isfinite(observation).all():
        raise ValueError('expected 39 finite values in the documented observation order')
    with np.load(args.directory / 'numpy_actor.npz') as archive:
        weights = {name: archive[name] for name in archive.files}
    bounds = np.asarray(config['residual_bounds_m'])
    previous = observation[32:36] * bounds
    limits = np.asarray(config['max_residual_change_m_per_decision'])
    action = predict(observation, weights)
    delta = previous + np.clip(action * bounds - previous, -limits, limits)
    result = {'normalized_proposal': action.tolist(), 'bounded_residual_m': delta.tolist(),
        'simulation_only': True, 'scope': 'residual actor only; frozen V3 system required'}
    if args.benchmark:
        for _ in range(100):
            predict(observation, weights)
        timings = []
        for _ in range(2000):
            started = time.perf_counter_ns()
            proposed = predict(observation, weights)
            previous + np.clip(proposed * bounds - previous, -limits, limits)
            timings.append((time.perf_counter_ns() - started) / 1e6)
        result['benchmark'] = {'samples': len(timings), 'median_ms': float(np.median(timings)),
            'p95_ms': float(np.quantile(timings, .95)), 'process_rss_bytes': resident_bytes(),
            'includes': 'NumPy residual actor plus residual limiting in standalone CPU process',
            'excludes': 'vision, RGB-D, IK, executor, hardware board and power', 'board_measured': False,
            'numpy_version': np.__version__}
        (args.directory / 'numpy_cpu_benchmark.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
