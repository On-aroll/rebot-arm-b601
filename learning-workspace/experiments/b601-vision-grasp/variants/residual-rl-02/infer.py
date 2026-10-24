"""CPU-only residual proposal; does not connect to or move hardware."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch


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
    torch.set_num_threads(1)
    config = json.loads((args.directory / 'inference_config.json').read_text())
    example = args.observation or args.directory / 'example_observation.npz'
    observation = np.load(example)['observation'].astype('float32')
    if observation.shape != (39,) or not np.isfinite(observation).all():
        raise ValueError('expected 39 finite observation values in documented order')
    model = torch.jit.load(str(args.directory / 'actor.torchscript')).eval()
    with torch.inference_mode():
        action = model(torch.from_numpy(observation[None]))[0].numpy()
    bounds = np.asarray(config['residual_bounds_m'])
    previous = observation[32:36] * bounds
    limits = np.asarray(config['max_residual_change_m_per_decision'])
    delta = previous + np.clip(action * bounds - previous, -limits, limits)
    result = {'normalized_proposal': action.tolist(), 'bounded_residual_m': delta.tolist(),
        'simulation_only': True, 'scope': 'residual proposal; requires frozen V3 vision, IK, timing and supervisor'}
    if args.benchmark:
        import time
        timings = []
        with torch.inference_mode():
            x = torch.from_numpy(observation[None])
            for _ in range(100):
                model(x)
            for _ in range(2000):
                started = time.perf_counter_ns()
                proposed = model(x)[0].numpy()
                previous + np.clip(proposed * bounds - previous, -limits, limits)
                timings.append((time.perf_counter_ns() - started) / 1e6)
        result['benchmark'] = {'samples': len(timings), 'threads': 1,
            'median_ms': float(np.median(timings)), 'p95_ms': float(np.quantile(timings, .95)),
            'process_rss_bytes': resident_bytes(),
            'includes': 'TorchScript residual actor and residual limiting in standalone CPU process',
            'excludes': 'RGB-D, vision model, IK, execution and hardware board', 'board_measured': False}
        (args.directory / 'standalone_cpu_benchmark.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
