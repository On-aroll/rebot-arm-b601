"""Measure the standalone CPU inference process, without loading MuJoCo."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import platform

import numpy as np
import torch
from chunk_policy import Inference


def rss():
    if os.name == 'nt':
        from ctypes import wintypes
        class Counters(ctypes.Structure):
            _fields_ = [('cb', wintypes.DWORD), ('faults', wintypes.DWORD)] + [
                (name, ctypes.c_size_t) for name in ('peak_working', 'working', 'peak_paged',
                'paged', 'peak_nonpaged', 'nonpaged', 'pagefile', 'peak_pagefile')]
        psapi = ctypes.WinDLL('psapi')
        kernel = ctypes.WinDLL('kernel32')
        kernel.GetCurrentProcess.restype = wintypes.HANDLE
        psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(Counters), wintypes.DWORD]
        counters = Counters()
        counters.cb = ctypes.sizeof(counters)
        if not psapi.GetProcessMemoryInfo(kernel.GetCurrentProcess(), ctypes.byref(counters), counters.cb):
            raise ctypes.WinError()
        return counters.working
    # Linux current resident pages, rather than peak RSS.
    return int(Path('/proc/self/statm').read_text().split()[1]) * os.sysconf('SC_PAGE_SIZE')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--observation', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    torch.set_num_threads(2)
    before = rss()
    policy = Inference(a.checkpoint, 'cpu')
    with np.load(a.observation, allow_pickle=False) as d:
        frame, state = {'rgb': d['rgb'], 'depth': d['depth_m']}, d['state']
    for _ in range(10):
        policy.predict(frame, state, 'A')
    policy.latencies.clear()
    for _ in range(200):
        policy.predict(frame, state, 'A')
    after = rss()
    result = {'platform': platform.platform(), 'processor': platform.processor(),
        'device': 'x86_64_CPU_2_threads', 'board_measured': False, 'samples': 200,
        'parameters': sum(p.numel() for p in policy.model.parameters()),
        'checkpoint_bytes': Path(a.checkpoint).stat().st_size,
        'median_ms': float(np.median(policy.latencies)), 'p95_ms': float(np.quantile(policy.latencies, .95)),
        'p99_ms': float(np.quantile(policy.latencies, .99)),
        'process_rss_before_model_bytes': before, 'process_rss_after_warmup_bytes': after,
        'rss_increment_bytes': after - before, 'torch': torch.__version__,
        'includes': 'resize, visual_summary, normalization, network, denormalization',
        'excludes': 'camera, transport, hardware executor, MuJoCo, power and thermal measurement'}
    Path(a.output).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
