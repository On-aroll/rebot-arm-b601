"""CPU fresh-chunk cost; queue-only calls must not hide network inference cost."""
import argparse
import platform
import time

from gate import upstream, demonstration, write, digest
from benchmark_inference import rss
import numpy as np
import torch


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    _, ACTPolicy, _, _ = upstream()
    torch.set_num_threads(2)
    data = demonstration()
    before = rss()
    policy = ACTPolicy.from_pretrained(args.checkpoint, local_files_only=True).cpu().eval()

    def predict():
        policy.reset()
        batch = {'observation.image': torch.from_numpy(data['rgb'][0].copy()).permute(2, 0, 1)[None].float() / 255,
                 'observation.state': torch.from_numpy(np.r_[data['state'][0], data['goal'][0]])[None].float()}
        return policy.select_action(batch)

    with torch.inference_mode():
        for _ in range(10):
            predict()
        latencies = []
        for _ in range(200):
            started = time.perf_counter()
            predict()
            latencies.append((time.perf_counter() - started) * 1000)
    after = rss()
    result = {'device': 'host_x86_CPU_two_threads', 'board_measured': False,
        'platform': platform.platform(), 'samples': len(latencies),
        'median_ms': float(np.median(latencies)), 'p95_ms': float(np.quantile(latencies, .95)),
        'p99_ms': float(np.quantile(latencies, .99)), 'process_rss_bytes': after,
        'model_warmup_rss_increment_bytes': after - before,
        'includes': 'uint8 RGB conversion, own-state/goal input, normalization, full fresh chunk, denormalization',
        'excludes': 'camera, transport, physical board, power/thermal measurement',
        'scope': 'no simulation instance created; gate/runtime modules imported',
        'source_sha256': digest(__file__)}
    write(args.output, result)
    print(result)
