#!/usr/bin/env python3
"""Standard-library client for the local simulator's action-program API."""
import argparse
import json
import time
import urllib.request
import urllib.error
from pathlib import Path


class RebotClient:
    def __init__(self, url='http://127.0.0.1:8765'):
        self.url = url.rstrip('/')

    def _request(self, path, body=None):
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        req = urllib.request.Request(self.url+path, data=data, headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=30) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            raise RuntimeError(json.load(exc).get('error', str(exc))) from exc

    def state(self):
        return self._request('/state')

    def submit(self, program):
        return self._request('/program', program)['job']

    def job(self, job_id):
        return self._request('/jobs/'+job_id)

    def control(self, action):
        return self._request('/command', {'action': action})

    def wait(self, job_id, timeout=180):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            job = self.job(job_id)
            if job['status'] not in ('running', 'paused'):
                if job['status'] != 'succeeded':
                    raise RuntimeError(f"{job['status']}: {job['error']}")
                return job
            time.sleep(.2)
        # Timeout is an observation failure, never falsely a successful motion.
        raise TimeoutError('等待超时，服务端程序可能仍在运行；查询任务或显式取消')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description='提交 JSON 动作程序到本地 MuJoCo 仿真')
    p.add_argument('program', type=Path)
    p.add_argument('--url', default='http://127.0.0.1:8765')
    p.add_argument('--wait', action='store_true')
    p.add_argument('--timeout', type=float, default=180)
    a = p.parse_args()
    client = RebotClient(a.url)
    job = client.submit(json.loads(a.program.read_text()))
    print(json.dumps(job, ensure_ascii=False, indent=2), flush=True)
    if a.wait:
        print(json.dumps(client.wait(job['id'], a.timeout), ensure_ascii=False, indent=2))
