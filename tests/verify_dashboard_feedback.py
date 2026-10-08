"""Exercise concurrent dashboard readers so rendering cannot starve feedback."""
import concurrent.futures
import json
import time
import urllib.request
from pathlib import Path

def reader(path):
    timings=[]
    until=time.monotonic()+25
    while time.monotonic()<until:
        started=time.monotonic()
        with urllib.request.urlopen('http://127.0.0.1:8765'+path,timeout=2.5) as response:
            data=response.read()
        timings.append(time.monotonic()-started)
        if path=='/state':
            state=json.loads(data)
            assert not state['error'] and time.time()-state['feedback_time_unix_s']<2
        time.sleep(.08 if path=='/frame.jpg' else .15)
    return timings

with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
    results=list(pool.map(reader,['/state','/state','/frame.jpg']))
samples=[v for group in results for v in group]
report=dict(success=True,duration_s=25,concurrent_readers=3,requests=len(samples),max_latency_s=max(samples),mean_latency_s=sum(samples)/len(samples))
(Path(__file__).resolve().parents[1]/'outputs/dashboard_feedback_verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
