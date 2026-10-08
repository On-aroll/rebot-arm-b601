"""Compose reusable steps with Python; no hardware connection is opened."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from rebot_client import RebotClient
from rebot_program import pick_place_steps

program = dict(version=1, name='抓取、搬运、放置，再抓回原位', steps=[])
points = [(.25, 0, .05), (.25, .065, .05), (.25, 0, .05)]
for source, destination in zip(points, points[1:]):
    program['steps'].extend(pick_place_steps(source, destination))

if __name__ == '__main__':
    client = RebotClient()
    client.control('reset')  # Explicitly reset this example's simulated scene.
    job = client.submit(program)
    print('任务已受理：', job['id'], flush=True)
    print(client.wait(job['id'], timeout=240)['status'])
