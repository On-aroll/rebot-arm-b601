"""Fetch LAN-GER/reBot-B601-RS-for-mujoco_sim into the ROS 2 workspace.

Why this is needed
------------------
`rebotarm_mujoco_rs` does NOT ship its MuJoCo model. Its rs_grasp_scene.xml is a
shell that <include>s

    ../../../third_party/reBot-B601-RS-for-mujoco_sim/assets/00_arm_rs_asm_v3/00_arm_rs_asm_v3.xml

and the README states the upstream repo has **no LICENSE file**, so it is not
vendored; `scripts/setup_rs_workspace.sh` is supposed to check out a pinned
revision into `third_party/`. That script is NOT present in the fetched repo, so
there is no upstream-pinned revision to follow — the commit below was chosen by
us on 2026-10-06 and must be recorded as such.

Layout: `../../../` from `src/rebotarm_mujoco_rs/models/` resolves to the repo
root, so third_party/ must sit next to `src/`.

⚠️ LICENSE: upstream has none (personal repo). Treat as unlicensed; do not
redistribute. Re-check before any product use.

Fetch method: jsDelivr CDN, file by file (codeload is throttled to ~20-40 KB/s
through this proxy and ignores Range). Manifest from the GitHub API tree.
"""
import json
import os
import ssl
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor

SHA = '1249cb6efdf393ba636056fc41df30dc6ba389aa'
REPO = 'LAN-GER/reBot-B601-RS-for-mujoco_sim'
ROOT = '/home/bonvoyage/Desktop/seeed Robotics'
DEST = ROOT + '/external/reBotArmController_ROS2/third_party/reBot-B601-RS-for-mujoco_sim'
CDN = 'https://cdn.jsdelivr.net/gh/%s@%s/' % (REPO, SHA)
TREE = 'https://api.github.com/repos/%s/git/trees/%s?recursive=1' % (REPO, SHA)
CACHE = ROOT + '/downloads/LAN-GER-mujoco-tree.json'

ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE


def get(u, timeout=60):
    return urllib.request.urlopen(
        urllib.request.Request(u, headers={'User-Agent': 'curl/8',
                                           'Accept': 'application/json'}),
        timeout=timeout, context=ctx)


print('== 1. 文件清单 ==')
if os.path.exists(CACHE):
    tree = json.loads(open(CACHE).read())
    print('   使用缓存:', CACHE)
else:
    raw = get(TREE).read()
    open(CACHE, 'wb').write(raw)
    tree = json.loads(raw)
files = [t['path'] for t in tree['tree'] if t['type'] == 'blob']
print('   文件数:', len(files), '| 截断:', tree.get('truncated'))

os.makedirs(DEST, exist_ok=True)
done, failed = [], []


def fetch(rel):
    dest = os.path.join(DEST, rel)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    for attempt in range(3):
        try:
            with get(CDN + rel) as r, open(dest, 'wb') as f:
                while True:
                    c = r.read(1 << 16)
                    if not c:
                        break
                    f.write(c)
            done.append(rel)
            return
        except Exception as e:
            if attempt == 2:
                failed.append((rel, '%s: %s' % (type(e).__name__, e)))
                return
            time.sleep(2)


print('== 2. 下载（6 并发） ==')
t0 = time.time()
with ThreadPoolExecutor(max_workers=6) as ex:
    list(ex.map(fetch, files))
el = time.time() - t0
total = sum(os.path.getsize(os.path.join(DEST, f)) for f in done
            if os.path.exists(os.path.join(DEST, f)))
print('   完成 %d/%d | %.1f MB | %.0fs' % (len(done), len(files), total / 1048576, el))

if failed:
    print('   失败 %d:' % len(failed))
    for f, e in failed[:10]:
        print('     ', f, '->', e)
    sys.exit(1)

# 关键文件闸门
need = [
    'assets/00_arm_rs_asm_v3/scene.xml',
    'assets/00_arm_rs_asm_v3/00_arm_rs_asm_v3.xml',
]
for n in need:
    p = os.path.join(DEST, n)
    print('   %-48s %s' % (n, 'OK %d bytes' % os.path.getsize(p) if os.path.exists(p) else '缺失!'))
print('OK ->', DEST)
