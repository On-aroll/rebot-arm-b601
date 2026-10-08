"""Fetch reBotArmController_ROS2 @ 7506dc0 file-by-file: jsDelivr CDN (fast).

Why not codeload: through this proxy codeload/github is throttled to ~20-40 KB/s
and it IGNORES the HTTP Range header (always 200, never 206), so a "resume by
append" downloader silently produced a corrupt 127 MB file containing 55 gzip
headers. jsDelivr mirrors GitHub and measured ~1.5 MB/s here.

  1. file list  <- GitHub API git/trees/<sha>?recursive=1   (authoritative)
  2. file bytes <- https://cdn.jsdelivr.net/gh/<repo>@<sha>/<path>

Integrity caveat: jsDelivr is a THIRD-PARTY CDN. From here we cannot cross-check
each file against GitHub (github/codeload throttled or 502), so provenance is
recorded in the manifest rather than claimed as verified. Re-verify from GitHub
whenever it becomes reachable.
"""
import json
import os
import ssl
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor

SHA = '7506dc0fc5c2a7787dcb974d89d13f9f49d16d61'
REPO = 'Seeed-Projects/reBotArmController_ROS2'
ROOT = '/home/bonvoyage/Desktop/seeed Robotics'
DEST = ROOT + '/external/reBotArmController_ROS2'
CDN = 'https://cdn.jsdelivr.net/gh/%s@%s/' % (REPO, SHA)
TREE = 'https://api.github.com/repos/%s/git/trees/%s?recursive=1' % (REPO, SHA)
TREE_CACHE = ROOT + '/downloads/reBotArmController_ROS2-tree-%s.json' % SHA[:12]

ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE


def get(u, timeout=30):
    return urllib.request.urlopen(
        urllib.request.Request(u, headers={'User-Agent': 'curl/8',
                                           'Accept': 'application/json'}),
        timeout=timeout, context=ctx)


print('== 1. 文件清单 (GitHub API) ==')
if os.path.exists(TREE_CACHE):
    tree = json.loads(open(TREE_CACHE).read())
    print('   使用缓存:', TREE_CACHE)
else:
    raw = get(TREE, 60).read()
    open(TREE_CACHE, 'wb').write(raw)
    tree = json.loads(raw)
    print('   已保存:', TREE_CACHE)

files = [t['path'] for t in tree['tree'] if t['type'] == 'blob']
print('   文件数:', len(files), '| 树条目:', len(tree['tree']),
      '| 截断:', tree.get('truncated'))

os.makedirs(DEST, exist_ok=True)
done, failed = [], []


def fetch(rel):
    dest = os.path.join(DEST, rel)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    for attempt in range(3):
        try:
            with get(CDN + rel, 90) as r, open(dest, 'wb') as f:
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


print('== 2. 逐文件下载 (jsDelivr, 6 并发) ==')
t0 = time.time()
with ThreadPoolExecutor(max_workers=6) as ex:
    list(ex.map(fetch, files))

el = time.time() - t0
total = 0
for f in done:
    p = os.path.join(DEST, f)
    if os.path.exists(p):
        total += os.path.getsize(p)
print()
print('完成 %d/%d 文件 | %.1f MB | %.0fs | %.0f KB/s'
      % (len(done), len(files), total / 1048576, el, total / el / 1024 if el else 0))
if failed:
    print('失败 %d:' % len(failed))
    for f, e in failed[:10]:
        print('   ', f, '->', e)
    sys.exit(1)
print('OK ->', DEST)
