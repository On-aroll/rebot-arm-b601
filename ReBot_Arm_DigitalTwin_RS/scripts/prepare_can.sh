#!/usr/bin/env bash
# No persistent privilege rules. Polkit asks the local user when needed.
set -Eeuo pipefail
umask 077
CAN_OWNER_DIR="/tmp/rebot-can-owner-${UID}"
mkdir -p "${CAN_OWNER_DIR}"
exec 8>"${CAN_OWNER_DIR}/can0.lock"
flock -n 8 || { echo 'CAN 正由控制器使用，禁止重新配置。' >&2; exit 1; }
python3 - <<'PY'
import json, pathlib, subprocess
result = subprocess.run(['/usr/sbin/ip', '-j', '-details', 'link', 'show', 'can0'], capture_output=True, text=True)
if result.returncode:
    raise SystemExit('未找到 can0：请连接 USB-CAN 适配器。')
device = json.loads(result.stdout)[0]
info = device.get('linkinfo', {}).get('info_data', {})
if 'UP' in device['flags'] and info.get('bittiming', {}).get('bitrate') == 1000000:
    if info.get('state') == 'BUS-OFF':
        raise SystemExit('CAN 为 BUS-OFF，请检查接线与供电；未自动重置接口。')
    print('CAN 已为 UP / 1 Mbps，复用当前配置。')
    raise SystemExit(0)
for name in ('all', 'fil', 'inv', 'sff', 'eff', 'err'):
    path = pathlib.Path('/proc/net/can') / ('rcvlist_' + name)
    if path.exists():
        for line in path.read_text().splitlines():
            fields = line.split()
            if fields and fields[0] in ('can0', 'any'):
                raise SystemExit('CAN 存在接收者，未重新配置。请先在支撑条件下结束原控制会话。')
if 'UP' in device['flags']:
    raise SystemExit('CAN 已启用但速率不是 1 Mbps；未自动关闭接口，请由维护人员检查。')
for args in (['link', 'set', 'can0', 'type', 'can', 'bitrate', '1000000'], ['link', 'set', 'can0', 'up']):
    subprocess.run(['pkexec', '/usr/sbin/ip', *args], check=True)
print('CAN 配置完成：UP / 1 Mbps。')
PY
