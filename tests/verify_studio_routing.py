#!/usr/bin/env python3
"""虚实联调控制台的静态路由与只读端点验收（不启动任何运动）。

背景：Studio 的 i18n 会把语言写到地址栏最前面（src/i18n/index.js 的
buildPathWithLocale），所以 /zh/studio/console 必须和 /studio/console 返回同一个
入口页，否则刷新或转发链接就是 404。

用法:
    python3 tests/verify_studio_routing.py                       # 默认 8765
    python3 tests/verify_studio_routing.py http://127.0.0.1:8766 # 验证实例
"""
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = (sys.argv[1] if len(sys.argv) > 1 else os.environ.get('REBOT_SIM_BASE')
        or 'http://127.0.0.1:8765').rstrip('/')
DIST = ROOT / 'external/motorbridge-studio/dist'


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=15) as response:
        return response.status, response.read()


def status_of(path):
    try:
        return get(path)[0]
    except urllib.error.HTTPError as error:
        return error.code


checks = {}

# 1. 控制台入口：新名字、旧别名、以及三种语言前缀必须给出同一份 index.html。
entry = get('/studio/console')[1]
assert b'/studio/assets/' in entry, '控制台入口未返回 Studio 构建产物'
assert get('/studio/writing')[1] == entry, '旧入口 /studio/writing 别名失效'
for locale in ('zh', 'en', 'es'):
    assert get(f'/{locale}/studio/console')[1] == entry, f'{locale} 语言前缀深链 404'
checks['console_route'] = True
checks['legacy_writing_alias'] = True
checks['locale_deep_links'] = True

# 2. 三维视图需要的构建产物与 URDF 资源仍可取到，且语言前缀不影响资源。
asset = next(p.relative_to(DIST).as_posix() for p in sorted(DIST.glob('assets/*.js')))
assert status_of(f'/studio/{asset}') == 200
assert status_of(f'/zh/studio/{asset}') == 200, '带语言前缀的资源取不到'
assert get('/studio-model/urdf/ReBot_Arm_RS.urdf')[1] == (
    ROOT / 'external/reBotArm_control_py/urdf/RS/urdf/ReBot_Arm_RS.urdf').read_bytes()
checks['assets_served'] = True
checks['identical_rs_model'] = True

# 3. 剥离语言前缀不能顺带放宽路径穿越。
for path in ('/studio/%2e%2e/package.json', '/zh/studio/%2e%2e/package.json',
             '/studio-model/%2e%2e/reBotArm_control_py/__init__.py'):
    assert status_of(path) == 403, f'路径穿越被接受：{path}'
checks['traversal_rejected'] = True

# 4. 实机侧只读端点：没接 CAN 时必须显式说明，而不是给出全零姿态冒充读数。
for path in ('/machine-state', '/machine-params'):
    body = json.loads(get(path)[1])
    assert body.get('present') is False or body.get('present') is True
    if not body.get('present'):
        assert body.get('reason') and body.get('hint'), '缺数据时要给出原因与补救命令'
    else:
        assert body.get('age_s') is not None
        if path == '/machine-state':
            assert len(body.get('q_deg') or []) == 6, '实机读数必须是六个关节'
            assert body.get('read_only_on_hardware') is True
            assert body.get('enables_motors') is False and body.get('sends_motion_to_hardware') is False
checks['machine_readonly_endpoints'] = True

# 5. 仿真反馈仍然自洽，改名没有动到执行链路。
state = json.loads(get('/studio-state')[1])
assert state['feedback_source'] == 'simulation' and state['backend'] == 'mujoco'
assert not state['error'] and len(state['q_rad']) == 6
checks['sim_feedback_coherent'] = True

report = dict(base=BASE, success=True, checks=checks,
              hardware_motion=False, hardware_writes=False,
              note='只读 HTTP 验收：不含实机指令，也不写电机参数。')
out = Path(os.environ.get('REBOT_OUTPUT_DIR', str(ROOT/'outputs'))) / 'studio_routing_verification.json'
out.parent.mkdir(exist_ok=True)
out.write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps(report, ensure_ascii=False))
