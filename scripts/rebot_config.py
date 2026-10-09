"""Private SOFTWARE configuration ledger. No CAN imports, sockets or actuator writes.

Approval records human intent, never physical validation or hardware permission.
A single atomic head contains the entire state; immutable history precedes head.
"""
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import tempfile
import threading
import time
from contextlib import contextmanager

SCHEMA = 1
FIELDS = {
    'kp': ('N*m/rad', 'MIT 刚度：可能突然出力', 'mit'),
    'kd': ('N*m*s/rad', 'MIT 阻尼', 'mit'),
    'tau_ff': ('N*m', 'MIT 前馈力矩', 'mit'),
    'torque_max': ('N*m', '软件输出力矩边界', 'trajectory'),
    'v_max': ('rad/s', '软件关节速度边界', 'trajectory'),
    'a_max': ('rad/s2', '软件关节加速度边界', 'trajectory'),
    'jerk_max': ('rad/s3', '软件关节加加速度边界', 'trajectory'),
    'error_max': ('rad', '软件跟踪误差边界', 'trajectory'),
    'timeout': ('s', '任务超时', 'trajectory'),
    'q_min': ('rad', '关节下限，待机械验收', 'trajectory'),
    'q_max': ('rad', '关节上限，待机械验收', 'trajectory'),
    'direction': ('sign', '映射方向', 'mapping'),
    'offset': ('deg', '映射偏移，不写零点', 'mapping'),
}
POSITIVE = {'torque_max','v_max','a_max','jerk_max','error_max','timeout'}
DEVICE_UNITS = {'mechPosRad':'rad','runMode':'enum','limitSpd':'rad/s','limitPpSpd':'rad/s','locKp':'firmware','limitCur':'A'}
LOCK_REASON = 'LOCKED：MIT 执行器、动态映射与停止策略尚未现场验收；本服务没有真实驱动通道'


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def envelope(state):
    return dict(checksum=digest(state), state=state)


def empty_state():
    return dict(schema_version=SCHEMA, revision=0, model='B601-RS', device_id=None,
                identity_status='unverified', approved={}, drafts={}, snapshots=[], events=[])


def axis_key(axis):
    if type(axis) is not int or axis not in range(1, 8):
        raise ValueError('axis_id 必须为 1–7')
    return str(axis)


def validate_fields(fields):
    if not isinstance(fields, dict) or set(fields) != set(FIELDS):
        raise ValueError('字段缺失或未知；需要完整字段集合，未确认项为 null')
    for name, field in fields.items():
        if not isinstance(field, dict) or set(field) != {'value','unit'} or field['unit'] != FIELDS[name][0]:
            raise ValueError(f'{name}: 类型或单位不匹配')
        value = field['value']
        if value is None:
            continue
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'{name}: 必须是有限数值或 null')
        if name in POSITIVE and value <= 0 or name in ('kp','kd') and value < 0:
            raise ValueError(f'{name}: 数值范围无效')
        if name == 'direction' and value not in (-1, 1):
            raise ValueError('direction: 只能为 -1 或 1')
    lo, hi = fields['q_min']['value'], fields['q_max']['value']
    if lo is not None and hi is not None and lo >= hi:
        raise ValueError('关节上下限顺序错误')


def blank_fields():
    return {k: dict(value=None, unit=v[0]) for k, v in FIELDS.items()}


def validate_state(state):
    if not isinstance(state, dict) or set(state) != set(empty_state()):
        raise ValueError('配置结构不兼容')
    if state['schema_version'] != SCHEMA or type(state['revision']) is not int or state['revision'] < 0 or state['model'] != 'B601-RS':
        raise ValueError('配置版本或机型不兼容')
    if state['identity_status'] != 'unverified' or state['device_id'] is not None and (not isinstance(state['device_id'], str) or len(state['device_id']) > 128):
        raise ValueError('设备身份无效；软件导入不能核实物理身份')
    for category in ('approved', 'drafts'):
        if not isinstance(state[category], dict):
            raise ValueError('配置分类类型错误')
        for key, profile in state[category].items():
            if key not in map(str, range(1, 8)) or not isinstance(profile, dict):
                raise ValueError('配置轴号错误')
            expected = {'schema_version','axis_id','model','source','measured_at','control_mode','review_status','fields','saved_at'}
            if category == 'approved':
                expected |= {'approved_at','confirmation','version'}
            if set(profile) != expected or profile['schema_version'] != SCHEMA or profile['axis_id'] != int(key) or profile['model'] != ('rs-06' if int(key) < 4 else 'rs-00'):
                raise ValueError('轴配置元数据无效')
            if profile['control_mode'] != ('MIT' if int(key) <= 6 else 'unknown') or profile['source'] not in ('human-software', 'user-provided-mit-initial-trial') or profile['measured_at'] is not None:
                raise ValueError('不能把软件配置伪装成设备读数')
            if profile['review_status'] != ('software-approved' if category == 'approved' else 'draft'):
                raise ValueError('审批状态无效')
            if category == 'approved' and (profile['confirmation'] != 'explicit-software-only' or type(profile['version']) is not int or profile['version'] < 1):
                raise ValueError('批准记录无效')
            validate_fields(profile['fields'])
    if not isinstance(state['snapshots'], list) or not isinstance(state['events'], list):
        raise ValueError('快照或历史类型错误')
    for snapshot in state['snapshots']:
        validate_snapshot(snapshot)
    canonical(state)


def validate_snapshot(snapshot):
    if not isinstance(snapshot, dict) or set(snapshot) != {'schema_version','axis_id','model','source','measured_at','control_mode','review_status','identity_status','device_id','fields'}:
        raise ValueError('快照结构错误')
    key = axis_key(snapshot['axis_id'])
    if snapshot['schema_version'] != SCHEMA or snapshot['model'] != ('rs-06' if int(key) < 4 else 'rs-00') or snapshot['source'] not in ('mock','authorized-browser-read','historical-file') or snapshot['review_status'] != 'collected' or snapshot['identity_status'] != 'unverified':
        raise ValueError('快照来源或身份状态无效')
    if snapshot['device_id'] is not None and (not isinstance(snapshot['device_id'], str) or len(snapshot['device_id']) > 128):
        raise ValueError('快照身份格式无效')
    if type(snapshot['measured_at']) not in (int,float) or not math.isfinite(snapshot['measured_at']):
        raise ValueError('采集完成时间无效')
    if snapshot['control_mode'] is not None and type(snapshot['control_mode']) is not int:
        raise ValueError('快照模式无效')
    if not isinstance(snapshot['fields'], dict) or set(snapshot['fields']) != set(DEVICE_UNITS):
        raise ValueError('快照字段缺失')
    for name, f in snapshot['fields'].items():
        if not isinstance(f,dict) or set(f) != {'value','unit','measured_at','status'} or f['unit'] != DEVICE_UNITS[name] or f['status'] not in ('ok','unknown','error'):
            raise ValueError('读回状态或单位无效')
        if f['status'] == 'ok':
            if type(f['value']) not in (int,float) or not math.isfinite(f['value']) or type(f['measured_at']) not in (int,float) or not math.isfinite(f['measured_at']) or f['measured_at'] > snapshot['measured_at'] or f['measured_at'] <= 0:
                raise ValueError('读回数值或时间无效')
        elif f['value'] is not None or f['measured_at'] is not None:
            raise ValueError('未知/失败读回不能伪造数值或时间')


def preparation_checks(state, now=None):
    """Explain missing evidence without granting execution, even if all values match."""
    now = time.time() if now is None else now
    rows = []
    for axis in range(1, 8):
        key = str(axis)
        reasons = ['设备身份未核验', '动态映射、TCP 与停止策略未现场验收']
        approved = state['approved'].get(key)
        if not approved:
            reasons.append('没有人工批准的软件配置')
        else:
            missing = [name for name, field in approved['fields'].items() if field['value'] is None]
            if missing: reasons.append('未确认字段：' + ', '.join(missing))
        snapshots = [item for item in state['snapshots'] if item['axis_id'] == axis]
        snapshot = snapshots[-1] if snapshots else None
        if not snapshot:
            reasons.append('没有设备快照')
        else:
            if snapshot['device_id'] != state['device_id']: reasons.append('快照设备身份不匹配')
            if snapshot['source'] != 'authorized-browser-read': reasons.append('历史/Mock 数据不能用于实机起点')
            if not 0 <= now - snapshot['measured_at'] <= 3: reasons.append('快照过期或时间异常')
            if axis <= 6 and snapshot['control_mode'] != 0: reasons.append('实测模式未知或与 MIT 路线不一致')
            for name, field in snapshot['fields'].items():
                if field['status'] != 'ok': reasons.append(name + ' 读回失败/未知')
                elif not 0 <= now - field['measured_at'] <= 3: reasons.append(name + ' 读数过期或时间异常')
        if axis == 7: reasons.append('夹爪模式与开合机构待独立核实')
        rows.append(dict(axis_id=axis, ready=False, reasons=reasons,
                         comparison='MIT 增益/轨迹是软件配置，无法用 PP/CSP 寄存器验证一致；只比较实测模式与预期路线'))
    return rows


def parameter_provenance(state):
    """Report evidence, never infer physical backup from a software profile."""
    snapshots = state['snapshots'] if state else []
    approved = state['approved'] if state else {}
    physical = [s for s in snapshots if s['source'] != 'mock']
    source_counts = {source: sum(s['source'] == source for s in snapshots)
                     for source in ('authorized-browser-read', 'historical-file', 'mock')}
    return dict(
        device_parameters=dict(source='explicit archived DeviceSnapshot only',
            status=('unavailable' if state is None else 'partial-unverified-archive' if physical else 'not-collected'),
            snapshot_count=len(physical), source_counts=source_counts,
            last_measured_at=max((s['measured_at'] for s in physical), default=None),
            full_backup_verified=False, identity_verified=False,
            note='软件配置已保存不等于真实设备参数已备份；快照只覆盖实际读回字段，无法证明完整性、当前值或掉电持久化'),
        mit_runtime=dict(source='real executor command evidence unavailable', status='unknown-not-backed-up',
            fields=['q_des','qd_des','Kp','Kd','tau_ff'], full_backup_verified=False,
            user_trial_axes=[int(k) for k,p in (state['drafts'].items() if state else []) if p['source']=='user-provided-mit-initial-trial'],
            note='逐帧命令不是 PP/CSP 寄存器；没有真实帧记录时不推断增益。B1 合成配置与帧单独标为 mock-only'),
        approved_profile=dict(source='explicit human software-only review', status=('unavailable' if state is None else 'software-approved' if approved else 'not-approved'),
            axis_versions={key:value['version'] for key,value in approved.items()},
            applied_to_device=False, physical_validation=False,
            note='批准记录仅证明软件审核，既不证明设备当前值，也不授予实机执行权限'))


class ConfigStore:
    def __init__(self, root):
        self.root = Path(root)
        self.mutex = threading.RLock()
        self.reviews = {}

    @contextmanager
    def locked(self):
        with self.mutex:
            self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
            os.chmod(self.root, 0o700)
            fd = os.open(self.root/'lock', os.O_CREAT | os.O_RDWR, 0o600)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX)
                yield
            finally:
                os.close(fd)

    def load(self):
        path = self.root/'machine_profile.json'
        if not path.exists():
            # Missing head after a previous commit must never silently reset approval.
            if any((self.root/'history').glob('*.json')):
                raise ValueError('配置主文件丢失；保留历史，等待人工恢复')
            return empty_state()
        data = json.loads(path.read_text())
        if set(data) != {'checksum','state'} or data['checksum'] != digest(data['state']):
            raise ValueError('配置校验和错误')
        validate_state(data['state'])
        return data['state']

    def status(self):
        try:
            with self.locked():
                state = self.load()
            error = ''
        except (ValueError, OSError, TypeError, KeyError) as e:
            state, error = None, f'配置不可用，保留原文件：{e}'
        return dict(state=state, error=error, hardware=False, armed=False, drill=True,
                    execution_state='LOCKED', provenance=parameter_provenance(state), reason=LOCK_REASON, preparation=preparation_checks(state) if state else [],
                    fields={k:dict(unit=v[0],label=v[1],group=v[2]) for k,v in FIELDS.items()})

    def fingerprint(self):
        with self.mutex:
            try:
                state = self.load()
                return digest(dict(approved=state['approved'], device_id=state['device_id'], schema=SCHEMA))
            except (ValueError, OSError, TypeError, KeyError):
                return 'configuration-unavailable-simulation-only'

    def _commit(self, state, event):
        state['revision'] += 1
        state['events'].append(dict(revision=state['revision'], at=time.time(), **event))
        validate_state(state)
        raw = canonical(envelope(state))
        history = self.root/'history'
        history.mkdir(mode=0o700, exist_ok=True)
        # Unique immutable record remains recoverable if power fails before head swap.
        record = history/f"{state['revision']:08d}-{digest(state)}.json"
        fd = os.open(record, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'wb') as out:
            out.write(raw); out.flush(); os.fsync(out.fileno())
        fd, name = tempfile.mkstemp(prefix='.pending-', dir=self.root)
        try:
            with os.fdopen(fd, 'wb') as out:
                out.write(raw); out.flush(); os.fsync(out.fileno())
            validate_state(json.loads(Path(name).read_text())['state'])
            os.replace(name, self.root/'machine_profile.json')
            fd = os.open(self.root, os.O_RDONLY)
            try: os.fsync(fd)
            finally: os.close(fd)
        finally:
            Path(name).unlink(missing_ok=True)
        return state

    def save_user_mit_trial(self, model_limits):
        """Explicit one-time user request, not a startup migration or motor write.

        Never replaces an existing different draft or approval. Conflicts are
        returned for a human decision; a single transaction saves all six axes.
        """
        with self.locked():
            state = self.load()
            gains = {'kp': 3., 'kd': 1., 'tau_ff': 0.}
            conflicts = []
            for axis in range(1, 7):
                for category in ('approved', 'drafts'):
                    profile = state[category].get(str(axis))
                    if profile:
                        for name, value in gains.items():
                            old = profile['fields'][name]['value']
                            if old is not None and old != value:
                                conflicts.append(dict(axis_id=axis, category=category, field=name, before=old, proposed=value))
            if conflicts:
                return dict(saved=False, conflicts=conflicts, hardware=False)
            changed = False
            filled_model_fields = {}
            for axis in range(1, 7):
                key = str(axis)
                fields = copy.deepcopy(state['drafts'].get(key, {}).get('fields') or
                                       state['approved'].get(key, {}).get('fields') or blank_fields())
                for name, value in gains.items(): fields[name]['value'] = value
                # Model data are identified in the audit event; not physical approval.
                for name, value in dict(q_min=float(model_limits[axis-1][0]),q_max=float(model_limits[axis-1][1]),direction=1,offset=0).items():
                    if fields[name]['value'] is None:
                        fields[name]['value'] = value
                        filled_model_fields.setdefault(key, []).append(name)
                validate_fields(fields)
                old = state['drafts'].get(key)
                if old and old['fields'] == fields and old['source'] == 'user-provided-mit-initial-trial': continue
                state['drafts'][key] = dict(schema_version=SCHEMA,axis_id=axis,model='rs-06' if axis<=3 else 'rs-00',
                    source='user-provided-mit-initial-trial',measured_at=None,control_mode='MIT',review_status='draft',fields=fields,saved_at=time.time())
                changed = True
            if changed:
                self._commit(state, dict(action='save-user-mit-initial-trial', label='用户提供的 MIT 初始试验配置',
                    axes=[1,2,3,4,5,6], field_sources={'kp/kd/tau_ff':'user task 2026-10-09',
                    'q_min/q_max':'existing Robot.lower/upper from locked URDF/MuJoCo; model-only',
                    'direction/offset':'official identity mapping and prior static confirmation'},
                    filled_model_fields=filled_model_fields, retained_fields='non-empty model/mapping/engineering fields retained from existing profile',
                    physical_validation=False, hardware_writes=False))
            return dict(saved=True, changed=changed, revision=state['revision'], hardware=False)

    def action(self, body, session='local-test'):
        with self.locked():
            state = self.load()
            action = body.get('action')
            if action == 'export':
                exported = copy.deepcopy(state)
                exported['device_id'] = None
                exported['snapshots'] = []
                exported['events'] = []
                return dict(backup=envelope(exported))
            if type(body.get('revision')) is not int or body['revision'] != state['revision']:
                raise ValueError('配置版本冲突；请刷新后重新审核（多标签页保护）')
            if action == 'draft':
                key = axis_key(body.get('axis_id'))
                validate_fields(body.get('fields'))
                state['drafts'][key] = dict(schema_version=SCHEMA,axis_id=int(key),model='rs-06' if int(key)<4 else 'rs-00',source='human-software',measured_at=None,control_mode='MIT' if int(key)<=6 else 'unknown',review_status='draft',fields=copy.deepcopy(body['fields']),saved_at=time.time())
                self._commit(state, dict(action='draft',axis_id=int(key)))
            elif action == 'snapshot':
                snapshot = copy.deepcopy(body.get('snapshot'))
                validate_snapshot(snapshot)
                state['snapshots'].append(snapshot)
                self._commit(state, dict(action='snapshot',axis_id=snapshot['axis_id']))
            elif action == 'review':
                key = axis_key(body.get('axis_id'))
                if key not in state['drafts']: raise ValueError('请先保存草稿')
                before = state['approved'].get(key, {}).get('fields', blank_fields())
                after = state['drafts'][key]['fields']
                changes = [dict(field=k, before=before[k], after=after[k]) for k in FIELDS if before[k] != after[k]]
                if not changes: raise ValueError('没有待批准的数值变化')
                token = secrets.token_urlsafe(32)
                self.reviews = {k:v for k,v in self.reviews.items() if v['expires'] > time.monotonic()}
                if len(self.reviews) >= 128: raise ValueError('审核请求过多')
                self.reviews[token] = dict(revision=state['revision'],axis=key,session=session,expires=time.monotonic()+120)
                return dict(review_token=token,changes=changes,expires_in_s=120,warning='只批准本地软件配置；刚度/力矩变化可能突然出力。未写电机、未核实身份、未完成物理验收。')
            elif action == 'approve':
                review = self.reviews.pop(body.get('review_token'), None)
                if not review or review['session'] != session or review['revision'] != state['revision'] or review['expires'] <= time.monotonic() or body.get('confirm') != 'software-only':
                    raise ValueError('审核已失效或未明确确认；请重新审核')
                key = review['axis']
                approved = copy.deepcopy(state['drafts'][key])
                approved.update(review_status='software-approved',approved_at=time.time(),confirmation='explicit-software-only',version=state['revision']+1)
                state['approved'][key] = approved
                self._commit(state, dict(action='approve-software-only',axis_id=int(key)))
            elif action == 'import':
                backup = body.get('backup', {})
                if not isinstance(backup,dict) or set(backup) != {'state','checksum'} or digest(backup['state']) != backup['checksum']: raise ValueError('备份校验失败')
                source = backup['state']; validate_state(source)
                if source['device_id'] not in (None,state['device_id']): raise ValueError('备份设备身份不匹配')
                # Import stages drafts ONLY, never restores approval or runtime permission.
                for key, profile in {**source['approved'], **source['drafts']}.items():
                    draft = copy.deepcopy(profile)
                    for field in ('approved_at','confirmation','version'): draft.pop(field,None)
                    draft.update(review_status='draft',saved_at=time.time())
                    state['drafts'][key] = draft
                self._commit(state, dict(action='import-as-drafts'))
            elif action == 'mock_apply':
                key = axis_key(body.get('axis_id'))
                if key not in state['drafts']: raise ValueError('请先保存草稿')
                outcome = body.get('outcome')
                if outcome not in ('match','mismatch','timeout'): raise ValueError('Mock 结果无效')
                self._commit(state, dict(action='mock-write-readback',axis_id=int(key),outcome=outcome,transport='mock',warning='仅模拟；真实失败可能部分生效，需人工复核；不自动回滚设备'))
            else:
                raise ValueError(LOCK_REASON)
            return dict(state=state,hardware=False,armed=False,drill=True,preparation=preparation_checks(state),provenance=parameter_provenance(state))


class ConfigAPI:
    """Loopback peer + exact origin + transient bearer session + one-use sequence.
    No unlock endpoint exists. A local malicious process is outside this boundary.
    """
    def __init__(self, store, port):
        self.store, self.port = store, port
        self.sessions = {}
        self.lock = threading.RLock()

    def request(self, method, path, headers, peer, body=None):
        with self.lock:
            host = headers.get('Host','')
            origin = headers.get('Origin','')
            if peer not in ('127.0.0.1','::1') or host not in (f'127.0.0.1:{self.port}',f'localhost:{self.port}') or origin != 'http://'+host or headers.get('Sec-Fetch-Site') not in (None,'same-origin'):
                raise ValueError('只允许本机同源软件配置请求，拒绝跨站/隧道')
            now = time.monotonic()
            self.sessions = {k:v for k,v in self.sessions.items() if v['expires'] > now}
            if method == 'POST' and path == '/config/session':
                if len(self.sessions)>=128: raise ValueError('会话过多')
                token = secrets.token_urlsafe(32)
                self.sessions[token] = dict(expires=now+1800,seq=0)
                return dict(token=token,seq=0,**self.store.status())
            token = headers.get('X-Config-Session')
            session = self.sessions.get(token)
            if not session: raise ValueError('软件配置会话失效，请刷新')
            if method != 'POST' or path != '/config/action': raise ValueError('配置接口不存在')
            if type(body) is not dict or type(body.get('seq')) is not int or body['seq'] != session['seq']+1: raise ValueError('请求序号重复或乱序')
            session['seq'] += 1
            if body.get('action') == 'status': return self.store.status()
            return self.store.action(body,session=token)
