"""External feedback controller tests: fake hardware and local HTTP only."""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from test_twin_program import console, program
from rebotarmcontroller import twin_algorithm as module


def config(**changes):
    return dict(schema=module.SCHEMA, name='test', endpoint='http://127.0.0.1:8089/step',
                output='joint_velocity', rate_hz=10, parameters={}, **changes)


def setup(tmp_path, monkeypatch, values=None):
    parts = console(tmp_path)
    t = parts[0]
    def reply(c, p):
        return dict(schema=module.SCHEMA, request_id=p['request_id'], output=c['output'], values=values or [.01]*6)
    monkeypatch.setattr(module, 'exchange', reply)
    return parts, t.extensions.controller


def checked(controller, output=None):
    cfg=config()
    if output:cfg['output']=output
    controller.check(dict(id='check', config=cfg, target=[.05]*6, speed=.1))
    controller.worker.join(2)
    assert controller.checked, controller.stats
    return controller.checked['id']


@pytest.mark.parametrize('change', [dict(endpoint='https://example.com/control'), dict(endpoint='http://localhost:8089/step'),
    dict(endpoint='http://127.0.0.1:8089/step?hidden=1'), dict(output='torque'), dict(rate_hz=1000),
    dict(parameters={'gain':float('nan')}), dict(script='os.system(...)')])
def test_config_rejects_nonlocal_ambiguous_and_unsupported(change):
    c=config();c.update(change)
    with pytest.raises(ValueError):module.configuration(c)


def test_check_is_inert_and_start_requires_matching_check(tmp_path, monkeypatch):
    (t,hw,q,g,events,writes,*_), c=setup(tmp_path,monkeypatch)
    id=checked(c)
    assert not writes and events[-1]['ok'] and not c.active
    t.session='owner';t.mode='HOLD'
    with pytest.raises(ValueError,match='失效'):c.start(dict(check_id='wrong'))
    q[0]=.1
    with pytest.raises(ValueError,match='变化'):c.start(dict(check_id=id))
    q[0]=0;hw.motion_ready=False
    with pytest.raises(ValueError,match='电机保持'):c.start(dict(check_id=id))
    assert not writes


@pytest.mark.parametrize('output', ['joint_velocity', 'joint_position'])
def test_feedback_loop_uses_shared_executor_and_stops_without_replay(tmp_path,monkeypatch,output):
    (t,hw,q,g,events,writes,checks,tick,_),c=setup(tmp_path,monkeypatch)
    id=checked(c,output);t.session='owner';t.mode='HOLD';c.start(dict(check_id=id))
    for _ in range(8):
        tick(.05)
        if c.worker:c.worker.join(1)
    assert writes and c.stats['cycles']>=2 and t.mode=='ALGORITHM'
    assert max(abs(v) for row in writes for v in row)<=.04
    count=len(writes);t._hold('pause');tick()
    assert not c.active and not c.checked and len(writes)==count


def test_delayed_response_cannot_resume_after_stop(tmp_path,monkeypatch):
    (t,hw,q,g,events,writes,checks,tick,_),c=setup(tmp_path,monkeypatch)
    id=checked(c);t.session='owner';t.mode='HOLD';c.start(dict(check_id=id))
    entered=threading.Event();release=threading.Event()
    def slow(config,p):
        entered.set();release.wait(2)
        return dict(schema=module.SCHEMA,request_id=p['request_id'],output=config['output'],values=[.01]*6)
    monkeypatch.setattr(module,'exchange',slow)
    tick();assert entered.wait(1)
    tick(.21);assert not c.active and t.mode=='HOLD'
    release.set();c.worker.join(1);tick()
    assert not writes and c.latest is None and c.stats['status']=='error'


@pytest.mark.parametrize('output',['joint_position','joint_velocity'])
def test_ruckig_hardware_adapter_uses_live_seed_and_only_can_owner_smooths(tmp_path,monkeypatch,output):
    (t,hw,q,g,events,writes,checks,tick,_),c=setup(tmp_path,monkeypatch)
    hw.continuous_jog_reference=True;hw.command_reference=[.001]*6
    sent=[];hw.teleop_stream=lambda token,seq,at,target,velocity,speed:sent.append((target,velocity))
    id=checked(c,output);t.session='owner';t.mode='HOLD';c.start(dict(check_id=id))
    for _ in range(5):
        tick(.05)
        if c.worker:c.worker.join(1)
    assert sent and not writes
    target,velocity=sent[-1]
    assert max(abs(v) for v in velocity)<=.1+1e-10
    assert all(abs((a-.001)/.05-v)<1e-10 for a,v in zip(target,velocity))
    assert not any(c.ramp.velocity) and not any(c.ruckig_ramp.velocity)


@pytest.mark.parametrize('change',[dict(request_id='old'),dict(values=[1.]*6),dict(values=[float('nan')]*6),dict(output='torque'),dict(enable=True)])
def test_invalid_output_never_becomes_a_reference(tmp_path,monkeypatch,change):
    (t,hw,q,g,events,writes,*_),c=setup(tmp_path,monkeypatch)
    def invalid(config,p):
        r=dict(schema=module.SCHEMA,request_id=p['request_id'],output=config['output'],values=[.01]*6);r.update(change);return r
    monkeypatch.setattr(module,'exchange',invalid)
    c.check(dict(id='bad',config=config(),target=[0.]*6,speed=.1));c.worker.join(1)
    assert not c.checked and not writes and events[-1]['ok'] is False


def test_algorithm_excludes_manual_program_and_tcp_inputs(tmp_path,monkeypatch):
    (t,hw,q,g,events,writes,checks,tick,_),c=setup(tmp_path,monkeypatch)
    id=checked(c);t.session='owner';t.mode='HOLD';c.start(dict(check_id=id))
    with pytest.raises(RuntimeError,match='算法'):t._jog(dict(twist=[0.]*6,speed=.1))
    with pytest.raises(RuntimeError):t.extensions.readonly(dict(op='program_preview',session='owner',program=program([dict(type='wait',seconds=1)])))
    with pytest.raises(ValueError):t.extensions.command(dict(op='tcp_target',pose=[0.]*6,speed=.1))
    t._release('lost lease');tick();assert not c.active and not writes


def test_http_protocol_round_trip_and_no_redirect():
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            b=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            self.send_response(200 if self.path=='/step' else 302);self.end_headers();self.wfile.write(json.dumps(b).encode())
        def log_message(self,*_):pass
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler);worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
    try:
        c=config();c['endpoint']=f'http://127.0.0.1:{server.server_port}/step'
        assert module.exchange(c,{'request_id':'test'})=={'request_id':'test'}
        c['endpoint']=c['endpoint'].replace('/step','/redirect')
        with pytest.raises(ValueError,match='302'):module.exchange(c,{})
    finally:server.shutdown();server.server_close()
