"""In-memory SDK only: real-time field scheduling and recovery classification."""
from dataclasses import replace
from types import SimpleNamespace
import threading
import time
import pytest
from test_csp_twin import Controller,Owner,wait_until,fixture
from rebotarmcontroller.csp_hardware import CspHardware
from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZTransport,NativeXYZExecutor,FeedbackTimeout,AXES
from test_twin_jog import console


def test_ros_revoke_before_owner_hold_preserves_telemetry_timeout_type():
 twin,hw,clock,writes,pauses,stops,jog=console();reasons=[]
 hw.hold_requested=threading.Event();hw.hold_requested.set();hw.hold_recoverable=True
 hw.suspend=lambda reason:reasons.append(reason) or True
 twin.tick()
 assert len(reasons)==1 and isinstance(reasons[0],FeedbackTimeout)
 assert not twin.session and twin.reference is None and not writes


def test_j7_startup_does_not_replay_an_expired_open_button():
 from rebotarmcontroller.csp_gripper import CspGripper
 from rebotarmcontroller.csp_runtime.rebot_mit_hardware import Sample
 clock=[100.];sent=[]
 hw=SimpleNamespace(clock=lambda:clock[0],output=None,motion_ready=True,
                    config=SimpleNamespace(policy=SimpleNamespace(command_age=.5)))
 g=CspGripper(hw);g.speed_max=1.;g.authorized=True;g.sample=Sample(7,0.,0.,5,100.,100.,iq_raw_a=0.)
 g._read=lambda check:g.sample.__class__(7,0.,0.,5,clock[0],clock[0],iq_raw_a=0.)
 def startup(check):
  clock[0]+=.3;g.reference=0.;g.history=[(clock[0],0.)]
  g.view=SimpleNamespace(active=True,permit=SimpleNamespace(expires_at=0),runtime={0x7018:16.},
                         write_register=lambda index,value,*a,**kw:sent.append(value))
 g._start=startup;g.manual=dict(at=100.,velocity=.5)
 g.step()
 assert g.manual is None and sent==[0.] and g.manual_velocity==0.
 g.manual=dict(at=clock[0],velocity=.5);clock[0]+=.1;g.step()
 assert sent[-1]>0  # Fresh held input can move; the old press cannot.


def test_old_current_is_refreshed_before_newer_mode_without_widening_600ms():
 clock=[100.];sdk=Controller();queries=[]
 bus=NativeXYZTransport(factory=lambda **kw:sdk,ownership=Owner(),clock=lambda:clock[0]);bus.open_readonly()
 hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:clock[0]);hw.phase='READONLY';hw.set_control_mode('full')
 executor=NativeXYZExecutor(hw.config,bus,fk=lambda q:[0,0,0],clock=lambda:clock[0])
 try:
  for i in AXES:
   m=bus.motors[i];f32,i8=m.robstride_get_param_f32_host_id,m.robstride_get_param_i8
   def read_f32(index,*args,read=f32,axis=i):clock[0]+=.008;queries.append((axis,index));return read(index,*args)
   def read_i8(index,*args,read=i8,axis=i):clock[0]+=.008;queries.append((axis,index));return read(index,*args)
   m.robstride_get_param_f32_host_id=read_f32;m.robstride_get_param_i8=read_i8
  hw._read();clock[0]+=.1;queries.clear()
  for i in AXES:
   s=replace(bus.latest_samples[i],requested_at=clock[0]-.1,received_at=clock[0]-.099,mode_requested_at=clock[0]-.1,diagnostics_requested_at=clock[0]-.1)
   bus.latest_samples[i]=bus.previous_samples[i]=s;hw.samples[i]=s
  bus.latest_samples[1]=hw.samples[1]=replace(bus.latest_samples[1],diagnostics_requested_at=clock[0]-.590)
  hw.phase='ACTIVE'
  def validate():clock[0]+=.005;executor.validate(dict(hw.samples))
  hw._read_cycle(on_io=validate)
  assert queries[0]==(1,0x701A)  # Old mode-first scheduler hit 603ms here.
  assert len(queries)==18 and hw.config.policy.feedback_age==.6 and not sdk.calls
 finally:bus.close()


@pytest.mark.parametrize('recoverable',[False,True])
def test_only_telemetry_timeout_offers_manual_readiness_recovery(fixture,recoverable):
 hw,sdk=fixture;hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token);wait_until(lambda:hw.motion_ready)
 reason=FeedbackTimeout('电流反馈过期') if recoverable else '用户暂停 / 页面失焦'
 hw.suspend(reason);wait_until(lambda:hw.phase=='HOLDING' and hw.holding['feedback_confirmed'])
 assert hw.holding['automatic_manual_recovery'] is recoverable
 calls=len(sdk.calls);time.sleep(.05);assert len(sdk.calls)==calls and not hw.teleop_owned
 token=hw.teleop_acquire();wait_until(lambda:hw.motion_ready)
 assert hw.target is None and hw.paused and all(m.enabled for m in sdk.motors.values())
 assert all(c[0]=='write' and c[2]==0x7016 for c in sdk.calls[calls:])
 hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY');assert hw.last_report['cleanup_ok']


def test_ros_revoke_preserves_owner_heartbeat_timeout_reason():
 twin,hw,clock,writes,pauses,stops,jog=console();reasons=[]
 hw.hold_requested=threading.Event();hw.hold_requested.set();hw.hold_recoverable=False
 hw.stop_reason='网页心跳超过700 ms，锁定暂停'
 hw.suspend=lambda reason:reasons.append(reason) or True
 twin.tick()
 assert reasons==[hw.stop_reason]
 assert not twin.session and twin.reference is None and not writes
