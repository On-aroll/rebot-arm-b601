#!/usr/bin/env python3
"""ROS transport acceptance using an in-process Fake driver in an isolated domain.

Run: source scripts/rs_env.sh && ROS_DOMAIN_ID=82 python3 scripts/verify_twin_console.py
Never imports HardwareManager, enables physical motors, or targets /rebotarm.
"""
import json
import math
import os
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

if os.environ.get("ROS_DOMAIN_ID") != "82":
    raise SystemExit("This simulation-only test requires ROS_DOMAIN_ID=82")

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rebotarm_msgs.msg import JointMitCmd
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from rebotarmcontroller.fake_rs_driver import FakeRsDriver


def main():
    rclpy.init(args=[])
    driver = FakeRsDriver()
    probe = Node("twin_acceptance_probe")
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(driver)
    executor.add_node(probe)
    worker = threading.Thread(target=executor.spin, daemon=True)
    state, events = {}, {}
    transport_lock = threading.RLock()
    sequence = 0
    session = ""
    heartbeat_on = threading.Event()
    finished = threading.Event()
    base = "/rebotarm_rs/twin"
    command = probe.create_publisher(String, base + "/command", 1)
    legacy = probe.create_publisher(JointMitCmd, "/rebotarm_rs/joints/joint1/cmd/mit", 1)
    probe.create_subscription(String, base + "/state", lambda msg: state.update(json.loads(msg.data)), 10)
    probe.create_subscription(String, base + "/events", lambda msg: events.update({json.loads(msg.data).get("id"): json.loads(msg.data)}), 10)
    worker.start()
    checks = []
    algorithm_server = None

    def wait(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise AssertionError(f"timeout; state={state}")

    def send(op, **args):
        nonlocal sequence
        with transport_lock:
            sequence += 1
            ident = f"check-{sequence}"
            payload = dict(op=op, session=session, seq=sequence, stamp_ms=time.time()*1000, id=ident)
            payload.update(args)
            command.publish(String(data=json.dumps(payload)))
            return ident

    def request(op, **args):
        # Match the page: a heartbeat must not replace an unacknowledged
        # request in the deliberately depth-one control queue.
        with transport_lock:
            ident = send(op, **args)
            wait(lambda: ident in events)
            return events[ident]

    def service(name):
        client = probe.create_client(Trigger, base + "/" + name)
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(Trigger.Request())
        wait(future.done)
        result = future.result()
        probe.destroy_client(client)
        return result

    def heartbeat():
        while not finished.wait(0.12):
            if heartbeat_on.is_set():
                send("heartbeat")

    beats = threading.Thread(target=heartbeat, daemon=True)
    beats.start()
    try:
        wait(lambda: bool(state))
        time.sleep(0.2)  # DDS publisher discovery
        start = list(state["actual"])
        assert state['motion_profile']['selected'] == 'ruckig'
        assert request('motion_profile',profile='quintic')['ok']
        wait(lambda:state['motion_profile']['selected']=='quintic')
        assert request('motion_profile',profile='ruckig')['ok']
        wait(lambda:state['motion_profile']['selected']=='ruckig')
        assert state['actual']==start and not state['owned']
        checks.append('Ruckig/default and baseline can switch through ROS while idle without motion')
        reduced=dict(acceleration_scale=.25,jerk_scale=.5)
        assert request('motion_profile',profile='ruckig',parameters=reduced)['ok']
        wait(lambda:state['motion_profile']['profiles']['ruckig']==reduced)
        assert state['actual']==start and not state['owned']
        assert not request('motion_profile',profile='ruckig',parameters=dict(acceleration_scale=2,jerk_scale=1))['ok']
        assert request('motion_profile',profile='ruckig',parameters=dict(acceleration_scale=1,jerk_scale=1))['ok']
        checks.append('trajectory tuning applies atomically through ROS, rejects envelope increases and stays inert')

        assert request("preview", pose=[0.3, 0, 0.3, 0, 0, 0])["ok"]
        assert state["actual"] == start and not state["owned"]
        checks.append("preview is read-only")
        acquired = service("acquire")
        assert acquired.success, acquired.message
        session = json.loads(acquired.message)["session"]
        heartbeat_on.set()
        assert not request('motion_profile',profile='quintic')['ok']
        assert state['motion_profile']['selected']=='ruckig'
        checks.append('owned motion profile cannot be switched during a session')
        assert not service("acquire").success
        checks.append("exclusive owner; a second client cannot acquire")
        legacy.publish(JointMitCmd(pos=0.8, vel=0.3))
        time.sleep(0.15)
        assert state["actual"] == start
        checks.append("legacy joint publishers cannot bypass the lease")
        plan = request("preview", pose=[0.3, 0, 0.3, 0, 0, 0])["plan"]
        assert request("goal", plan_id=plan["id"])["ok"]
        wait(lambda: state.get("mode") == "GOAL")
        wait(lambda: state.get("mode") == "HOLD" and "已到达" in state.get("message", ""), plan["duration"]+5)
        result = state["last_result"]
        assert result["position_error_mm"] <= 2 and result["rotation_error_deg"] <= 2
        checks.append({"coordinate_goal": result, "duration_s": plan["duration"]})
        before = list(state["pose"])
        assert state['jog_input']['policy'] == 'latest_only'
        assert state['jog_input']['dispatch_period_ms'] == 20
        for _ in range(30):
            send("jog", twist=[0.02,0,0,0,0,0], speed=0.15)
            time.sleep(0.02)
        assert state['jog_smoothing']['enabled'] and any(abs(v)>1e-6 for v in state['jog_smoothing']['velocity_rad_s'])
        # Keep the browser's deadman held and send fresh neutral input. This
        # must brake to HOLD, not depend on the missing-command watchdog.
        for _ in range(20):
            assert request('jog',twist=[0]*6,speed=.15)['ok']
            time.sleep(.05)
            if state['mode']=='HOLD' and '平滑减速' in state['message']:break
        assert state['mode']=='HOLD' and '平滑减速' in state['message']
        assert not any(state['jog_smoothing']['velocity_rad_s'])
        after = list(state["pose"])
        assert after[0] > before[0]+0.001
        frozen = list(state["actual"])
        time.sleep(0.2)
        assert max(abs(a-b) for a,b in zip(frozen,state["actual"])) < 1e-5
        checks.append("50 Hz Cartesian jog uses latest-only dispatcher; fresh neutral input brakes to HOLD without drift")
        before = list(state["pose"])
        for _ in range(10):
            send("jog", twist=[0,0,0,0,0,0.1], speed=0.15)
            time.sleep(0.05)
        request("hold")
        wait(lambda: state["mode"] == "HOLD")
        assert not any(state['jog_smoothing']['velocity_rad_s'])
        assert abs(state["pose"][5]-before[5]) > 0.005
        checks.append("orientation jog changes measured TCP yaw")
        assert request("jog", twist=[0.02,0,0,0,0,0], speed=0.15)['ok']
        wait(lambda: state["mode"] == "JOG")
        wait(lambda: state["mode"] == "HOLD", 0.6)
        checks.append("missing jog samples hold within 300 ms despite continuing heartbeat")
        before_grip = state['extensions']['gripper']['position']
        for _ in range(12):
            assert request('jog',twist=[0]*6,speed=.15,gripper_velocity=.2)['ok']
            time.sleep(.05)
        assert state['extensions']['gripper']['position'] > before_grip+.05
        assert request('hold')['ok']
        wait(lambda:state['mode']=='HOLD')
        frozen_grip=state['extensions']['gripper']['position']
        time.sleep(.35)
        assert abs(state['extensions']['gripper']['position']-frozen_grip)<.002
        checks.append('J7 shares the jog lease and deadman; hold freezes gripper without a separate authorization UI')
        assert not request("jog", twist=[math.nan,0,0,0,0,0])["ok"]
        assert not request("jog", twist=[1,0,0,0,0,0])["ok"]
        assert not request("preview", pose=[2,0,0,0,0,0])["ok"]
        checks.append("NaN, excessive velocity and unreachable points are rejected")
        assert not request("jog", twist=[0.02,0,0,0,0,0], stamp_ms=time.time()*1000-1000)["ok"]
        assert not request("jog", twist=[0.02,0,0,0,0,0], seq=0)["ok"]
        assert not request("goal", plan_id="expired-preview")["ok"]
        checks.append("old timestamps, replayed sequence numbers and invalid preview IDs are rejected")
        plan = request("preview", pose=[0.3,0,0.32,0,0,0])["plan"]
        assert request("goal", plan_id=plan["id"])["ok"]
        wait(lambda: state["mode"] == "GOAL")
        heartbeat_on.clear()
        wait(lambda: not state["owned"], 1.2)
        assert state["mode"] == "IDLE"
        frozen = list(state["actual"])
        time.sleep(0.2)
        assert max(abs(a-b) for a,b in zip(frozen,state["actual"])) < 1e-5
        checks.append("lost browser heartbeat interrupts point motion and releases ownership")
        acquired = service("acquire")
        assert acquired.success, acquired.message
        session = json.loads(acquired.message)["session"]
        heartbeat_on.set()
        assert request("preview",pose=list(state["pose"]))["ok"]
        reset = service("reset")
        assert reset.success, reset.message
        heartbeat_on.clear()
        wait(lambda: not state["owned"] and (state.get("reset_status") or {}).get("status") == "succeeded")
        assert state["goal"] is None and state["reference"] is None and state["last_result"] is None
        assert not request("power_hold")["ok"]
        assert not request("goal",plan_id="old")["ok"]
        checks.append("reset service clears owner/preview/reference/result; old lease cannot execute")
        # Workbench uses the same guarded command topic, never a second executor.
        program = dict(schema_version="1.0",robot_model="b601_rs",base_frame="base_link",
            calibration_id="rs-identity-v1",units=dict(position="m",angle="rad",time="s"),name="isolated workbench",
            steps=[dict(type="wait",seconds=.5),dict(type="joint",joints=[state["actual"][0]+.01,*state["actual"][1:]],speed=.15),
                   dict(type="gripper",position=.05,speed=.2),dict(type="wait",seconds=.1)])
        compiled = request("program_preview",program=program)
        assert compiled["ok"],compiled
        checks.append("whole imported program compiles read-only with J7 and waits")
        acquired = service("acquire");assert acquired.success,acquired.message
        session = json.loads(acquired.message)["session"];heartbeat_on.set()
        assert request("gripper_authorize",enabled=True)["ok"]
        assert request("program_start",plan_id=compiled["program_plan"]["id"])["ok"]
        wait(lambda: state["extensions"]["program"].get("status")=="running")
        assert request("program_pause")["ok"]
        wait(lambda: state["extensions"]["program"].get("status")=="paused")
        assert request("program_resume")["ok"]
        wait(lambda: state["extensions"]["program"].get("status")=="completed",8)
        assert abs(state["extensions"]["gripper"]["position"]-.05)<.003
        checks.append("program pause/resume rechecks remaining path; arm and J7 arrive via shared lease")
        frozen = list(state["actual"])
        target = list(state["pose"]);target[0]+=.004
        for _ in range(10):
            assert request("tcp_target",pose=target,orientation=False,speed=.15)["ok"]
            time.sleep(.05)
        wait(lambda: state["mode"]=="HOLD",.7)
        assert max(abs(a-b) for a,b in zip(frozen,state["actual"]))>1e-5
        checks.append("dragged TCP follows through jog executor; stale drag holds without replay")
        assert not request("program_preview",program=dict(program,base_frame="camera"))["ok"]
        assert not request("program_preview",program=dict(program,steps=[dict(type="shell",code="no")]))["ok"]
        checks.append("imported frame mismatch and executable commands are rejected")
        compiled=request("program_preview",program=dict(program,steps=[dict(type="wait",seconds=5)]))
        assert compiled["ok"] and request("program_start",plan_id=compiled["program_plan"]["id"])["ok"]
        assert request("program_cancel")["ok"]
        wait(lambda: state["extensions"]["program"].get("status")=="cancelled")
        assert service("stop").success;heartbeat_on.clear()
        wait(lambda:not state["owned"])
        assert not request("gripper",position=.1,speed=.1)["ok"]
        checks.append("program cancel freezes targets; stopped lease cannot control J7")
        # Independent local feedback-control algorithm: no planner or LLM.
        algorithm_delay = [0.]
        class Algorithm(BaseHTTPRequestHandler):
            def do_POST(self):
                payload=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                time.sleep(algorithm_delay[0])
                q=payload['state']['position'];goal=payload['reference']['position'];speed=payload['limits']['velocity']
                values=[max(-speed,min(speed,1.5*(g-a))) for a,g in zip(q,goal)]
                response=dict(schema='b601-controller/1.0',request_id=payload['request_id'],output='joint_velocity',values=values)
                self.send_response(200);self.end_headers()
                try:self.wfile.write(json.dumps(response).encode())
                except (BrokenPipeError,ConnectionResetError):pass
            def log_message(self,*_):pass
        algorithm_server=ThreadingHTTPServer(('127.0.0.1',0),Algorithm)
        threading.Thread(target=algorithm_server.serve_forever,daemon=True).start()
        cfg=dict(schema='b601-controller/1.0',name='isolated P controller',endpoint=f'http://127.0.0.1:{algorithm_server.server_port}/step',rate_hz=10,output='joint_velocity',parameters={})
        frozen=list(state['actual']);desired=[frozen[0]+.01,*frozen[1:]]
        result=request('controller_check',config=cfg,target=desired,speed=.1)
        assert result['ok'],result
        assert not state['owned'] and state['actual']==frozen
        checks.append('control algorithm single-step check reads feedback without acquiring or moving')
        acquired=service('acquire');assert acquired.success
        session=json.loads(acquired.message)['session'];heartbeat_on.set()
        assert request('controller_start',check_id=result['check_id'])['ok']
        wait(lambda:state['extensions']['controller']['cycles']>=4 and state['actual'][0]>frozen[0]+.002,3)
        assert request('controller_stop')['ok']
        wait(lambda:state['mode']=='HOLD' and not state['extensions']['controller']['active'])
        frozen=list(state['actual']);time.sleep(.3)
        assert max(abs(a-b) for a,b in zip(frozen,state['actual']))<1e-5
        checks.append('independent feedback controller drives Fake through shared executor; stop freezes references')
        result=request('controller_check',config=cfg,target=desired,speed=.1);assert result['ok']
        assert request('controller_start',check_id=result['check_id'])['ok']
        wait(lambda:state['extensions']['controller']['active'])
        algorithm_delay[0]=.4
        wait(lambda:not state['extensions']['controller']['active'] and state['mode']=='HOLD',1.5)
        frozen=list(state['actual']);time.sleep(.5)
        assert max(abs(a-b) for a,b in zip(frozen,state['actual']))<1e-5
        checks.append('slow controller response holds and late results cannot revive motion')
        for engine in ('p','pd','pid','scheduled_pid','ladrc','mpc'):
            cfg_builtin=dict(schema='b601-controller/1.0',engine=engine,name=engine,output='joint_velocity',rate_hz=10,parameters={})
            frozen=list(state['actual']);desired=[frozen[0]+.005,*frozen[1:]]
            result=request('controller_check',config=cfg_builtin,target=desired,speed=.1);assert result['ok'],result
            assert state['actual']==frozen
            assert request('controller_start',check_id=result['check_id'])['ok']
            wait(lambda:state['extensions']['controller']['cycles']>=3 and state['actual'][0]>frozen[0]+.0001,3)
            assert state['extensions']['controller']['engine']==engine
            assert state['extensions']['controller']['tracking']['samples']>=2
            assert request('controller_stop')['ok']
            wait(lambda:not state['extensions']['controller']['active'])
        checks.append('six built-in controllers including scheduled PID, LADRC and MPC use ROS feedback and shared Fake executor; independent tuning metrics and stop work')

        assert service('stop').success;heartbeat_on.clear();wait(lambda:not state['owned'])
        mujoco = probe.create_publisher(JointState, "/rebotarm_rs/mujoco/joint_states", 1)
        time.sleep(0.2)
        for _ in range(5):
            mujoco.publish(JointState(name=[f"joint{i}" for i in range(1,7)], position=list(state["actual"])))
            time.sleep(0.04)
        wait(lambda: state.get("feedback_source", "").endswith("mujoco/joint_states"))
        acquired = service("acquire")
        assert acquired.success, acquired.message
        session = json.loads(acquired.message)["session"]
        heartbeat_on.set()
        wait(lambda: not state["owned"] and not state["feedback_valid"], 1.2)
        assert state["feedback_source"].endswith("mujoco/joint_states")
        assert not service("acquire").success
        checks.append("lost MuJoCo feedback releases control; never silently substitutes Fake feedback")
        output = Path(__file__).resolve().parents[1] / "outputs/twin-console/acceptance.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({"ok":True,"domain":82,"backend":"FakeRsDriver, no CAN", "checks":checks},ensure_ascii=False,indent=2)+"\n")
        print(output)
        print(json.dumps(checks,ensure_ascii=False,indent=2))
    finally:
        if algorithm_server:
            algorithm_server.shutdown();algorithm_server.server_close()
        heartbeat_on.clear(); finished.set(); beats.join(timeout=1)
        driver.twin_control.shutdown()
        driver.leader_teleop.shutdown()
        executor.shutdown(timeout_sec=2); worker.join(timeout=2)
        probe.destroy_node(); driver.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
