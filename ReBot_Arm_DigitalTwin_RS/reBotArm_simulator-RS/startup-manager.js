'use strict';
const fs = require('node:fs');
const path = require('node:path');
const net = require('node:net');
const {spawn, execFile} = require('node:child_process');
const {promisify} = require('node:util');
const exec = promisify(execFile);
const ROOT = path.resolve(__dirname, '..');
const KEYS = ['ros','bridge','can','simulation','hardware','camera','mcp','text'];
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));

// Subscribe only: no publisher, service call, control lease, enable or CAN socket.
class FeedbackObserver {
  constructor() { this.samples = {}; this.socket = null; this.retryAt = 0; }
  connect() {
    if (this.socket || Date.now() < this.retryAt) return;
    this.retryAt = Date.now() + 2000;
    const socket = this.socket = new WebSocket('ws://127.0.0.1:9090');
    socket.onopen = () => {
      for (const [key, topic, type] of [
        ['hardware','/rebotarm/twin/state','std_msgs/msg/String'],
        ['simulation','/rebotarm_rs/twin/state','std_msgs/msg/String'],
        ['mujoco','/rebotarm_rs/mujoco/joint_states','sensor_msgs/msg/JointState'],
        ['camera','/camera/color/preview/compressed','sensor_msgs/msg/CompressedImage']
      ]) socket.send(JSON.stringify({op:'subscribe',id:`startup-${key}`,topic,type,throttle_rate:200,queue_length:1}));
    };
    socket.onmessage = event => {
      try {
        const message = JSON.parse(event.data);
        if (message.op !== 'publish') return;
        const key = message.topic === '/rebotarm/twin/state' ? 'hardware' : message.topic === '/rebotarm_rs/twin/state' ? 'simulation' : message.topic === '/rebotarm_rs/mujoco/joint_states' ? 'mujoco' : message.topic === '/camera/color/preview/compressed' ? 'camera' : null;
        if (!key) return;
        const state = ['mujoco','camera'].includes(key) ? {} : JSON.parse(message.msg.data);
        const valid=key==='camera'?typeof message.msg.data==='string'&&message.msg.data.length>100:key==='mujoco'||state.feedback_valid===true;
        this.samples[key] = {time:Date.now(),valid,phase:state.hardware_phase || null,owned:Boolean(state.owned),prepared:Boolean(state.control_prepared),recoveryRequired:Boolean(state.recovery_required),message:String(state.message || '').slice(0,600)};
      } catch (_) { /* Reject malformed bridge messages. */ }
    };
    // Closing from error can re-enter the handler while CONNECTING in Node.
    // Clear on error too: a refused connection may not emit close.
    const disconnected = () => { if (this.socket === socket) this.socket = null; };
    socket.onerror = disconnected;
    socket.onclose = disconnected;
  }
  snapshot() {
    this.connect();
    return {connected:this.socket?.readyState === WebSocket.OPEN,...Object.fromEntries(Object.entries(this.samples).map(([key,s])=>[key,{...s,fresh:Date.now()-s.time<1500}]))};
  }
  close() { this.socket?.close(); }
}
function portOpen(port) {
  return new Promise(resolve => {
    const socket = net.connect({port,host:'127.0.0.1'});
    const done = value => {socket.destroy();resolve(value);};
    socket.setTimeout(350);socket.once('connect',()=>done(true));socket.once('error',()=>done(false));socket.once('timeout',()=>done(false));
  });
}
function processes() {
  const result = {bridge:false,simulation:false,mcp:false,text:false,hardware:false,camera:false};
  for (const item of fs.readdirSync('/proc')) {
    if (!/^\d+$/.test(item)) continue;
    try {
      const argv = fs.readFileSync(`/proc/${item}/cmdline`,'utf8').split('\0').filter(Boolean);
      const cmd=argv.join(' '), names=argv.map(arg=>path.basename(arg));
      const cwd = fs.realpathSync(`/proc/${item}/cwd`);
      if (cwd !== ROOT && !cmd.includes(`${ROOT}/rebotarm_ros2_RS/`)) continue;
      if (names.includes('rosbridge_websocket')) result.bridge=true;
      if (names.some(name=>['FakeRsDriver','mujoco_rs_sync','start_rs_sim.sh'].includes(name))) result.simulation=true;
      if (names.includes('rebotarm_mcp_server')) result.mcp=true;
      if (argv.includes('rebotarm_agent.rebotarm_text_agent') && argv.includes('-m')) result.text=true;
      if (argv.includes('rebotarmcontroller.csp_twin_driver') && argv.includes('-m')) result.hardware=true;
      if (names.includes('orbbec_ros_camera_bridge.py')) result.camera=true;
    } catch (_) { /* Process may exit during inspection. */ }
  }
  return result;
}
function cameraPresent() {
  try{return fs.readdirSync('/dev/v4l/by-id').some(name=>/^usb-Orbbec_R__Orbbec_Gemini_336L_.*-video-index0$/.test(name));}
  catch(_){return false;}
}
async function canStatus() {
  try {
    const {stdout} = await exec('/usr/bin/python3',[path.join(ROOT,'scripts/can_interface.py'),'--status'],{timeout:2500});
    return JSON.parse(stdout);
  } catch (_) { return {ready:false,present:false,message:'无法核对 USB-CAN 接口，请查看 CAN 日志。'}; }
}
function launch(key) {
  const directory = path.join(ROOT,'outputs/startup');
  fs.mkdirSync(directory,{recursive:true});
  const logfile = path.join(directory,`${key}.log`), fd = fs.openSync(logfile,'a');
  const script = path.join(ROOT,'scripts',key === 'can' ? 'prepare_can.sh' : 'start_component.sh');
  const args = key === 'can' ? [script] : [script,key === 'ros' ? 'check' : key];
  const child = spawn('/bin/bash',args,{cwd:ROOT,detached:true,stdio:['ignore',fd,fd]});
  fs.closeSync(fd);child.unref();
  const completion = new Promise((resolve,reject)=>{
    const timer=['ros','can'].includes(key)?setTimeout(()=>{try{process.kill(-child.pid,'SIGTERM');}catch(_){}reject(new Error('本步骤超时，请查看日志后重试。'));},key==='can'?120000:20000):null;
    const finish=(error)=>{if(timer)clearTimeout(timer);error?reject(error):resolve();};
    child.once('error',finish);child.once('exit',(code)=>finish(code===0?null:new Error(`${key} 进程退出（${code}），请展开启动日志。`)));
  });
  // Long-lived components are observed by feedback, but an early exit is recorded.
  completion.catch(()=>{});
  return {pid:child.pid,completion};
}
function createStartupManager(options={}) {
  const observer = options.observer || new FeedbackObserver();
  const hardware = options.hardware || require('./hardware-launcher');
  const inspect = options.processes || processes, readCan = options.canStatus || canStatus, port = options.portOpen || portOpen;
  const run = options.launch || launch;
  const hasCamera=options.cameraPresent||cameraPresent;
  const jobs = {}, ros = {ready:false,message:'点击检查已安装的 ROS 2 与项目环境。'};
  let statusPromise,allJob=null;
  async function status() {
    if (statusPromise) return statusPromise;
    statusPromise = (async()=>{
      const feedback = observer.snapshot(), proc = inspect();
      const [can,bridgePort,mcpPort,textPort] = await Promise.all([readCan(),port(9090),port(8081),port(8082)]);
      const services = {
        web:{ready:true,message:'网页服务已运行 · 本机 3002'},
        ros:{...ros},
        bridge:{ready:Boolean(proc.bridge && feedback.connected),running:proc.bridge,message:feedback.connected && proc.bridge?'ROSBridge 已连接 · 9090':bridgePort?'9090 已占用，尚未确认本项目 ROSBridge':'尚未启动 · 网页与 ROS 的联动接口'},
        can,
        simulation:{ready:Boolean(feedback.connected && feedback.simulation?.fresh && feedback.simulation.valid && feedback.mujoco?.fresh),running:proc.simulation,message:feedback.simulation?.fresh && feedback.simulation.valid && feedback.mujoco?.fresh?'仿真控制反馈与 MuJoCo 状态均在更新':proc.simulation?'进程运行中，等待仿真控制反馈和 MuJoCo 状态':'尚未启动 · Fake 控制器 + MuJoCo'},
        hardware:{ready:Boolean((hardware.runningPid() || proc.hardware) && feedback.connected && feedback.hardware?.fresh && feedback.hardware.valid),running:Boolean(hardware.runningPid() || proc.hardware),needsRecovery:Boolean(feedback.hardware?.fresh&&(feedback.hardware.recoveryRequired||['FAULT','RECOVERY'].includes(feedback.hardware.phase))),message:feedback.hardware?.fresh?`实机反馈${feedback.hardware.valid?'有效':'未确认'} · ${feedback.hardware.phase || '未知阶段'}${feedback.hardware.recoveryRequired?' · 需在重置与维护中恢复':feedback.hardware.owned?' · 有控制会话':''}`:'未收到有效实机状态；启动服务只读取反馈'},
        camera:{ready:Boolean(proc.camera&&feedback.connected&&feedback.camera?.fresh&&feedback.camera.valid),running:Boolean(proc.camera),present:hasCamera(),message:proc.camera&&feedback.camera?.fresh&&feedback.camera.valid?'Gemini 实时画面已更新':proc.camera?'相机采集运行中，尚无新鲜图像':hasCamera()?'相机已接入，尚未启动采集':'未发现 Gemini 彩色相机'},
        mcp:{ready:Boolean(proc.mcp && mcpPort),running:proc.mcp,message:proc.mcp && mcpPort?'仿真 MCP 接口已运行 · 8081':mcpPort?'8081 被其他服务占用':'可选 · 仿真任务 / MCP 接口'},
        text:{ready:Boolean(proc.text && textPort),running:proc.text,message:proc.text && textPort?'仿真文本代理已运行 · 8082':textPort?'8082 被其他服务占用':'可选 · 需模型配置；主控制台对话也可直接接模型'}
      };
      for (const key of KEYS) services[key].job=jobs[key] || null;
      return {ok:true,root:ROOT,services,feedback,bundle:allJob,checkedAt:new Date().toISOString()};
    })().finally(()=>{statusPromise=null;});
    return statusPromise;
  }
  async function execute(key) {
    if (key === 'ros') ros.ready=false;
    const before = await status(), services = before.services;
    if (key !== 'ros' && services[key].ready) return {ok:true,reused:true,message:'已就绪，复用当前服务。'};
    if (key !== 'ros' && !ros.ready) throw new Error('请先完成第 1 步：检查 ROS 2 环境。');
    if (['simulation','hardware','camera','mcp','text'].includes(key) && !services.bridge.ready) throw new Error('请先启动 ROS 联动接口。');
    if (key==='camera'&&!services.camera.present)throw new Error('未发现 Gemini 彩色相机，请检查 USB 连接。');
    if (key === 'hardware' && !services.can.ready) throw new Error('请先启用 CAN 接口，确认 UP / 1 Mbps。');
    if (key === 'mcp' && !services.simulation.ready) throw new Error('MCP 服务用于仿真，请先启动仿真。');
    if (key === 'text' && !services.mcp.ready) throw new Error('请先启动仿真 MCP 接口。');
    if (key === 'can' && services.hardware.running) throw new Error('实机驱动运行中，禁止重新配置 CAN。');
    if (services[key].running) throw new Error('服务进程已存在但尚未就绪；请查看反馈与日志，未启动第二份进程。');
    if ((key === 'bridge' && await port(9090)) || (key === 'mcp' && await port(8081)) || (key === 'text' && await port(8082))) throw new Error('端口已有服务占用，未启动重复进程。');
    const launched = key === 'hardware' ? hardware.connect() : run(key);
    if (key === 'ros' || key === 'can') {
      await launched.completion;
      if (key === 'ros') { ros.ready=true;ros.message='ROS 2、项目工作空间与 Python 依赖检查通过。'; }
      else if (!(await readCan()).ready) throw new Error('CAN 尚未达到 UP / 1 Mbps，请查看日志。');
      return {ok:true,message:key === 'ros' ? ros.message : 'CAN 已启用 · 1 Mbps。'};
    }
    let earlyError;
    launched.completion?.catch(error=>{earlyError=error;});
    const deadline=Date.now()+30000;
    while (Date.now()<deadline) {
      if (earlyError) throw earlyError;
      if ((await status()).services[key].ready) return {ok:true,pid:launched.pid,message:key === 'hardware'?'实机反馈已更新；未使能电机。':'服务已就绪。'};
      await delay(350);
    }
    throw new Error('进程已发起，但 30 秒内未确认就绪。请展开日志；不会自动重复启动或重启服务。');
  }
  function start(key) {
    if(key==='all')return startAll();
    if (!KEYS.includes(key)) throw new Error('未知启动步骤');
    if(allJob?.state==='running')throw new Error('一键启动正在进行，请等待完成。');
    if (jobs[key]?.state === 'running') return {ok:true,reused:true,job:jobs[key]};
    const job=jobs[key]={state:'running',startedAt:new Date().toISOString(),message:key === 'can'?'正在检查 CAN；如弹出系统认证窗口，请在本机输入密码。':'正在检查并启动…'};
    execute(key).then(result=>Object.assign(job,{state:'done',...result}),error=>Object.assign(job,{state:'error',message:error.message}));
    return {ok:true,job};
  }
  function startAll() {
    if(allJob?.state==='running')return {ok:true,reused:true,job:allJob};
    if(Object.values(jobs).some(j=>j.state==='running'))throw new Error('已有启动步骤正在进行，请等待完成。');
    const job=allJob={id:'startup-'+Date.now(),state:'running',startedAt:new Date().toISOString(),message:'正在一键检查并启动只读服务…',warnings:[]};
    (async()=>{
      for(const key of ['ros','bridge','can','hardware','camera']) {
        if(key==='camera'&&!hasCamera()){job.warnings.push('未发现相机，可在接入后单独启动。');continue;}
        job.step=key;job.message=key==='camera'?'正在启动相机画面…':'正在检查：'+key;
        const component=jobs[key]={state:'running',startedAt:new Date().toISOString(),message:job.message};
        try{Object.assign(component,{state:'done',...await execute(key)});}
        catch(error){Object.assign(component,{state:'error',message:error.message});if(key!=='camera')throw error;job.warnings.push('相机：'+error.message);}
      }
      const result=await status();
      job.state='done';job.message=result.services.hardware.needsRecovery?'反馈已连接，仍需进入“重置与维护”恢复控制状态。':'只读反馈已就绪，可以进入操作工作台。';
      if(job.warnings.length)job.message+=' '+job.warnings.join(' ');
    })().catch(error=>{job.state='error';job.message=error.message;});
    return {ok:true,job};
  }
  function logs(key) {
    if (!KEYS.includes(key)) throw new Error('未知日志');
    const filename=key==='hardware'?path.join(ROOT,'outputs/twin-console/csp-driver.log'):path.join(ROOT,'outputs/startup',`${key}.log`);
    try { const fd=fs.openSync(filename,'r');try {const size=fs.fstatSync(fd).size,b=Buffer.alloc(Math.min(size,12000));fs.readSync(fd,b,0,b.length,Math.max(0,size-b.length));return b.toString('utf8');} finally {fs.closeSync(fd);} }
    catch (_) {return key === 'hardware'?'实机驱动日志：outputs/twin-console/csp-driver.log；既有服务直接复用。':'本步骤尚无启动日志；已运行的旧服务可直接复用。';}
  }
  return {status,start,logs,busy:()=>allJob?.state==='running'||Object.values(jobs).some(j=>j.state==='running'),close:()=>observer.close()};
}
module.exports={createStartupManager,KEYS};
