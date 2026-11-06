'use strict';
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto');
const {inventory,stopProcesses}=require('./project-shutdown');
const ROOT=path.resolve(__dirname,'..');
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));

function recovered(state) {
  const gripper=state.extensions?.gripper;
  return state.feedback_valid===true && state.hardware_phase==='READONLY' && state.mode==='IDLE' &&
    state.owned===false && state.control_prepared===false && state.goal==null && state.reference==null &&
    !state.recovery_required && state.cleanup_ok===true &&
    Object.keys(state.drive_modes||{}).length===6 &&
    [1,2,3,4,5,6].every(i=>state.drive_modes[String(i)]===0) &&
    Boolean(gripper) && gripper.active===false && gripper.authorized===false && !gripper.recovery_required;
}

// Only subscribes and calls the existing restore-only service. Never advertises
// commands, acquires a lease, opens CAN, chooses a snapshot or enables a drive.
function resetHardware({Socket=WebSocket,timeout=60000}={}) {
  return new Promise((resolve,reject)=>{
    const socket=new Socket('ws://127.0.0.1:9090'),id='maintenance-'+crypto.randomUUID();
    let settled=false,resetId='',latest=null,requested=false;
    const timer=setTimeout(()=>finish(new Error('恢复结果超时；未确认恢复成功，请查看驱动日志和最新状态。')),timeout);
    function finish(error,state) {
      if(settled)return;settled=true;clearTimeout(timer);socket.close();
      error?reject(error):resolve({message:state.reset_status.message,resetId:state.reset_status.id});
    }
    function checkResult() {
      if(!resetId || latest?.reset_status?.id!==resetId)return;
      const status=latest.reset_status;
      if(status.status==='failed')finish(new Error(status.message));
      else if(status.status==='succeeded') {
        if(!recovered(latest))finish(new Error('重置已返回，但只读、失能或恢复读回尚未通过；未确认恢复成功。'));
        else finish(null,latest);
      }
    }
    socket.onopen=()=>socket.send(JSON.stringify({op:'subscribe',id:id+'-state',topic:'/rebotarm/twin/state',type:'std_msgs/msg/String',throttle_rate:100,queue_length:1}));
    socket.onmessage=event=>{
      try {
        const m=JSON.parse(event.data);
        if(m.op==='publish' && m.topic==='/rebotarm/twin/state') {
          latest=JSON.parse(m.msg.data);
          if(!requested) {
            if(latest.feedback_valid!==true || ['FAULT','FEEDBACK_WAIT'].includes(latest.hardware_phase)) {
              finish(new Error('驱动尚无有效反馈，未执行恢复；请检查本体供电与 CAN 接线。'));return;
            }
            requested=true;
            socket.send(JSON.stringify({op:'call_service',id,service:'/rebotarm/twin/reset',type:'std_srvs/srv/Trigger',args:{}}));
          }
          checkResult();
        } else if(m.op==='service_response' && m.id===id) {
          if(!m.result || !m.values?.success)throw new Error(m.values?.message||'控制器拒绝重置');
          const status=JSON.parse(m.values.message);
          if(typeof status.id!=='string' || !status.id)throw new Error('控制器未返回恢复任务编号');
          resetId=status.id;checkResult();
        }
      } catch(error) {finish(error);}
    };
    socket.onerror=()=>finish(new Error('ROS 联动连接异常，恢复结果未确认。'));
    socket.onclose=()=>{if(!settled)finish(new Error('恢复期间联动连接中断，恢复结果未确认。'));};
  });
}

function createMaintenanceManager(options={}) {
  const startup=options.startup,scan=options.inventory||inventory,stop=options.stopProcesses||stopProcesses;
  const reset=options.resetHardware||resetHardware,wait=options.sleep||sleep;
  const root=options.root||ROOT,blocked=options.blocked||(()=>false);
  let job=null;
  const status=()=>({ok:true,job:job?JSON.parse(JSON.stringify(job)):{state:'idle',steps:[]}});
  function save() {
    const dir=path.join(root,'outputs/maintenance');fs.mkdirSync(dir,{recursive:true});
    const file=path.join(dir,job.id+'.json'),tmp=file+'.tmp';
    const fd=fs.openSync(tmp,'w',0o600);
    try{fs.writeFileSync(fd,JSON.stringify(job,null,2));fs.fsyncSync(fd);}finally{fs.closeSync(fd);}
    fs.renameSync(tmp,file);
  }
  function step(message) {
    if(blocked())throw new Error('结束项目已优先，维护已取消。');
    job.message=message;job.steps.push({message,time:new Date().toISOString()});save();
  }
  async function startComponent(key) {
    if(blocked())throw new Error('结束项目已优先，维护已取消。');
    const {job:component}=startup.start(key),deadline=Date.now()+(key==='can'?125000:35000);
    while(component.state==='running') {
      if(Date.now()>deadline)throw new Error('启动步骤超时：'+key+'；请查看启动台日志。');
      await wait(200);
    }
    if(component.state!=='done')throw new Error(component.message);
  }
  async function execute() {
    let before=await startup.status();
    const rows=scan().filter(p=>p.kind==='hardware');
    if(rows.length>1)throw new Error('检测到多份实机驱动，未自动处理；请核对 CAN 所有者。');
    job.previousDrivers=rows;save();
    const reusable=()=>before.services.bridge.ready &&
      before.feedback.hardware?.fresh && before.feedback.hardware.valid &&
      !['FAULT','FEEDBACK_WAIT'].includes(before.feedback.hardware.phase);
    const workerFault=before.feedback.hardware?.fresh && before.feedback.hardware.phase==='FAULT';
    if(rows.length===1 && !reusable() && !workerFault) {
      step('保留现有 CAN 驱动，等待有效反馈。');
      for(let attempt=0;attempt<12&&!reusable();attempt++) {
        await wait(250);
        before=await startup.status();
      }
      if(!reusable())throw new Error('现有驱动尚未恢复有效反馈，已保留 CAN 连接。请核对本体供电与 CAN 接线。');
    }
    const healthy=rows.length===1 && reusable();
    if(!healthy) {
      step('正在正常退出故障驱动；保留原始恢复记录…');
      await stop(rows); // Identity/birth checked SIGTERM; no hardware SIGKILL.
      if(scan().some(p=>p.kind==='hardware'))throw new Error('实机驱动仍存在，未重配 CAN 或启动第二份。');
      step('正在检查 ROS 环境与联动接口…');
      await startComponent('ros');
      if(!(await startup.status()).services.bridge.ready) {
        await stop(scan().filter(p=>p.kind==='bridge'));
        await startComponent('bridge');
      }
      step('正在检查 USB-CAN；需要认证时请在系统窗口输入密码…');
      await startComponent('can');
      step('正在连接唯一实机驱动，只读取反馈…');
      await startComponent('hardware');
    }
    const current=scan().filter(p=>p.kind==='hardware');
    if(current.length!==1 || (healthy && (current[0].pid!==rows[0].pid || current[0].birth!==rows[0].birth)))
      throw new Error('实机驱动身份发生变化，未继续恢复，请重新核对当前状态。');
    job.driver=current[0];
    step('正在清除旧目标、失能，并按原始记录核对恢复参数…');
    job.result=await reset();
    const after=await startup.status();
    if(after.services.camera?.present&&!after.services.camera.ready) {
      step('机械臂已恢复，正在恢复相机采集…');
      try {
        if(!after.services.ros.ready)await startComponent('ros');
        if(after.services.camera.running)await stop(scan().filter(p=>p.kind==='camera'));
        await startComponent('camera');
      } catch(error) {job.cameraWarning=error.message;}
    }
    job.state='done';job.message='恢复完成：已失能并回到只读，旧目标已清空。可以重新选择范围、准备控制。';
    if(job.cameraWarning)job.message+=' 相机仍需处理：'+job.cameraWarning;
  }
  function start(confirmed) {
    if(confirmed!==true)throw new Error('请通过“已支撑，恢复连接与状态”明确确认现场安全并允许失能。');
    if(job?.state==='running')return status();
    if(blocked() || startup.busy())throw new Error('启动或结束项目正在进行，请完成后再恢复。');
    job={id:crypto.randomUUID(),state:'running',confirmedAt:new Date().toISOString(),
      confirmation:'用户点击恢复按钮：已可靠支撑，允许失能与必要的只读重连；不授权使能或运动',
      message:'已收到现场确认，正在检查连接与控制状态…',steps:[]};
    try{save();}catch(error){job.state='error';job.message='现场确认记录保存失败，未开始恢复：'+error.message;throw new Error(job.message);}
    execute().catch(error=>{job.state='error';job.message=error.message;}).finally(()=>{
      job.finishedAt=new Date().toISOString();
      try{save();}catch(error){job.state='error';job.message='恢复结果记录保存失败：'+error.message;}
    });
    return status();
  }
  return {status,start,busy:()=>job?.state==='running'};
}
module.exports={createMaintenanceManager,resetHardware,recovered};
