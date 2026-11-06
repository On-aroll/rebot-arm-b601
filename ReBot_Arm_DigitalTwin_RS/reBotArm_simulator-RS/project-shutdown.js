'use strict';
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto');
const ROOT=path.resolve(__dirname,'..');
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const ORDER=['hardware','text','mcp','camera','simulation','bridge'];

// Scope by project path, uid and an exact entry point, never by a broad pkill.
function classify(argv,cwd,root) {
  if(typeof cwd!=='string')return null;
  if(cwd!==root&&!cwd.startsWith(root+'/'))return null;
  const names=argv.map(a=>path.basename(a));
  if(argv.includes('rebotarmcontroller.csp_twin_driver'))return 'hardware';
  if(names.includes('orbbec_ros_camera_bridge.py')||names.includes('orbbec_color_detector.py'))return 'camera';
  if(argv.includes('rebotarm_agent.rebotarm_text_agent')||names.includes('start_rs_text_agent.sh'))return 'text';
  if(names.includes('rebotarm_mcp_server')||names.includes('rebotarm_mcp.launch.py'))return 'mcp';
  if(names.some(n=>['start_rs_sim.sh','FakeRsDriver','mujoco_rs_sync','fake_rs_bringup.launch.py','mujoco_rs.launch.py','robot_state_publisher'].includes(n)))return 'simulation';
  if(names.some(n=>['rosbridge_websocket','rosapi_node','rosbridge_websocket_launch.xml'].includes(n)))return 'bridge';
  return null;
}
function readProcess(pid) {
  try {
    const dir='/proc/'+pid,uid=fs.statSync(dir).uid;
    if(uid!==process.getuid())return null;
    const stat=fs.readFileSync(dir+'/stat','utf8');
    const fields=stat.slice(stat.lastIndexOf(')')+2).split(' ');
    if(fields[0]==='Z')return null;
    const argv=fs.readFileSync(dir+'/cmdline','utf8').split('\0').filter(Boolean);
    let cwd;
    try{cwd=fs.realpathSync(dir+'/cwd');}
    catch(error){
      // Sandboxed browser/system processes may hide cwd even from their uid.
      // A recognizable project entry point must remain inspectable to exit.
      if(error.code!=='EACCES'||classify(argv,ROOT,ROOT))throw error;
      cwd=null;
    }
    return {pid:Number(pid),ppid:Number(fields[1]),birth:fields[19],
      uid,cwd,argv};
  }catch(error){if(['ENOENT','ESRCH'].includes(error.code))return null;throw error;}
}
function inventory(root=ROOT) {
  const all=fs.readdirSync('/proc').filter(p=>/^\d+$/.test(p)).map(readProcess).filter(p=>p&&p.uid===process.getuid()&&p.pid!==process.pid);
  const selected=new Map();
  for(const p of all){const kind=classify(p.argv,p.cwd,root);if(kind)selected.set(p.pid,{...p,kind});}
  // Include children in other working directories while their ancestry is known.
  let changed=true;
  while(changed){changed=false;for(const p of all)if(!selected.has(p.pid)&&selected.has(p.ppid)){
    selected.set(p.pid,{...p,kind:selected.get(p.ppid).kind});changed=true;
  }}
  return [...selected.values()];
}
function sameProcess(row) {const now=readProcess(row.pid);return Boolean(now&&now.uid===row.uid&&now.birth===row.birth);}
async function stopProcesses(rows,{signal=(pid,s)=>process.kill(pid,s),alive=sameProcess,wait=sleep,timeout=10000}={}) {
  // No negative pid/group kill: a process group may contain an unrelated service.
  for(const p of rows)if(alive(p))signal(p.pid,'SIGTERM');
  for(let elapsed=0;elapsed<timeout;elapsed+=100){if(rows.every(p=>!alive(p)))return;await wait(100);}
  const remaining=rows.filter(alive);
  if(remaining.some(p=>p.kind==='hardware'))throw new Error('实机驱动尚未正常退出；保留联动接口，不强制结束。');
  for(const p of remaining)if(alive(p))signal(p.pid,'SIGKILL');
  await wait(100);
  if(rows.some(alive))throw new Error('部分后台进程未退出，请查看退出进度。');
}
function sealHardware() {
  // The confirmed exit revokes control and waits for drive disable replies.
  return new Promise((resolve,reject)=>{
    const socket=new WebSocket('ws://127.0.0.1:9090');
    const timer=setTimeout(()=>finish(new Error('未收到电机失能确认，实机驱动仍保留。')),15000);
    function finish(error,result){clearTimeout(timer);socket.close();error?reject(error):resolve(result);}
    socket.onopen=()=>socket.send(JSON.stringify({op:'call_service',id:'project-supported-exit',service:'/rebotarm/twin/shutdown_stop',type:'std_srvs/srv/Trigger',args:{}}));
    socket.onmessage=event=>{try{
      const m=JSON.parse(event.data);
      if(m.op==='service_response'&&m.id==='project-supported-exit'){
        if(!m.result||!m.values?.success)finish(new Error(m.values?.message||'实机未确认失能，后台保留。'));
        else {
          const result=JSON.parse(m.values.message);
          if(!result.ok || ![1,2,3,4,5,6].every(axis=>result.states?.[axis]?.mode_state===0))
            finish(new Error('六轴失能回包不完整，后台保留。'));
          else finish(null,result);
        }
      }
    }catch(error){finish(error);}};
    socket.onerror=()=>finish(new Error('无法确认实机状态；后台服务保持运行。'));
  });
}
function canStoppedWithoutReceivers(status) {
  const can=status.services.can;
  if(!can?.present || can.up!==false || can.state!=='STOPPED' ||
      !status.feedback.hardware?.fresh || status.feedback.hardware.phase!=='FAULT')return false;
  try {
    return ['all','fil','inv','sff','eff','err'].every(type=>
      !fs.readFileSync('/proc/net/can/rcvlist_'+type,'utf8').split('\n').some(line=>{
        const fields=line.trim().split(/\s+/);
        return fields.length>=5 && ['any',can.interface].includes(fields[0]);
      }));
  }catch(_){return false;}
}
function createShutdownManager(options={}) {
  const scan=options.inventory||inventory,stop=options.stopProcesses||stopProcesses;
  const seal=options.sealHardware||sealHardware,starting=options.starting||(()=>false);
  const offline=options.offlineCanOnly||(()=>false);
  const root=options.root||ROOT;
  let job=null;
  const publicJob=()=>job?JSON.parse(JSON.stringify(job)):{state:'idle',steps:[]};
  function status(){return {ok:true,job:publicJob()};}
  function blocked(){return Boolean(job&&job.state!=='error');}
  function start(confirmed,powerOff=false) {
    if(confirmed!==true)throw new Error('请确认机械臂已有可靠支撑，允许失能撤力。');
    if(starting())throw new Error('启动步骤仍在进行，请等待完成后结束项目。');
    if(job&&job.state!=='error')return status();
    job={id:crypto.randomUUID(),state:'running',message:'正在检查后台与实机退出条件…',steps:[]};
    const current=job;
    (async()=>{
      let rows;
      for(const kind of ORDER){
        current.message=({hardware:'优先撤销实机控制并核对失能',text:'结束文本代理',mcp:'结束MCP接口',camera:'结束相机采集服务',simulation:'结束仿真与模型进程',bridge:'结束ROS联动接口'})[kind];
        // Refresh the inventory so children created during teardown are included.
        rows=scan();const targets=rows.filter(p=>p.kind===kind);
        if(kind==='hardware'&&targets.length){
          if(targets.filter(p=>p.argv?.includes('rebotarmcontroller.csp_twin_driver')).length>1)
            throw new Error('检测到多份实机驱动，无法确认唯一失能执行器。');
          current.disable=powerOff===true && await offline()
            ? {source:'现场已确认本体电机电源断开',canAcknowledged:false}
            : await seal();
          const owners=scan().filter(p=>p.kind==='hardware');
          if(owners.some(p=>!targets.some(t=>p.pid===t.pid&&p.birth===t.birth)))
            throw new Error('实机驱动身份变化，保留后台。');
        }
        await stop(targets);
        if(scan().some(p=>p.kind===kind))throw new Error(current.message+'未完成；其余服务保留。');
        current.steps.push({kind,state:'done',count:targets.length});
      }
      if(scan().length)throw new Error('检测到新启动的项目进程，尚未结束网页服务，请重试。');
      current.state='ready';current.message='后台服务已停止，正在准备退出网页服务。';
    })().catch(error=>{current.state='error';current.message=error.message;})
      .finally(()=>{
        try{
          const dir=path.join(root,'outputs/startup');fs.mkdirSync(dir,{recursive:true});
          fs.writeFileSync(path.join(dir,'last-shutdown.json'),JSON.stringify(publicJob(),null,2));
        }catch(error){current.state='error';current.message='退出记录保存失败：'+error.message;}
      });
    return status();
  }
  function finish(id){
    if(!job||job.id!==id||job.state!=='ready')throw new Error('后台尚未全部结束，网页服务保留。');
    if(scan().length)throw new Error('项目进程已重新出现，网页服务保留。');
    job.state='exiting';
    return {ok:true,job:publicJob()};
  }
  return {status,start,finish,blocked};
}
module.exports={createShutdownManager,inventory,classify,readProcess,sameProcess,stopProcesses,canStoppedWithoutReceivers};
