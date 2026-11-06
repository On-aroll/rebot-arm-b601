'use strict';
const fs = require('node:fs');
const path = require('node:path');
const { spawn } = require('node:child_process');

const root = path.resolve(__dirname, '..');
const pidFile = path.join(process.env.XDG_RUNTIME_DIR || '/tmp', `rebotarm-csp-twin-${process.getuid()}.pid`);
const script = path.join(root,'scripts/start_rs_twin.sh');
function runningPid() {
  try {
    const pid = Number(fs.readFileSync(pidFile,'utf8'));
    if (!Number.isSafeInteger(pid) || pid <= 1) return null;
    const command = fs.readFileSync(`/proc/${pid}/cmdline`,'utf8');
    const cwd = fs.realpathSync(`/proc/${pid}/cwd`);
    return cwd === root && (command.includes('rebotarmcontroller.csp_twin_driver') || command.includes(script)) ? pid : null;
  } catch (_) { return null; }
}
function connect() {
  const pid = runningPid();
  if (pid) return { ok:true, reused:true, pid };
  const directory = path.join(root,'outputs/twin-console');
  fs.mkdirSync(directory,{recursive:true});
  const logfile = path.join(directory,'csp-driver.log');
  const log = fs.openSync(logfile,'a'), logStart = fs.fstatSync(log).size;
  const child = spawn('/bin/bash',[script],{cwd:root,detached:true,stdio:['ignore',log,log]});
  const completion = new Promise((resolve,reject)=>{
    child.once('error',reject);
    child.once('exit',(code,signal)=>{
      if(code===0){resolve();return;}
      let detail='';
      try {
        const fd=fs.openSync(logfile,'r');
        try {
          const size=fs.fstatSync(fd).size,start=Math.max(logStart,size-2000);
          const buffer=Buffer.alloc(Math.max(0,size-start));
          fs.readSync(fd,buffer,0,buffer.length,start);
          detail=buffer.toString('utf8').trim().split('\n').slice(-2).join(' ');
        } finally {fs.closeSync(fd);}
      } catch (_) {}
      reject(new Error(`实机反馈服务已退出（${signal||code}）：${detail||'请查看实机反馈日志'}`));
    });
  });
  completion.catch(()=>{});
  child.unref(); fs.closeSync(log);
  if (!child.pid) throw new Error('无法启动实机反馈服务');
  fs.writeFileSync(pidFile,String(child.pid),{mode:0o600});
  return {ok:true,reused:false,pid:child.pid,completion,message:'正在启动 CSP 只读反馈；连接不会使能电机'};
}
function disconnect() {
  const pid = runningPid();
  if (pid) process.kill(pid,'SIGTERM');
  return {ok:true,stopping:Boolean(pid),message:'已请求停止控制并释放实机反馈服务'};
}
module.exports={connect,disconnect,runningPid};
