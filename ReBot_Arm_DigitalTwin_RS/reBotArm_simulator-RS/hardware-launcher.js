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
  const log = fs.openSync(path.join(directory,'csp-driver.log'),'a');
  const child = spawn('/bin/bash',[script],{cwd:root,detached:true,stdio:['ignore',log,log]});
  child.on('error',()=>{});
  child.unref(); fs.closeSync(log);
  if (!child.pid) throw new Error('无法启动实机反馈服务');
  fs.writeFileSync(pidFile,String(child.pid),{mode:0o600});
  return {ok:true,reused:false,pid:child.pid,message:'正在启动 CSP 只读反馈；连接不会使能电机'};
}
function disconnect() {
  const pid = runningPid();
  if (pid) process.kill(pid,'SIGTERM');
  return {ok:true,stopping:Boolean(pid),message:'已请求停止控制并释放实机反馈服务'};
}
module.exports={connect,disconnect,runningPid};
