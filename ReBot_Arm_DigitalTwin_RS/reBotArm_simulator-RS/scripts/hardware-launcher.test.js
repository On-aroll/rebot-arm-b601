'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');

test('a failed driver reports its current launch error without waiting 30 seconds',async t=>{
  const root=fs.mkdtempSync(path.join(os.tmpdir(),'rebot-launch-')),web=path.join(root,'web');
  fs.mkdirSync(web);fs.mkdirSync(path.join(root,'scripts'));
  const filename=path.join(web,'hardware-launcher.js');
  fs.copyFileSync(path.join(__dirname,'../hardware-launcher.js'),filename);
  fs.writeFileSync(path.join(root,'scripts/start_rs_twin.sh'),
    'echo "RuntimeError: test motor feedback parameter 0x7019 unavailable" >&2\nexit 1\n');
  const previous=process.env.XDG_RUNTIME_DIR;
  process.env.XDG_RUNTIME_DIR=root;
  const launcher=require(filename);
  if(previous===undefined)delete process.env.XDG_RUNTIME_DIR;else process.env.XDG_RUNTIME_DIR=previous;
  t.after(()=>fs.rmSync(root,{recursive:true,force:true}));
  const launched=launcher.connect();
  assert.ok(launched.completion instanceof Promise);
  // The launcher intentionally detaches/unrefs the child; keep the isolated
  // test alive until its real exit event arrives.
  const timer=setTimeout(()=>{},3000);t.after(()=>clearTimeout(timer));
  await assert.rejects(launched.completion,/0x7019 unavailable/);
  assert.equal(launcher.runningPid(),null);
});
