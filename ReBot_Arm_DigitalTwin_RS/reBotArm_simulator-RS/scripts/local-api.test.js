const {test}=require('node:test'),assert=require('node:assert/strict');
const http=require('node:http'),fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {spawn}=require('node:child_process'),{once}=require('node:events');
const {allowed}=require('../local-api');
const request=(headers={},method='GET',address='127.0.0.1')=>({method,socket:{remoteAddress:address},headers:{host:'127.0.0.1:3002',...headers}});

test('API rejects remote peers, rebinding hosts and cross-origin browsers; local CLI reads remain available',()=>{
  assert.equal(allowed(request()),true);
  assert.equal(allowed(request({},'GET','192.168.1.5')),false);
  assert.equal(allowed(request({host:'attacker.example:3002'})),false);
  assert.equal(allowed(request({origin:'http://attacker.example'})),false);
  assert.equal(allowed(request({'sec-fetch-site':'cross-site'})),false);
  assert.equal(allowed(request({'sec-fetch-site':'same-site'})),false);
  assert.equal(allowed(request({origin:'http://127.0.0.1:3002'},'POST')),true);
  assert.equal(allowed(request({},'POST')),false);
  assert.equal(allowed(request({origin:'null'},'POST')),false);
  assert.equal(allowed(request({origin:'https://127.0.0.1:3002'},'POST'),true),true);
  assert.equal(allowed(request({origin:'http://127.0.0.1:3002'},'POST'),true),false);
  assert.equal(allowed(request({host:'[::1]:3002'},'GET','::1')),true);
});

function get(port,url,headers={},method='GET') {
  return new Promise((resolve,reject)=>{
    const req=http.request({hostname:'127.0.0.1',port,path:url,method,headers},res=>{
      let body='';res.on('data',chunk=>body+=chunk);res.on('end',()=>resolve({status:res.statusCode,body}));
    });req.on('error',reject);req.end();
  });
}

test('isolated actual server gates config, quick-start and LLM before dispatch, and rejects static path escapes',async t=>{
  // Inert replacement managers: a dispatched control request fails this test.
  const root=fs.mkdtempSync(path.join(os.tmpdir(),'rebot-api-')),web=path.join(root,'web');fs.mkdirSync(web);
  t.after(()=>fs.rmSync(root,{recursive:true,force:true}));
  for(const name of ['server.js','local-api.js'])fs.copyFileSync(path.join(__dirname,'..',name),path.join(web,name));
  fs.writeFileSync(path.join(web,'hardware-launcher.js'),'module.exports={};');
  fs.writeFileSync(path.join(web,'startup-manager.js'),'exports.createStartupManager=()=>({busy:()=>false});');
  fs.writeFileSync(path.join(web,'workbench-api.js'),'exports.createWorkbench=()=>({handle:()=>{throw Error("dispatch forbidden")}});');
  fs.writeFileSync(path.join(web,'project-shutdown.js'),'exports.createShutdownManager=()=>({blocked:()=>false});');
  fs.mkdirSync(path.join(web,'public'));fs.writeFileSync(path.join(web,'public/index.html'),'local workbench');
  fs.mkdirSync(path.join(web,'public-private'));fs.writeFileSync(path.join(web,'public-private/secret'),'hidden');
  const child=spawn(process.execPath,['server.js'],{cwd:web,env:{...process.env,HTTPS:'0',PORT:'0',MOTORBRIDGE_WS_TOKEN:'test-secret'},stdio:['ignore','pipe','pipe']});
  t.after(async()=>{if(child.exitCode===null){child.kill('SIGTERM');await once(child,'exit');}});
  const port=await new Promise((resolve,reject)=>{
    let output='';const timer=setTimeout(()=>reject(Error('HTTP start timed out')),5000);
    child.stdout.on('data',chunk=>{output+=chunk;const match=output.match(/Local: http:\/\/127\.0\.0\.1:(\d+)/);if(match){clearTimeout(timer);resolve(Number(match[1]));}});
    child.once('exit',code=>{clearTimeout(timer);reject(Error(`early exit ${code}`));});
  });
  for(const url of ['/api/config','/api/mcp/config','/api/urdf']){
    const denied=await get(port,url,{origin:'https://attacker.example'});assert.equal(denied.status,403);assert.ok(!denied.body.includes('test-secret'));
    assert.equal((await get(port,url,{host:'attacker.example'})).status,403);
  }
  for(const url of ['/api/quick-start','/api/llm/chat','/api/hardware/connect','/api/startup/start/hardware','/api/workbench/profiles']){
    assert.equal((await get(port,url,{},'POST')).status,403);
    assert.equal((await get(port,url,{origin:'null'},'POST')).status,403);
  }
  const config=await get(port,'/api/config');assert.equal(config.status,200);assert.equal(JSON.parse(config.body).motorbridge.token,'test-secret');
  assert.equal((await get(port,'/')).body,'local workbench');
  assert.equal((await get(port,'/../public-private/secret')).status,403);
  assert.equal((await get(port,'/%E0%A4%A')).status,403);
  // The listener itself must be loopback-only, not merely protected at HTTP level.
  const tcp=fs.readFileSync('/proc/net/tcp','utf8').split('\n').find(line=>line.split(/\s+/)[2]===`0100007F:${port.toString(16).toUpperCase().padStart(4,'0')}`);
  assert.ok(tcp,'expected a 127.0.0.1 listening socket');
});
