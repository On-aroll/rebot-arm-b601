'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const tick=()=>new Promise(resolve=>setImmediate(resolve));
function page(snapshot) {
  const elements=new Map(),calls=[],local=new Map(),session=new Map();
  function el(id){if(!elements.has(id))elements.set(id,{id,dataset:{},textContent:'',disabled:false,handlers:{},attributes:{},
    addEventListener(type,fn){this.handlers[type]=fn;},setAttribute(k,v){this.attributes[k]=v;},getAttribute(k){return this.attributes[k];},
    async click(){let prevented=false;await this.handlers.click?.({preventDefault(){prevented=true;}});return prevented;}});return elements.get(id);}
  const buttons=['ros','bridge','can','hardware','camera','mcp','text'].map(key=>{
    const button=el(key);button.dataset.start=key;const article=el('article-'+key);article.dataset.service=key;
    article.querySelector=selector=>el(key+selector);button.closest=()=>article;return button;
  });
  const document={hidden:false,getElementById:el,querySelectorAll:selector=>selector==='[data-start]'?buttons:[]};
  const context={document,AbortSignal,localStorage:{setItem:(k,v)=>local.set(k,v)},sessionStorage:{setItem:(k,v)=>session.set(k,v),removeItem:k=>session.delete(k)},
    setInterval(){},async fetch(url,options){calls.push([url,options.method]);return {ok:true,json:async()=>url.endsWith('/status')?snapshot:{ok:true,job:{message:'已开始'}}};}};
  vm.runInNewContext(fs.readFileSync(require.resolve('../public/js/startup.js'),'utf8'),context);
  return {el,calls,local,session};
}
function snapshot() {
  return {ok:true,root:'/project',services:Object.fromEntries(['ros','bridge','can','hardware','camera','simulation','mcp','text'].map(key=>[key,{ready:true,message:'已就绪'}])),bundle:null};
}
test('ordinary page load only reads; one click launches all and startup entry records a readonly connection',async()=>{
  const p=page(snapshot());await tick();assert.deepEqual(p.calls,[['/api/startup/status','GET']]);
  assert.equal(p.el('startup-all').disabled,false);await p.el('startup-all').click();
  assert.deepEqual(p.calls.filter(c=>c[1]==='POST'),[['/api/startup/start/all','POST']]);
  assert.equal(await p.el('startup-enter').click(),false);
  assert.equal(p.session.get('rebotarm.startup.connect'),'hardware');
});
test('live recovery lock is shown even when the feedback process is ready; running bundle cannot be double started',async()=>{
  const s=snapshot();s.services.hardware.needsRecovery=true;s.bundle={state:'running',message:'正在检查相机'};
  const p=page(s);await tick();assert.equal(p.el('startup-all').disabled,true);
  assert.equal(p.el('hardware.badge').textContent,'需恢复');assert.equal(p.el('startup-enter').getAttribute('href'),'/#hub-reset-section');
  assert.equal(p.el('startup-message').textContent,'正在检查相机');assert.equal(p.calls.filter(c=>c[1]==='POST').length,0);
});
test('a fault with missing feedback opens maintenance directly without starting a connection or restore',async()=>{
  const s=snapshot();
  s.services.hardware={ready:false,running:true,needsRecovery:true,message:'实机反馈未确认 · FAULT',
    job:{state:'error',startedAt:'2026-11-06T06:03:24Z',message:'服务进程已存在但尚未就绪'}};
  s.feedback={hardware:{fresh:true,valid:false,message:'0x7019 读取超时'}};
  const p=page(s);await tick();
  assert.equal(p.el('startup-recovery').hidden,false);
  assert.equal(p.el('hardware').disabled,true);
  assert.match(p.el('hardware.service-message').textContent,/0x7019/);
  assert.equal(p.el('startup-enter').getAttribute('href'),'/#hub-reset-section');
  assert.equal(p.el('startup-enter').getAttribute('aria-disabled'),'false');
  p.session.set('rebotarm.startup.connect','hardware');
  assert.equal(await p.el('startup-enter').click(),false);
  assert.equal(p.local.get('rebotarm.rs.ros.target'),'hardware');
  assert.equal(p.session.has('rebotarm.startup.connect'),false);
  await p.el('startup-recovery').click();
  assert.deepEqual(p.calls,[['/api/startup/status','GET']]);
});
