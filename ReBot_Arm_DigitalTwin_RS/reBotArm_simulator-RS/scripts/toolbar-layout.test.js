const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const source=fs.readFileSync(require.resolve('../public/js/operation-hub.js'),'utf8');

function toolbar(saved) {
  const handlers={},store=new Map(saved===undefined?[]:[['rebot-feature-order-v1',saved]]);
  let focused;
  let hover, captured=false;
  const bar={children:[],querySelectorAll:()=>bar.children,
    contains:button=>bar.children.includes(button),setPointerCapture(){captured=true;},hasPointerCapture:()=>captured,releasePointerCapture(){captured=false;},
    append(button){bar.insertBefore(button,null);},
    insertBefore(button,target){bar.children=bar.children.filter(b=>b!==button);const i=bar.children.indexOf(target);bar.children.splice(i<0?bar.children.length:i,0,button);},
    addEventListener:(name,fn)=>handlers[name]=fn};
  const buttons=['programs','tcp','keyboard'].map(key=>({
    dataset:{},attributes:[{name:'data-hub-mode',value:key}],
    append(){},classList:{add(){},remove(){}},focus(){focused=key;},
    closest(){return this;},get nextSibling(){return bar.children[bar.children.indexOf(this)+1]||null;},
    getBoundingClientRect:()=>({top:0,height:40})
  }));
  bar.children=[...buttons];
  const context={$:()=>bar,document:{createElement:()=>({setAttribute(){}}),elementFromPoint:()=>hover},localStorage:{getItem:k=>store.get(k),setItem:(k,v)=>store.set(k,v)}};
  vm.runInNewContext(source.slice(source.indexOf('  const toolbar ='),source.indexOf('  const projectMeta =')),context);
  const fire=(name,target,extra={})=>handlers[name]({target,preventDefault(){},...extra});
  return {buttons,bar,store,fire,setHover:button=>hover=button,focused:()=>focused,order:()=>bar.children.map(b=>b.dataset.toolbarKey)};
}
test('pointer grip drag persists order and suppresses its navigation click',()=>{
  const t=toolbar(),[program,tcp,keyboard]=t.buttons;
  const grip={closest:selector=>selector==='.toolbar-grip'?grip:keyboard};
  t.fire('pointerdown',grip,{button:0,pointerId:7,clientY:80});
  t.setHover(program);t.fire('pointermove',grip,{pointerId:7,clientY:5,clientX:10});
  t.fire('pointerup',grip,{pointerId:7});
  assert.equal(t.order()[0],'data-hub-mode:keyboard');
  assert.deepEqual(toolbar(t.store.get('rebot-feature-order-v1')).order(),t.order());
  let blocked=false;t.fire('click',keyboard,{stopImmediatePropagation(){blocked=true;}});assert.equal(blocked,true);
});
test('drag moves an entry and restores saved order without clicking it',()=>{
  const t=toolbar(),[program,tcp,keyboard]=t.buttons;
  const dataTransfer={setData(){}};
  t.fire('dragstart',keyboard,{dataTransfer});t.fire('dragover',tcp,{dataTransfer});
  t.fire('drop',program,{clientY:5});t.fire('dragend',keyboard);
  assert.deepEqual(t.order(),['data-hub-mode:keyboard','data-hub-mode:programs','data-hub-mode:tcp']);
  assert.deepEqual(toolbar(t.store.get('rebot-feature-order-v1')).order(),t.order());
});
test('Alt arrow reorder persists and keeps focus; malformed/unknown entries are ignored',()=>{
  const t=toolbar();t.fire('keydown',t.buttons[1],{altKey:true,key:'ArrowUp'});
  assert.equal(t.order()[0],'data-hub-mode:tcp');assert.equal(t.focused(),'tcp');
  const order=toolbar(JSON.stringify(['unknown','data-hub-mode:tcp','data-hub-mode:tcp'])).order();
  assert.equal(new Set(order).size,3);assert.equal(order[0],'data-hub-mode:tcp');
  assert.equal(toolbar('{bad').order().length,3);
});

test('each view shows its own panel and reuses the single settings node below it',()=>{
  const panels=Object.fromEntries(['control','programs','teach','controller','system','prepare','maintenance'].map(key=>[key,{hidden:false,append(node){this.last=node;}}]));
  const nodes=Object.fromEntries(['operation-hub','hub-system','hub-project-eyebrow','hub-project-description','hub-project-local','hub-title','toolbar-toggle'].map(id=>[id,{textContent:'old',setAttribute(){},removeAttribute(){}}]));
  const motion={},session={},deck={},forms={},project={},drag=[];
  const context={$:id=>nodes[id],text:(id,value)=>nodes[id].textContent=value,
    document:{body:{dataset:{},classList:{contains:()=>false,remove(){}}},
      querySelector:s=>panels[s.match(/="(.*?)"/)?.[1]],querySelectorAll:s=>s==='[data-workbench-panel]'?Object.values(panels):[]},
    projectWorkspace:project,projectPanels:{},motionSection:motion,sessionPanel:session,twinDeck:deck,workspaceForms:forms,
    titles:{programs:'动作程序'},window:{reBotSim:{setTwinDrag:enabled=>drag.push(enabled)}}};
  vm.createContext(context);
  vm.runInContext(source.slice(source.indexOf('  function selectView('),source.indexOf('  const activeMode =')),context);
  for(const name of ['programs','teach','tcp','controller','system','prepare','maintenance','programs']) {
    context.selectView(name);
    const selected=panels[['tcp'].includes(name)?'control':name];
    assert.equal(Object.values(panels).filter(p=>!p.hidden).length,1);
    assert.equal(selected.hidden,false);
    assert.equal(motion.hidden,['system','prepare','maintenance'].includes(name));
    if(!motion.hidden)assert.equal(selected.last,motion);
    assert.equal(deck.hidden,name!=='tcp');
    assert.equal(drag.at(-1),name==='tcp');
  }
  assert.equal(session.hidden,true);assert.equal(nodes['hub-project-description'].textContent,'');
});

test('service, preparation and maintenance navigation target three distinct panels',()=>{
  const button=dataset=>({dataset,addEventListener(name,fn){this.click=fn;}});
  const system=button({}),prepare=button({hubSystemSection:'hub-expert-start'}),maintenance=button({hubSystemSection:'hub-reset-section'});
  const panels={'hub-expert-start':{dataset:{workbenchPanel:'prepare'}},'hub-reset-section':{dataset:{workbenchPanel:'maintenance'}}};
  const selected=[],scrolled=[];
  const context={document:{querySelector:()=>system,querySelectorAll:()=>[prepare,maintenance]},
    $:id=>panels[id],selectView:(name,entry)=>selected.push([name,entry]),scroll:id=>scrolled.push(id)};
  vm.runInNewContext(source.slice(source.indexOf("  document.querySelector('[data-hub-system]')"),source.indexOf("  window.addEventListener('rebot-workbench-changed'")),context);
  system.click();prepare.click();maintenance.click();
  assert.deepEqual(selected,[['system',system],['prepare',prepare],['maintenance',maintenance]]);
  assert.deepEqual(scrolled,['hub-title','hub-title','hub-title']);
});

test('TCP marker reconciles a local animation stop and preserves legacy drag',()=>{
  const sim=fs.readFileSync(require.resolve('../public/js/rebot-sim.js'),'utf8');
  const code=sim.slice(sim.indexOf('    setTwinDrag(enabled) {'),sim.indexOf('    attachObject(color)',sim.indexOf('    setTwinDrag(enabled) {'))).replace('setTwinDrag(enabled) {','function setTwinDrag(enabled) {').replace(/},\s*$/,'}');
  const ctx={twinDrag:true,dragMode:false,draggingTcp:false,toggleDragMode(){ctx.dragMode=!ctx.dragMode;},updateDragMarker(){},endTcpDrag(){throw Error('unexpected drag interruption');}};
  vm.createContext(ctx);vm.runInContext(code,ctx);ctx.setTwinDrag(true);assert.equal(ctx.dragMode,true);
  ctx.draggingTcp=true;ctx.setTwinDrag(true);assert.equal(ctx.dragMode,true);
  ctx.draggingTcp=false;ctx.setTwinDrag(false);assert.equal(ctx.dragMode,false);
  ctx.dragMode=true;ctx.setTwinDrag(false);assert.equal(ctx.dragMode,true);
});
