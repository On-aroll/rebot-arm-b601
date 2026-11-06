'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict');
const {projector,create,build}=require('../public/js/writing-plane');
const glyphs=require('../public/programs/writing-strokes.json').templates;
const settings=(axis='+Z',extra={})=>({axis,tilt_deg:[0,0,0],spin_deg:0,retract_sign:1,offset_m:[0,0,0],...extra});
const fixed={xyz:[0,0,0],rpy:[0,0,0]};
const dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0);
const near=(a,b)=>a.every((v,i)=>Math.abs(v-b[i])<1e-9);
test('side-facing gripper plane displays full character height instead of flattening its Z component',()=>{
  const p=projector({kind:'tool_normal',frame_id:'base_link',origin:[.5,.1,.08],u:[0,-1,0],v:[0,0,1],normal:[-1,0,0]});
  const result=p([.5,.01,.1]);
  assert.ok(Math.abs(result[0]-.09)<1e-12);assert.ok(Math.abs(result[1]-.02)<1e-12);
  assert.deepEqual(projector()([1,2,3]),[1,2]);
});
test('all six signed wrist axes have an unambiguous right-handed writing frame',()=>{
  for(const axis of ['+X','-X','+Y','-Y','+Z','-Z']){
    const p=create([1,2,3,0,0,0],fixed,settings(axis));
    const n=[0,0,0];n['XYZ'.indexOf(axis[1])]=axis[0]==='+'?1:-1;
    assert.ok(near(p.normal,n));assert.ok(near(p.retract_direction,n));
    assert.ok(Math.abs(dot(p.u,p.v))+Math.abs(dot(p.u,p.normal))+Math.abs(dot(p.v,p.normal))<1e-12);
    assert.ok(near([p.u[1]*p.v[2]-p.u[2]*p.v[1],p.u[2]*p.v[0]-p.u[0]*p.v[2],p.u[0]*p.v[1]-p.u[1]*p.v[0]],p.normal));
  }
});
test('positive rotation follows fixed wrist right-hand axes; lift sign never changes text orientation',()=>{
  const p=create([0,0,0,0,0,0],fixed,settings('+Y',{tilt_deg:[90,0,0]}));
  assert.ok(near(p.normal,[0,0,1]));
  const forward=create([0,0,0,0,0,0],fixed,settings('+Z'));
  const reverse=create([0,0,0,0,0,0],fixed,settings('+Z',{retract_sign:-1}));
  assert.deepEqual(reverse.u,forward.u);assert.deepEqual(reverse.v,forward.v);
  assert.ok(near(reverse.retract_direction,[0,0,-1]));
  const spun=create([0,0,0,0,0,0],fixed,settings('+Z',{spin_deg:90}));
  assert.ok(near(spun.u,[0,1,0]));assert.ok(near(spun.v,[-1,0,0]));
});
test('wrist reference is derived from the real fixed TCP transform, rather than guessing TCP axes',()=>{
  const transform={xyz:[0,0,.16621],rpy:[Math.PI,-Math.PI/2,0]};
  // R_base_tcp equals the fixed transform, so the wrist axes are base XYZ.
  const p=create([0,0,.16621,...transform.rpy],transform,settings('+Z',{offset_m:[.01,.02,.03]}));
  assert.ok(near(p.normal,[0,0,1]));assert.ok(near(p.wrist_origin,[0,0,0]));
  assert.ok(near(p.origin,[.01,.02,.19621]));
});
test('both templates freeze direction, stay in their plane and retract along the chosen sign',()=>{
  for(const text of ['BIT103','国庆快乐']){
    const input=settings('-X',{tilt_deg:[15,-20,30],retract_sign:-1}),p=create([.3,.1,.2,.2,-.4,.7],fixed,input);
    const program=build(glyphs[text],p,{text,width:.09,height:.02,lift:.005});
    assert.ok(program.steps.length<=250);assert.equal(program.writing_plane.preview_only,true);
    input.tilt_deg[0]=90;assert.equal(p.configuration.tilt_deg[0],15);
    const uv=projector(p),positions=[];
    for(const step of program.steps){
      const point=step.pose.slice(0,3),distance=dot(point.map((x,i)=>x-p.origin[i]),p.normal);
      if(step.label.endsWith('书写')||step.label.endsWith('落笔')){assert.ok(Math.abs(distance)<1e-10);positions.push(uv(point));}
      else assert.ok(Math.abs(distance+.005)<1e-10);
    }
    if(text==='BIT103'){
      const xs=positions.map(point=>point[0]);assert.ok(Math.max(...xs)-Math.min(...xs)<=.1+1e-12);
      assert.throws(()=>build(glyphs[text],p,{text,width:.101}),/100mm/);
    }
  }
});
test('invalid axis, transform, offsets and rotations cannot generate a misleading frame',()=>{
  for(const value of [settings('X'),settings('+Q'),settings('+X',{tilt_deg:[NaN,0,0]}),settings('+X',{offset_m:[1,0,0]}),settings('+Z',{retract_sign:0})])
    assert.throws(()=>create([0,0,0,0,0,0],fixed,value),/法线轴/);
});
test('invalid or nonorthogonal frame cannot make a misleading writing preview',()=>{
  assert.throws(()=>projector({kind:'tool_normal',frame_id:'base_link',origin:[0,0,0],u:[1,0,0],v:[1,0,0],normal:[0,0,1]}),/书写平面/);
});
