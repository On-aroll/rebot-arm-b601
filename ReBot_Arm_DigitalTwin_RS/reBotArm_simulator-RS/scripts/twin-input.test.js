const { test } = require('node:test');
const assert = require('node:assert/strict');
const { keyboard,gamepad,twist,deadzone,response } = require('../public/js/twin-input.js');
const pad = () => ({ mapping:'standard', axes:[0,0,0,0], buttons:Array.from({length:17},() => ({value:0})) });
test('quick speed scales tool/gripper requests while preserving hard joint cap',()=>{
  const {manualSpeeds}=require('../public/js/twin-input.js');
  const base={joint:.1,linear:25,angular:10,gripper:.5},limits={minimum:.005,joint:1,linear:40,angular:20,gripper:.8};
  assert.deepEqual(manualSpeeds(base,limits,200),{joint:.1,linear:40,angular:20,gripper:.8});
  assert.deepEqual(manualSpeeds(base,limits,100,true),{joint:.1,linear:6.25,angular:2.5,gripper:.125});
  assert.equal(manualSpeeds(base,limits,10000).gripper,.8);
  assert.equal(manualSpeeds(base,limits,-100).linear,2.5);
  assert.equal(manualSpeeds(base,limits,NaN).linear,25);
});
test('X/Y speed edges do not repeat while held, arm on connect, or bypass mapping',()=>{
  const {SpeedShortcut}=require('../public/js/twin-input.js');const s=new SpeedShortcut(),p=pad();
  p.buttons[3].value=1;assert.equal(s.sample(p,false),0);assert.equal(s.sample(p,true),0);
  p.buttons[3].value=0;s.sample(p,true);p.buttons[3].value=1;
  assert.equal(s.sample(p,true),1);assert.equal(s.sample(p,true),0);
  p.buttons[2].value=1;assert.equal(s.sample(p,true),0);
  p.buttons[3].value=0;assert.equal(s.sample(p,true),-1);
  p.mapping='';s.reset();assert.equal(s.sample(p,true),0);
});
test('keyboard requires Space, opposite keys cancel',() => {
  assert.deepEqual(keyboard(new Set(['KeyW'])),[0,0,0,0,0,0]);
  assert.deepEqual(keyboard(new Set(['Space','KeyW','KeyS','KeyA','KeyU'])),[0,1,0,1,0,0]);
});
test('gamepad never moves without RB and refuses unknown mapping',() => {
  const p = pad(); p.axes[1] = -1;
  assert.deepEqual(gamepad(p).values,[0,0,0,0,0,0]);
  p.buttons[5].value = 1;
  assert.equal(gamepad(p).values[0],1);
  p.mapping = ''; assert.equal(gamepad(p).enabled,false);
});
test('standard mapping controls all six axes and supports precision speed',() => {
  const p = pad(); p.buttons[5].value=1; p.axes=[-1,-1,-1,-1]; p.buttons[12].value=1; p.buttons[7].value=1;
  assert.deepEqual(gamepad(p).values,[1,1,1,1,1,1]);
  p.buttons[4].value=1; assert.deepEqual(gamepad(p).values,[.25,.25,.25,.25,.25,.25]);
});
test('neutral guard, deadzone and missing axes do not produce motion',() => {
  assert.equal(gamepad(pad()).neutral,true);
  assert.equal(deadzone(.15),0); assert.equal(deadzone(NaN),0); assert.equal(deadzone(undefined),0);
});
test('diagonal input obeys vector speed caps',() => {
  const result=twist([1,1,1,1,1,1],.04,.35);
  assert.ok(Math.hypot(...result.slice(0,3)) <= .040001);
  assert.ok(Math.hypot(...result.slice(3)) <= .350001);
});
test('soft response is symmetric, monotone, fine near center and full at the edge',()=>{
  let last=0;
  for(let i=0;i<=100;i++) {
    const x=i/100,y=response(x);
    assert.ok(y>=last && y<=1);assert.ok(Math.abs(response(-x)+y)<1e-12);last=y;
  }
  assert.equal(response(.16),0);assert.equal(response(1),1);
  assert.ok(response(.58) < .251 && response(.58) > .249);
  assert.equal(response(NaN),0);assert.equal(response(undefined),0);
});
test('J7 shares Space/RB deadman and neutral guard, opposite controls cancel',()=>{
  const {keyboardGripper}=require('../public/js/twin-input.js');
  assert.equal(keyboardGripper(new Set(['KeyQ'])),0);
  assert.equal(keyboardGripper(new Set(['Space','KeyQ'])),1);
  assert.equal(keyboardGripper(new Set(['Space','KeyE'])),-1);
  assert.equal(keyboardGripper(new Set(['Space','KeyQ','KeyE'])),0);
  const p=pad();p.buttons[0].value=1;
  assert.equal(gamepad(p).gripper,0);assert.equal(gamepad(p).neutral,false);
  p.buttons[5].value=1;assert.equal(gamepad(p).gripper,1);
  p.buttons[4].value=1;assert.equal(gamepad(p).gripper,.25);
  p.buttons[1].value=1;assert.equal(gamepad(p).gripper,0);
});
test('speed presets use current device cap including lower device limits',()=>{
  const {speedPreset}=require('../public/js/twin-input.js');
  for(const cap of [.02,.37,1]){
    let previous=0;
    for(const name of ['slow','normal','fast','max']) {
      const s=speedPreset(name,cap,.005,40,20);
      assert.ok(s.joint>=previous&&s.joint<=cap&&s.linear<=40&&s.angular<=20);previous=s.joint;
    }
    assert.equal(speedPreset('max',cap).joint,cap);
  }
});
