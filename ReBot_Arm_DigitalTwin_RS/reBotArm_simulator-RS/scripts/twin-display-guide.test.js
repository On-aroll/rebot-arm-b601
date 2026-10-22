const {test}=require('node:test');
const assert=require('node:assert/strict');
const Display=require('../public/js/twin-feedback-display.js');
const {padIndicators,showPad}=require('../public/js/twin-guides.js');
const {gamepad}=require('../public/js/twin-input.js');

test('repeated 50Hz status does not restart a 100ms measured-position blend',()=>{
  const d=new Display();d.push({joint1:0},0,100);d.push({joint1:1},100,100);
  for(let t=120;t<200;t+=20){d.push({joint1:1},t,100);assert.ok(Math.abs(d.sample(t).joint1-(t-100)/100)<1e-12);}
  assert.equal(d.sample(200).joint1,1);
});
test('100ms encoder ramp displays continuously between updates and never extrapolates',()=>{
  const d=new Display(); d.push({joint1:0},0,100);
  for(let t=100;t<=600;t+=100){
    d.push({joint1:t/100},t,100);
    if(t>100)for(let dt=10;dt<100;dt+=10)assert.ok(Math.abs(d.sample(t+dt).joint1-((t-100+dt)/100))<1e-12);
  }
  assert.equal(d.sample(900).joint1,6);
});
test('per-axis timing is independent, raw values remain accessible, no overshoot on reversal',()=>{
  const d=new Display();d.push({joint1:0,joint2:2},0,100);d.push({joint1:1,joint2:2},100,100);
  d.push({joint1:1,joint2:1},150,100);
  assert.equal(d.sample(175).joint1,.75);assert.equal(d.sample(175).joint2,1.75);
  assert.deepEqual(d.sample(175,true),{joint1:1,joint2:1});
  d.push({joint1:-1},180,100);
  for(let t=180;t<450;t+=10)assert.ok(d.sample(t).joint1>=-1&&d.sample(t).joint1<=.8);
});
test('stale or reset display cannot continue playback; reconnect does not blend from old pose',()=>{
  const d=new Display();d.push({joint1:1},0);assert.equal(d.sample(500),null);
  d.push({joint1:3},500,100);assert.equal(d.sample(500).joint1,3);
  d.reset();assert.equal(d.sample(501),null);
});
test('220ms ACTIVE polling keeps moving between replies instead of pausing at 150ms',()=>{
  const d=new Display();d.push({joint1:0},0,240);d.push({joint1:1},220,240);
  const late=d.sample(420).joint1;
  assert.ok(late>d.sample(400).joint1&&late<1);
  d.push({joint1:2},440,240);
  assert.ok(d.sample(460).joint1>d.sample(440).joint1);
  assert.equal(d.sample(460,true).joint1,2);
});
test('display delay is bounded to 300ms even with slow feedback and invalid numbers are ignored',()=>{
  const d=new Display();d.push({joint1:0},0);d.push({joint1:1,joint2:NaN},50,1000);
  assert.equal(d.sample(350).joint1,1);assert.equal(d.sample(350).joint2,undefined);
});
test('GameSir branding never bypasses non-standard mapping restriction',()=>{
  const p={id:'GameSir Nova 2 Lite',mapping:'',axes:[-1,-1,-1,-1],buttons:Array.from({length:17},()=>({value:1}))};
  assert.deepEqual(padIndicators(p).buttons,[]);assert.equal(gamepad(p).enabled,false);
  p.mapping='standard'; assert.ok(padIndicators(p).buttons.every(Boolean));assert.equal(gamepad(p).enabled,true);
});
test('controller guide highlights standard inputs without publishing robot commands',()=>{
  const nodes=[0,1,4,5,6,7,12,13].map(i=>({dataset:{padButton:String(i)},classList:{toggle(_name,v){this.active=v;}}}));
  const stick={dataset:{padStick:'0'},classList:{toggle(_name,v){this.active=v;}},setAttribute(_name,v){this.transform=v;}};
  const status={textContent:''};
  const prior=global.document;
  global.document={querySelectorAll:s=>s==='[data-pad-button]'?nodes:[stick],getElementById:()=>status};
  try{
    const p={mapping:'standard',axes:[-1,0,0,0],buttons:Array.from({length:17},(_,i)=>({value:i===0||i===5?1:0}))};
    showPad(p); assert.ok(nodes[0].classList.active);assert.ok(nodes[3].classList.active);assert.equal(stick.transform,'translate(-9 0)');assert.match(status.textContent,/standard/);
    showPad(null);assert.ok(nodes.every(n=>!n.classList.active));assert.equal(stick.transform,'translate(0 0)');assert.match(status.textContent,/未检测/);
  }finally{global.document=prior;}
});
