const {test}=require('node:test');
const assert=require('node:assert/strict');
const {review,example}=require('../public/js/vision-replay.js');
const {Recorder,analyze}=require('../public/js/grasp-recording.js');

test('RGBD replay projects pixels in metres and composes image-time hand-eye transforms',()=>{
  const p=example(),r=review(p);
  assert.deepEqual(r.target.position,[.10800000000000001,0,.2]);
  assert.equal(r.hardware_execution_allowed,false);assert.equal(r.object_verified,false);assert.equal(r.stamp_ms,1000);
  p.robot_pose.T_base_tool=[[0,-1,0,.1],[1,0,0,0],[0,0,1,-.2],[0,0,0,1]];
  assert.ok(review(p).target.position.every((x,i)=>Math.abs(x-[.1,.008,.2][i])<1e-12));
  p.calibration.T_tool_camera[0][3]=.02;
  assert.ok(Math.abs(review(p).target.position[1]-.028)<1e-12);
});
test('fixed-camera path does not depend on current wrist pose',()=>{
  const p=example();p.calibration.mount='fixed';p.calibration.T_base_camera=p.robot_pose.T_base_tool;delete p.calibration.T_tool_camera;delete p.robot_pose;
  assert.deepEqual(review(p).target.position,[.10800000000000001,0,.2]);
});
test('unsynchronised frames, mismatched calibration and unrectified/unaligned depth are rejected',()=>{
  const bad=[p=>p.depth.stamp_ms+=51,p=>p.robot_pose.stamp_ms+=51,p=>p.robot_pose.tool_frame='flange',
    p=>p.calibration.camera_id='wrong',p=>p.robot_pose.calibration_id='wrong',p=>p.camera.rectified=false,
    p=>p.depth.aligned_to='depth',p=>p.camera.K[1]=1,p=>p.depth.scale_m=0,
    p=>p.detection.confidence=.49,p=>p.detection.center_px=[640,0],p=>p.calibration.T_tool_camera[0][0]=-1,
    p=>p.calibration.T_tool_camera[0][3]=1,p=>p.source.kind='live',p=>p.depth.roi.x=700,
    p=>p.depth.roi.values[0][0]=null,p=>p.depth.roi.values[0][0]=NaN];
  for(const mutate of bad){const p=example();mutate(p);assert.throws(()=>review(p));}
});
test('depth holes and foreground/background mixture cannot silently produce targets',()=>{
  const p=example();p.depth.roi.values=p.depth.roi.values.map(row=>row.map(()=>0));assert.throws(()=>review(p),/深度不足/);
  const edge=example();edge.depth.roi.values=edge.depth.roi.values.map((row,y)=>row.map(()=>y<2?800:400));assert.throws(()=>review(edge),/深度跨度/);
});
const sample=(stamp,position=.2,current=.5)=>({fresh:true,sample_at:stamp,position,velocity:0,current_a:current,target:position});
test('readonly recorder deduplicates source timestamps and rejects stale, restarted and switched sources',()=>{
  const r=new Recorder('empty','hardware');assert.equal(r.append(sample(1),'hardware',1000),true);
  assert.equal(r.append(sample(1),'hardware',1010),false);assert.equal(r.duplicates,1);assert.equal(r.trial.samples.length,1);
  for(const [g,source] of [[sample(.9),'hardware'],[sample(1.7),'hardware'],[{...sample(1.1),fresh:false},'hardware'],[sample(1.1),'fake'],[{...sample(1.1),current_a:NaN},'hardware']])assert.throws(()=>r.append(g,source,1100));
});
function trial(label,source,id,position,current){
  const r=new Recorder(label,source);r.trial.id=id;
  for(let i=0;i<21;i++)r.append(sample(1+i*.1,position,current),source,1000+i*100);
  return r.trial;
}
function log(source='hardware'){
  return {schema_version:'b601-j7-recording/1.0',trials:Array.from({length:3},(_,i)=>[
    trial('empty',source,`empty-${i}`,0,.1),trial('object',source,`object-${i}`,.3,.6)]).flat()};
}
test('J7 distributions use independent stationary real runs; no executable thresholds are manufactured',()=>{
  const a=analyze(log());assert.deepEqual(a.hardware_stable_trials,{empty:3,object:3});
  assert.equal(a.observed.empty_position_median_rad,0);assert.equal(a.observed.min_endpoint_separation_rad,.3);
  assert.equal(a.hardware_execution_allowed,false);assert.equal(a.calibration_ready,false);assert.equal(a.object_verified,false);
  assert.equal(a.contact_current_a,undefined);assert.equal(a.abort_current_a,undefined);
  assert.equal(analyze(log('fake')).observed,undefined);
  const moving=log();moving.trials.forEach(t=>t.samples.at(-1).velocity=.02);assert.equal(analyze(moving).observed,undefined);
});
test('imported corrupt J7 recordings cannot acquire calibration evidence',()=>{
  for(const modify of [v=>v.trials[0].samples[2].sample_at=1,v=>v.trials[0].samples[2].sample_at=9,
    v=>v.trials[0].samples[0].current_a=null,v=>v.trials[1].id=v.trials[0].id,v=>v.trials[0].source='unknown']){
    const p=log();modify(p);assert.throws(()=>analyze(p));
  }
});
test('near-overlapping J7 endpoints are reported as insufficient separation',()=>{
  const p=log();p.trials.filter(t=>t.label==='object').forEach(t=>t.samples.forEach(s=>s.position=.04));assert.match(analyze(p).message,/无法可靠区分/);
});
function page(){
  const fs=require('node:fs'),vm=require('node:vm'),elements=new Map(),events={},timers=[];
  const element=id=>{if(!elements.has(id))elements.set(id,{value:id==='wb-j7-label'?'empty':'',textContent:'',dataset:{},disabled:false,handlers:{},addEventListener(event,fn){this.handlers[event]=fn;}});return elements.get(id);};
  let now=1000,state={extensions:{gripper:sample(1)}},fresh=true;
  const context={window:{ReBotTwin:{hardware:true,fresh:()=>fresh,state:()=>state,request:()=>assert.fail('记录不应请求运动')},
    ReBotGraspRecording:require('../public/js/grasp-recording.js'),ReBotVisionReplay:require('../public/js/vision-replay.js'),addEventListener:(e,fn)=>events[e]=fn},
    document:{getElementById:element},Date,performance:{now:()=>now},setInterval:fn=>timers.push(fn)};
  vm.runInNewContext(fs.readFileSync(require.resolve('../public/js/twin-offline-tools.js'),'utf8'),context);
  return {element,click:id=>element(id).handlers.click(),tick:(stamp,dt=100)=>{now+=dt;state={extensions:{gripper:sample(stamp)}};events['rebot-twin-state']({detail:state});},
    lose:()=>{fresh=false;timers.forEach(fn=>fn());},timeout:()=>{now+=601;timers.forEach(fn=>fn());}};
}
test('actual page recorder collects raw distinct samples, analyzes, and never sends robot requests',()=>{
  const p=page();p.click('wb-j7-start');p.tick(1);for(let i=1;i<=20;i++)p.tick(1+i*.1);
  assert.match(p.element('wb-j7-status').textContent,/21 个不同采样/);p.click('wb-j7-stop');p.click('wb-j7-analyze');
  const report=JSON.parse(p.element('wb-j7-report').textContent);assert.equal(report.trials[0].samples,21);assert.equal(report.hardware_execution_allowed,false);
});
test('actual page stops recording on disconnection, no-new-sample timeout or clock restart',()=>{
  for(const stop of [p=>p.lose(),p=>p.timeout(),p=>p.tick(.5)]){
    const p=page();p.click('wb-j7-start');stop(p);assert.equal(p.element('wb-j7-start').disabled,false);
    assert.match(p.element('wb-j7-status').textContent,/本页共 1 段/);
  }
});
