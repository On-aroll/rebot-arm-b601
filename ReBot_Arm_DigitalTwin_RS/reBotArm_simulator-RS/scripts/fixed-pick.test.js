const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const {build,validateProfile}=require('../public/js/fixed-pick.js');
const {validateProgram}=require('../workbench-api.js');
const profile=()=>({empty_position:0,empty_tolerance:.02,min_gap:.05,contact_current_a:.5,
  abort_current_a:2,stall_velocity:.01,min_travel:.05,dwell:.3,hold_time:.3,timeout:30});
const request=()=>({pose:[.25,-.16,.31,0,0,0],approach:.03,openPosition:1,jointSpeed:.1,
  graspSpeed:.1,profile:profile(),calibrationId:'rs-identity-v1'});
test('point builder produces inert native program with sampled approach and a conditional final grasp',()=>{
  const p=build(request());assert.equal(validateProgram(p),p);
  assert.equal(p.steps[0].type,'gripper');assert.equal(p.steps.at(-1).type,'grasp');
  const poses=p.steps.filter(s=>s.type==='pose');assert.equal(poses.length,7);
  for(let i=1;i<poses.length;i++){
    assert.ok(poses[i-1].pose[2]-poses[i].pose[2]<=.005+1e-10);
    assert.deepEqual(poses[i].pose.slice(3),[0,0,0]);assert.equal(poses[i].orientation,true);
  }
  assert.deepEqual(poses.at(-1).pose,request().pose);
  assert.ok(!p.steps.some(s=>s.type==='enable'));
});
test('missing calibration/current data and bad opening cannot produce an executable proposal',()=>{
  for(const change of [{calibrationId:''},{openPosition:.01},{graspSpeed:.2},{pose:[NaN,0,0,0,0,0]},
    {profile:{}},{profile:{...profile(),contact_current_a:0}},{profile:{...profile(),extra:1}}]){
    assert.throws(()=>build({...request(),...change}));
  }
  assert.throws(()=>validateProfile({...profile(),abort_current_a:.5}));
});
test('saved/imported grasp program uses the same strict thresholds as the point builder',()=>{
  const p=build(request());assert.equal(validateProgram(JSON.parse(JSON.stringify(p))).steps.at(-1).type,'grasp');
  for(const change of [{speed:.2},{profile:{...profile(),contact_current_a:true}},
    {profile:{...profile(),timeout:.1}},{profile:{...profile(),abort_current_a:100}}]){
    const bad=structuredClone(p);Object.assign(bad.steps.at(-1),change);assert.throws(()=>validateProgram(bad));
  }
});
test('page loads builder before workbench and includes explicit calibration and feedback fields',()=>{
  const page=fs.readFileSync(require.resolve('../public/index.html'),'utf8');
  assert.ok(page.indexOf('js/fixed-pick.js')<page.indexOf('js/twin-workbench.js'));
  for(const id of ['wb-pick-build','wb-pick-capture','wb-grasp-empty_position','wb-grasp-contact_current_a','wb-grasp-abort_current_a','wb-grasp-status'])assert.ok(page.includes(`id="${id}"`));
  for(const id of ['empty_position','contact_current_a','abort_current_a']){
    const input=page.match(new RegExp(`<input id="wb-grasp-${id}"[^>]+>`))[0];assert.ok(!input.includes(' value='));
  }
});
