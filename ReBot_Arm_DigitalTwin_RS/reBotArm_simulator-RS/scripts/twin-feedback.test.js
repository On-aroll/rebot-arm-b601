const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

// Exercise the actual display function with a URDF-style joint. This never
// loads ROS or CAN and cannot publish a hardware command.
const source = fs.readFileSync(require.resolve('../public/js/rebot-sim.js'),'utf8');
const start = source.indexOf('  function setJoint(');
const end = source.indexOf('  function setGhostJoint(',start);
assert.ok(start >= 0 && end > start);
function display() {
  const joint = {ignoreLimits:false,angle:0,setJointValue(value) {
    this.angle = this.ignoreLimits ? value : Math.max(0,Math.min(3.14,value));
  }};
  const commands=[];
  const context={jointDefs:[{name:'joint2',min:0,max:3.14}],currentAngles:{},robot:{},
    clamp:(v,a,b)=>Math.max(a,Math.min(b,v)),getJoint:()=>joint,
    document:{getElementById:()=>null},updateJointLabel:()=>{},
    emitCommand:c=>commands.push(c),performance:{now:()=>0}};
  vm.createContext(context);
  vm.runInContext(source.slice(start,end),context);
  return {context,joint,commands};
}
test('encoder feedback preserves measured out-of-limit pose without transmitting',()=>{
  const {context,joint,commands}=display();
  const measured=-.22*Math.PI/180;
  context.setJoint('joint2',measured,false,{source:'ros',forceFeedback:true});
  assert.equal(context.currentAngles.joint2,measured);
  assert.equal(joint.angle,measured);
  assert.equal(joint.ignoreLimits,false);
  assert.deepEqual(commands,[]);
});
test('user and local simulation targets retain nominal limits',()=>{
  for (const [fromUser,options] of [[true,{source:'user'}],[false,{source:'sim'}]]) {
    const {context,joint}=display();
    context.setJoint('joint2',-.1,fromUser,options);
    assert.equal(context.currentAngles.joint2,0);
    assert.equal(joint.angle,0);
    assert.equal(joint.ignoreLimits,false);
  }
});
