const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const source=fs.readFileSync(require.resolve('../public/js/twin-console.js'),'utf8');
const start=source.indexOf('    if (session && client.connected && !document.hidden && now-lastHeartbeatAt');
const gate=source.slice(start,source.indexOf('\n  function sampleManualInput',start));
function poll({op='program_preview',compiling=false,receivedAt=110,sentAt=100}={}){
  const sent=[];
  const context={session:'owned',client:{connected:true},document:{hidden:false},now:200,lastHeartbeatAt:0,
    pending:new Map([['request',{op,sentAt}]]),feedback:{extensions:{program_compiling:compiling}},receivedAt,
    publish:op=>sent.push(op),forgetSession(){},tell(){}};
  vm.createContext(context);vm.runInContext('(function(){\n'+gate+')();',context);
  return sent;
}
test('heartbeat cannot replace a program preview before controller acceptance',()=>{
  assert.deepEqual(poll(),[]);
  assert.deepEqual(poll({op:'support_preview'}),[]);
  assert.deepEqual(poll({op:'program_resume'}),[]);
});
test('long preview keeps heartbeat only after a fresh compiling status',()=>{
  assert.deepEqual(poll({compiling:true,receivedAt:90}),[]);
  for(const op of ['program_preview','support_preview','program_resume'])
    assert.deepEqual(poll({op,compiling:true}),['heartbeat']);
  assert.deepEqual(poll({op:'program_start',compiling:true}),[]);
});
