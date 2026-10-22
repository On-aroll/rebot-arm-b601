(function(){
  'use strict';
  const $=id=>document.getElementById(id),twin=window.ReBotTwin;
  if(!twin||!$('wb-j7-tools'))return;
  let recorder=null,trials=[],report=null,visionReport=null,lastSampleAt=0,exportedCount=0;
  const json=value=>JSON.stringify(value,null,2),source=()=>twin.hardware?'hardware':'fake';
  function status(id,message,error=false){$(id).textContent=message;$(id).dataset.error=String(error);}
  function download(value,name){const url=URL.createObjectURL(new Blob([json(value)],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
  function action(id,out,fn){$(id).addEventListener('click',()=>{try{fn();}catch(e){status(out,e.message,true);}});}
  async function read(input){const f=input.files?.[0];if(!f||f.size>8*1024*1024)throw new Error('请选择小于8MB的JSON文件');return JSON.parse(await f.text());}
  function stop(reason='手动结束'){
    if(!recorder)return;
    if(recorder.trial.samples.length)trials.push({...recorder.trial,stop_reason:reason,duplicates_ignored:recorder.duplicates});
    recorder=null;$('wb-j7-start').disabled=false;
    status('wb-j7-status',`${reason} · 本页共 ${trials.length} 段。请导出保存。`);
  }
  function consume(state){
    if(!recorder)return;
    try{
      if(!twin.fresh())throw new Error('机器人反馈中断');
      if(recorder.append(state.extensions?.gripper,source(),Date.now()))lastSampleAt=performance.now();
      status('wb-j7-status',`${recorder.trial.source==='hardware'?'实机':'Fake'} · ${recorder.trial.samples.length} 个不同采样 · 重复反馈忽略 ${recorder.duplicates} 次 · 已存 ${trials.length} 段`);
    }catch(e){stop(e.message);}
  }
  action('wb-j7-start','wb-j7-status',()=>{
    if(recorder)throw new Error('请先结束当前记录');
    if(trials.length>=100)throw new Error('最多100段，请导出并清空');
    const g=twin.state()?.extensions?.gripper;
    if(!twin.fresh()||!g?.fresh)throw new Error('需要新鲜J7反馈；记录按钮不会启动或使能设备');
    if(!Number.isFinite(g.sample_at))throw new Error('当前驱动缺少J7原始采样时间戳；需在可现场核查时加载新版驱动');
    recorder=new window.ReBotGraspRecording.Recorder($('wb-j7-label').value,source(),$('wb-j7-note').value);
    lastSampleAt=performance.now();$('wb-j7-start').disabled=true;report=null;consume(twin.state());
  });
  action('wb-j7-stop','wb-j7-status',()=>stop());
  const log=()=>({schema_version:'b601-j7-recording/1.0',exported_ms:Date.now(),trials});
  action('wb-j7-export','wb-j7-status',()=>{if(recorder)throw new Error('请先结束本段再导出');if(!trials.length)throw new Error('尚无记录');download(log(),'b601-j7-recording.json');exportedCount=trials.length;});
  function showAnalysis(value){
    report=null;$('wb-j7-report').textContent='';$('wb-j7-summary').textContent='';
    report=window.ReBotGraspRecording.analyze(value);$('wb-j7-report').textContent=json(report);status('wb-j7-status',report.message);
    const o=report.observed;
    $('wb-j7-summary').textContent=o?`空夹中位位置 ${o.empty_position_median_rad.toFixed(4)} rad · 空夹/夹物最小间隔 ${o.min_endpoint_separation_rad.toFixed(4)} rad · 夹物静止电流 ${o.object_abs_current_range_a.map(x=>x.toFixed(3)).join('–')} A。此处不自动设置夹持阈值。`:`已分析 ${report.trials.length} 段 · 实机稳定空夹 ${report.hardware_stable_trials.empty} 段 / 夹物 ${report.hardware_stable_trials.object} 段`;
  }
  action('wb-j7-analyze','wb-j7-status',()=>{if(recorder)throw new Error('请先结束本段');showAnalysis(log());});
  $('wb-j7-import').addEventListener('change',async e=>{try{showAnalysis(await read(e.target));}catch(err){report=null;status('wb-j7-status',err.message,true);}finally{e.target.value='';}});
  action('wb-j7-report-export','wb-j7-status',()=>{if(!report)throw new Error('请先分析记录');download(report,'b601-j7-analysis.json');});
  action('wb-j7-clear','wb-j7-status',()=>{if(recorder)throw new Error('请先结束本段');if(exportedCount!==trials.length)throw new Error('还有未导出的记录，请先导出保存');trials=[];exportedCount=0;status('wb-j7-status','本页原始记录已清空');});
  function showVision(input){
    visionReport=null;$('wb-replay-report').textContent='';$('wb-replay-summary').textContent='';visionReport=window.ReBotVisionReplay.review(input);$('wb-replay-report').textContent=json(visionReport);
    status('wb-replay-status',`${visionReport.source.kind==='synthetic'?'合成示例（未运行识别）':'录制数据回放'} · 基座XYZ ${visionReport.target.position.map(x=>(x*1000).toFixed(2)).join(', ')} mm · 仅供坐标复核`);
    const g=visionReport.geometry;$('wb-replay-summary').textContent=`目标附近有效深度 ${g.valid_depth_pixels} 点 · 中位深度 ${(g.depth_median_m*1000).toFixed(2)} mm · 深度跨度 ${(g.depth_spread_m*1000).toFixed(2)} mm。夹取姿态、夹持中心和真实精度仍待验证。`;
  }
  action('wb-replay-example','wb-replay-status',()=>showVision(window.ReBotVisionReplay.example()));
  action('wb-replay-template','wb-replay-status',()=>download(window.ReBotVisionReplay.example(),'b601-synthetic-rgbd-replay.json'));
  $('wb-replay-import').addEventListener('change',async e=>{try{showVision(await read(e.target));}catch(err){visionReport=null;status('wb-replay-status',err.message,true);}finally{e.target.value='';}});
  action('wb-replay-export','wb-replay-status',()=>{if(!visionReport)throw new Error('请先完成坐标复核');download(visionReport,'b601-vision-review.json');});
  window.addEventListener('rebot-twin-state',e=>consume(e.detail));
  window.addEventListener('pagehide',()=>stop('页面已关闭'));
  setInterval(()=>{if(recorder&&(!twin.fresh()||performance.now()-lastSampleAt>600))stop('超过600ms没有新的反馈');},200);
})();
