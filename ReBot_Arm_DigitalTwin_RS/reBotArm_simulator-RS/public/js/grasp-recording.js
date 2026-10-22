(function(root){
  'use strict';
  const finite=v=>typeof v==='number'&&Number.isFinite(v);
  const quantile=(a,p)=>{a=[...a].sort((x,y)=>x-y);const i=(a.length-1)*p,j=Math.floor(i);return a[j]+(a[Math.ceil(i)]-a[j])*(i-j);};
  function validateSamples(samples){
    if(!Array.isArray(samples)||samples.length<1||samples.length>5000)throw new Error('每段记录需要1–5000个样本');
    samples.forEach((s,i)=>{
      if(![s.sample_at,s.received_ms,s.position,s.velocity,s.current_a].every(finite)||s.position<0||s.position>5||
        Math.abs(s.velocity)>10||Math.abs(s.current_a)>50||s.sample_at<0||s.received_ms<0||
        i>0&&(s.sample_at<=samples[i-1].sample_at||s.sample_at-samples[i-1].sample_at>.6))throw new Error('样本无效、重复、倒序或间隔超过600ms');
    });
  }
  class Recorder{
    constructor(label,source,note=''){
      if(!['empty','object','free'].includes(label)||!['hardware','fake'].includes(source))throw new Error('记录类型或来源无效');
      this.trial={id:globalThis.crypto?.randomUUID?.()||`trial-${Date.now()}-${Math.random()}`,label,source,note:String(note).slice(0,200),samples:[]};
      this.duplicates=0;
    }
    append(g,source,receivedMs){
      if(source!==this.trial.source||!g?.fresh)throw new Error('反馈中断或来源改变，记录已停止');
      const samples=this.trial.samples,last=samples.at(-1);
      if(last&&g.sample_at===last.sample_at){this.duplicates++;return false;}
      const sample={sample_at:g.sample_at,received_ms:receivedMs,position:g.position,velocity:g.velocity,current_a:g.current_a,
        reference:finite(g.target)?g.target:null,grasp_status:g.grasp?.status||''};
      validateSamples(last?[last,sample]:[sample]);
      if(samples.length>=5000||samples.length&&sample.sample_at-samples[0].sample_at>120)throw new Error('达到120秒/5000样本上限，记录已停止');
      samples.push(sample);return true;
    }
  }
  function summary(samples){
    const values=k=>samples.map(s=>k==='current_a'?Math.abs(s[k]):s[k]);
    return Object.fromEntries(['position','velocity','current_a'].map(k=>{const a=values(k);return [k,{min:Math.min(...a),median:quantile(a,.5),p95:quantile(a,.95),max:Math.max(...a)}];}));
  }
  function analyze(log){
    if(log?.schema_version!=='b601-j7-recording/1.0'||!Array.isArray(log.trials)||log.trials.length<1||log.trials.length>100)throw new Error('需要包含1–100段记录的 b601-j7-recording/1.0 文件');
    const ids=new Set();
    const trials=log.trials.map(t=>{
      if(typeof t.id!=='string'||!t.id||ids.has(t.id)||!['empty','object','free'].includes(t.label)||!['hardware','fake'].includes(t.source))throw new Error('记录ID重复或类型/来源无效');
      ids.add(t.id);validateSamples(t.samples);
      const last=t.samples.at(-1),tail=t.samples.filter(s=>s.sample_at>=last.sample_at-1.2);
      // Last stationary second, without combining repeated feedback or separate runs.
      const stable=tail.length>=5&&last.sample_at-tail[0].sample_at>=1&&tail.every(s=>Math.abs(s.velocity)<=.01)&&Math.max(...tail.map(s=>s.position))-Math.min(...tail.map(s=>s.position))<=.02;
      return {id:t.id,label:t.label,source:t.source,note:t.note,samples:t.samples.length,duration_s:last.sample_at-t.samples[0].sample_at,
        all:summary(t.samples),stationary:stable?summary(tail):null};
    });
    const empty=trials.filter(t=>t.label==='empty'&&t.source==='hardware'&&t.stationary),objects=trials.filter(t=>t.label==='object'&&t.source==='hardware'&&t.stationary);
    const ready=empty.length>=3&&objects.length>=3;
    const result={schema_version:'b601-j7-analysis/1.0',hardware_execution_allowed:false,object_verified:false,trials,
      hardware_stable_trials:{empty:empty.length,object:objects.length},calibration_ready:false,
      message:ready?'已有至少三组实机空夹/夹物稳定片段；下面仅为观测分布，仍需核对工件、行程、保护上限和现场表现。':'每类至少记录三组实机空夹/夹物，末尾保持静止一秒；Fake不参与实机标定。',
      required:['核实实际工件与夹具接触情况','中止电流须结合现有设备上限和现场验证确定','角度到夹爪开口及TCP仍需标定']};
    if(ready){
      const positions=empty.map(t=>t.stationary.position.median),emptyMax=Math.max(...empty.map(t=>t.stationary.position.max)),objectMin=Math.min(...objects.map(t=>t.stationary.position.min));
      result.observed={empty_position_median_rad:quantile(positions,.5),empty_position_range_rad:[Math.min(...empty.map(t=>t.stationary.position.min)),emptyMax],
        object_min_position_rad:objectMin,min_endpoint_separation_rad:objectMin-emptyMax,
        object_abs_current_range_a:[Math.min(...objects.map(t=>t.stationary.current_a.min)),Math.max(...objects.map(t=>t.stationary.current_a.max))]};
      if(objectMin-emptyMax<=.07)result.message+=' 空夹与夹物位置间隔不足默认0.02+0.05rad，当前默认判定无法可靠区分。';
    }
    return result;
  }
  const api={Recorder,analyze};if(typeof module==='object'&&module.exports)module.exports=api;else root.ReBotGraspRecording=api;
})(typeof window==='object'?window:globalThis);
