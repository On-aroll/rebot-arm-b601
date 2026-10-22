(function (root) {
  'use strict';
  const limits={empty_position:[0,4.8],empty_tolerance:[.005,.1],min_gap:[.02,.5],
    contact_current_a:[.01,15.9],abort_current_a:[.02,16],stall_velocity:[.001,.03],
    min_travel:[.01,.5],dwell:[.3,2],hold_time:[.3,3],timeout:[2,180]};
  function number(value,label,lo,hi){
    if(typeof value!=='number'||!Number.isFinite(value)||value<lo||value>hi)throw new Error(`${label} 需要 ${lo}–${hi} 内有限数值`);
    return value;
  }
  function validateProfile(raw){
    if(!raw||typeof raw!=='object'||Array.isArray(raw)||Object.keys(raw).length!==Object.keys(limits).length||Object.keys(raw).some(k=>!limits[k]))throw new Error('夹持标定字段不完整或不支持');
    const p=Object.fromEntries(Object.entries(limits).map(([k,[lo,hi]])=>[k,number(raw[k],k,lo,hi)]));
    if(p.contact_current_a>=p.abort_current_a)throw new Error('接触电流必须小于中止电流');
    if(p.dwell+p.hold_time>=p.timeout)throw new Error('超时必须大于接触与保持确认时间');
    return p;
  }
  function build({pose,approach,openPosition,jointSpeed,graspSpeed,profile,calibrationId}){
    if(!Array.isArray(pose)||pose.length!==6)throw new Error('夹取点需要XYZ和RPY六个数值');
    pose=pose.map((v,i)=>number(v,`夹取点${i+1}`,i<3?-.8:-Math.PI,i<3?.8:Math.PI));
    number(approach,'接近高度',.005,.2);number(openPosition,'张开电机角',0,5);
    number(jointSpeed,'机械臂速度',.005,1.5);number(graspSpeed,'夹持闭合速度',.005,.1);
    const p=validateProfile(profile);
    if(graspSpeed<=p.stall_velocity*2)throw new Error('闭合速度须大于停滞阈值的两倍');
    if(openPosition<p.empty_position+p.empty_tolerance+p.min_gap+p.min_travel)throw new Error('张开角度不足，不能判定已发生有效闭合');
    if(typeof calibrationId!=='string'||!calibrationId)throw new Error('缺少机器人标定版本');
    const pre=[...pose];pre[2]+=approach;
    if(pre[2]>.8)throw new Error('接近点超出工作空间参考');
    const steps=[{type:'gripper',position:openPosition,speed:graspSpeed,label:'张开J7'},
      {type:'pose',pose:pre,orientation:true,speed:jointSpeed,label:'到达夹取点上方'}];
    const count=Math.ceil(approach/.005);
    for(let i=1;i<=count;i++){
      const point=[...pose];point[2]=i===count?pose[2]:pre[2]-approach*i/count;
      steps.push({type:'pose',pose:point,orientation:true,speed:jointSpeed,label:`垂直接近 ${i}/${count}`});
    }
    steps.push({type:'grasp',speed:graspSpeed,profile:p,label:'闭合J7并验证接触保持（结束后原位保持）'});
    return {schema_version:'1.0',robot_model:'b601_rs',base_frame:'base_link',calibration_id:calibrationId,
      units:{position:'m',angle:'rad',time:'s'},name:'指定点夹取 · 原位验证',steps};
  }
  const api={validateProfile,build};
  if(typeof module==='object'&&module.exports)module.exports=api;
  else root.ReBotFixedPick=api;
})(typeof window==='object'?window:globalThis);
