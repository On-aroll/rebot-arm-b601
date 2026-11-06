(function(root){
  'use strict';
  const finite=v=>Array.isArray(v)&&v.length===3&&v.every(x=>typeof x==='number'&&Number.isFinite(x));
  const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const add=(a,b)=>a.map((x,i)=>x+b[i]),scale=(v,k)=>v.map(x=>x*k);
  const mv=(r,v)=>r.map(row=>dot(row,v));
  const transpose=r=>r[0].map((_,i)=>r.map(row=>row[i]));
  const mm=(a,b)=>a.map(row=>transpose(b).map(column=>dot(row,column)));
  function rotation([roll,pitch,yaw]){
    const cr=Math.cos(roll),sr=Math.sin(roll),cp=Math.cos(pitch),sp=Math.sin(pitch),cy=Math.cos(yaw),sy=Math.sin(yaw);
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
      [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]];
  }
  function create(pose,fixed,settings){
    if(!Array.isArray(pose)||pose.length!==6||!pose.every(Number.isFinite)||
       !finite(fixed?.rpy)||!finite(fixed?.xyz)||!finite(settings?.tilt_deg)||!finite(settings?.offset_m)||
       !/^[+-][XYZ]$/.test(settings.axis)||![1,-1].includes(settings.retract_sign)||
       !Number.isFinite(settings.spin_deg)||settings.tilt_deg.some(v=>Math.abs(v)>180)||
       Math.abs(settings.spin_deg)>180||settings.offset_m.some(v=>Math.abs(v)>.3))
      throw new Error('请选择带正负号的腕部法线轴，偏角与原点偏移须在允许范围内');
    const tcp=rotation(pose.slice(3)),wrist=mm(tcp,transpose(rotation(fixed.rpy)));
    const i='XYZ'.indexOf(settings.axis[1]),sign=settings.axis[0]==='+'?1:-1;
    const n=[0,0,0],right=[0,0,0];n[i]=sign;right[(i+1)%3]=1;
    const up=cross(n,right),r=rotation(settings.tilt_deg.map(v=>v*Math.PI/180));
    const normal=mv(wrist,mv(r,n)),u0=mv(wrist,mv(r,right)),v0=mv(wrist,mv(r,up));
    const angle=settings.spin_deg*Math.PI/180,c=Math.cos(angle),s=Math.sin(angle);
    const u=add(scale(u0,c),scale(v0,s)),v=add(scale(u0,-s),scale(v0,c));
    const plane={kind:'tool_normal',frame_id:'base_link',tcp_frame:'gripper_end',reference_frame:'link6',
      origin:add(pose.slice(0,3),mv(wrist,settings.offset_m)),orientation_rpy:pose.slice(3),
      normal,u,v,retract_direction:scale(normal,settings.retract_sign),
      wrist_axes:transpose(wrist),wrist_origin:add(pose.slice(0,3),scale(mv(wrist,fixed.xyz),-1)),
      captured_pose:pose.slice(),fixed_transform:{rpy:fixed.rpy.slice(),xyz:fixed.xyz.slice()},
      configuration:JSON.parse(JSON.stringify(settings)),frozen:true,preview_only:true,pen_tip_calibrated:false};
    projector(plane);return plane;
  }
  function projector(plane){
    if(!plane)return p=>p.slice(0,2);
    if(plane.kind!=='tool_normal'||plane.frame_id!=='base_link'||!finite(plane.origin)||
       !['u','v','normal'].every(k=>finite(plane[k])&&Math.abs(dot(plane[k],plane[k])-1)<1e-6)||
       Math.abs(dot(plane.u,plane.v))+Math.abs(dot(plane.u,plane.normal))+Math.abs(dot(plane.v,plane.normal))>1e-6)
      throw new Error('书写平面坐标无效，请重新记录末端位置与朝向');
    return p=>{const d=p.map((x,i)=>x-plane.origin[i]);return [dot(d,plane.u),dot(d,plane.v)];};
  }
  function build(template,plane,{text,width=.09,height=.02,lift=.005,speed=.05}={}){
    projector(plane);
    if(!template||!Array.isArray(template.strokes)||!Number.isFinite(template.advance)||template.advance<=0||
       !['BIT103','国庆快乐'].includes(text)||![width,height,lift,speed].every(Number.isFinite)||
       width<=0||width>(text==='BIT103'?.1:.3)||height<=0||height>.1||lift<=0||lift>.03||speed<.005||speed>.1)
      throw new Error('书写尺寸或速度无效；BIT103总宽须不超过100mm');
    const steps=[],strokes=[],direction=plane.retract_direction||plane.normal;
    for(const row of template.strokes){
      if(!Array.isArray(row.points)||row.points.length<2||row.points.some(p=>!Array.isArray(p)||p.length!==2||!p.every(Number.isFinite)))throw new Error('笔画数据无效');
      const points=row.points.map(([x,y])=>add(plane.origin,add(scale(plane.u,x*width/template.advance),scale(plane.v,y*height))));
      strokes.push({...row,points});
      const push=(p,action)=>steps.push({type:'pose',pose:[...p,...plane.orientation_rpy],orientation:true,speed,label:`${row.char}·${String(row.stroke).padStart(2,'0')}·${action}`});
      push(add(points[0],scale(direction,lift)),'移笔');push(points[0],'落笔');
      const spacing=text==='BIT103'?.003:.009;
      for(let i=1;i<points.length;i++){
        const a=points[i-1],b=points[i],count=Math.max(1,Math.ceil(Math.hypot(...a.map((x,j)=>b[j]-x))/spacing));
        for(let k=1;k<=count;k++)push(a.map((x,j)=>x+(b[j]-x)*k/count),'书写');
      }
      push(add(points.at(-1),scale(direction,lift)),'抬笔');
    }
    if(steps.length>250)throw new Error('路径超过250步，请减小文字尺寸');
    return {schema_version:'1.0',robot_model:'b601_rs',base_frame:'base_link',calibration_id:'rs-identity-v1',
      units:{position:'m',angle:'rad',time:'s'},name:`${text} · 腕部轴法线 · ${width*1000}mm · 几何预览`,
      writing_plane:plane,writing_setup:{text,width,height,lift,speed},steps};
  }
  const api={projector,create,build};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  else root.ReBotWritingPlane=api;
})(typeof window!=='undefined'?window:globalThis);
