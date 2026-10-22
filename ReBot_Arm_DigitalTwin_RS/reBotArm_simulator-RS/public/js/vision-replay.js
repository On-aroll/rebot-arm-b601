(function(root){
  'use strict';
  const finite=v=>typeof v==='number'&&Number.isFinite(v);
  const vector=(v,n)=>Array.isArray(v)&&v.length===n&&v.every(finite);
  function requireValue(ok,message){if(!ok)throw new Error(message);}
  function rigid(m){
    requireValue(Array.isArray(m)&&m.length===4&&m.every(r=>vector(r,4)),'需要有限的4×4变换矩阵');
    requireValue(m[3].every((v,i)=>Math.abs(v-(i===3?1:0))<1e-6),'变换矩阵末行无效');
    for(let i=0;i<3;i++)for(let j=0;j<3;j++)requireValue(Math.abs(m[i].slice(0,3).reduce((s,x,k)=>s+x*m[j][k],0)-(i===j?1:0))<1e-5,'旋转矩阵不正交');
    const det=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    requireValue(Math.abs(det-1)<1e-5,'旋转矩阵包含镜像');return m;
  }
  function multiply(a,b){return a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0)));}
  function quantile(values,p){const a=[...values].sort((x,y)=>x-y),i=(a.length-1)*p,lo=Math.floor(i);return a[lo]+(a[Math.ceil(i)]-a[lo])*(i-lo);}
  // Offline review only. No live observation/program contract or motion API is exposed.
  function review(input){
    requireValue(input?.schema_version==='b601-rgbd-replay/1.0','需要 b601-rgbd-replay/1.0 文件');
    const {source,camera,depth,detection,calibration,robot_pose:robot}=input;
    requireValue(['synthetic','recorded'].includes(source?.kind)&&typeof source.capture_id==='string'&&source.capture_id.length>0,'回放必须声明合成/录制来源及采集ID');
    requireValue(typeof camera?.id==='string'&&camera.id&&camera.frame_id==='camera_optical_frame','缺少相机ID或光学坐标系');
    requireValue(Number.isInteger(camera.width)&&Number.isInteger(camera.height)&&camera.width>0&&camera.height>0&&camera.width<=8192&&camera.height<=8192,'图像尺寸无效');
    requireValue(camera.rectified===true&&vector(camera.K,9)&&camera.K[0]>0&&camera.K[4]>0&&[1,3,6,7].every(i=>camera.K[i]===0)&&camera.K[8]===1,'需要已校正图像对应的无斜切内参K');
    requireValue(finite(input.stamp_ms)&&input.stamp_ms>=0&&finite(depth?.stamp_ms)&&Math.abs(input.stamp_ms-depth.stamp_ms)<=50,'RGB与深度时间差须不超过50ms');
    requireValue(depth.aligned_to==='color'&&finite(depth.scale_m)&&depth.scale_m>0&&depth.scale_m<=1,'需要与彩色图对齐的深度及米/单位比例');
    requireValue(typeof detection?.label==='string'&&detection.label&&finite(detection.confidence)&&detection.confidence>=.5&&detection.confidence<=1,'目标置信度至少0.5');
    requireValue(vector(detection.center_px,2)&&detection.center_px[0]>=0&&detection.center_px[0]<camera.width&&detection.center_px[1]>=0&&detection.center_px[1]<camera.height,'目标像素超出彩色图');
    const roi=depth.roi,rows=roi?.values;
    requireValue(Number.isInteger(roi?.x)&&Number.isInteger(roi?.y)&&roi.x>=0&&roi.y>=0&&Array.isArray(rows)&&rows.length>0&&rows.length<=256&&Array.isArray(rows[0])&&rows[0].length>0&&rows[0].length<=256,'深度ROI无效（最大256×256）');
    requireValue(rows.every(r=>vector(r,rows[0].length)&&r.every(v=>v>=0))&&roi.x+rows[0].length<=camera.width&&roi.y+rows.length<=camera.height,'深度ROI必须为图像内的有限非负矩形');
    const [u,v]=detection.center_px;
    requireValue(u>=roi.x&&u<=roi.x+rows[0].length-1&&v>=roi.y&&v<=roi.y+rows.length-1,'深度ROI未覆盖目标中心');
    // Center 5x5 depth window, matching the upstream median fallback convention.
    const samples=[];let total=0;
    rows.forEach((row,y)=>row.forEach((raw,x)=>{if(Math.abs(roi.x+x-Math.round(u))<=2&&Math.abs(roi.y+y-Math.round(v))<=2){total++;const z=raw*depth.scale_m;if(z>=.05&&z<=1)samples.push(z);}}));
    requireValue(total>=9&&samples.length>=9&&samples.length/total>=.6,'目标附近有效深度不足');
    requireValue(quantile(samples,.9)-quantile(samples,.1)<=.02,'目标附近深度跨度过大，可能跨越边缘或背景');
    const z=quantile(samples,.5),point=[(u-camera.K[2])*z/camera.K[0],(v-camera.K[5])*z/camera.K[4],z,1];
    requireValue(typeof calibration?.id==='string'&&calibration.id&&calibration.camera_id===camera.id,'相机与标定ID不匹配');
    let transform;
    if(calibration.mount==='fixed'){
      requireValue(!calibration.T_tool_camera,'固定相机不可混入腕部变换');transform=rigid(calibration.T_base_camera);
    }else{
      requireValue(calibration.mount==='eye_in_hand'&&!calibration.T_base_camera,'相机安装类型必须为 fixed 或 eye_in_hand');
      requireValue(typeof calibration.tool_frame==='string'&&calibration.tool_frame&&typeof calibration.robot_calibration_id==='string'&&calibration.robot_calibration_id,'腕部标定缺少末端坐标系/机器人标定ID');
      requireValue(robot?.frame_id==='base_link'&&robot.tool_frame===calibration.tool_frame&&robot.calibration_id===calibration.robot_calibration_id,'机器人位姿与手眼标定坐标系不匹配');
      requireValue(finite(robot.stamp_ms)&&Math.abs(robot.stamp_ms-input.stamp_ms)<=50,'腕部相机需要拍摄时50ms内的机器人位姿');
      transform=multiply(rigid(robot.T_base_tool),rigid(calibration.T_tool_camera));
    }
    const position=transform.slice(0,3).map(row=>row.reduce((s,x,i)=>s+x*point[i],0));
    requireValue(position.every(x=>Math.abs(x)<=.8),'投影位置超出±0.8m参考范围');
    return {schema_version:'b601-vision-review/1.0',source,stamp_ms:input.stamp_ms,calibration_id:calibration.id,
      hardware_execution_allowed:false,object_verified:false,
      target:{label:detection.label,confidence:detection.confidence,frame_id:'base_link',position},
      geometry:{camera_position:point.slice(0,3),T_base_camera:transform,valid_depth_pixels:samples.length,depth_median_m:z,depth_spread_m:quantile(samples,.9)-quantile(samples,.1)},
      pending:['真实检测与深度质量验证','夹取姿态和夹持中心TCP标定','IK/碰撞与现场验证']};
  }
  function example(){
    const identity=()=>[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
    const t=identity();t[0][3]=.1;t[2][3]=-.2;
    return {schema_version:'b601-rgbd-replay/1.0',source:{kind:'synthetic',capture_id:'synthetic-geometry-only'},stamp_ms:1000,
      camera:{id:'synthetic-camera',frame_id:'camera_optical_frame',width:640,height:480,rectified:true,K:[600,0,320,0,600,240,0,0,1]},
      detection:{label:'合成目标（未运行识别）',confidence:1,center_px:[332,240],obb_px:[[328,238],[336,238],[336,242],[328,242]]},
      depth:{stamp_ms:1000,aligned_to:'color',scale_m:.001,roi:{x:328,y:238,values:Array.from({length:5},()=>Array(9).fill(400))}},
      calibration:{id:'synthetic-hand-eye',camera_id:'synthetic-camera',mount:'eye_in_hand',tool_frame:'gripper_end',robot_calibration_id:'rs-identity-v1',T_tool_camera:identity()},
      robot_pose:{stamp_ms:1000,frame_id:'base_link',tool_frame:'gripper_end',calibration_id:'rs-identity-v1',T_base_tool:t}};
  }
  const api={review,example};if(typeof module==='object'&&module.exports)module.exports=api;else root.ReBotVisionReplay=api;
})(typeof window==='object'?window:globalThis);
