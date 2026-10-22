(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const video = $('twin-video'), image = $('twin-camera-image');
  const source = $('twin-camera-source'), devices = $('twin-camera-device');
  let stream = null, topic = '', generation = 0, live = false, frameCallback = 0, lastFrameAt = 0, sourceStamp = null;
  const client = window.reBotRos;
  function status(text, ok = false) {
    $('twin-camera-state').textContent = text;
    $('twin-camera-state').dataset.ok = String(ok);
  }
  function message(text) { $('twin-camera-message').textContent = text; }
  function show(kind) {
    live = Boolean(kind);
    video.hidden = kind !== 'video'; image.hidden = kind !== 'image';
    $('twin-camera-placeholder').hidden = live;
  }
  function disconnect() {
    generation++;
    if (frameCallback && video.cancelVideoFrameCallback) video.cancelVideoFrameCallback(frameCallback);
    frameCallback = 0;
    if (stream) { stream.getTracks().forEach((track) => track.stop()); stream = null; }
    if (topic && client) client.unsubscribe(topic);
    topic = '';
    video.pause(); video.srcObject = null; video.removeAttribute('src'); video.load();
    image.onload = image.onerror = null; image.removeAttribute('src');
    video.removeAttribute('crossorigin'); image.removeAttribute('crossorigin');
    lastFrameAt = 0; sourceStamp = null;
    show(null); status('未连接相机');
    $('twin-camera-timing').textContent = '相机与关节反馈独立采集，视频延迟尚未标定';
  }
  async function listDevices() {
    if (!navigator.mediaDevices?.enumerateDevices) return;
    const selected = devices.value;
    const cameras = (await navigator.mediaDevices.enumerateDevices()).filter((device) => device.kind === 'videoinput');
    devices.replaceChildren(new Option('系统默认相机',''));
    cameras.forEach((device,i) => devices.add(new Option(device.label || `相机 ${i+1}（连接后显示名称）`,device.deviceId)));
    if ([...devices.options].some((option) => option.value === selected)) devices.value = selected;
  }
  function sourceChanged() {
    disconnect();
    const type = source.value;
    $('twin-camera-device-row').hidden = type !== 'usb';
    $('twin-camera-url-row').hidden = type !== 'mjpeg' && type !== 'video';
    $('twin-camera-cors-row').hidden = type !== 'mjpeg' && type !== 'video';
    $('twin-camera-topic-row').hidden = type !== 'ros';
    message({usb:'USB 相机属于打开网页的这台电脑；连接时浏览器会请求授权。',mjpeg:'填写相机提供的 HTTP MJPEG/JPEG 地址。RTSP 需要先转为浏览器可播放的流。',video:'填写浏览器可播放的 HTTP MP4 / WebM 地址。实时 RTSP 请先通过视频网关转换。',ros:'订阅 ROS 主机上的 sensor_msgs/msg/CompressedImage；需要先连接 ROS。'}[type]);
  }
  function watchVideo(epoch) {
    if (!video.requestVideoFrameCallback) return;
    frameCallback = video.requestVideoFrameCallback(() => {
      if (epoch !== generation) return;
      lastFrameAt = performance.now();
      status('画面接收中',true);
      $('twin-camera-timing').textContent = '浏览器正在接收视频帧 · 拍摄至显示的延迟尚未标定';
      watchVideo(epoch);
    });
  }
  $('twin-camera-connect').addEventListener('click',async () => {
    disconnect(); const epoch = generation;
    $('twin-camera-connect').disabled = true; status('连接中…');
    try {
      const type = source.value;
      if (type === 'usb') {
        if (!navigator.mediaDevices?.getUserMedia) throw new Error('当前浏览器环境无法访问相机；请使用 localhost 或 HTTPS，并检查相机权限。');
        const selectedDevice = devices.value;
        const acquired = await navigator.mediaDevices.getUserMedia({audio:false,video:{deviceId:selectedDevice ? {exact:selectedDevice} : undefined,width:{ideal:1280},height:{ideal:720}}});
        if (epoch !== generation) { acquired.getTracks().forEach((track) => track.stop()); return; }
        stream = acquired; video.srcObject = stream;
        stream.getVideoTracks().forEach((track) => track.addEventListener('ended',() => { if (epoch === generation) { disconnect(); message('相机已断开，请检查设备或重新连接。'); } }));
        await video.play();
        if (epoch !== generation) return;
        show('video'); status('本机相机已连接',true);
        message('相机在当前网页显示；点击算法识别或勾选对话附图并发送时，才向你选择的服务发送一帧。');
        await listDevices(); watchVideo(epoch);
      } else if (type === 'ros') {
        if (!client?.connected) throw new Error('请先连接 ROS，再连接图像话题');
        const name = $('twin-camera-topic').value.trim();
        if (!/^\/[a-zA-Z0-9_/]+$/.test(name)) throw new Error('请填写以 / 开头的 ROS 压缩图像话题');
        topic = name;
        client.subscribe(topic,'sensor_msgs/msg/CompressedImage',(sample) => {
          if (epoch !== generation) return;
          if (typeof sample.data !== 'string' || sample.data.length > 28000000) { message('图像不是有效的 base64 压缩帧，或帧过大'); return; }
          if (!live) message('已收到图像；相机与关节反馈独立采集，尚未做时间/空间标定。');
          image.src = `data:${/png/i.test(sample.format) ? 'image/png' : 'image/jpeg'};base64,${sample.data}`;
          lastFrameAt = performance.now();
          show('image'); status('ROS 图像接收中',true);
          const stamp = sample.header?.stamp;
          sourceStamp = stamp ? stamp.sec*1000+stamp.nanosec/1e6 : null;
          const age = stamp ? Date.now()-(stamp.sec*1000+stamp.nanosec/1e6) : NaN;
          $('twin-camera-timing').textContent = Number.isFinite(age) && age >= 0 && age < 10000 ? `图像消息时间差 ${Math.round(age)} ms（需主机时钟同步）` : '已收到 ROS 图像 · 时间差未标定';
        },{throttleRate:80});
        image.onerror = () => { status('图像解码失败'); message('请确认该话题发布 JPEG / PNG CompressedImage'); };
        message(`正在等待 ${topic}。若无画面，请检查该话题是否有发布者。`);
        status('等待 ROS 图像');
      } else {
        const url = new URL($('twin-camera-url').value.trim());
        if (!['http:','https:'].includes(url.protocol)) throw new Error('浏览器无法直接播放 RTSP；请使用 HTTP(S) MJPEG、MP4 / WebM 或 ROS 压缩图像。');
        if (url.username || url.password) throw new Error('请使用相机提供的可访问视频地址，勿在 URL 中携带用户名和密码。');
        if (location.protocol === 'https:' && url.protocol === 'http:') throw new Error('HTTPS 页面会拦截 HTTP 相机流；请使用 HTTPS 视频地址或本机 HTTP 页面。');
        if ($('twin-camera-cors').checked) { image.crossOrigin='anonymous'; video.crossOrigin='anonymous'; }
        if (type === 'mjpeg') {
          image.onload = () => { if (epoch === generation) { lastFrameAt=performance.now(); show('image'); status('网络画面已连接',true); } };
          image.onerror = () => { if (epoch === generation) { show(null); status('网络画面连接失败'); message('请检查地址、相机登录状态及浏览器是否允许访问该来源。'); } };
          image.src = url.href;
        } else {
          video.src = url.href; await video.play();
          if (epoch !== generation) return;
          show('video'); status('网络视频已连接',true); watchVideo(epoch);
        }
        message('网络画面直接由浏览器读取；关节反馈与视频尚未进行时间/空间标定。');
      }
    } catch (error) {
      if (epoch === generation) { disconnect(); status('相机连接失败'); message(error.name === 'NotAllowedError' ? '相机权限被拒绝，请在浏览器中允许此页面使用相机后重试。' : error.message); }
    } finally { $('twin-camera-connect').disabled = false; }
  });
  $('twin-camera-disconnect').addEventListener('click',disconnect);
  source.addEventListener('change',sourceChanged);
  navigator.mediaDevices?.addEventListener('devicechange',() => void listDevices().catch(() => {}));
  client?.addEventListener('status',(event) => {
    if (topic && ['error','closed'].includes(event.detail.state)) { disconnect(); message('ROS 已断开，相机话题订阅已关闭。'); }
  });
  video.addEventListener('error',() => { if (live) { status('视频播放中断'); message('网络视频播放失败，请检查地址及浏览器支持的编码。'); } });
  window.addEventListener('pagehide',disconnect);
  setInterval(() => {
    if (live && lastFrameAt && performance.now()-lastFrameAt > 1500) {
      const age = ((performance.now()-lastFrameAt)/1000).toFixed(1);
      status(`画面已 ${age}s 未更新`);
      $('twin-camera-timing').textContent = '当前画面可能已过期，请检查相机或图像话题';
    }
  },500);
  void listDevices().catch(() => {});
  window.ReBotTwinCamera = {
    capture() {
      if (!live || !lastFrameAt || performance.now()-lastFrameAt>1500) throw new Error('需要正在更新的相机帧；MJPEG无法确认新帧时请使用ROS图像或视频流');
      if (sourceStamp!==null && (Date.now()-sourceStamp>1500 || sourceStamp>Date.now()+500)) throw new Error('相机消息时间戳已过期或时钟未同步');
      const element=video.hidden?image:video,w=video.hidden?image.naturalWidth:video.videoWidth,h=video.hidden?image.naturalHeight:video.videoHeight;
      if(!w||!h)throw new Error('图像尚未解码，请稍后再试');
      const ratio=Math.min(1,960/w,720/h),canvas=document.createElement('canvas');canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);
      try{canvas.getContext('2d').drawImage(element,0,0,canvas.width,canvas.height);const data=canvas.toDataURL('image/jpeg',.8).split(',')[1];return {mime:'image/jpeg',data_base64:data,stamp_ms:sourceStamp??Date.now(),frame_id:'camera_optical_frame'};}
      catch(_){throw new Error('网络相机需允许CORS，并勾选“用于算法取帧”后重新连接；也可改用ROS压缩图像');}
    }
  };
})();
