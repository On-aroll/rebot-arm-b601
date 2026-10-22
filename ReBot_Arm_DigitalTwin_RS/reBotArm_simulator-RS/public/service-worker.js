const CACHE_NAME = 'rebot-arm-rs-pwa-v165-preview';
const APP_SHELL = [
  '/',
  '/index.html',
  '/startup.html',
  '/css/project-shutdown.css?v=20261018-exit156',
  '/js/project-shutdown.js?v=20261018-exit156',
  '/css/startup.css?v=20261017-start02',
  '/js/startup.js?v=20261017-start02',
  '/manifest.webmanifest',
  '/favicon.png',
  '/css/rebot-sim.css?v=20260813-rs-guide43',
  '/css/twin-console.css?v=20261018-speed159',
  '/js/twin-input.js?v=20261018-speed159',
  '/js/operation-hub.js?v=20261018-sections157',
  '/js/twin-feedback-display.js?v=20261018-recover155',
  '/js/twin-guides.js?v=20261018-recover155',
  '/js/twin-console.js?v=20261020-preview165',
  '/js/twin-algorithm.js?v=20261017-mit01',
  '/js/twin-camera.js?v=20261016-workbench13',
  '/js/fixed-pick.js?v=20261019-grasp160',
  '/js/vision-replay.js?v=20261019-offline162',
  '/js/grasp-recording.js?v=20261019-offline162',
  '/js/twin-offline-tools.js?v=20261019-offline162',
  '/js/twin-workbench.js?v=20261019-tcpdrop163',
  '/js/quick-start.js?v=20261015-rs-quick01',
  '/js/pwa.js?v=20260812-rs-ctrl32',
  '/js/i18n.js?v=20261016-feedback01',
  '/js/rebot-sim.js?v=20261019-tcpdrop163',
  '/js/wrist-camera.js?v=20261009-rs-camera03',
  '/js/ros/rebot-ros-client.js?v=20260921-rs-preset-smooth02',
  '/js/control-mode.js?v=20261018-rs-ctrl34',
  '/js/ros/rebot-ros-ui.js?v=20261016-workbench13',
  '/js/ros/leader-model.js?v=20261007-leader02',
  '/js/ros/rebot-leader-ui.js?v=20261007-leader02',
  '/css/leader-teleop.css?v=20261007-leader02',
  '/models/leader-arm102/urdf/leader.urdf',
  '/models/leader-arm102/meshes/base_link.STL',
  '/models/leader-arm102/meshes/link1.STL',
  '/models/leader-arm102/meshes/link2.STL',
  '/models/leader-arm102/meshes/link3.STL',
  '/models/leader-arm102/meshes/link4.STL',
  '/models/leader-arm102/meshes/link5.STL',
  '/models/leader-arm102/meshes/link6.STL',
  '/models/leader-arm102/meshes/link7_left.STL',
  '/models/leader-arm102/meshes/link7_right.STL',
  '/js/rebot-llm.js?v=20260813-rs-guide43',
  '/lib/three-r128.min.js',
  '/lib/STLLoader-umd.js',
  '/lib/ColladaLoader.js',
  '/models/wrist-cameras/urdf/d405.urdf',
  '/models/wrist-cameras/urdf/d435i.urdf',
  '/models/wrist-cameras/urdf/gemini2.urdf',
  '/models/wrist-cameras/urdf/uvc32.urdf',
  '/models/wrist-cameras/meshes/mounts/UVC32_mount.stl',
  '/models/wrist-cameras/meshes/uvc32/board.stl',
  '/models/wrist-cameras/meshes/uvc32/lens.stl',
  '/models/wrist-cameras/meshes/uvc32/glass.stl',
  '/models/wrist-cameras/meshes/gemini2/base_link.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_botter_screw_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_color_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_color_optical_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_depth_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_infra1_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_infra1_optical_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_infra2_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_infra2_optical_frame.STL',
  '/models/wrist-cameras/meshes/mounts/D405_305_Mount.stl',
  '/models/wrist-cameras/meshes/mounts/D435_Gemini2_Mount.stl',
  '/models/wrist-cameras/meshes/realsense/d405.stl',
  '/models/wrist-cameras/meshes/realsense/d435.dae',
  '/lib/URDFLoader.js',
  '/js/motorbridge/rebot-motorbridge-client.js?v=20260812-rs-ctrl32',
  '/js/motorbridge/rebot-motorbridge-ui.js?v=20261018-single-axis-live04'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then((cache) => cache.addAll(APP_SHELL))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(
        keys.filter((key) => key !== CACHE_NAME).map((key) => caches.delete(key))
      ))
      .then(() => self.clients.claim())
  );
});

self.addEventListener('fetch', (event) => {
  const request = event.request;
  const url = new URL(request.url);

  if (request.method !== 'GET' || url.origin !== self.location.origin) return;

  if (url.pathname.startsWith('/api/')) {
    event.respondWith(fetch(request).catch(function () {
      return new Response('{"error":"network error"}', {
        status: 502,
        headers: { 'Content-Type': 'application/json' }
      });
    }));
    return;
  }

  const isControlAsset = request.mode === 'navigate' ||
    url.pathname.endsWith('.html') ||
    url.pathname.endsWith('.js') ||
    url.pathname.endsWith('.css');

  if (isControlAsset) {
    event.respondWith(
      fetch(request)
        .then((response) => {
          if (!response || response.status !== 200) return response;
          const copy = response.clone();
          caches.open(CACHE_NAME).then((cache) => cache.put(request, copy));
          return response;
        })
        .catch(() => caches.match(request).then((cached) => {
          if (cached) return cached;
          if (request.mode === 'navigate') return caches.match('/index.html');
          return Response.error();
        }))
    );
    return;
  }

  event.respondWith(
    caches.match(request).then((cached) => {
      if (cached) return cached;
      return fetch(request).then((response) => {
        if (!response || response.status !== 200) return response;
        const copy = response.clone();
        caches.open(CACHE_NAME).then((cache) => cache.put(request, copy));
        return response;
      });
    })
  );
});
