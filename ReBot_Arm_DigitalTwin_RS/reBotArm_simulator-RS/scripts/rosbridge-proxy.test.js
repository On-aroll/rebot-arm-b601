'use strict';

const { test } = require('node:test');
const assert = require('node:assert/strict');
const http = require('node:http');
const net = require('node:net');
const { attachRosbridgeProxy } = require('../rosbridge-proxy');

function listen(server) {
  return new Promise((resolve, reject) => {
    server.once('error', reject);
    server.listen(0, '127.0.0.1', () => resolve(server.address().port));
  });
}

function close(server) {
  return new Promise(resolve => server.close(() => resolve()));
}

function websocketRequest(port, path, origin) {
  return new Promise((resolve, reject) => {
    const socket = net.connect({ host: '127.0.0.1', port });
    const key = 'dGhlIHNhbXBsZSBub25jZQ==';
    let received = Buffer.alloc(0);
    let settled = false;
    const timer = setTimeout(() => {
      if (!settled) reject(new Error('upgrade response timed out'));
      socket.destroy();
    }, 1500);
    socket.once('connect', () => socket.write([
      `GET ${path} HTTP/1.1`,
      `Host: 127.0.0.1:${port}`,
      'Upgrade: websocket',
      'Connection: keep-alive, Upgrade',
      `Origin: ${origin}`,
      `Sec-WebSocket-Key: ${key}`,
      'Sec-WebSocket-Version: 13',
      '',
      ''
    ].join('\r\n')));
    socket.on('data', chunk => {
      received = Buffer.concat([received, chunk]);
      const end = received.indexOf('\r\n\r\n');
      if (end < 0 || settled) return;
      settled = true;
      clearTimeout(timer);
      resolve({ status: received.subarray(0, end).toString('ascii').split('\r\n')[0], socket });
    });
    socket.once('error', error => {
      if (!settled) reject(error);
    });
  });
}

test('same-origin loopback WebSocket is proxied to rosbridge root', async t => {
  const forwarded = [];
  let backendConnections = 0;
  const backend = net.createServer(socket => {
    backendConnections++;
    let request = Buffer.alloc(0);
    socket.on('data', chunk => {
      request = Buffer.concat([request, chunk]);
      const end = request.indexOf('\r\n\r\n');
      if (end < 0) return;
      forwarded.push(request.subarray(0, end).toString('ascii'));
      socket.write('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n\r\n');
      socket.removeAllListeners('data');
    });
  });
  const backendPort = await listen(backend);
  const frontend = http.createServer();
  const frontendPort = await listen(frontend);
  attachRosbridgeProxy(frontend, { serverPort: frontendPort, upstreamPort: backendPort });
  t.after(async () => {
    await close(frontend);
    await close(backend);
  });

  const origin = `http://127.0.0.1:${frontendPort}`;
  const success = await websocketRequest(frontendPort, '/rosbridge?source=console', origin);
  assert.match(success.status, / 101 /);
  success.socket.destroy();
  assert.equal(backendConnections, 1);
  assert.match(forwarded[0], /^GET \/\?source=console HTTP\/1\.1/m);
  assert.match(forwarded[0], new RegExp(`Host: 127\\.0\\.0\\.1:${backendPort}`));

  const crossOrigin = await websocketRequest(frontendPort, '/rosbridge', 'http://example.invalid');
  assert.match(crossOrigin.status, / 403 /);
  crossOrigin.socket.destroy();
  const wrongPath = await websocketRequest(frontendPort, '/other', origin);
  assert.match(wrongPath.status, / 404 /);
  wrongPath.socket.destroy();
  assert.equal(backendConnections, 1);
});
