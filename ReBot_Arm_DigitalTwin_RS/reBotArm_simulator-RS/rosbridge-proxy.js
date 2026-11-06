'use strict';

const net = require('node:net');

function rejectUpgrade(socket, status, reason) {
  if (socket.destroyed) return;
  socket.end(`HTTP/1.1 ${status} ${reason}\r\nConnection: close\r\nContent-Length: 0\r\n\r\n`);
}

function isLoopbackUpgrade(req, { protocol, serverPort }) {
  const host = String(req.headers.host || '').toLowerCase();
  const origin = String(req.headers.origin || '');
  if (!host || !origin || req.headers.upgrade?.toLowerCase() !== 'websocket' ||
      !String(req.headers.connection || '').toLowerCase().split(',').some(value => value.trim() === 'upgrade')) {
    return false;
  }

  let hostUrl;
  let originUrl;
  try {
    hostUrl = new URL(`${protocol}://${host}`);
    originUrl = new URL(origin);
  } catch (_) {
    return false;
  }

  const loopback = ['127.0.0.1', 'localhost', '[::1]'].includes(hostUrl.hostname.toLowerCase());
  const effectivePort = Number(hostUrl.port || (protocol === 'https' ? 443 : 80));
  return loopback && effectivePort === Number(serverPort) && originUrl.origin === hostUrl.origin;
}

function attachRosbridgeProxy(server, {
  serverPort,
  protocol = 'http',
  upstreamHost = '127.0.0.1',
  upstreamPort = 9090
} = {}) {
  if (!server || typeof server.on !== 'function') throw new TypeError('an HTTP server is required');
  if (!Number.isInteger(Number(serverPort)) || !['http', 'https'].includes(protocol)) {
    throw new TypeError('serverPort and http/https protocol are required');
  }

  server.on('upgrade', (req, client, head) => {
    let requestUrl;
    try {
      requestUrl = new URL(req.url, `${protocol}://${req.headers.host || '127.0.0.1'}`);
    } catch (_) {
      rejectUpgrade(client, 400, 'Bad Request');
      return;
    }
    if (requestUrl.pathname !== '/rosbridge') {
      rejectUpgrade(client, 404, 'Not Found');
      return;
    }
    if (!isLoopbackUpgrade(req, { protocol, serverPort })) {
      rejectUpgrade(client, 403, 'Forbidden');
      return;
    }

    const key = String(req.headers['sec-websocket-key'] || '');
    if (Buffer.from(key, 'base64').length !== 16 || req.headers['sec-websocket-version'] !== '13') {
      rejectUpgrade(client, 400, 'Bad Request');
      return;
    }

    const upstream = net.connect({ host: upstreamHost, port: upstreamPort });
    let upgraded = false;
    let responseStarted = false;
    upstream.once('connect', () => {
      const targetPath = `/${requestUrl.search}`;
      const headers = [
        `GET ${targetPath} HTTP/1.1`,
        `Host: ${upstreamHost}:${upstreamPort}`,
        'Upgrade: websocket',
        'Connection: Upgrade',
        `Origin: ${req.headers.origin}`,
        `Sec-WebSocket-Key: ${key}`,
        'Sec-WebSocket-Version: 13'
      ];
      if (req.headers['sec-websocket-protocol']) {
        headers.push(`Sec-WebSocket-Protocol: ${req.headers['sec-websocket-protocol']}`);
      }
      upstream.write(`${headers.join('\r\n')}\r\n\r\n`);
      if (head.length) upstream.write(head);
      client.pipe(upstream);
      upstream.pipe(client);
    });
    let responseHeaders = Buffer.alloc(0);
    upstream.on('data', chunk => {
      responseStarted = true;
      if (upgraded) return;
      responseHeaders = Buffer.concat([responseHeaders, chunk]);
      if (responseHeaders.length > 16384) return;
      const end = responseHeaders.indexOf('\r\n\r\n');
      if (end < 0) return;
      upgraded = /^HTTP\/1\.1 101\b/.test(responseHeaders.subarray(0, end).toString('ascii'));
    });
    upstream.on('error', () => {
      if (!responseStarted) rejectUpgrade(client, 502, 'Bad Gateway');
      else client.destroy();
    });
    client.on('error', () => upstream.destroy());
    client.on('close', () => upstream.destroy());
    upstream.on('close', () => {
      if (!client.destroyed) {
        if (!responseStarted) rejectUpgrade(client, 502, 'Bad Gateway');
        else client.destroy();
      }
    });
  });
}

module.exports = { attachRosbridgeProxy, isLoopbackUpgrade };
