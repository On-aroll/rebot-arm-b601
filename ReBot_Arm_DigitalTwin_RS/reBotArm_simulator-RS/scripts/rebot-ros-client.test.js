'use strict';

const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function loadClient() {
  class MockWebSocket extends EventTarget {
    static CONNECTING = 0;
    static OPEN = 1;
    static CLOSING = 2;
    static CLOSED = 3;
    static instances = [];

    constructor(url) {
      super();
      this.url = url;
      this.readyState = MockWebSocket.CONNECTING;
      this.sent = [];
      MockWebSocket.instances.push(this);
    }

    send(value) { this.sent.push(JSON.parse(value)); }
    open() {
      this.readyState = MockWebSocket.OPEN;
      this.dispatchEvent(new Event('open'));
    }
  }

  class MockCustomEvent extends Event {
    constructor(type, options) {
      super(type);
      this.detail = options && options.detail;
    }
  }

  const window = {
    setTimeout,
    clearTimeout,
    crypto: { getRandomValues: values => values.fill(1) }
  };
  vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../public/js/ros/rebot-ros-client.js'), 'utf8'), {
    window,
    WebSocket: MockWebSocket,
    EventTarget,
    Event,
    CustomEvent: MockCustomEvent,
    Uint8Array,
    Math,
    setTimeout,
    clearTimeout
  });
  return { Client: window.ReBotRosClient, MockWebSocket };
}

test('camera subscriptions can use a latest-only queue without changing other topics', () => {
  const { Client, MockWebSocket } = loadClient();
  const client = new Client({ url: 'ws://localhost/rosbridge' });
  client.subscribe('/camera/image', 'sensor_msgs/msg/CompressedImage', () => {}, {
    throttleRate: 250,
    queueLength: 1
  });
  client.subscribe('/robot/joint_states', 'sensor_msgs/msg/JointState', () => {}, {
    throttleRate: 0
  });
  client.connect();
  MockWebSocket.instances[0].open();

  assert.deepEqual(
    MockWebSocket.instances[0].sent.map(({ topic, throttle_rate, queue_length }) => ({ topic, throttle_rate, queue_length })),
    [
      { topic: '/camera/image', throttle_rate: 250, queue_length: 1 },
      { topic: '/robot/joint_states', throttle_rate: 80, queue_length: undefined }
    ]
  );
});
