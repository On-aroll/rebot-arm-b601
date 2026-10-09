import { useEffect, useRef, useState } from 'react';
import { WsGatewayClient } from '../wsGatewayClient';
import { useI18n } from '../i18n';
import { normalizeGatewayCapabilities, WS_V1_FALLBACK_CAPABILITIES } from '../lib/wsCapabilities';

export function useGatewayBridge({
  wsUrl,
  channel,
  wsTokenEnabled,
  wsToken,
  pushLog,
  setStateSnapshot,
  onGatewayState,
  onGatewayParams,
}) {
  const { t } = useI18n();
  const [connText, setConnText] = useState(t('conn_disconnected'));
  const [connected, setConnected] = useState(false);
  const [targetTransport, setTargetTransport] = useState('auto');
  const [targetSerialPort, setTargetSerialPort] = useState('');
  const [gatewayCapabilities, setGatewayCapabilities] = useState(WS_V1_FALLBACK_CAPABILITIES);
  const [capabilitiesSource, setCapabilitiesSource] = useState('fallback');
  const clientRef = useRef(null);
  const connectingRef = useRef(false);
  const lastConnectUrlRef = useRef('');
  const connectConfigRef = useRef({ wsUrl, wsTokenEnabled, wsToken });
  const onGatewayStateRef = useRef(onGatewayState);
  const onGatewayParamsRef = useRef(onGatewayParams);
  connectConfigRef.current = { wsUrl, wsTokenEnabled, wsToken };
  onGatewayStateRef.current = onGatewayState;
  onGatewayParamsRef.current = onGatewayParams;

  const buildWsConnectUrl = () => {
    const {
      wsUrl: nextWsUrl,
      wsTokenEnabled: nextWsTokenEnabled,
      wsToken: nextWsToken,
    } = connectConfigRef.current;
    const baseUrl = nextWsUrl.trim();
    if (!nextWsTokenEnabled) return baseUrl;
    if (!nextWsToken.trim()) {
      throw new Error(t('ws_token_required'));
    }
    const url = new URL(baseUrl);
    url.searchParams.set('motorbridge_ws_token', nextWsToken.trim());
    return url.toString();
  };

  const connectNow = (client, statusText = '', forceReconnect = false) => {
    const nextUrl = buildWsConnectUrl();
    const isAlreadyConnected = connected || client?.isConnected?.();
    if (connectingRef.current) return;
    if (isAlreadyConnected && !forceReconnect && nextUrl === lastConnectUrlRef.current) return;
    connectingRef.current = true;
    setConnText(statusText || t('conn_connecting'));
    try {
      lastConnectUrlRef.current = nextUrl;
      client.connect(nextUrl);
    } catch (e) {
      connectingRef.current = false;
      pushLog(e.message || e, 'err');
      throw e;
    }
  };

  const ensureClient = () => {
    if (!clientRef.current) {
      clientRef.current = new WsGatewayClient({
        onEvent: (msg, lvl = 'info') => {
          const text = String(msg || '');
          if (
            text.startsWith('ws msg:') ||
            text.startsWith('async ws message:') ||
            text.startsWith('tx:')
          ) {
            return;
          }
          pushLog(msg, lvl);
        },
        onState: (st) => {
          setStateSnapshot(JSON.stringify(st, null, 2));
          onGatewayStateRef.current?.(st);
        },
        onParams: (data) => {
          setStateSnapshot(JSON.stringify(data, null, 2));
          onGatewayParamsRef.current?.(data);
        },
        onMessage: (msg) => {
          if (msg?.type === 'event' && msg?.event === 'connected' && msg?.data) {
            const d = msg.data;
            const target = d.default_target || {};
            const transport = target.transport ?? d.transport ?? 'auto';
            const serialPort = target.serial_port ?? d.serial_port ?? '';
            setTargetTransport(transport);
            setTargetSerialPort(serialPort);
            setStateSnapshot(
              JSON.stringify(
                {
                  gateway: {
                    router_mode: d.router_mode || 'unknown',
                    connected_bus: d.connected_bus ?? null,
                  },
                  target_default: {
                    vendor: target.vendor ?? d.vendor ?? null,
                    transport,
                    channel: target.channel ?? d.channel ?? null,
                    serial_port: serialPort || null,
                    model: target.model ?? d.model ?? null,
                    motor_id: target.motor_id ?? d.motor_id ?? null,
                    feedback_id: target.feedback_id ?? d.feedback_id ?? null,
                  },
                },
                null,
                2
              )
            );
            return;
          }

          if (msg?.ok === true && msg?.op === 'set_target') {
            const d = msg?.data || {};
            if (typeof d.transport === 'string') setTargetTransport(d.transport);
            if (typeof d.serial_port === 'string') setTargetSerialPort(d.serial_port);
          }
        },
        onOpen: () => {
          connectingRef.current = false;
          setConnected(true);
          setConnText(t('conn_connected'));
          pushLog(
            t('log_connected', {
              url: lastConnectUrlRef.current || connectConfigRef.current.wsUrl,
            }),
            'ok'
          );
          clientRef.current
            ?.send('capabilities', {}, 3000)
            .then((ret) => {
              if (!ret?.ok) throw new Error(ret?.error || 'capabilities failed');
              setGatewayCapabilities(normalizeGatewayCapabilities(ret));
              setCapabilitiesSource('gateway');
              pushLog(t('log_capabilities_gateway'), 'ok');
            })
            .catch((e) => {
              setGatewayCapabilities(WS_V1_FALLBACK_CAPABILITIES);
              setCapabilitiesSource('fallback');
              pushLog(t('log_capabilities_fallback', { err: e.message || e }), 'info');
            });
        },
        onClose: () => {
          connectingRef.current = false;
          setConnected(false);
          setConnText(t('conn_disconnected'));
          pushLog(t('log_disconnected'), 'err');
          // Reconnection requires an explicit click; never replay target/setup.
        },
        onError: () => {
          connectingRef.current = false;
          pushLog(t('log_error'), 'err');
        },
      });
    }
    return clientRef.current;
  };

  const sendCmd = async (op, payload = {}, timeoutMs = 8000, opts = undefined) => {
    const client = ensureClient();
    return client.send(op, payload, timeoutMs, opts);
  };

  const connectWs = () => {
    try {
      connectNow(ensureClient());
    } catch {
      connectingRef.current = false;
      setConnected(false);
      setConnText(t('conn_disconnected'));
    }
  };

  const disconnectWs = () => {
    connectingRef.current = false;
    lastConnectUrlRef.current = '';
    setConnected(false);
    setConnText(t('conn_disconnected'));
    ensureClient().disconnect();
  };

  const closeBusQuietly = async () => {
    try {
      const ret = await sendCmd('close_bus', {}, 3000);
      if (!ret.ok) pushLog(t('log_close_bus_warn', { err: ret.error || 'unknown' }), 'err');
    } catch (e) {
      pushLog(t('log_close_bus_warn', { err: e.message || e }), 'err');
    }
  };

  const setTargetFor = async (vendor, model, motorId, feedbackId, options = {}) => {
    const enableStreams = options.enableStreams !== false;
    const ret = await sendCmd(
      'set_target',
      {
        vendor,
        channel,
        model,
        motor_id: motorId,
        feedback_id: feedbackId,
      },
      10000
    );
    if (!ret.ok) throw new Error(ret.error || 'set_target failed');
    // Selection changes connection routing only. Never write 0x7026, enable active
    // reports, or start streams implicitly. Refresh uses explicit register reads.
    if (enableStreams) pushLog('只读目标已选择；主动上报配置不修改，请使用读取/刷新参数', 'info');
  };

  useEffect(() => () => {
    clientRef.current?.disconnect();
  }, []);

  return {
    connText,
    connected,
    targetTransport,
    targetSerialPort,
    gatewayCapabilities,
    capabilitiesSource,
    connectWs,
    disconnectWs,
    sendCmd,
    closeBusQuietly,
    setTargetFor,
  };
}
