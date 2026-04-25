// Burst Motion - Web Config App (Phase 2.1)
// 自動再接続 + HID テスト + 簡易ルール登録

import { h, render } from 'preact';
import { useState, useEffect, useRef, useCallback } from 'preact/hooks';
import htm from 'htm';
import { SerialClient } from './lib/SerialClient.js';
import { BleClient }    from './lib/BleClient.js';
import { IMUViewer }    from './lib/IMUViewer.js';

const html = htm.bind(h);

// ==========================================================
// グローバル: 接続クライアント (USB / BLE どちらか active)
// ==========================================================
const serialClient = new SerialClient();
const bleClient    = new BleClient();
let activeClient   = null;

// localStorage キー
const LS_TRANSPORT = 'burst_motion_transport';
const LS_AUTOCONNECT = 'burst_motion_autoconnect';

// ==========================================================
// App 本体
// ==========================================================
function App() {
  const [transport, setTransport] = useState(
    localStorage.getItem(LS_TRANSPORT) || 'usb'
  );
  const [connected, setConnected] = useState(false);
  const [deviceInfo, setDeviceInfo] = useState(null);
  const [sensor, setSensor] = useState(null);
  const [streamRate, setStreamRate] = useState(0);
  const [log, setLog] = useState([]);
  const [autoConnect, setAutoConnect] = useState(
    localStorage.getItem(LS_AUTOCONNECT) !== 'false'
  );
  const [authorizedPortCount, setAuthorizedPortCount] = useState(0);
  const [autoTried, setAutoTried] = useState(false);

  // HID test 状態
  const [testKey, setTestKey] = useState('a');
  const [testText, setTestText] = useState('Hello');
  // Rule editor 状態
  const [ruleAccelTh, setRuleAccelTh] = useState(2.5);
  const [ruleKey, setRuleKey] = useState('a');
  const [ruleMode, setRuleMode] = useState('oneshot');

  const logRef = useRef(null);
  const canvasRef = useRef(null);
  const viewerRef = useRef(null);

  // 3D viewer 初期化
  useEffect(() => {
    if (!canvasRef.current) return;
    if (viewerRef.current) return;
    viewerRef.current = new IMUViewer(canvasRef.current);
    return () => {
      if (viewerRef.current) {
        viewerRef.current.destroy();
        viewerRef.current = null;
      }
    };
  }, [canvasRef.current]);

  // sensor 受信時に 3D viewer 更新
  useEffect(() => {
    if (sensor && viewerRef.current && sensor.qw !== undefined) {
      viewerRef.current.setQuaternion(sensor.qw, sensor.qx, sensor.qy, sensor.qz);
    }
  }, [sensor]);

  const addLog = useCallback((dir, text) => {
    setLog((prev) => {
      const next = [...prev, { dir, text, t: Date.now() }];
      if (next.length > 200) next.splice(0, next.length - 200);
      return next;
    });
  }, []);

  // localStorage 永続化
  useEffect(() => { localStorage.setItem(LS_TRANSPORT, transport); }, [transport]);
  useEffect(() => { localStorage.setItem(LS_AUTOCONNECT, autoConnect ? 'true' : 'false'); }, [autoConnect]);

  // 認可済みポート数を確認 (UI 表示用)
  useEffect(() => {
    let cancelled = false;
    serialClient.getAuthorizedPorts().then((ports) => {
      if (!cancelled) setAuthorizedPortCount(ports.length);
    }).catch(() => {});
    return () => { cancelled = true; };
  }, [connected]);

  // 両クライアントに同じハンドラを取付
  useEffect(() => {
    const onConnected = () => setConnected(true);
    const onDisconnected = () => {
      setConnected(false);
      setDeviceInfo(null);
      setSensor(null);
    };
    const onRaw = (ev) => addLog('rx', ev.detail);
    const onSent = (ev) => addLog('tx', JSON.stringify(ev.detail));
    const onSensor = (ev) => setSensor(ev.detail);
    const onDevInfo = (ev) => setDeviceInfo(ev.detail);
    const onPong = (ev) => setDeviceInfo((prev) => ({ ...prev, ...ev.detail }));

    [serialClient, bleClient].forEach((c) => {
      c.addEventListener('connected', onConnected);
      c.addEventListener('disconnected', onDisconnected);
      c.addEventListener('raw', onRaw);
      c.addEventListener('sent', onSent);
      c.addEventListener('type:sensor', onSensor);
      c.addEventListener('type:device.info', onDevInfo);
      c.addEventListener('type:pong', onPong);
      c.addEventListener('type:boot', onPong);
    });

    return () => {
      [serialClient, bleClient].forEach((c) => {
        c.removeEventListener('connected', onConnected);
        c.removeEventListener('disconnected', onDisconnected);
        c.removeEventListener('raw', onRaw);
        c.removeEventListener('sent', onSent);
        c.removeEventListener('type:sensor', onSensor);
        c.removeEventListener('type:device.info', onDevInfo);
        c.removeEventListener('type:pong', onPong);
        c.removeEventListener('type:boot', onPong);
      });
    };
  }, [addLog]);

  // 自動接続: 起動 1 度だけ試行
  useEffect(() => {
    if (autoTried || !autoConnect || connected || transport !== 'usb') return;
    setAutoTried(true);
    (async () => {
      try {
        const ok = await serialClient.autoConnect(115200);
        if (ok) {
          activeClient = serialClient;
          await new Promise((r) => setTimeout(r, 200));
          await activeClient.send({ cmd: 'ping' });
        }
      } catch (err) {
        console.warn('[autoConnect] failed:', err);
      }
    })();
  }, [autoTried, autoConnect, connected, transport]);

  useEffect(() => {
    if (logRef.current) logRef.current.scrollTop = logRef.current.scrollHeight;
  }, [log]);

  const handleConnect = async () => {
    try {
      if (transport === 'usb') {
        activeClient = serialClient;
        await serialClient.connect(115200);
      } else {
        activeClient = bleClient;
        await bleClient.connect();
      }
      await new Promise((r) => setTimeout(r, 200));
      await activeClient.send({ cmd: 'ping' });
    } catch (err) {
      activeClient = null;
      alert(err.message || String(err));
    }
  };

  const handleDisconnect = async () => {
    try {
      if (streamRate > 0 && activeClient) await activeClient.send({ cmd: 'sensor.stream', rate_hz: 0 });
    } catch (_) {}
    if (activeClient === serialClient) await serialClient.disconnect();
    else if (activeClient === bleClient) await bleClient.disconnect();
    activeClient = null;
    setStreamRate(0);
  };

  const handleForget = async () => {
    if (!confirm('全 USB Serial ポート許可を解除します。次回手動で選び直しが必要になります。OK?')) return;
    await serialClient.forgetAllPorts();
    setAuthorizedPortCount(0);
  };

  const sendCmd = async (cmd) => {
    if (!activeClient) return;
    try { await activeClient.send(cmd); }
    catch (err) { alert(err.message); }
  };

  const handlePing       = () => sendCmd({ cmd: 'ping' });
  const handleDeviceInfo = () => sendCmd({ cmd: 'device.info' });
  const handleStreamToggle = async () => {
    const next = streamRate === 0 ? 50 : 0;
    await sendCmd({ cmd: 'sensor.stream', rate_hz: next });
    setStreamRate(next);
  };
  const handleBleStart = () => sendCmd({ cmd: 'ble.start' });
  const handleCalibrate = () => {
    if (!confirm('キャリブレーション中は Controller を 1 秒間静止させてください。OK?')) return;
    sendCmd({ cmd: 'calibrate.simple', duration_ms: 1000 });
  };

  // HID Test
  const hidTest = (action, extra = {}) => sendCmd({ cmd: 'test.hid', action, ...extra });

  // Rule
  const handleAddRule = () => {
    sendCmd({
      cmd: 'rule.add',
      r: {
        id: Date.now() & 0xffff,
        name: `${ruleMode}_${ruleKey}`,
        ui_mode: ruleMode,
        accel_abs_threshold: parseFloat(ruleAccelTh),
        key: ruleKey,
        cooldown_ms: 500,
      },
    });
  };
  const handleListRules  = () => sendCmd({ cmd: 'rule.list' });
  const handleClearRules = () => sendCmd({ cmd: 'rule.clear' });

  const usbSupported = 'serial' in navigator;
  const bleSupported = 'bluetooth' in navigator;

  return html`
  <div class="max-w-6xl mx-auto p-4">
    <header class="flex items-center justify-between mb-4 pb-3 border-b border-slate-200">
      <div>
        <h1 class="text-2xl font-bold">🎮 Burst Motion — 設定アプリ</h1>
        <p class="text-sm text-slate-500">Web Serial (USB) / Web Bluetooth (BLE NUS) 両対応 — Phase 2</p>
      </div>
      <div class="flex items-center gap-3">
        ${!connected ? html`
          <div class="flex gap-1 bg-slate-100 rounded-lg p-1">
            <button onClick=${() => setTransport('usb')}
              class="px-3 py-1 text-sm rounded ${transport === 'usb' ? 'bg-white shadow font-semibold' : 'text-slate-500'}"
              disabled=${!usbSupported}>📡 USB</button>
            <button onClick=${() => setTransport('ble')}
              class="px-3 py-1 text-sm rounded ${transport === 'ble' ? 'bg-white shadow font-semibold' : 'text-slate-500'}"
              disabled=${!bleSupported}>📶 BLE</button>
          </div>
        ` : null}
        ${connected ? html`
          <span class="chip bg-green-100 text-green-700 mr-2">${transport === 'usb' ? 'USB' : 'BLE'} 接続中</span>
          <button onClick=${handleDisconnect} class="px-4 py-2 bg-red-500 hover:bg-red-600 text-white rounded-lg">切断</button>
        ` : html`
          <button onClick=${handleConnect}
            disabled=${(transport === 'usb' && !usbSupported) || (transport === 'ble' && !bleSupported)}
            class="px-4 py-2 bg-blue-500 hover:bg-blue-600 text-white rounded-lg disabled:opacity-50">
            🔌 ${transport === 'usb' ? (authorizedPortCount > 0 ? '前回ポートに接続' : 'USB Serial で接続') : 'BLE NUS で接続'}
          </button>
        `}
      </div>
    </header>

    ${!connected && transport === 'usb' && usbSupported ? html`
      <div class="mb-4 bg-blue-50 text-blue-800 p-3 rounded-lg text-sm flex items-center justify-between flex-wrap gap-2">
        <div>
          ${authorizedPortCount > 0 ? html`
            <span>✅ 認可済みポート ${authorizedPortCount} 個 (${autoConnect ? '次回起動時に自動接続' : '自動接続 OFF'})</span>
          ` : html`
            <span>📌 初回は <b>USB Serial で接続</b> ボタンを押して COM8 を選択。次回以降は自動接続できます。</span>
          `}
        </div>
        <div class="flex items-center gap-2">
          <label class="flex items-center gap-1 cursor-pointer">
            <input type="checkbox" checked=${autoConnect} onChange=${(e) => setAutoConnect(e.target.checked)} />
            自動接続
          </label>
          ${authorizedPortCount > 0 ? html`
            <button onClick=${handleForget} class="text-xs text-red-600 underline">許可を解除</button>
          ` : null}
        </div>
      </div>
    ` : null}

    <div class="grid grid-cols-1 lg:grid-cols-2 gap-4">
      <!-- デバイス情報パネル -->
      <div class="bg-white rounded-lg shadow-sm border border-slate-200 p-4">
        <h2 class="font-semibold mb-3">🔧 デバイス</h2>
        ${deviceInfo ? html`
          <dl class="grid grid-cols-2 gap-x-2 gap-y-1 text-sm">
            <dt class="text-slate-500">FW:</dt><dd>${deviceInfo.fw || '—'}</dd>
            <dt class="text-slate-500">Board:</dt><dd>${deviceInfo.board || '—'}</dd>
            <dt class="text-slate-500">IMU:</dt><dd>${deviceInfo.imu || '—'} ${deviceInfo.imu_ok === false ? '❌' : ''}</dd>
            <dt class="text-slate-500">Uptime:</dt><dd>${deviceInfo.uptime ? (deviceInfo.uptime / 1000).toFixed(1) + 's' : '—'}</dd>
            <dt class="text-slate-500">BLE HID:</dt><dd>${deviceInfo.ble_connected ? '✅' : '—'}</dd>
            <dt class="text-slate-500">BLE NUS:</dt><dd>${deviceInfo.ble_nus_started ? (deviceInfo.ble_nus_connected ? '✅' : '⚪') : '—'}</dd>
          </dl>
        ` : html`<p class="text-sm text-slate-400">未接続、または ping 応答待ち</p>`}
        <div class="mt-3 flex gap-2 flex-wrap">
          <button onClick=${handlePing} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 hover:bg-slate-300 rounded disabled:opacity-40">Ping</button>
          <button onClick=${handleDeviceInfo} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 hover:bg-slate-300 rounded disabled:opacity-40">Info</button>
          <button onClick=${handleBleStart} disabled=${!connected} class="px-3 py-1 text-sm bg-purple-200 hover:bg-purple-300 rounded disabled:opacity-40">BLE Start</button>
          <button onClick=${handleCalibrate} disabled=${!connected} class="px-3 py-1 text-sm bg-orange-200 hover:bg-orange-300 rounded disabled:opacity-40">Calibrate</button>
          <button onClick=${handleStreamToggle} disabled=${!connected} class="px-3 py-1 text-sm bg-emerald-200 hover:bg-emerald-300 rounded disabled:opacity-40">
            ${streamRate === 0 ? 'Stream ON' : 'Stream OFF'}
          </button>
        </div>
      </div>

      <!-- センサー + 3D ビュー -->
      <div class="bg-white rounded-lg shadow-sm border border-slate-200 p-4">
        <div class="flex justify-between items-center mb-2">
          <h2 class="font-semibold">📊 センサー / 🎨 3D 姿勢</h2>
          <button onClick=${() => viewerRef.current?.reset()} class="text-xs px-2 py-1 bg-slate-200 rounded">3D Reset</button>
        </div>
        <canvas ref=${canvasRef} style="width:100%; height:200px; display:block; border-radius:6px; background:#f1f5f9;"></canvas>
        ${sensor ? html`
          <div class="grid grid-cols-3 gap-2 text-xs font-mono mt-2">
            <div class="bg-sky-50 rounded p-2">
              <div class="text-slate-500">Accel [m/s²]</div>
              <div>X: ${sensor.ax?.toFixed(2)}</div>
              <div>Y: ${sensor.ay?.toFixed(2)}</div>
              <div>Z: ${sensor.az?.toFixed(2)}</div>
            </div>
            <div class="bg-pink-50 rounded p-2">
              <div class="text-slate-500">Gyro [°/s]</div>
              <div>X: ${sensor.gx?.toFixed(1)}</div>
              <div>Y: ${sensor.gy?.toFixed(1)}</div>
              <div>Z: ${sensor.gz?.toFixed(1)}</div>
            </div>
            <div class="bg-emerald-50 rounded p-2">
              <div class="text-slate-500">Euler [°]</div>
              <div>R: ${sensor.roll?.toFixed(1)}</div>
              <div>P: ${sensor.pitch?.toFixed(1)}</div>
              <div>Y: ${sensor.yaw?.toFixed(1)}</div>
            </div>
          </div>
        ` : html`<p class="text-xs text-slate-400 mt-2 text-center">${streamRate === 0 ? 'Stream OFF (3D は QW/Q* 受信で動作)' : '待機中…'}</p>`}
      </div>

      <!-- HID テスト -->
      <div class="bg-white rounded-lg shadow-sm border border-slate-200 p-4">
        <h2 class="font-semibold mb-3">🧪 HID 直接テスト</h2>
        <p class="text-xs text-slate-500 mb-2">事前に「BLE Start」を押し、PC で Burst Motion をペアリング</p>
        <div class="flex items-center gap-2 mb-2">
          <label class="text-sm">キー:</label>
          <input type="text" value=${testKey} onInput=${(e) => setTestKey(e.target.value)}
            maxlength="1" class="border rounded px-2 py-1 w-12 text-center font-mono" />
          <button onClick=${() => hidTest('fire', { key: testKey })}
            disabled=${!connected} class="px-3 py-1 text-sm bg-blue-200 hover:bg-blue-300 rounded disabled:opacity-40">
            Press '${testKey}'
          </button>
        </div>
        <div class="flex items-center gap-2 mb-2">
          <label class="text-sm">テキスト:</label>
          <input type="text" value=${testText} onInput=${(e) => setTestText(e.target.value)}
            class="border rounded px-2 py-1 flex-1 font-mono text-sm" />
          <button onClick=${() => hidTest('text', { text: testText })}
            disabled=${!connected} class="px-3 py-1 text-sm bg-blue-200 hover:bg-blue-300 rounded disabled:opacity-40">
            Type
          </button>
        </div>
        <div class="flex gap-2 flex-wrap">
          <button onClick=${() => hidTest('mouse_move', { dx: 50, dy: 0 })} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 rounded disabled:opacity-40">→ Mouse 50,0</button>
          <button onClick=${() => hidTest('mouse_move', { dx: -50, dy: 0 })} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 rounded disabled:opacity-40">← Mouse -50,0</button>
          <button onClick=${() => hidTest('mouse_click', { button: 'left' })} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 rounded disabled:opacity-40">Click</button>
        </div>
      </div>

      <!-- 簡易ルール -->
      <div class="bg-white rounded-lg shadow-sm border border-slate-200 p-4">
        <h2 class="font-semibold mb-3">🎯 簡易アクションルール (試作)</h2>
        <p class="text-xs text-slate-500 mb-2">加速度トリガーで HID キー発火</p>
        <div class="flex items-center gap-2 mb-2 flex-wrap">
          <label class="text-sm">モード:</label>
          <select value=${ruleMode} onChange=${(e) => setRuleMode(e.target.value)} class="border rounded px-2 py-1 text-sm">
            <option value="oneshot">ONESHOT (1発)</option>
            <option value="hold_start_only">HOLD_START_ONLY (押下保持)</option>
          </select>
        </div>
        <div class="flex items-center gap-2 mb-2 flex-wrap">
          <label class="text-sm">加速度しきい値 [g]:</label>
          <input type="number" min="0.5" max="10" step="0.1" value=${ruleAccelTh}
            onInput=${(e) => setRuleAccelTh(e.target.value)} class="border rounded px-2 py-1 w-20 font-mono text-sm" />
          <label class="text-sm ml-2">キー:</label>
          <input type="text" value=${ruleKey} onInput=${(e) => setRuleKey(e.target.value)}
            maxlength="1" class="border rounded px-2 py-1 w-12 text-center font-mono" />
        </div>
        <div class="flex gap-2 flex-wrap">
          <button onClick=${handleAddRule} disabled=${!connected} class="px-3 py-1 text-sm bg-emerald-200 hover:bg-emerald-300 rounded disabled:opacity-40">Add Rule</button>
          <button onClick=${handleListRules} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 rounded disabled:opacity-40">List</button>
          <button onClick=${handleClearRules} disabled=${!connected} class="px-3 py-1 text-sm bg-red-200 hover:bg-red-300 rounded disabled:opacity-40">Clear All</button>
        </div>
      </div>
    </div>

    <!-- ログペイン -->
    <div class="mt-4 bg-slate-900 rounded-lg shadow-sm p-3">
      <div class="flex items-center justify-between mb-2">
        <h2 class="font-semibold text-slate-200">📜 通信ログ</h2>
        <button onClick=${() => setLog([])} class="text-xs text-slate-400 hover:text-slate-200">Clear</button>
      </div>
      <div ref=${logRef} class="h-56 overflow-y-auto bg-slate-950 rounded p-2">
        ${log.map((l) => html`
          <div class="log-line ${l.dir === 'tx' ? 'text-yellow-300' : 'text-green-300'}">
            ${l.dir === 'tx' ? '→ ' : '← '}${l.text}
          </div>
        `)}
      </div>
    </div>

    <footer class="mt-4 text-center text-xs text-slate-400">
      Burst Motion — Phase 2.1 | USB:115200 / BLE NUS | JSON Lines | Auto-reconnect 対応
    </footer>
  </div>
  `;
}

render(html`<${App} />`, document.getElementById('app'));
