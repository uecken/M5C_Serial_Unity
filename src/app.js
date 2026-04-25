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
  const [hidDelayMs, setHidDelayMs] = useState(1000);
  const [hidCountdown, setHidCountdown] = useState(0);  // 0 = idle, >0 = カウントダウン中
  // Rule editor 状態
  const [ruleAccelTh, setRuleAccelTh] = useState(2.5);
  const [ruleAccelEnabled, setRuleAccelEnabled] = useState(true);
  const [ruleKey, setRuleKey] = useState('a');
  const [ruleMode, setRuleMode] = useState('oneshot');
  const [ruleName, setRuleName] = useState('');
  const [ruleList, setRuleList] = useState([]);   // FW から取得した rule 一覧
  const [triggerFlash, setTriggerFlash] = useState(null);  // {id, phase, name, t}
  const [watchEnabled, setWatchEnabled] = useState(false);

  // 姿勢キャプチャ (Euler [r,p,y]、tol [r,p,y])
  const [startPosture, setStartPosture] = useState(null);   // {euler:[r,p,y], tol:[r,p,y]} | null
  const [endPosture, setEndPosture] = useState(null);
  const [postureTol, setPostureTol] = useState(15);   // ±degrees

  // 修飾キー
  const [modCtrl,  setModCtrl ] = useState(false);
  const [modShift, setModShift] = useState(false);
  const [modAlt,   setModAlt  ] = useState(false);
  const [modGui,   setModGui  ] = useState(false);   // Win / Cmd

  // Profile 状態
  const [profileList, setProfileList] = useState([]);
  const [profileName, setProfileName] = useState('default');
  const [activeProfile, setActiveProfile] = useState('');

  // 6 点キャリブ ウィザード
  const [calib6Step, setCalib6Step] = useState(-1);  // -1=idle、0..5=待機、6=完了待機
  const [calib6Instruction, setCalib6Instruction] = useState('');
  const [calib6Result, setCalib6Result] = useState(null);

  const logRef = useRef(null);
  const canvasRef = useRef(null);
  const viewerRef = useRef(null);
  const hidTimerRef = useRef(null);

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

    const onRuleList = (ev) => setRuleList(ev.detail.rules || []);
    const onProfileList = (ev) => {
      setProfileList(ev.detail.profiles || []);
      if (ev.detail.active) setActiveProfile(ev.detail.active);
    };
    const onProfileActive = (ev) => setActiveProfile(ev.detail.name || '');
    const onCalibStep = (ev) => {
      setCalib6Step(ev.detail.step);
      setCalib6Instruction(ev.detail.instruction || '');
    };
    const onCalibAck = (ev) => {
      const d = ev.detail;
      if (d.cmd === 'calibrate.full.finish' && d.ok) {
        setCalib6Result(d);
        setCalib6Step(-1);
      }
    };
    const onTriggerHit = (ev) => {
      const d = ev.detail;
      setTriggerFlash({ id: d.id, name: d.rule_name, phase: d.phase, t: Date.now() });
      // 1 秒後にフラッシュを消す
      setTimeout(() => setTriggerFlash((cur) => cur && cur.t === d.t ? null : cur), 1000);
    };
    const onAck = (ev) => {
      const d = ev.detail;
      if (d.cmd === 'rule.add' || d.cmd === 'rule.clear' || d.cmd === 'rule.remove') {
        if (activeClient) activeClient.send({ cmd: 'rule.list' }).catch(() => {});
      }
      if (d.cmd === 'profile.save' || d.cmd === 'profile.delete' || d.cmd === 'profile.load') {
        if (activeClient) activeClient.send({ cmd: 'profile.list' }).catch(() => {});
      }
    };

    [serialClient, bleClient].forEach((c) => {
      c.addEventListener('connected', onConnected);
      c.addEventListener('disconnected', onDisconnected);
      c.addEventListener('raw', onRaw);
      c.addEventListener('sent', onSent);
      c.addEventListener('type:sensor', onSensor);
      c.addEventListener('type:device.info', onDevInfo);
      c.addEventListener('type:pong', onPong);
      c.addEventListener('type:boot', onPong);
      c.addEventListener('type:rule.list', onRuleList);
      c.addEventListener('type:trigger.hit', onTriggerHit);
      c.addEventListener('type:ack', onAck);
      c.addEventListener('type:profile.list', onProfileList);
      c.addEventListener('type:profile.active', onProfileActive);
      c.addEventListener('type:calibration.step', onCalibStep);
      c.addEventListener('type:ack', onCalibAck);
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
        c.removeEventListener('type:rule.list', onRuleList);
        c.removeEventListener('type:trigger.hit', onTriggerHit);
        c.removeEventListener('type:ack', onAck);
        c.removeEventListener('type:profile.list', onProfileList);
        c.removeEventListener('type:profile.active', onProfileActive);
        c.removeEventListener('type:calibration.step', onCalibStep);
        c.removeEventListener('type:ack', onCalibAck);
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
    if (!confirm('簡易キャリブレーション (1秒静止) を実行します。OK?')) return;
    sendCmd({ cmd: 'calibrate.simple', duration_ms: 1000 });
  };
  // 6 点キャリブレーション
  const handleCalib6Start = () => {
    setCalib6Result(null);
    sendCmd({ cmd: 'calibrate.full.start' });
  };
  const handleCalib6Capture = () => {
    sendCmd({ cmd: 'calibrate.full.capture' });
  };
  const handleCalib6Finish = () => {
    sendCmd({ cmd: 'calibrate.full.finish' });
  };
  const handleCalib6Cancel = () => {
    sendCmd({ cmd: 'calibrate.full.cancel' });
    setCalib6Step(-1);
  };

  // HID Test
  // マウス系 (mouse_move / mouse_click) は即実行 (動きが見えてわかりやすい)
  // キーボード系 (fire / text) は遅延付き (フォーカス先のアプリに切り替える時間)
  const hidTest = (action, extra = {}) => {
    const isMouse = action === 'mouse_move' || action === 'mouse_click';
    // 既に走ってるカウントダウンがあればキャンセル
    if (hidTimerRef.current) {
      clearInterval(hidTimerRef.current);
      hidTimerRef.current = null;
    }
    const delay = isMouse ? 0 : (parseInt(hidDelayMs) || 0);
    if (delay <= 0) {
      sendCmd({ cmd: 'test.hid', action, ...extra });
      return;
    }
    // カウントダウン開始 (キーボード系のみ)
    setHidCountdown(delay);
    const startTime = Date.now();
    hidTimerRef.current = setInterval(() => {
      const remaining = Math.max(0, delay - (Date.now() - startTime));
      setHidCountdown(remaining);
      if (remaining <= 0) {
        clearInterval(hidTimerRef.current);
        hidTimerRef.current = null;
        setHidCountdown(0);
        sendCmd({ cmd: 'test.hid', action, ...extra });
      }
    }, 50);
  };
  const hidTestCancel = () => {
    if (hidTimerRef.current) {
      clearInterval(hidTimerRef.current);
      hidTimerRef.current = null;
    }
    setHidCountdown(0);
  };

  // unmount 時にタイマークリア
  useEffect(() => () => {
    if (hidTimerRef.current) clearInterval(hidTimerRef.current);
  }, []);

  // 姿勢キャプチャ (現在の sensor から)
  const captureStartPosture = () => {
    if (!sensor) { alert('センサーストリーム ON にしてから姿勢を取得してください'); return; }
    const tol = parseInt(postureTol) || 15;
    setStartPosture({
      euler: [sensor.roll, sensor.pitch, sensor.yaw],
      euler_tol: [tol, tol, tol * 6]  // yaw は許容大きめ
    });
  };
  const captureEndPosture = () => {
    if (!sensor) { alert('センサーストリーム ON にしてから姿勢を取得してください'); return; }
    const tol = parseInt(postureTol) || 15;
    setEndPosture({
      euler: [sensor.roll, sensor.pitch, sensor.yaw],
      euler_tol: [tol, tol, tol * 6]
    });
  };
  const clearStartPosture = () => setStartPosture(null);
  const clearEndPosture   = () => setEndPosture(null);

  // 修飾キービット (BleCombo の HID キーコード規約に近い形)
  // bit0=Ctrl, bit1=Shift, bit2=Alt, bit3=GUI(Win)
  const buildModifiers = () => {
    let m = 0;
    if (modCtrl)  m |= 0x01;
    if (modShift) m |= 0x02;
    if (modAlt)   m |= 0x04;
    if (modGui)   m |= 0x08;
    return m;
  };

  // Rule
  const handleAddRule = () => {
    const r = {
      id: Date.now() & 0xffff,
      name: ruleName || `${ruleMode}_${ruleKey}`,
      ui_mode: ruleMode,
      key: ruleKey,
      cooldown_ms: 500,
    };
    if (ruleAccelEnabled && parseFloat(ruleAccelTh) > 0) {
      r.accel_abs_threshold = parseFloat(ruleAccelTh);
    }
    if (startPosture) {
      r.posture = { euler: startPosture.euler, euler_tol: startPosture.euler_tol };
    }
    if (ruleMode === 'hold_start_end' && endPosture) {
      r.end_posture = { euler: endPosture.euler, euler_tol: endPosture.euler_tol };
    }
    const mods = buildModifiers();
    if (mods > 0) r.modifiers = mods;
    sendCmd({ cmd: 'rule.add', r });
  };
  const handleListRules  = () => sendCmd({ cmd: 'rule.list' });
  const handleClearRules = () => {
    if (!confirm('登録済みルールをすべて削除します。OK?')) return;
    sendCmd({ cmd: 'rule.clear' });
  };
  const handleToggleWatch = () => {
    const next = !watchEnabled;
    setWatchEnabled(next);
    sendCmd({ cmd: 'watch.set', enabled: next });
  };
  // 接続成功時に自動で rule.list + profile.list + watch を要求
  useEffect(() => {
    if (connected && activeClient) {
      const t = setTimeout(() => {
        activeClient.send({ cmd: 'rule.list' }).catch(() => {});
        activeClient.send({ cmd: 'profile.list' }).catch(() => {});
        activeClient.send({ cmd: 'watch.set', enabled: true }).catch(() => {});
        setWatchEnabled(true);
      }, 500);
      return () => clearTimeout(t);
    }
  }, [connected]);

  // Profile 操作
  const handleProfileSave = () => {
    const name = profileName.trim();
    if (!name) { alert('プロファイル名を入力してください'); return; }
    sendCmd({ cmd: 'profile.save', name });
  };
  const handleProfileLoad = (name) => {
    sendCmd({ cmd: 'profile.load', name });
    setProfileName(name);
  };
  const handleProfileDelete = (name) => {
    if (!confirm(`プロファイル '${name}' を削除します。OK?`)) return;
    sendCmd({ cmd: 'profile.delete', name });
  };

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

        <!-- 遅延設定 -->
        <div class="flex items-center gap-2 mb-3 p-2 bg-amber-50 rounded">
          <label class="text-sm font-semibold">⏱ 送信遅延:</label>
          <input type="number" min="0" max="10000" step="100" value=${hidDelayMs}
            onInput=${(e) => setHidDelayMs(e.target.value)}
            class="border rounded px-2 py-1 w-20 font-mono text-sm" />
          <span class="text-xs text-slate-500">ms (押下後この時間待ってから送信、メモ帳等にフォーカス移動用)</span>
        </div>

        ${hidCountdown > 0 ? html`
          <div class="mb-3 p-2 bg-orange-100 border border-orange-300 rounded flex items-center justify-between">
            <span class="text-sm font-semibold text-orange-700">🚀 ${(hidCountdown/1000).toFixed(1)}秒後に送信</span>
            <button onClick=${hidTestCancel} class="px-2 py-0.5 text-xs bg-red-300 hover:bg-red-400 rounded">キャンセル</button>
          </div>
        ` : null}

        <div class="flex items-center gap-2 mb-2">
          <label class="text-sm">キー:</label>
          <input type="text" value=${testKey} onInput=${(e) => setTestKey(e.target.value)}
            maxlength="1" class="border rounded px-2 py-1 w-12 text-center font-mono" />
          <button onClick=${() => hidTest('fire', { key: testKey })}
            disabled=${!connected || hidCountdown > 0} class="px-3 py-1 text-sm bg-blue-200 hover:bg-blue-300 rounded disabled:opacity-40">
            Press '${testKey}'
          </button>
        </div>
        <div class="flex items-center gap-2 mb-2">
          <label class="text-sm">テキスト:</label>
          <input type="text" value=${testText} onInput=${(e) => setTestText(e.target.value)}
            class="border rounded px-2 py-1 flex-1 font-mono text-sm" />
          <button onClick=${() => hidTest('text', { text: testText })}
            disabled=${!connected || hidCountdown > 0} class="px-3 py-1 text-sm bg-blue-200 hover:bg-blue-300 rounded disabled:opacity-40">
            Type
          </button>
        </div>
        <div class="flex gap-2 flex-wrap items-center">
          <span class="text-xs text-slate-500 mr-1">マウス (即実行):</span>
          <button onClick=${() => hidTest('mouse_move', { dx: 50, dy: 0 })} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 hover:bg-slate-300 rounded disabled:opacity-40">→ 50,0</button>
          <button onClick=${() => hidTest('mouse_move', { dx: -50, dy: 0 })} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 hover:bg-slate-300 rounded disabled:opacity-40">← -50,0</button>
          <button onClick=${() => hidTest('mouse_move', { dx: 0, dy: 50 })} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 hover:bg-slate-300 rounded disabled:opacity-40">↓ 0,50</button>
          <button onClick=${() => hidTest('mouse_move', { dx: 0, dy: -50 })} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 hover:bg-slate-300 rounded disabled:opacity-40">↑ 0,-50</button>
          <button onClick=${() => hidTest('mouse_click', { button: 'left' })} disabled=${!connected} class="px-3 py-1 text-sm bg-slate-200 hover:bg-slate-300 rounded disabled:opacity-40">Click</button>
        </div>
      </div>

      <!-- 簡易ルール -->
      <div class="bg-white rounded-lg shadow-sm border border-slate-200 p-4">
        <div class="flex justify-between items-center mb-3">
          <h2 class="font-semibold">🎯 アクションルール</h2>
          <label class="flex items-center gap-1 cursor-pointer text-xs">
            <input type="checkbox" checked=${watchEnabled} onChange=${handleToggleWatch} disabled=${!connected} />
            発火通知 (watch)
          </label>
        </div>

        <!-- 発火フラッシュ -->
        ${triggerFlash ? html`
          <div class="mb-3 p-2 bg-yellow-100 border border-yellow-400 rounded animate-pulse text-sm font-semibold text-yellow-800">
            🔥 #${triggerFlash.id} ${triggerFlash.name} ${triggerFlash.phase}
          </div>
        ` : null}

        <!-- 登録済みルール一覧 -->
        ${ruleList.length > 0 ? html`
          <div class="mb-3 max-h-32 overflow-y-auto border rounded">
            <table class="w-full text-xs">
              <thead class="bg-slate-100 sticky top-0">
                <tr>
                  <th class="px-2 py-1 text-left">ID</th>
                  <th class="px-2 py-1 text-left">Name</th>
                  <th class="px-2 py-1 text-center">States</th>
                  <th class="px-2 py-1 text-center">Loop</th>
                </tr>
              </thead>
              <tbody>
                ${ruleList.map((r) => html`
                  <tr class="${triggerFlash && triggerFlash.id === r.id ? 'bg-yellow-100' : ''} border-t">
                    <td class="px-2 py-1 font-mono">${r.id}</td>
                    <td class="px-2 py-1">${r.name}</td>
                    <td class="px-2 py-1 text-center">${r.states_count}</td>
                    <td class="px-2 py-1 text-center">${r.loop ? '🔁' : '➡️'}</td>
                  </tr>
                `)}
              </tbody>
            </table>
          </div>
        ` : html`<p class="text-xs text-slate-400 mb-2">未登録 — 下のフォームから追加</p>`}

        <!-- 追加フォーム -->
        <div class="border-t pt-2 mt-2 space-y-2">
          <div class="flex items-center gap-2 flex-wrap">
            <label class="text-sm">名前:</label>
            <input type="text" value=${ruleName} onInput=${(e) => setRuleName(e.target.value)}
              placeholder="(自動)" class="border rounded px-2 py-1 text-sm flex-1 max-w-32" />
            <label class="text-sm">モード:</label>
            <select value=${ruleMode} onChange=${(e) => setRuleMode(e.target.value)} class="border rounded px-2 py-1 text-sm">
              <option value="oneshot">ONESHOT (1発)</option>
              <option value="hold_start_only">HOLD_START_ONLY (押下保持)</option>
              <option value="hold_start_end">HOLD_START_END (開始/終了 別姿勢)</option>
            </select>
          </div>

          <!-- 加速度トリガ -->
          <div class="flex items-center gap-2 flex-wrap">
            <label class="text-sm flex items-center gap-1">
              <input type="checkbox" checked=${ruleAccelEnabled} onChange=${(e) => setRuleAccelEnabled(e.target.checked)} />
              加速度 ≥
            </label>
            <input type="number" min="0" max="10" step="0.1" value=${ruleAccelTh}
              disabled=${!ruleAccelEnabled}
              onInput=${(e) => setRuleAccelTh(e.target.value)} class="border rounded px-2 py-1 w-16 font-mono text-sm disabled:opacity-40" />
            <span class="text-sm">g</span>
          </div>

          <!-- 姿勢トリガ -->
          <div class="border rounded p-2 bg-slate-50">
            <div class="text-xs font-semibold text-slate-600 mb-1">姿勢条件 (任意)</div>
            <div class="flex items-center gap-2 mb-1 flex-wrap">
              <span class="text-xs">許容 ±</span>
              <input type="number" min="5" max="90" step="5" value=${postureTol}
                onInput=${(e) => setPostureTol(e.target.value)} class="border rounded px-1 py-0.5 w-12 font-mono text-xs" />
              <span class="text-xs">°</span>
            </div>
            <div class="flex items-center gap-2 mb-1 flex-wrap text-xs">
              <button onClick=${captureStartPosture} disabled=${!connected || !sensor}
                class="px-2 py-0.5 bg-cyan-200 hover:bg-cyan-300 rounded disabled:opacity-40">📷 開始姿勢</button>
              ${startPosture ? html`
                <span class="font-mono text-cyan-700">R:${startPosture.euler[0].toFixed(0)} P:${startPosture.euler[1].toFixed(0)} Y:${startPosture.euler[2].toFixed(0)}</span>
                <button onClick=${clearStartPosture} class="text-xs text-red-600 hover:underline">×</button>
              ` : html`<span class="text-slate-400">未取得 (Stream ON で取得可)</span>`}
            </div>
            ${ruleMode === 'hold_start_end' ? html`
              <div class="flex items-center gap-2 flex-wrap text-xs">
                <button onClick=${captureEndPosture} disabled=${!connected || !sensor}
                  class="px-2 py-0.5 bg-orange-200 hover:bg-orange-300 rounded disabled:opacity-40">📷 終了姿勢</button>
                ${endPosture ? html`
                  <span class="font-mono text-orange-700">R:${endPosture.euler[0].toFixed(0)} P:${endPosture.euler[1].toFixed(0)} Y:${endPosture.euler[2].toFixed(0)}</span>
                  <button onClick=${clearEndPosture} class="text-xs text-red-600 hover:underline">×</button>
                ` : html`<span class="text-slate-400">未取得</span>`}
              </div>
            ` : null}
          </div>

          <!-- 出力アクション -->
          <div class="border rounded p-2 bg-emerald-50">
            <div class="text-xs font-semibold text-slate-600 mb-1">出力 HID キー</div>
            <div class="flex items-center gap-1 mb-1 flex-wrap text-xs">
              <span>修飾:</span>
              <label class="flex items-center gap-0.5"><input type="checkbox" checked=${modCtrl} onChange=${(e)=>setModCtrl(e.target.checked)} />Ctrl</label>
              <label class="flex items-center gap-0.5"><input type="checkbox" checked=${modShift} onChange=${(e)=>setModShift(e.target.checked)} />Shift</label>
              <label class="flex items-center gap-0.5"><input type="checkbox" checked=${modAlt} onChange=${(e)=>setModAlt(e.target.checked)} />Alt</label>
              <label class="flex items-center gap-0.5"><input type="checkbox" checked=${modGui} onChange=${(e)=>setModGui(e.target.checked)} />Win</label>
            </div>
            <div class="flex items-center gap-2">
              <span class="text-xs">+ キー:</span>
              <input type="text" value=${ruleKey} onInput=${(e) => setRuleKey(e.target.value)}
                maxlength="1" class="border rounded px-2 py-1 w-12 text-center font-mono" />
              <span class="text-xs text-slate-500">
                ${modCtrl?'Ctrl+':''}${modShift?'Shift+':''}${modAlt?'Alt+':''}${modGui?'Win+':''}${ruleKey}
              </span>
            </div>
          </div>

          <!-- アクション -->
          <div class="flex gap-2 flex-wrap pt-1">
            <button onClick=${handleAddRule} disabled=${!connected}
              class="px-3 py-1 text-sm bg-emerald-200 hover:bg-emerald-300 rounded disabled:opacity-40 font-semibold">
              ✚ ルール追加
            </button>
            <button onClick=${handleListRules} disabled=${!connected}
              class="px-3 py-1 text-sm bg-slate-200 rounded disabled:opacity-40">↻</button>
            <button onClick=${handleClearRules} disabled=${!connected || ruleList.length === 0}
              class="px-3 py-1 text-sm bg-red-200 hover:bg-red-300 rounded disabled:opacity-40">Clear All</button>
          </div>
        </div>
      </div>
    </div>

    <!-- 6 点キャリブレーション ウィザード -->
    <div class="mt-4 bg-white rounded-lg shadow-sm border border-slate-200 p-4">
      <div class="flex justify-between items-center mb-2">
        <h2 class="font-semibold">🎯 加速度キャリブレーション</h2>
      </div>
      <p class="text-xs text-slate-500 mb-2">
        簡易: 1秒静止で gyro bias 補正 (起動時自動実行済み)<br/>
        フル 6 点: 各面に向けて 6 回キャプチャ、accel bias + scale 補正
      </p>

      ${calib6Step < 0 ? html`
        <div class="flex gap-2 flex-wrap">
          <button onClick=${handleCalibrate} disabled=${!connected}
            class="px-3 py-1 text-sm bg-orange-200 hover:bg-orange-300 rounded disabled:opacity-40">
            ⚡ 簡易キャリブ (1秒)
          </button>
          <button onClick=${handleCalib6Start} disabled=${!connected}
            class="px-3 py-1 text-sm bg-purple-200 hover:bg-purple-300 rounded disabled:opacity-40">
            🎯 フル 6 点キャリブ 開始
          </button>
          ${calib6Result ? html`
            <span class="text-xs text-emerald-700 font-mono">
              ✅ accel_bias=[${calib6Result.accel_bias_ms2?.map(v => v.toFixed(3)).join(', ')}] scale=[${calib6Result.accel_scale?.map(v => v.toFixed(3)).join(', ')}]
            </span>
          ` : null}
        </div>
      ` : html`
        <div class="border-2 border-purple-400 rounded p-3 bg-purple-50">
          <div class="flex justify-between items-center mb-2">
            <span class="font-semibold text-purple-700">ステップ ${Math.min(calib6Step+1, 6)}/6</span>
            <button onClick=${handleCalib6Cancel} class="text-xs text-red-600 hover:underline">✕ キャンセル</button>
          </div>
          <div class="text-sm font-semibold mb-2">${calib6Instruction || '...'}</div>
          ${sensor ? html`
            <div class="text-xs font-mono text-slate-500 mb-2">
              現在: ax=${sensor.ax?.toFixed(2)} ay=${sensor.ay?.toFixed(2)} az=${sensor.az?.toFixed(2)}
            </div>
          ` : html`<div class="text-xs text-amber-600 mb-2">⚠ Stream ON にしてください</div>`}
          ${calib6Step < 6 ? html`
            <button onClick=${handleCalib6Capture} class="px-3 py-1 text-sm bg-emerald-300 hover:bg-emerald-400 rounded font-semibold">
              📷 静止して キャプチャ
            </button>
          ` : html`
            <button onClick=${handleCalib6Finish} class="px-3 py-1 text-sm bg-blue-300 hover:bg-blue-400 rounded font-semibold">
              ✅ 計算 + 適用
            </button>
          `}
        </div>
      `}
    </div>

    <!-- FW 書込み (esp-web-tools) -->
    <div class="mt-4 bg-white rounded-lg shadow-sm border border-slate-200 p-4">
      <div class="flex justify-between items-center mb-3">
        <h2 class="font-semibold">⚡ FW 書込み (Web から)</h2>
        ${deviceInfo?.fw ? html`<span class="chip bg-green-100 text-green-700">現在: ${deviceInfo.fw}</span>` : null}
      </div>
      <p class="text-xs text-slate-500 mb-2">
        esp-web-tools を使ってブラウザから直接 FW を書き込み。
        ${connected ? '⚠️ 書込みには Serial 切断が必要 (上の「切断」ボタン押下後に Install)' : ''}
      </p>
      <div class="flex items-center gap-2 flex-wrap">
        <esp-web-install-button manifest="./firmware/m5stickc-v2/manifest.json">
          <button slot="activate" class="px-4 py-2 bg-orange-500 hover:bg-orange-600 text-white rounded-lg disabled:opacity-50">
            ⚡ M5StickC v2 を書き込み
          </button>
          <span slot="unsupported" class="text-sm text-red-600">
            このブラウザは WebSerial 非対応です。Chrome/Edge を使ってください。
          </span>
          <span slot="not-allowed" class="text-sm text-red-600">
            HTTPS 環境でのみ動作します。
          </span>
        </esp-web-install-button>
        <span class="text-xs text-slate-500 ml-2">manifest: <a href="./firmware/m5stickc-v2/manifest.json" class="underline">m5stickc-v2</a></span>
      </div>
    </div>

    <!-- プロファイル管理 -->
    <div class="mt-4 bg-white rounded-lg shadow-sm border border-slate-200 p-4">
      <div class="flex justify-between items-center mb-3">
        <h2 class="font-semibold">📁 プロファイル (LittleFS 永続化)</h2>
        ${activeProfile ? html`<span class="chip bg-blue-100 text-blue-700">アクティブ: ${activeProfile}</span>` : null}
      </div>
      <p class="text-xs text-slate-500 mb-2">登録済みルールを名前付きで保存。次回起動時に自動ロード。</p>
      <div class="flex items-center gap-2 mb-3 flex-wrap">
        <label class="text-sm">名前:</label>
        <input type="text" value=${profileName} onInput=${(e) => setProfileName(e.target.value)}
          class="border rounded px-2 py-1 font-mono text-sm" placeholder="default" />
        <button onClick=${handleProfileSave} disabled=${!connected}
          class="px-3 py-1 text-sm bg-emerald-200 hover:bg-emerald-300 rounded disabled:opacity-40">
          💾 現在のルールを保存
        </button>
      </div>
      ${profileList.length > 0 ? html`
        <div class="border rounded">
          <table class="w-full text-xs">
            <thead class="bg-slate-100">
              <tr>
                <th class="px-2 py-1 text-left">プロファイル名</th>
                <th class="px-2 py-1 text-center">操作</th>
              </tr>
            </thead>
            <tbody>
              ${profileList.map((name) => html`
                <tr class="${activeProfile === name ? 'bg-blue-50' : ''} border-t">
                  <td class="px-2 py-1">
                    ${activeProfile === name ? '⭐ ' : ''}${name}
                  </td>
                  <td class="px-2 py-1 text-center">
                    <button onClick=${() => handleProfileLoad(name)} disabled=${!connected}
                      class="px-2 py-0.5 text-xs bg-blue-200 hover:bg-blue-300 rounded disabled:opacity-40 mr-1">
                      📥 Load
                    </button>
                    <button onClick=${() => handleProfileDelete(name)} disabled=${!connected}
                      class="px-2 py-0.5 text-xs bg-red-200 hover:bg-red-300 rounded disabled:opacity-40">
                      🗑 Delete
                    </button>
                  </td>
                </tr>
              `)}
            </tbody>
          </table>
        </div>
      ` : html`<p class="text-xs text-slate-400 mt-2">保存済みプロファイルなし</p>`}
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
