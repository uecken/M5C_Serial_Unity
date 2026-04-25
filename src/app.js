// Burst Motion - Web Config App (Phase 2.1)
// 自動再接続 + HID テスト + 簡易ルール登録

import { h, render } from 'preact';
import { useState, useEffect, useRef, useCallback } from 'preact/hooks';
import htm from 'htm';
import { SerialClient } from './lib/SerialClient.js?v=20260426-084324';
import { BleClient }    from './lib/BleClient.js?v=20260426-084324';
import { IMUViewer }    from './lib/IMUViewer.js?v=20260426-084324';
import { PitchRollGrid } from './lib/PitchRollGrid.js?v=20260426-084324';
import { TimeSeriesChart } from './lib/TimeSeriesChart.js?v=20260426-084324';

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
  // キー入力モード: 'single' (単一キー、名前付き対応) / 'macro' (連続入力、カンマ区切り)
  const [ruleInputMode, setRuleInputMode] = useState('single');
  const [ruleKeysList, setRuleKeysList] = useState('');  // 例: "DOWN,RIGHT,p"
  const [ruleKeyInterval, setRuleKeyInterval] = useState(30);
  // HOLD_START_END で終了側に別キー (空なら同じキーを release のみ)
  const [endKey, setEndKey] = useState('');
  // 終了側の修飾キー
  const [endModCtrl,  setEndModCtrl ] = useState(false);
  const [endModShift, setEndModShift] = useState(false);
  const [endModAlt,   setEndModAlt  ] = useState(false);
  const [endModGui,   setEndModGui  ] = useState(false);
  const [ruleList, setRuleList] = useState([]);   // FW から取得した rule 一覧
  const [triggerFlash, setTriggerFlash] = useState(null);  // {id, phase, name, t}
  const [watchEnabled, setWatchEnabled] = useState(false);

  // 姿勢キャプチャ (Euler [r,p,y]、tol [r,p,y])
  const [startPosture, setStartPosture] = useState(null);   // {euler:[r,p,y], tol:[r,p,y]} | null
  const [endPosture, setEndPosture] = useState(null);
  const [postureTol, setPostureTol] = useState(15);   // ±degrees (一律、簡易)
  // 軸別 tol オーバーライド (Phase 5.15、空 or 0 なら postureTol 使用、180 で軸を実質除外)
  const [postureTolRoll, setPostureTolRoll] = useState('');
  const [postureTolPitch, setPostureTolPitch] = useState('');
  // 姿勢判定方法: "euler" (default、Roll/Pitch tol)、"quat" (Quaternion 内積)
  const [postureJudgeBy, setPostureJudgeBy] = useState('euler');

  // 修飾キー
  const [modCtrl,  setModCtrl ] = useState(false);
  const [modShift, setModShift] = useState(false);
  const [modAlt,   setModAlt  ] = useState(false);
  const [modGui,   setModGui  ] = useState(false);   // Win / Cmd

  // Profile 状態
  const [profileList, setProfileList] = useState([]);
  const [profileName, setProfileName] = useState('default');
  const [activeProfile, setActiveProfile] = useState('');

  // サンプルプロファイル (リポジトリ同梱、Phase 5)
  const [samples, setSamples] = useState([]);
  const [sampleLoading, setSampleLoading] = useState(null);  // 進行中のサンプル id
  const [sampleStatus, setSampleStatus] = useState('');

  // Hardware 定義 (機種別ボタン GPIO)
  const [hardwareDefs, setHardwareDefs] = useState({});  // {m5stickc: {title, buttons: [...]}, ...}
  const [selectedHardware, setSelectedHardware] = useState(
    localStorage.getItem('burst_motion_hardware') || 'm5stickc'
  );

  // ルール作成時のボタン条件 (デフォルト: Btn3 押下中、M5StickC レガシー互換)
  const [ruleButtonEnabled, setRuleButtonEnabled] = useState(true);
  const [ruleButtonIdx, setRuleButtonIdx] = useState(3);
  const [ruleButtonState, setRuleButtonState] = useState(0);  // 0=押下中、1=解放中

  // 6 点キャリブ ウィザード
  const [calib6Step, setCalib6Step] = useState(-1);  // -1=idle、0..5=待機、6=完了待機
  const [calib6Instruction, setCalib6Instruction] = useState('');
  const [calib6Result, setCalib6Result] = useState(null);

  // Stream タイミング debug
  const [streamGaps, setStreamGaps] = useState([]);  // 直近 100 サンプルの ms gap
  const [streamStats, setStreamStats] = useState({ count: 0, avg: 0, min: 0, max: 0, p95: 0 });
  const lastStreamTRef = useRef(null);   // 直近の receive 時刻 (performance.now)
  const lastFwTRef = useRef(null);       // 直近の sensor.t (FW 側 timestamp)

  const logRef = useRef(null);
  const canvasRef = useRef(null);
  const viewerRef = useRef(null);
  const gridCanvasRef = useRef(null);
  const gridRef = useRef(null);
  const hidTimerRef = useRef(null);
  const accelChartCanvasRef = useRef(null);
  const accelChartRef = useRef(null);
  const gyroChartCanvasRef = useRef(null);
  const gyroChartRef = useRef(null);

  // 3D 表示オプション (旧 UI 互換)
  const [showWorldAxes, setShowWorldAxes] = useState(false);
  const [showBodyAxes, setShowBodyAxes] = useState(false);
  const [showGravity, setShowGravity] = useState(false);

  // ルール姿勢から計算した登録参照点
  const [ruleReferences, setRuleReferences] = useState([]);  // [{id, name, roll, pitch, yaw, qw, qx, qy, qz}]
  const ruleReferencesRef = useRef([]);   // onTriggerHit など closure 経由で参照する用
  const [closestRuleIdx, setClosestRuleIdx] = useState(-1);

  // ruleReferences を ref にも反映 (event handler の closure 内で最新を参照)
  useEffect(() => { ruleReferencesRef.current = ruleReferences; }, [ruleReferences]);

  // Closest-only モード: FW 側で最近傍ルールだけ発火させる
  const [closestOnlyMode, setClosestOnlyMode] = useState(false);
  // Button-edge lock パラメータ
  const [lockWindowMs, setLockWindowMs] = useState(500);
  const [lockCooldownMs, setLockCooldownMs] = useState(300);
  // Lock 中のルール ID (FW から watch event で受信)
  const [lockedRuleId, setLockedRuleId] = useState(-1);
  const [lockedAt, setLockedAt] = useState(0);   // 受信タイムスタンプ (ms)

  // 初回 device.info 取得時に FW の状態を反映
  useEffect(() => {
    if (deviceInfo && typeof deviceInfo.closest_only === 'boolean') {
      setClosestOnlyMode(deviceInfo.closest_only);
    }
    if (deviceInfo?.lock_window_ms) setLockWindowMs(deviceInfo.lock_window_ms);
    if (deviceInfo?.lock_cooldown_ms !== undefined) setLockCooldownMs(deviceInfo.lock_cooldown_ms);
  }, [deviceInfo?.closest_only, deviceInfo?.lock_window_ms, deviceInfo?.lock_cooldown_ms]);

  // 'lock' イベント受信 (FW: lock.acquired / lock.fired / lock.expired)
  useEffect(() => {
    const onLock = (ev) => {
      const d = ev.detail;
      if (d.phase === 'lock.acquired') {
        setLockedRuleId(d.id);
        setLockedAt(Date.now());
      } else {
        // fired or expired → リセット
        setLockedRuleId(-1);
      }
    };
    [serialClient, bleClient].forEach((c) => c.addEventListener('type:lock', onLock));
    return () => [serialClient, bleClient].forEach((c) => c.removeEventListener('type:lock', onLock));
  }, []);

  // Lock 設定を FW に送信
  const handleSendLockParams = () => {
    sendCmd({
      cmd: 'engine.lock.set',
      window_ms: parseInt(lockWindowMs, 10),
      cooldown_ms: parseInt(lockCooldownMs, 10),
    });
  };

  // App version (deploy 時に生成される version.json から読み込み)
  // index.html の <meta name="app-version"> が deploy 時に置換されるのでそれをまず読む
  // version.json は no-cache で fetch、デプロイ時に必ず更新
  const [appVersion, setAppVersion] = useState(() => {
    const meta = document.querySelector('meta[name="app-version"]');
    const v = meta?.getAttribute('content') || '';
    return v && !v.includes('__BUILD_VERSION__') ? v : '';
  });
  const [appDeployedAt, setAppDeployedAt] = useState('');
  useEffect(() => {
    fetch(`./version.json?nocache=${Date.now()}`, { cache: 'no-store' })
      .then((r) => r.ok ? r.json() : null)
      .then((d) => {
        if (!d) return;
        if (d.version) setAppVersion(d.version);
        if (d.deployed_at) {
          // YYYY-MM-DD だけ表示
          setAppDeployedAt(String(d.deployed_at).split('T')[0]);
        }
      })
      .catch(() => {});
  }, []);

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

  // 2D グリッド初期化
  useEffect(() => {
    if (!gridCanvasRef.current) return;
    if (gridRef.current) return;
    gridRef.current = new PitchRollGrid(gridCanvasRef.current);
    gridRef.current.resize();
    const onResize = () => gridRef.current?.resize();
    window.addEventListener('resize', onResize);
    return () => window.removeEventListener('resize', onResize);
  }, [gridCanvasRef.current]);

  // 時系列波形チャート初期化 (Accel & Gyro、各 XYZ + RMS)
  useEffect(() => {
    if (!accelChartCanvasRef.current || accelChartRef.current) return;
    accelChartRef.current = new TimeSeriesChart(accelChartCanvasRef.current, {
      channels: [
        { key: 'x',   color: '#ef4444', label: 'X' },
        { key: 'y',   color: '#10b981', label: 'Y' },
        { key: 'z',   color: '#3b82f6', label: 'Z' },
        { key: 'rms', color: '#1e293b', label: 'RMS', width: 2 },
      ],
      yMin: -2, yMax: 2, autoScale: true, bufferSize: 200,
    });
    accelChartRef.current.resize();
  }, [accelChartCanvasRef.current]);
  useEffect(() => {
    if (!gyroChartCanvasRef.current || gyroChartRef.current) return;
    gyroChartRef.current = new TimeSeriesChart(gyroChartCanvasRef.current, {
      channels: [
        { key: 'x',   color: '#ef4444', label: 'X' },
        { key: 'y',   color: '#10b981', label: 'Y' },
        { key: 'z',   color: '#3b82f6', label: 'Z' },
        { key: 'rms', color: '#1e293b', label: 'RMS', width: 2 },
      ],
      yMin: -200, yMax: 200, autoScale: true, bufferSize: 200,
    });
    gyroChartRef.current.resize();
  }, [gyroChartCanvasRef.current]);

  // チャート resize 監視
  useEffect(() => {
    const onResize = () => {
      accelChartRef.current?.resize();
      gyroChartRef.current?.resize();
    };
    window.addEventListener('resize', onResize);
    return () => window.removeEventListener('resize', onResize);
  }, []);

  // sensor 受信時に時系列波形へ push (RMS は Web 側で計算: sqrt(x^2+y^2+z^2))
  useEffect(() => {
    if (!sensor) return;
    if (sensor.ax !== undefined && accelChartRef.current) {
      // accel は m/s² → g 換算で見やすく
      const ax_g = sensor.ax / 9.80665;
      const ay_g = sensor.ay / 9.80665;
      const az_g = sensor.az / 9.80665;
      const rms_g = Math.sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
      accelChartRef.current.push({ x: ax_g, y: ay_g, z: az_g, rms: rms_g });
    }
    if (sensor.gx !== undefined && gyroChartRef.current) {
      const rms = Math.sqrt(sensor.gx*sensor.gx + sensor.gy*sensor.gy + sensor.gz*sensor.gz);
      gyroChartRef.current.push({ x: sensor.gx, y: sensor.gy, z: sensor.gz, rms });
    }
  }, [sensor?.t]);

  // 3D viewer の軸表示切替
  useEffect(() => { viewerRef.current?.setShowWorldAxes(showWorldAxes); }, [showWorldAxes]);
  useEffect(() => { viewerRef.current?.setShowBodyAxes(showBodyAxes); }, [showBodyAxes]);
  useEffect(() => { viewerRef.current?.setShowGravity(showGravity); }, [showGravity]);

  // sensor 受信時に 3D viewer + 2D グリッド + 最近傍ルール更新
  useEffect(() => {
    if (!sensor || !viewerRef.current) return;
    if (sensor.qw !== undefined) {
      viewerRef.current.setQuaternion(sensor.qw, sensor.qx, sensor.qy, sensor.qz);
      // 球面の現在位置 dot
      viewerRef.current.setCurrentDot(sensor.qw, sensor.qx, sensor.qy, sensor.qz);
    }
    if (sensor.ax !== undefined) {
      viewerRef.current.setGravityVector(sensor.ax, sensor.ay, sensor.az);
    }
    // 2D グリッド
    if (gridRef.current && sensor.roll !== undefined) {
      gridRef.current.setCurrent(sensor.roll, sensor.pitch);
    }
    // 最近傍ルール計算 (Quaternion angleTo)
    if (ruleReferences.length > 0 && sensor.qw !== undefined) {
      const findClosest = () => {
        let minAngle = Infinity;
        let idx = -1;
        const cur = { w: sensor.qw, x: sensor.qx, y: sensor.qy, z: sensor.qz };
        ruleReferences.forEach((r, i) => {
          if (r.qw === undefined) return;
          // 内積
          let dot = cur.w*r.qw + cur.x*r.qx + cur.y*r.qy + cur.z*r.qz;
          if (dot < 0) dot = -dot;
          if (dot > 1) dot = 1;
          const angle = 2 * Math.acos(dot);
          if (angle < minAngle) { minAngle = angle; idx = i; }
        });
        return idx;
      };
      const idx = findClosest();
      if (idx !== closestRuleIdx) setClosestRuleIdx(idx);
      if (idx >= 0) {
        const r = ruleReferences[idx];
        viewerRef.current.setClosestDot(r.qw, r.qx, r.qy, r.qz);
        gridRef.current?.setClosest(idx);
      }
    } else {
      viewerRef.current.setClosestDot(null);
      gridRef.current?.setClosest(-1);
    }
  }, [sensor, ruleReferences]);

  // ruleList 更新時に reference 座標を更新
  // 優先: FW rule.list 応答の posture (新 FW)、フォールバック: localStorage (姿勢キャプチャ時保存)
  useEffect(() => {
    const stored = JSON.parse(localStorage.getItem('burst_motion_rule_postures') || '{}');
    const refs = ruleList.map((r) => {
      // 1. FW 応答に posture が含まれていれば優先 (Phase 5.9 で quat も含む、Phase 5.16 で tol も)
      if (r.posture && r.posture.euler) {
        const local = stored[r.id];
        const fwQuat = r.posture.quat;
        const localQuat = local?.quat;
        const q = (fwQuat && fwQuat.length === 4) ? fwQuat : localQuat;
        const tol = r.posture.euler_tol;
        return {
          id: r.id, name: r.name,
          roll: r.posture.euler[0], pitch: r.posture.euler[1], yaw: r.posture.euler[2],
          rollTol:  Array.isArray(tol) ? tol[0] : undefined,
          pitchTol: Array.isArray(tol) ? tol[1] : undefined,
          yawTol:   Array.isArray(tol) ? tol[2] : undefined,
          qw: q?.[0], qx: q?.[1], qy: q?.[2], qz: q?.[3],
        };
      }
      // 2. localStorage から
      const p = stored[r.id];
      if (!p) return null;
      return { id: r.id, name: r.name,
               roll: p.euler[0], pitch: p.euler[1], yaw: p.euler[2],
               rollTol:  p.euler_tol?.[0],
               pitchTol: p.euler_tol?.[1],
               yawTol:   p.euler_tol?.[2],
               qw: p.quat?.[0], qx: p.quat?.[1], qy: p.quat?.[2], qz: p.quat?.[3] };
    }).filter(Boolean);
    setRuleReferences(refs);
    if (gridRef.current) {
      gridRef.current.setReferences(refs);
    }
    if (viewerRef.current) {
      const quats = refs.filter(r => r.qw !== undefined).map(r => ({
        w: r.qw, x: r.qx, y: r.qy, z: r.qz
      }));
      import('three').then((THREE) => {
        const tquats = quats.map(q => new THREE.Quaternion(-q.x, q.z, q.y, q.w));
        viewerRef.current?.setReferenceQuaternions(tquats);
      });
    }
  }, [ruleList]);

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
    // ノイズ除外: 自動ポーリングしている hw.buttons.get の送受信はログに出さない
    //   (200ms 周期で 5/秒 流れてログを埋めるため)
    //   ヘッダの 🔍 Test ボタンで明示送信した場合は除外できないが、
    //   それ以外の hw.buttons は UI 表示のための裏通信なので隠す
    const isNoiseTx = (obj) => obj?.cmd === 'hw.buttons.get';
    const isNoiseRx = (line) => typeof line === 'string' && line.includes('"type":"hw.buttons"');
    const onRaw = (ev) => {
      if (isNoiseRx(ev.detail)) return;
      addLog('rx', ev.detail);
    };
    const onSent = (ev) => {
      if (isNoiseTx(ev.detail)) return;
      addLog('tx', JSON.stringify(ev.detail));
    };
    const onSensor = (ev) => {
      setSensor(ev.detail);
      // Stream タイミング統計
      const now = performance.now();
      const fwT = ev.detail.t;
      const last = lastStreamTRef.current;
      const lastFw = lastFwTRef.current;
      lastStreamTRef.current = now;
      lastFwTRef.current = fwT;
      if (last !== null && lastFw !== null) {
        const browserGap = now - last;       // ブラウザ受信 gap
        const fwGap = fwT - lastFw;          // FW timestamp 差 (本来 ~20ms @50Hz)
        setStreamGaps((prev) => {
          const next = [...prev, { fw: fwGap, browser: browserGap }];
          if (next.length > 100) next.splice(0, next.length - 100);
          return next;
        });
      }
    };
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
      setTriggerFlash({
        id: d.id, name: d.rule_name, phase: d.phase, t: Date.now(),
        keys: d.keys, action_type: d.action_type, modifiers: d.modifiers,
      });
      // 1 秒後にフラッシュを消す
      setTimeout(() => setTriggerFlash((cur) => cur && cur.t === d.t ? null : cur), 1000);
      // 2D マップで該当ルール点を緑フラッシュ (500ms 後に元の色)
      if (gridRef.current && d.phase === 'enter') {
        const idx = ruleReferencesRef.current.findIndex((r) => r.id === d.id);
        if (idx >= 0) gridRef.current.setFiring(idx, 500);
      }
    };
    const onAck = (ev) => {
      const d = ev.detail;
      if (d.cmd === 'rule.add' || d.cmd === 'rule.clear' || d.cmd === 'rule.remove') {
        if (activeClient) activeClient.send({ cmd: 'rule.list' }).catch(() => {});
      }
      if (d.cmd === 'profile.save' || d.cmd === 'profile.delete' || d.cmd === 'profile.load') {
        if (activeClient) activeClient.send({ cmd: 'profile.list' }).catch(() => {});
      }
      if (d.cmd === 'engine.closest_only' && typeof d.enabled === 'boolean') {
        setClosestOnlyMode(d.enabled);
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

  // streamGaps が更新されたら統計再計算
  useEffect(() => {
    if (streamGaps.length === 0) return;
    const fwGaps = streamGaps.map((g) => g.fw).filter((v) => v > 0 && v < 1000);
    if (fwGaps.length === 0) return;
    const sorted = [...fwGaps].sort((a, b) => a - b);
    const sum = fwGaps.reduce((a, b) => a + b, 0);
    setStreamStats({
      count: fwGaps.length,
      avg: sum / fwGaps.length,
      min: sorted[0],
      max: sorted[sorted.length - 1],
      p95: sorted[Math.floor(sorted.length * 0.95)],
    });
  }, [streamGaps]);

  const handleClearStreamStats = () => {
    setStreamGaps([]);
    setStreamStats({ count: 0, avg: 0, min: 0, max: 0, p95: 0 });
  };

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
  const handleBleStop  = () => sendCmd({ cmd: 'ble.stop' });
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

  // 姿勢キャプチャ (現在の sensor から、quat も保存)
  const captureStartPosture = () => {
    if (!sensor) { alert('センサーストリーム ON にしてから姿勢を取得してください'); return; }
    const tol = parseInt(postureTol) || 15;
    setStartPosture({
      euler: [sensor.roll, sensor.pitch, sensor.yaw],
      euler_tol: [tol, tol, tol * 6],
      quat: [sensor.qw, sensor.qx, sensor.qy, sensor.qz],
    });
  };
  const captureEndPosture = () => {
    if (!sensor) { alert('センサーストリーム ON にしてから姿勢を取得してください'); return; }
    const tol = parseInt(postureTol) || 15;
    setEndPosture({
      euler: [sensor.roll, sensor.pitch, sensor.yaw],
      euler_tol: [tol, tol, tol * 6],
      quat: [sensor.qw, sensor.qx, sensor.qy, sensor.qz],
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
    // 姿勢ありルール推奨ガード (姿勢なしは Closest-only モードで lock を抜けて誤発火しやすい)
    if (!startPosture) {
      const ok = confirm(
        '姿勢条件 (📷 開始姿勢) が未取得です。\n\n' +
        '姿勢なしルールは Closest-only モードで lock 機構を抜けて誤発火しやすいため、推奨されません。\n' +
        '可能なら姿勢キャプチャしてからルール追加してください。\n\n' +
        'それでも姿勢なしで追加しますか?'
      );
      if (!ok) return;
    }
    // ジンバルロック領域 (|Pitch| > 65°) チェック — Euler 判定が不安定になる
    if (startPosture && Math.abs(startPosture.euler[1]) > 65) {
      const ok = confirm(
        '⚠ ジンバルロック領域です\n\n' +
        `Pitch = ${startPosture.euler[1].toFixed(1)}° は |Pitch| > 65° のジンバルロック領域に該当します。\n` +
        'Mahony フィルタの Euler 出力は Pitch = asin で ±90° で縮退するため、\n' +
        'Roll/Yaw が安定して取れず、ルール判定が不安定になります。\n\n' +
        '推奨対応:\n' +
        ' ・姿勢を Pitch < ±65° の範囲で取り直す\n' +
        ` ・判定方法を「Quaternion 内積」に切替 (現在の判定: ${postureJudgeBy})\n\n` +
        'このまま登録しますか?'
      );
      if (!ok) return;
    }
    const id = Date.now() & 0xffff;
    const r = {
      id,
      name: ruleName || `${ruleMode}_${ruleKey}`,
      ui_mode: ruleMode,
      key: ruleKey,
      cooldown_ms: 500,
    };
    // 追加時の姿勢を localStorage に保存 (3D/2D 表示で使う、FW へは送らない)
    if (startPosture && sensor) {
      const stored = JSON.parse(localStorage.getItem('burst_motion_rule_postures') || '{}');
      stored[id] = {
        euler: startPosture.euler,
        euler_tol: startPosture.euler_tol,
        quat: [sensor.qw, sensor.qx, sensor.qy, sensor.qz],
      };
      localStorage.setItem('burst_motion_rule_postures', JSON.stringify(stored));
    }
    if (ruleAccelEnabled && parseFloat(ruleAccelTh) > 0) {
      r.accel_abs_threshold = parseFloat(ruleAccelTh);
    }
    if (startPosture) {
      r.posture = {
        euler: startPosture.euler,
        euler_tol: startPosture.euler_tol,
        judge_by: postureJudgeBy,   // "euler" (default) or "quat"
      };
      if (startPosture.quat) r.posture.quat = startPosture.quat;
    }
    if (ruleButtonEnabled) {
      r.button_idx = parseInt(ruleButtonIdx, 10);
      r.button_state = parseInt(ruleButtonState, 10);
    }
    if (ruleMode === 'hold_start_end' && endPosture) {
      r.end_posture = { euler: endPosture.euler, euler_tol: endPosture.euler_tol };
      if (endPosture.quat) r.end_posture.quat = endPosture.quat;
      // 終了側に別キーが指定されていれば送る (空なら従来通り start key の release のみ)
      if (endKey && endKey.length > 0) {
        r.end_key = endKey[0];
        let endMods = 0;
        if (endModCtrl)  endMods |= 0x01;
        if (endModShift) endMods |= 0x02;
        if (endModAlt)   endMods |= 0x04;
        if (endModGui)   endMods |= 0x08;
        if (endMods > 0) r.end_modifiers = endMods;
        r.end_duration_ms = 30;
      }
    }
    // キー入力 (単一 or 連続マクロ)
    if (ruleInputMode === 'macro' && ruleKeysList.trim()) {
      // カンマ区切り (各カンマ要素はそのステップで押下中のキー集合、"+" 連結可)
      // 例: "DOWN, DOWN+RIGHT, RIGHT, RIGHT+p"
      const arr = ruleKeysList.split(',').map((s) => s.trim()).filter(Boolean);
      if (arr.length > 0) {
        // Phase 5.14: 同時押し macro 展開 ("+" を含む要素があれば prefix 配列に変換)
        r.keys = expandKeySteps(arr);
        r.interval_ms = parseInt(ruleKeyInterval, 10) || 30;
        delete r.key;
      }
    } else if (ruleKey) {
      r.key = ruleKey;   // 1 文字 ASCII or 名前付き ("ARROW_RIGHT" 等)
    }
    const mods = buildModifiers();
    if (mods > 0) r.modifiers = mods;
    sendCmd({ cmd: 'rule.add', r });
  };
  const handleListRules  = () => sendCmd({ cmd: 'rule.list' });
  const handleClearRules = () => {
    if (!confirm('登録済みルールをすべて削除します。OK?')) return;
    // 楽観的更新: FW 応答を待たずに UI 即時クリア (ack→rule.list の往復で遅延感あるため)
    setRuleList([]);
    setRuleReferences([]);
    setClosestRuleIdx(-1);
    sendCmd({ cmd: 'rule.clear' });
  };
  const handleRemoveRule = (id, name) => {
    if (!confirm(`ルール「${name}」(id=${id}) を削除します。OK?`)) return;
    // 楽観的更新
    setRuleList((prev) => prev.filter((r) => r.id !== id));
    setRuleReferences((prev) => prev.filter((r) => r.id !== id));
    sendCmd({ cmd: 'rule.remove', id });
  };
  const handleToggleWatch = () => {
    const next = !watchEnabled;
    setWatchEnabled(next);
    sendCmd({ cmd: 'watch.set', enabled: next });
  };
  // 接続成功時に自動で device.info + rule.list + profile.list + watch を要求
  useEffect(() => {
    if (connected && activeClient) {
      const t = setTimeout(() => {
        activeClient.send({ cmd: 'device.info' }).catch(() => {});  // FW Phase / build 取得
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

  // サンプルプロファイル一覧を初回 fetch
  useEffect(() => {
    fetch('./profiles/index.json', { cache: 'no-store' })
      .then((r) => r.ok ? r.json() : null)
      .then((d) => { if (d && Array.isArray(d.profiles)) setSamples(d.profiles); })
      .catch(() => {});
  }, []);

  // Hardware 定義を初回 fetch
  useEffect(() => {
    fetch('./hardware_profiles.json', { cache: 'no-store' })
      .then((r) => r.ok ? r.json() : null)
      .then((d) => { if (d && d.hardware) setHardwareDefs(d.hardware); })
      .catch(() => {});
  }, []);

  // device.info.board が来たら自動で hardware 選択
  useEffect(() => {
    if (deviceInfo?.board && hardwareDefs[deviceInfo.board]) {
      setSelectedHardware(deviceInfo.board);
    }
  }, [deviceInfo?.board, hardwareDefs]);

  // selectedHardware を localStorage 永続化
  useEffect(() => {
    localStorage.setItem('burst_motion_hardware', selectedHardware);
  }, [selectedHardware]);

  // 現在 Hardware の buttons 配列 (空配列なら定義未取得 or 該当機種なし)
  const currentButtons = hardwareDefs[selectedHardware]?.buttons || [];

  // Hardware 切替時、ボタン条件のデフォルト idx を最後 (= 通常使用される独立ボタン) に追従
  // M5StickC なら 3 (Btn3=G26)、M5Atom S3 なら 1 (Btn=G41)
  useEffect(() => {
    if (currentButtons.length > 0) {
      const lastIdx = currentButtons[currentButtons.length - 1].idx;
      // 現在 idx が選択肢にない場合のみ追従 (ユーザー手動選択を尊重)
      const exists = currentButtons.some((b) => b.idx === ruleButtonIdx);
      if (!exists) setRuleButtonIdx(lastIdx);
    }
  }, [selectedHardware, currentButtons.length]);

  // FW 側の現在のボタン GPIO 構成 (hw.buttons.get の応答)
  const [fwButtons, setFwButtons] = useState(null);  // null = 未取得、配列 = FW の現在値
  // 「FW に適用」ボタンの結果メッセージ
  const [fwButtonsStatus, setFwButtonsStatus] = useState('');
  // ライブ ボタン bitmap (sensor.btn または hw.buttons.get.bitmap から)
  const [liveBtnBitmap, setLiveBtnBitmap] = useState(0);
  // 直近のボタン受信タイムスタンプ (UI で「データ来てる?」確認用)
  const [liveBtnUpdatedMs, setLiveBtnUpdatedMs] = useState(0);

  // hw.buttons 受信ハンドラ (FW から GPIO 構成取得 + bitmap)
  useEffect(() => {
    const onHwButtons = (ev) => {
      setFwButtons(ev.detail.buttons || []);
      if (ev.detail.bitmap !== undefined) {
        setLiveBtnBitmap(ev.detail.bitmap);
        setLiveBtnUpdatedMs(Date.now());
      }
    };
    [serialClient, bleClient].forEach((c) => {
      c.addEventListener('type:hw.buttons', onHwButtons);
    });
    return () => {
      [serialClient, bleClient].forEach((c) => {
        c.removeEventListener('type:hw.buttons', onHwButtons);
      });
    };
  }, []);

  // sensor.btn 受信時に liveBtnBitmap 更新
  useEffect(() => {
    if (sensor && sensor.btn !== undefined) {
      setLiveBtnBitmap(sensor.btn);
      setLiveBtnUpdatedMs(Date.now());
    }
  }, [sensor?.btn, sensor?.t]);

  // ボタン rising edge 検出 → 3D 球面に押下時の姿勢を紫ドット表示
  // (旧 motion_controller.js 互換、Stream ON 前提)
  const lastBtnRef = useRef(0);
  useEffect(() => {
    if (!sensor || sensor.btn === undefined) return;
    const prev = lastBtnRef.current;
    const curr = sensor.btn;
    const edge = curr & ~prev;  // 立ち上がり
    lastBtnRef.current = curr;
    if (edge !== 0 && viewerRef.current && sensor.qw !== undefined) {
      viewerRef.current.setButtonPressDot(sensor.qw, sensor.qx, sensor.qy, sensor.qz, 2500);
    }
  }, [sensor?.btn]);

  // Stream OFF 時は 500ms 周期で hw.buttons.get をポーリング (ボタン状態を切らさない)
  // ログには出さない (onRaw/onSent でフィルタ)
  useEffect(() => {
    if (!connected || !activeClient) return;
    if (streamRate > 0) return;  // Stream ON なら sensor.btn から取得済
    const id = setInterval(() => {
      activeClient.send({ cmd: 'hw.buttons.get' }).catch(() => {});
    }, 500);
    return () => clearInterval(id);
  }, [connected, streamRate]);

  // 接続成功時に hw.buttons.get を投げて FW 現在値を取得
  useEffect(() => {
    if (connected && activeClient) {
      const t = setTimeout(() => {
        activeClient.send({ cmd: 'hw.buttons.get' }).catch(() => {});
      }, 800);
      return () => clearTimeout(t);
    }
  }, [connected]);

  // 現 Hardware 定義を FW に適用
  const handleApplyButtonsToFw = async () => {
    if (!activeClient || !connected) return;
    if (currentButtons.length === 0) return;
    const buttons = currentButtons.map((b) => ({
      gpio: b.gpio,
      active_low: b.active_low ?? true,
      pull_mode: b.pull_mode ?? 1,
    }));
    setFwButtonsStatus('FW へ送信中…');
    try {
      await activeClient.send({ cmd: 'hw.buttons.set', buttons });
      // 直後に hw.buttons.get で確認
      await new Promise((r) => setTimeout(r, 200));
      await activeClient.send({ cmd: 'hw.buttons.get' });
      setFwButtonsStatus(`✅ ${buttons.length} ボタンを FW に適用 (NVS 保存済み)`);
    } catch (e) {
      setFwButtonsStatus(`❌ 失敗: ${e.message || e}`);
    }
    setTimeout(() => setFwButtonsStatus(''), 5000);
  };

  // FW 現在構成と Web 側 currentButtons の GPIO 一致判定
  const buttonsMatchFw = (() => {
    if (!fwButtons || currentButtons.length === 0) return null;
    if (fwButtons.length !== currentButtons.length) return false;
    return fwButtons.every((b, i) =>
      b.gpio === currentButtons[i].gpio &&
      !!b.active_low === !!currentButtons[i].active_low &&
      (b.pull_mode | 0) === (currentButtons[i].pull_mode | 0)
    );
  })();

  // HID キーコード → 表示名 (BleCombo 規約、ASCII printable はそのまま)
  const hidCodeToName = (c) => {
    if (typeof c !== 'number') return '?';
    if (c >= 0x20 && c <= 0x7e) return String.fromCharCode(c);
    const map = {
      0x80: 'L-Ctrl', 0x81: 'L-Shift', 0x82: 'L-Alt', 0x83: 'L-Win',
      0x84: 'R-Ctrl', 0x85: 'R-Shift', 0x86: 'R-Alt', 0x87: 'R-Win',
      0xB0: 'Enter', 0xB1: 'Esc', 0xB2: 'BS', 0xB3: 'Tab',
      0xC1: 'Caps', 0xCE: 'PrtSc',
      0xC2: 'F1', 0xC3: 'F2', 0xC4: 'F3', 0xC5: 'F4', 0xC6: 'F5', 0xC7: 'F6',
      0xC8: 'F7', 0xC9: 'F8', 0xCA: 'F9', 0xCB: 'F10', 0xCC: 'F11', 0xCD: 'F12',
      0xD1: 'Ins', 0xD2: 'Home', 0xD3: 'PgUp', 0xD4: 'Del', 0xD5: 'End', 0xD6: 'PgDn',
      0xD7: '→', 0xD8: '←', 0xD9: '↓', 0xDA: '↑',
    };
    return map[c] || `0x${c.toString(16).padStart(2, '0')}`;
  };

  // FIRE_MACRO の (keys[], key_modes[]) を「各ステップで押下中のキー集合」として再構築 (Phase 5.14.5)
  // 例: keys=[DOWN, RIGHT, DOWN, a, 0], modes=[PRESS, PRESS, RELEASE, PRESS, RELEASE_ALL]
  //   →  [{↓}, {↓+→}, {→}, {→+a}, {}]  ← 各ステップで押下中の集合
  // UI で「↓ → ↓+→ → →+a」のようにブロック並べて視覚化、同時押しが一目で分かる。
  const renderMacroSteps = (keys, modes) => {
    if (!Array.isArray(keys) || keys.length === 0) return null;
    const steps = [];
    const pressed = [];   // 配列で順序保持 (同時押し集合の表示順)
    const setHas = (name) => pressed.includes(name);
    const setAdd = (name) => { if (!setHas(name)) pressed.push(name); };
    const setDel = (name) => { const i = pressed.indexOf(name); if (i >= 0) pressed.splice(i, 1); };
    for (let i = 0; i < keys.length; i++) {
      const mode = (modes && modes[i] !== undefined) ? modes[i] : 0;
      const name = hidCodeToName(keys[i]);
      if (mode === 3) {            // RELEASE_ALL
        pressed.length = 0;
        steps.push([]);
      } else if (mode === 1) {      // PRESS
        setAdd(name); steps.push([...pressed]);
      } else if (mode === 2) {      // RELEASE
        setDel(name); steps.push([...pressed]);
      } else {                      // FIRE (press + release)
        setAdd(name); steps.push([...pressed]);
        setDel(name);
      }
    }
    return steps.map((set, i) => {
      if (set.length === 0) {
        return html`<span class="text-slate-400 mx-0.5" title="全 release">∅</span>`;
      }
      return html`<span class="bg-amber-100 text-amber-800 px-1 rounded mr-0.5 border border-amber-300" title="ステップ ${i+1}: ${set.length} キー同時押下">${set.join('+')}</span>`;
    });
  };

  // 同時押し macro 形式の展開 (Phase 5.14)
  // 入力: ["DOWN", "DOWN+RIGHT", "RIGHT", "RIGHT+p"]  (各ステップで押下中のキー集合)
  // 出力: ["+DOWN", "+RIGHT", "-DOWN", "+p", "!"]    (FW 内部 prefix 形式)
  // 1 要素に "+" を含まない単独キー (例: "p", "ARROW_RIGHT") は従来通り FIRE 扱い (押→離)。
  // 1 要素でも "+" を含む同時押しを検出したら、全体を「キーセット遷移」として展開する。
  const expandKeySteps = (keysInput) => {
    if (!Array.isArray(keysInput)) return keysInput;
    const hasComboStep = keysInput.some((s) => typeof s === 'string' && s.includes('+'));
    if (!hasComboStep) return keysInput;  // 全部単独キー → 従来形式そのまま
    const result = [];
    let prev = new Set();
    for (const step of keysInput) {
      if (typeof step !== 'string') continue;
      if (step === '!') {
        // 明示的 release-all
        for (const k of prev) result.push('-' + k);
        prev = new Set();
        continue;
      }
      const curr = new Set(step.split('+').map((s) => s.trim()).filter(Boolean));
      // release: prev にあって curr にないキー
      for (const k of prev) if (!curr.has(k)) result.push('-' + k);
      // press: curr にあって prev にないキー
      for (const k of curr) if (!prev.has(k)) result.push('+' + k);
      prev = curr;
    }
    // 最後に残っているキーを全 release
    if (prev.size > 0) result.push('!');
    return result;
  };

  // Euler [deg] → Quaternion [w,x,y,z] 変換 (ZYX 順、Mahony Filter / 一般的な航空規約)
  // サンプルプロファイルの posture.euler から quat を生成、rule.add に含める。
  // (FW 側で quat 未指定時は「現在の sensor quat」が使われてしまい、
  //  全ルールが同じ quat になり Closest-only 計算が機能不全になる問題への対処)
  const eulerToQuat = (rollDeg, pitchDeg, yawDeg) => {
    const r = (rollDeg  * Math.PI / 180) / 2;
    const p = (pitchDeg * Math.PI / 180) / 2;
    const y = (yawDeg   * Math.PI / 180) / 2;
    const cr = Math.cos(r), sr = Math.sin(r);
    const cp = Math.cos(p), sp = Math.sin(p);
    const cy = Math.cos(y), sy = Math.sin(y);
    return [
      cr*cp*cy + sr*sp*sy,   // w
      sr*cp*cy - cr*sp*sy,   // x (roll 軸)
      cr*sp*cy + sr*cp*sy,   // y (pitch 軸)
      cr*cp*sy - sr*sp*cy,   // z (yaw 軸)
    ];
  };

  // サンプル → ルール送信 → profile.save
  const handleSampleLoad = async (sample) => {
    if (!activeClient || !connected) {
      alert('先にデバイスへ接続してください');
      return;
    }
    if (!confirm(`サンプル「${sample.title}」を読み込みます。\n現在登録中のルールは全て上書きされます。\n続けますか?`)) return;

    setSampleLoading(sample.id);
    setSampleStatus('JSON 取得中…');
    try {
      const resp = await fetch(`./profiles/${sample.file}`, { cache: 'no-store' });
      if (!resp.ok) throw new Error('fetch failed');
      const data = await resp.json();
      if (!data.rules || !Array.isArray(data.rules)) throw new Error('invalid schema');

      setSampleStatus('既存ルール削除中…');
      await activeClient.send({ cmd: 'rule.clear' });

      // engine 設定 (Closest-only モード + Button lock window + Cooldown)
      if (data.engine) {
        if (typeof data.engine.closest_only === 'boolean') {
          await activeClient.send({ cmd: 'engine.closest_only', enabled: data.engine.closest_only });
          setClosestOnlyMode(data.engine.closest_only);
          await new Promise((res) => setTimeout(res, 30));
        }
        const winMs = data.engine.button_lock_window_ms;
        const coolMs = data.engine.cooldown_ms;
        if (winMs !== undefined || coolMs !== undefined) {
          const payload = { cmd: 'engine.lock.set' };
          if (winMs !== undefined) { payload.window_ms = winMs; setLockWindowMs(winMs); }
          if (coolMs !== undefined) { payload.cooldown_ms = coolMs; setLockCooldownMs(coolMs); }
          await activeClient.send(payload);
          await new Promise((res) => setTimeout(res, 30));
        }
      }

      // FW Phase チェック: 同時押し macro を含むサンプルは Phase 5.14+ FW 必須
      const fwPhase = deviceInfo?.fw_phase || '';
      const fwPhaseNum = parseFloat(fwPhase) || 0;
      const hasComboMacro = data.rules.some((r) =>
        Array.isArray(r.keys) && r.keys.some((s) => typeof s === 'string' && s.includes('+'))
      );
      if (hasComboMacro && fwPhaseNum < 5.14) {
        const ok = confirm(
          `⚠ FW Phase ${fwPhase || '不明'} は同時押し macro 未対応です\n\n` +
          `このサンプルには同時押しコマンド (DOWN+RIGHT 等) が含まれていますが、\n` +
          `FW Phase 5.14 未満では正しく解釈できず、"!" 文字が入力されてしまう問題があります。\n\n` +
          `FW を Phase 5.14+ にフラッシュしてから再度適用してください。\n\n` +
          `そのまま適用しますか? (誤動作する可能性が高いです)`
        );
        if (!ok) {
          setSampleLoading(null);
          setSampleStatus('❌ キャンセル: FW Phase 不足');
          setTimeout(() => setSampleStatus(''), 5000);
          return;
        }
      }

      const baseId = (Date.now() & 0xff00);
      // ack 待ちヘルパ (rule.add の ok=true ack を待つ、タイムアウト 2 秒)
      const waitForAck = (cmdName, expectedId) => new Promise((resolve) => {
        const timeout = setTimeout(() => {
          activeClient.removeEventListener('type:ack', handler);
          activeClient.removeEventListener('type:err', errHandler);
          resolve({ ok: false, reason: 'timeout' });
        }, 2000);
        const handler = (ev) => {
          const d = ev.detail;
          if (d.cmd === cmdName && (expectedId === undefined || d.id === expectedId)) {
            clearTimeout(timeout);
            activeClient.removeEventListener('type:ack', handler);
            activeClient.removeEventListener('type:err', errHandler);
            resolve({ ok: !!d.ok });
          }
        };
        const errHandler = (ev) => {
          // parse_error 等 cmd 不明エラーも検出
          clearTimeout(timeout);
          activeClient.removeEventListener('type:ack', handler);
          activeClient.removeEventListener('type:err', errHandler);
          resolve({ ok: false, reason: ev.detail?.err || 'err' });
        };
        activeClient.addEventListener('type:ack', handler);
        activeClient.addEventListener('type:err', errHandler);
      });

      let failedCount = 0;
      for (let i = 0; i < data.rules.length; i++) {
        const r = { ...data.rules[i], id: baseId + i };
        if (r.posture && Array.isArray(r.posture.euler) && !r.posture.quat) {
          r.posture = {
            ...r.posture,
            quat: eulerToQuat(r.posture.euler[0], r.posture.euler[1], r.posture.euler[2]),
          };
        }
        if (r.end_posture && Array.isArray(r.end_posture.euler) && !r.end_posture.quat) {
          r.end_posture = {
            ...r.end_posture,
            quat: eulerToQuat(r.end_posture.euler[0], r.end_posture.euler[1], r.end_posture.euler[2]),
          };
        }
        if (Array.isArray(r.keys)) {
          r.keys = expandKeySteps(r.keys);
        }
        setSampleStatus(`ルール ${i + 1}/${data.rules.length}: ${r.name} 送信中…`);
        await activeClient.send({ cmd: 'rule.add', r });
        // ack 待ち (request-response、Phase 5.14.6: rule.add drop の根本対策)
        const ackResult = await waitForAck('rule.add', r.id);
        if (!ackResult.ok) {
          failedCount++;
          console.warn(`rule.add ${r.name} (id=${r.id}) failed:`, ackResult);
          // 1 回リトライ
          await new Promise((res) => setTimeout(res, 200));
          await activeClient.send({ cmd: 'rule.add', r });
          const retry = await waitForAck('rule.add', r.id);
          if (retry.ok) failedCount--;
        }
      }
      if (failedCount > 0) {
        setSampleStatus(`⚠ ${failedCount} 件のルール登録に失敗 (タイムアウト/parse_error)`);
      }

      setSampleStatus('プロファイル保存中…');
      await activeClient.send({ cmd: 'profile.save', name: sample.id });
      // rule.list / profile.list 更新
      await activeClient.send({ cmd: 'rule.list' });
      await activeClient.send({ cmd: 'profile.list' });
      setProfileName(sample.id);
      setSampleStatus(`✅ "${sample.title}" を ${data.rules.length} ルールで読み込み完了`);
    } catch (e) {
      setSampleStatus(`❌ エラー: ${e.message || e}`);
    } finally {
      setSampleLoading(null);
      // 5 秒後にステータスをクリア
      setTimeout(() => setSampleStatus(''), 5000);
    }
  };

  const usbSupported = 'serial' in navigator;
  const bleSupported = 'bluetooth' in navigator;

  return html`
  <div class="max-w-6xl mx-auto p-4">
    <header class="flex items-center justify-between mb-4 pb-3 border-b border-slate-200 gap-3 flex-wrap">
      <div>
        <h1 class="text-2xl font-bold">🎮 Burst Motion — 設定アプリ</h1>
        <p class="text-sm text-slate-500">Web Serial (USB) / Web Bluetooth (BLE NUS) 両対応 — Phase 2</p>
      </div>
      <div class="flex items-center gap-3 flex-wrap">
        <!-- 大型ボタン状態インジケータ (常時表示、Stream ON/OFF 不問) -->
        ${connected && currentButtons.length > 0 ? html`
          <div class="flex items-center gap-2 px-3 py-2 bg-violet-50 border border-violet-200 rounded-lg">
            <span class="text-xs text-slate-600 font-semibold">BTN:</span>
            ${currentButtons.map((b) => {
              const pressed = ((liveBtnBitmap >> (b.idx - 1)) & 1) === 1;
              return html`
                <div class="flex flex-col items-center gap-0.5"
                     title="idx=${b.idx} GPIO ${b.gpio} (${b.active_low ? 'active_low' : 'active_high'}, ${b.pull_mode === 1 ? 'PU' : b.pull_mode === 2 ? 'PD' : 'INPUT'})">
                  <div class="w-8 h-8 rounded-full flex items-center justify-center text-sm font-bold border-2 transition-all
                    ${pressed
                      ? 'bg-emerald-500 text-white border-emerald-700 scale-110 shadow-lg animate-pulse'
                      : 'bg-slate-100 text-slate-400 border-slate-300'}">
                    ${b.idx}
                  </div>
                  <span class="text-[9px] font-mono ${pressed ? 'text-emerald-700 font-bold' : 'text-slate-400'}">G${b.gpio}</span>
                </div>
              `;
            })}
            <div class="flex flex-col items-end gap-0.5 ml-1">
              <span class="text-[10px] font-mono ${liveBtnUpdatedMs && (Date.now() - liveBtnUpdatedMs) < 2000 ? 'text-emerald-700' : 'text-amber-600'}">
                ${liveBtnUpdatedMs ? `${Date.now() - liveBtnUpdatedMs}ms` : '⚠ 未受信'}
              </span>
              <button onClick=${() => sendCmd({ cmd: 'hw.buttons.get' })}
                class="text-[10px] px-1.5 py-0.5 bg-violet-200 hover:bg-violet-300 rounded font-semibold"
                title="hw.buttons.get を即送信 (ログで送受信を確認)">
                🔍 Test
              </button>
            </div>
          </div>
        ` : null}
        <span class="text-[10px] font-mono px-2 py-1 bg-slate-100 rounded text-slate-500" title="${appDeployedAt}">
          v${appVersion || 'dev'}
        </span>
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

    <!-- FW Phase 古い警告 (Closest-only ON で Phase < 5.15 = tol 重み付け未対応) -->
    ${connected && closestOnlyMode && deviceInfo?.fw_phase && parseFloat(deviceInfo.fw_phase) < 5.15 ? html`
      <div class="mb-3 p-3 bg-amber-50 border-l-4 border-amber-400 text-sm">
        <div class="font-semibold text-amber-800 mb-1">⚠ FW Phase ${deviceInfo.fw_phase} は Closest 計算の tol 重み付けに未対応</div>
        <div class="text-xs text-amber-700">
          Phase 5.15+ で「<code>euler_tol = 180</code> の軸を Closest 計算で実質除外する」アルゴリズムが導入されました。
          現在の FW では shoryuken のような「Roll 任意 / Pitch 厳密」ルールが期待通り選定されない可能性があります。<br/>
          → Chrome タブで「切断」→ 開発者に FW フラッシュを依頼 → Phase 5.15+ にしてください。
        </div>
      </div>
    ` : null}

    <!-- ボタン未受信時の診断バナー -->
    ${connected && currentButtons.length > 0 && (!liveBtnUpdatedMs || (Date.now() - liveBtnUpdatedMs) > 3000) ? html`
      <div class="mb-3 p-3 bg-amber-50 border-l-4 border-amber-400 text-sm">
        <div class="font-semibold text-amber-800 mb-1">⚠ ボタン状態が FW から受信できていません</div>
        <ul class="text-xs text-amber-700 list-disc ml-5 space-y-0.5">
          <li>新 FW (Phase 5.2 以降) を焼いていますか? 旧 FW は <code>hw.buttons.get</code> コマンド未対応です。
            ヘッダの <b>🔍 Test</b> ボタンを押下 → 下のログに <code>← {"type":"err",...,"err":"unknown_cmd"}</code> が出れば旧 FW 確定。</li>
          <li>Web ページを <b>Ctrl+F5</b> で強制リロード (キャッシュ確認)。ヘッダ右の <code>v...</code> が <code>v20260425-22xxxx</code> 以降なら最新。</li>
          <li>Stream が ON なら sensor の "btn" フィールドから取得、OFF なら 200ms 周期で <code>hw.buttons.get</code> 自動送信中。
            ログにも何もない場合は <b>接続が切れている</b>可能性。ヘッダの "切断" ボタンが見えるか確認してください。</li>
        </ul>
      </div>
    ` : null}

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
          <button onClick=${handleBleStop} disabled=${!connected} class="px-3 py-1 text-sm bg-rose-200 hover:bg-rose-300 rounded disabled:opacity-40">BLE Stop</button>
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
          <div class="flex gap-1">
            <button onClick=${() => { viewerRef.current?.initBase(); }} class="text-xs px-2 py-1 bg-blue-200 hover:bg-blue-300 rounded">Init Yaw</button>
            <button onClick=${() => { viewerRef.current?.resetBase(); }} class="text-xs px-2 py-1 bg-slate-200 rounded">Reset Base</button>
          </div>
        </div>
        <div class="flex flex-wrap gap-2 mb-2 text-xs">
          <label class="flex items-center gap-1 cursor-pointer">
            <input type="checkbox" checked=${showWorldAxes} onChange=${(e) => setShowWorldAxes(e.target.checked)} />
            <span>World 軸 (赤=X 緑=Y 青=Z)</span>
          </label>
          <label class="flex items-center gap-1 cursor-pointer">
            <input type="checkbox" checked=${showBodyAxes} onChange=${(e) => setShowBodyAxes(e.target.checked)} />
            <span>Body 軸</span>
          </label>
          <label class="flex items-center gap-1 cursor-pointer">
            <input type="checkbox" checked=${showGravity} onChange=${(e) => setShowGravity(e.target.checked)} />
            <span>重力ベクトル (水色)</span>
          </label>
        </div>
        <canvas ref=${canvasRef} style="width:100%; height:240px; display:block; border-radius:6px; background:#000;"></canvas>
        ${sensor ? html`
          <div class="grid grid-cols-2 md:grid-cols-4 gap-2 text-xs font-mono mt-2">
            <div class="bg-sky-50 rounded p-2">
              <div class="text-slate-500">Accel [g]</div>
              <div><span class="text-red-600">X:</span> ${(sensor.ax/9.80665)?.toFixed(2)}</div>
              <div><span class="text-emerald-600">Y:</span> ${(sensor.ay/9.80665)?.toFixed(2)}</div>
              <div><span class="text-blue-600">Z:</span> ${(sensor.az/9.80665)?.toFixed(2)}</div>
              <div class="font-bold border-t border-sky-200 pt-0.5 mt-0.5">RMS: ${(Math.sqrt((sensor.ax/9.80665)**2 + (sensor.ay/9.80665)**2 + (sensor.az/9.80665)**2))?.toFixed(2)}</div>
            </div>
            <div class="bg-pink-50 rounded p-2">
              <div class="text-slate-500">Gyro [°/s]</div>
              <div><span class="text-red-600">X:</span> ${sensor.gx?.toFixed(1)}</div>
              <div><span class="text-emerald-600">Y:</span> ${sensor.gy?.toFixed(1)}</div>
              <div><span class="text-blue-600">Z:</span> ${sensor.gz?.toFixed(1)}</div>
              <div class="font-bold border-t border-pink-200 pt-0.5 mt-0.5">RMS: ${(Math.sqrt(sensor.gx*sensor.gx + sensor.gy*sensor.gy + sensor.gz*sensor.gz))?.toFixed(1)}</div>
            </div>
            <div class="bg-emerald-50 rounded p-2">
              <div class="text-slate-500">Euler [°]</div>
              <div>R: ${sensor.roll?.toFixed(1)}</div>
              <div>P: ${sensor.pitch?.toFixed(1)}</div>
              <div>Y: ${sensor.yaw?.toFixed(1)}</div>
            </div>
            <div class="bg-violet-50 rounded p-2">
              <div class="text-slate-500">Btn (FW bitmap)</div>
              ${sensor.btn === undefined ? html`
                <div class="text-amber-600 text-[10px]">btn 未受信<br/>FW 古い?</div>
              ` : html`
                <div class="font-mono">0x${(sensor.btn).toString(16).padStart(2,'0')} = ${(sensor.btn).toString(2).padStart(Math.max(3, currentButtons.length), '0')}b</div>
                <div class="flex gap-1 mt-1 flex-wrap">
                  ${currentButtons.map((b) => {
                    const pressed = ((sensor.btn >> (b.idx - 1)) & 1) === 1;
                    return html`<span class="px-1 rounded ${pressed ? 'bg-violet-500 text-white font-bold' : 'bg-slate-200 text-slate-500'}">
                      ${b.idx}=${pressed ? '🔴' : '⚪'}
                    </span>`;
                  })}
                </div>
              `}
            </div>
          </div>
        ` : html`<p class="text-xs text-slate-400 mt-2 text-center">${streamRate === 0 ? 'Stream OFF (3D は QW/Q* 受信で動作)' : '待機中…'}</p>`}

        <!-- 時系列波形チャート (Accel & Gyro 各 XYZ + RMS) -->
        <div class="grid grid-cols-1 md:grid-cols-2 gap-2 mt-3">
          <div>
            <div class="text-[10px] text-slate-500 mb-0.5 flex items-center gap-2 font-mono">
              <span>Accel [g] 時系列</span>
              <span class="text-red-600">━ X</span>
              <span class="text-emerald-600">━ Y</span>
              <span class="text-blue-600">━ Z</span>
              <span class="text-slate-900 font-bold">━ RMS</span>
            </div>
            <canvas ref=${accelChartCanvasRef} style="width:100%; height:80px; display:block; border-radius:4px;"></canvas>
          </div>
          <div>
            <div class="text-[10px] text-slate-500 mb-0.5 flex items-center gap-2 font-mono">
              <span>Gyro [°/s] 時系列</span>
              <span class="text-red-600">━ X</span>
              <span class="text-emerald-600">━ Y</span>
              <span class="text-blue-600">━ Z</span>
              <span class="text-slate-900 font-bold">━ RMS</span>
            </div>
            <canvas ref=${gyroChartCanvasRef} style="width:100%; height:80px; display:block; border-radius:4px;"></canvas>
          </div>
        </div>
      </div>

      <!-- Roll/Pitch 2D グリッド -->
      <div class="bg-white rounded-lg shadow-sm border border-slate-200 p-4 lg:col-span-2">
        <div class="flex justify-between items-center mb-2">
          <h2 class="font-semibold">📐 Roll / Pitch 2D マップ</h2>
          <span class="text-xs text-slate-500">
            🔴 現在  🟠 登録ルール  🟢 最近傍
            ${closestRuleIdx >= 0 && ruleReferences[closestRuleIdx] ?
              html` (最近傍: <b>${ruleReferences[closestRuleIdx].name}</b>)` : null}
          </span>
        </div>
        <canvas ref=${gridCanvasRef} style="width:100%; height:180px; display:block; border-radius:6px; background:#f1f5f9;"></canvas>
        <div class="mt-2 p-2 bg-emerald-50 rounded space-y-2">
          <div class="flex items-center justify-between">
            <label class="flex items-center gap-2 text-sm">
              <input type="checkbox" checked=${closestOnlyMode}
                onChange=${(e) => {
                  const v = e.target.checked;
                  setClosestOnlyMode(v);
                  sendCmd({ cmd: 'engine.closest_only', enabled: v });
                }}
                disabled=${!connected} />
              <b>Closest-only モード</b>
              <span class="text-xs text-slate-500">
                (ボタン押下時の姿勢で最近傍ルール lock → window 内で他条件成立 → 発火)
              </span>
            </label>
            <span class="text-xs ${closestOnlyMode ? 'text-emerald-700 font-semibold' : 'text-slate-400'}">
              ${closestOnlyMode ? 'ON' : 'OFF: 全マッチ並列発火'}
            </span>
          </div>
          ${closestOnlyMode ? html`
            <div class="flex items-center gap-2 text-xs flex-wrap pt-1 border-t border-emerald-200">
              <span class="font-semibold">⏱ Button lock window:</span>
              <input type="number" min="50" max="5000" step="50" value=${lockWindowMs}
                onInput=${(e) => setLockWindowMs(e.target.value)}
                class="border rounded px-1 py-0.5 w-16 font-mono" /> ms
              <span class="text-slate-500">(ボタン押下後この時間内に他条件成立で発火)</span>
              <span class="font-semibold ml-3">🚫 Cooldown:</span>
              <input type="number" min="0" max="5000" step="50" value=${lockCooldownMs}
                onInput=${(e) => setLockCooldownMs(e.target.value)}
                class="border rounded px-1 py-0.5 w-16 font-mono" /> ms
              <span class="text-slate-500">(発火後この時間は再 lock 不可)</span>
              <button onClick=${handleSendLockParams} disabled=${!connected}
                class="ml-auto px-2 py-0.5 bg-emerald-200 hover:bg-emerald-300 rounded font-semibold disabled:opacity-40">
                📤 適用
              </button>
            </div>
            ${lockedRuleId >= 0 ? html`
              <div class="text-xs text-amber-700 font-semibold animate-pulse">
                🔒 Button-locked: rule id=${lockedRuleId} (${Math.round((Date.now() - lockedAt))}ms)
              </div>
            ` : null}
          ` : null}
        </div>
        <p class="text-xs text-slate-500 mt-1">
          現在の姿勢と登録ルール姿勢を 2D 平面に投影 (Roll: -180~180°、Pitch: -90~90°)。
          ルール追加時に「📷 開始姿勢」キャプチャ後の姿勢が橙ドットで表示される。
        </p>
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

        <!-- Hardware 選択 (ボタン定義用) -->
        ${Object.keys(hardwareDefs).length > 0 ? html`
          <div class="mb-3 p-2 bg-slate-100 rounded text-xs">
            <div class="flex items-center gap-2 flex-wrap">
              <span class="font-semibold">⚙ Hardware:</span>
              <select value=${selectedHardware}
                onChange=${(e) => setSelectedHardware(e.target.value)}
                class="border rounded px-2 py-0.5 text-xs">
                ${Object.entries(hardwareDefs).map(([id, def]) => html`
                  <option value=${id}>${def.title} (${def.buttons?.length || 0} btn)</option>
                `)}
              </select>
              ${deviceInfo?.board && deviceInfo.board === selectedHardware ? html`
                <span class="ml-auto text-emerald-700 font-semibold">✓ 接続中デバイスと一致</span>
              ` : deviceInfo?.board ? html`
                <span class="ml-auto text-amber-600">⚠ 接続中 board: ${deviceInfo.board}</span>
              ` : null}
            </div>
            ${currentButtons.length > 0 ? html`
              <table class="mt-2 w-full text-xs">
                <thead class="text-slate-500">
                  <tr>
                    <th class="text-left px-1">idx</th>
                    <th class="text-left px-1">名前</th>
                    <th class="text-left px-1">GPIO</th>
                    <th class="text-left px-1">論理</th>
                    <th class="text-left px-1">PU</th>
                    <th class="text-left px-1">場所</th>
                  </tr>
                </thead>
                <tbody>
                  ${currentButtons.map((b) => html`
                    <tr class="border-t border-slate-200">
                      <td class="px-1 font-mono">${b.idx}</td>
                      <td class="px-1">${b.name}</td>
                      <td class="px-1 font-mono">G${b.gpio}</td>
                      <td class="px-1">${b.active_low ? 'active_low' : 'active_high'}</td>
                      <td class="px-1">${b.pull_mode === 1 ? 'PU' : b.pull_mode === 2 ? 'PD' : '-'}</td>
                      <td class="px-1 text-slate-500">${b.location || ''}</td>
                    </tr>
                  `)}
                </tbody>
              </table>
              <div class="mt-2 flex items-center gap-2 flex-wrap">
                <button onClick=${handleApplyButtonsToFw} disabled=${!connected}
                  class="px-2 py-0.5 bg-emerald-200 hover:bg-emerald-300 rounded disabled:opacity-40 text-xs font-semibold">
                  📤 FW に適用 (hw.buttons.set + NVS 保存)
                </button>
                ${buttonsMatchFw === true ? html`
                  <span class="text-emerald-700 font-semibold">✓ FW 構成と一致</span>
                ` : buttonsMatchFw === false ? html`
                  <span class="text-amber-600">⚠ FW 構成と異なる (適用が必要)</span>
                ` : html`
                  <span class="text-slate-400">FW 構成 未取得</span>`}
                ${fwButtonsStatus ? html`
                  <span class="text-emerald-700">${fwButtonsStatus}</span>
                ` : null}
              </div>
              ${fwButtons ? html`
                <div class="mt-1 text-slate-500">
                  FW 現在: ${fwButtons.map((b, i) => html`
                    ${i > 0 ? ' / ' : ''}idx${b.idx}=G${b.gpio}(${b.active_low ? 'L' : 'H'},${b.pull_mode === 1 ? 'PU' : b.pull_mode === 2 ? 'PD' : '-'})
                  `)}
                </div>
              ` : null}
            ` : null}
          </div>
        ` : null}

        <!-- 発火フラッシュ (発火時に何が入力されたか見えるように) -->
        ${triggerFlash ? html`
          <div class="mb-3 p-2 bg-yellow-100 border border-yellow-400 rounded animate-pulse text-sm font-semibold text-yellow-800">
            🔥 #${triggerFlash.id} <b>${triggerFlash.name}</b> ${triggerFlash.phase}
            ${triggerFlash.keys && triggerFlash.keys.length > 0 ? html`
              → <span class="ml-1 px-2 py-0.5 bg-yellow-300 rounded font-mono">
                ${triggerFlash.keys.map(hidCodeToName).join(' ')}
              </span>
            ` : null}
          </div>
        ` : null}

        <!-- 登録済みルール一覧 -->
        ${ruleList.length > 0 ? html`
          <div class="mb-3 max-h-48 overflow-y-auto border rounded">
            <table class="w-full text-xs">
              <thead class="bg-slate-100 sticky top-0">
                <tr>
                  <th class="px-2 py-1 text-left">ID</th>
                  <th class="px-2 py-1 text-left">Name</th>
                  <th class="px-2 py-1 text-center">St.</th>
                  <th class="px-2 py-1 text-center">Loop</th>
                  <th class="px-2 py-1 text-left">Btn</th>
                  <th class="px-2 py-1 text-left">Posture</th>
                  <th class="px-2 py-1 text-left">Accel</th>
                  <th class="px-2 py-1 text-left">Action</th>
                  <th class="px-2 py-1 text-center"></th>
                </tr>
              </thead>
              <tbody>
                ${ruleList.map((r) => {
                  // ボタン条件評価 (sensor.btn と r.button.idx を比較)
                  let btnEval = '';
                  if (r.button) {
                    const idx = r.button.idx;
                    const wantPressed = r.button.state === 0;
                    const wantReleased = r.button.state === 1;
                    const isPressed = sensor && sensor.btn !== undefined ? ((sensor.btn >> (idx - 1)) & 1) === 1 : null;
                    const ok = (r.button.state === 2) ||
                              (wantPressed && isPressed === true) ||
                              (wantReleased && isPressed === false);
                    btnEval = `idx${idx}${r.button.state===0?'押':r.button.state===1?'離':'?'}${isPressed===null?'':ok?'✓':'✗'}`;
                  }
                  return html`
                    <tr class="${triggerFlash && triggerFlash.id === r.id ? 'bg-yellow-100' : ''} border-t">
                      <td class="px-2 py-1 font-mono">${r.id}</td>
                      <td class="px-2 py-1">${r.name}</td>
                      <td class="px-2 py-1 text-center">${r.states_count}${r.current_state >= 0 ? `🟢${r.current_state}` : ''}</td>
                      <td class="px-2 py-1 text-center">${r.loop ? '🔁' : '➡️'}</td>
                      <td class="px-2 py-1 font-mono ${r.button ? (btnEval.endsWith('✓') ? 'text-emerald-700' : btnEval.endsWith('✗') ? 'text-red-600' : '') : 'text-slate-300'}">
                        ${btnEval || '-'}
                      </td>
                      <td class="px-2 py-1 font-mono ${r.posture ? '' : 'text-slate-300'}">
                        ${r.posture ? `R${r.posture.euler[0]?.toFixed(0)}P${r.posture.euler[1]?.toFixed(0)}±${r.posture.euler_tol[0]?.toFixed(0)}` : '-'}
                      </td>
                      <td class="px-2 py-1 font-mono ${r.accel ? '' : 'text-slate-300'}">
                        ${r.accel ? `≥${r.accel.abs_threshold?.toFixed(1)}g` : '-'}
                      </td>
                      <td class="px-2 py-1 font-mono ${r.action && r.action.keys && r.action.keys.length > 0 ? '' : 'text-red-400'}">
                        ${r.action ? html`
                          <span class="text-[9px] text-slate-400">${r.action.type_name || '?'}</span>
                          ${r.action.keys && r.action.keys.length > 0 ? html`
                            <span class="ml-1 inline-flex flex-wrap items-center">
                              ${r.action.type_name === 'fire_macro'
                                ? renderMacroSteps(r.action.keys, r.action.key_modes)
                                : r.action.keys.map(hidCodeToName).join(' ')}
                            </span>
                          ` : html`<span class="ml-1 text-red-500">空!</span>`}
                        ` : '-'}
                      </td>
                      <td class="px-1 py-1 text-center">
                        <button onClick=${() => handleRemoveRule(r.id, r.name)}
                          disabled=${!connected}
                          class="px-1.5 py-0.5 text-xs bg-red-200 hover:bg-red-400 hover:text-white rounded disabled:opacity-30"
                          title="ルール ${r.name} (id=${r.id}) を削除">🗑</button>
                      </td>
                    </tr>
                  `;
                })}
              </tbody>
            </table>
          </div>
          <div class="text-xs text-slate-500 mb-2">
            St.列の 🟢N は state[N] 滞在中。Btn列の ✓=条件成立、✗=不成立 (現在 sensor.btn で判定)。
            <button onClick=${handleListRules} disabled=${!connected} class="ml-2 px-2 py-0.5 bg-slate-200 hover:bg-slate-300 rounded text-xs">🔄 rule.list</button>
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
            <div class="flex items-center gap-2 mb-1 flex-wrap text-xs">
              <span>許容 ±</span>
              <input type="number" min="5" max="90" step="5" value=${postureTol}
                onInput=${(e) => setPostureTol(e.target.value)} class="border rounded px-1 py-0.5 w-12 font-mono" />
              <span>°</span>
              <span class="ml-3">判定:</span>
              <select value=${postureJudgeBy} onChange=${(e) => setPostureJudgeBy(e.target.value)}
                class="border rounded px-1 py-0.5">
                <option value="euler">Euler 角 (Roll/Pitch tol、推奨)</option>
                <option value="quat">Quaternion 内積</option>
              </select>
            </div>
            <div class="flex items-center gap-2 mb-1 flex-wrap text-xs">
              <button onClick=${captureStartPosture} disabled=${!connected || !sensor}
                class="px-2 py-0.5 bg-cyan-200 hover:bg-cyan-300 rounded disabled:opacity-40">📷 開始姿勢</button>
              ${startPosture ? html`
                <span class="font-mono text-cyan-700">R:${startPosture.euler[0].toFixed(0)} P:${startPosture.euler[1].toFixed(0)} Y:${startPosture.euler[2].toFixed(0)}</span>
                <button onClick=${clearStartPosture} class="text-xs text-red-600 hover:underline">×</button>
                ${Math.abs(startPosture.euler[1]) > 65 ? html`
                  <span class="text-[11px] text-red-700 font-semibold bg-red-100 px-2 py-0.5 rounded border border-red-300"
                        title="Pitch が ±65° を超えるとジンバルロック領域に入り、Mahony Euler 出力が不安定になります。Roll/Yaw が縮退してルール判定が機能しません。">
                    ⚠ Pitch ${startPosture.euler[1].toFixed(0)}° はジンバルロック (>±65°) — Euler 判定不可
                  </span>
                ` : null}
              ` : html`<span class="text-slate-400">未取得 (Stream ON で取得可)</span>`}
            </div>
            ${ruleMode === 'hold_start_end' ? html`
              <div class="flex items-center gap-2 flex-wrap text-xs">
                <button onClick=${captureEndPosture} disabled=${!connected || !sensor}
                  class="px-2 py-0.5 bg-orange-200 hover:bg-orange-300 rounded disabled:opacity-40">📷 終了姿勢</button>
                ${endPosture ? html`
                  <span class="font-mono text-orange-700">R:${endPosture.euler[0].toFixed(0)} P:${endPosture.euler[1].toFixed(0)} Y:${endPosture.euler[2].toFixed(0)}</span>
                  <button onClick=${clearEndPosture} class="text-xs text-red-600 hover:underline">×</button>
                  ${Math.abs(endPosture.euler[1]) > 65 ? html`
                    <span class="text-[11px] text-red-700 font-semibold bg-red-100 px-2 py-0.5 rounded border border-red-300"
                          title="Pitch が ±65° を超えるとジンバルロック領域に入り、Euler 判定が不安定になります。">
                      ⚠ Pitch ${endPosture.euler[1].toFixed(0)}° はジンバルロック (>±65°) — Euler 判定不可
                    </span>
                  ` : null}
                ` : html`<span class="text-slate-400">未取得</span>`}
              </div>
            ` : null}
          </div>

          <!-- ボタン条件 (Hardware に応じて利用可能ボタンを提示) -->
          <div class="border rounded p-2 bg-violet-50">
            <div class="flex items-center justify-between mb-1">
              <label class="flex items-center gap-1 text-xs font-semibold text-slate-600 cursor-pointer">
                <input type="checkbox" checked=${ruleButtonEnabled}
                  onChange=${(e) => setRuleButtonEnabled(e.target.checked)}
                  disabled=${currentButtons.length === 0} />
                ボタン条件 (任意)
              </label>
              <span class="text-xs text-slate-500">
                Hardware: <b>${hardwareDefs[selectedHardware]?.title || selectedHardware}</b>
              </span>
            </div>
            ${currentButtons.length === 0 ? html`
              <div class="text-xs text-slate-400">
                Hardware 定義が未取得 or ボタン定義なし
              </div>
            ` : html`
              <div class="flex items-center gap-2 flex-wrap text-xs">
                <span>ボタン:</span>
                <select value=${ruleButtonIdx}
                  onChange=${(e) => setRuleButtonIdx(parseInt(e.target.value, 10))}
                  disabled=${!ruleButtonEnabled}
                  class="border rounded px-1 py-0.5 disabled:opacity-40">
                  ${currentButtons.map((b) => html`
                    <option value=${b.idx}>
                      idx=${b.idx} ${b.name} (GPIO ${b.gpio}, ${b.location})
                    </option>
                  `)}
                </select>
                <span>状態:</span>
                <select value=${ruleButtonState}
                  onChange=${(e) => setRuleButtonState(parseInt(e.target.value, 10))}
                  disabled=${!ruleButtonEnabled}
                  class="border rounded px-1 py-0.5 disabled:opacity-40">
                  <option value="0">押下中</option>
                  <option value="1">解放中</option>
                  <option value="2">どちらでも</option>
                </select>
              </div>
            `}
          </div>

          <!-- 出力アクション -->
          <div class="border rounded p-2 bg-emerald-50">
            <div class="text-xs font-semibold text-slate-600 mb-1">
              出力 HID キー${ruleMode === 'hold_start_end' ? ' (開始姿勢で press → 終了姿勢で release)' : ''}
            </div>
            <div class="flex items-center gap-1 mb-1 flex-wrap text-xs">
              <span>修飾:</span>
              <label class="flex items-center gap-0.5"><input type="checkbox" checked=${modCtrl} onChange=${(e)=>setModCtrl(e.target.checked)} />Ctrl</label>
              <label class="flex items-center gap-0.5"><input type="checkbox" checked=${modShift} onChange=${(e)=>setModShift(e.target.checked)} />Shift</label>
              <label class="flex items-center gap-0.5"><input type="checkbox" checked=${modAlt} onChange=${(e)=>setModAlt(e.target.checked)} />Alt</label>
              <label class="flex items-center gap-0.5"><input type="checkbox" checked=${modGui} onChange=${(e)=>setModGui(e.target.checked)} />Win</label>
              <span class="ml-3">入力:</span>
              <select value=${ruleInputMode} onChange=${(e) => setRuleInputMode(e.target.value)}
                class="border rounded px-1 py-0.5">
                <option value="single">単一キー</option>
                <option value="macro">連続入力 (macro)</option>
              </select>
            </div>
            ${ruleInputMode === 'single' ? html`
              <div class="flex items-center gap-2 flex-wrap text-xs">
                <span>キー:</span>
                <input type="text" value=${ruleKey} onInput=${(e) => setRuleKey(e.target.value)}
                  placeholder="a / ARROW_RIGHT / F1 / ENTER 等"
                  class="border rounded px-2 py-1 w-44 font-mono" />
                <span class="text-slate-500">
                  ${modCtrl?'Ctrl+':''}${modShift?'Shift+':''}${modAlt?'Alt+':''}${modGui?'Win+':''}<b>${ruleKey}</b>
                </span>
              </div>
              <div class="text-[10px] text-slate-500 mt-1">
                対応: 1 文字 ASCII / ARROW_LEFT,RIGHT,UP,DOWN / ENTER / ESC / TAB / SPACE / BACKSPACE / DELETE / HOME / END / PAGE_UP / PAGE_DOWN / F1〜F24
              </div>
            ` : html`
              <div class="flex items-center gap-2 flex-wrap text-xs">
                <span>キー列:</span>
                <input type="text" value=${ruleKeysList} onInput=${(e) => setRuleKeysList(e.target.value)}
                  placeholder="DOWN, RIGHT, p"
                  class="border rounded px-2 py-1 flex-1 font-mono min-w-48" />
                <span>間隔:</span>
                <input type="number" min="10" max="500" step="10" value=${ruleKeyInterval}
                  onInput=${(e) => setRuleKeyInterval(e.target.value)}
                  class="border rounded px-1 py-0.5 w-16 font-mono" /> ms
              </div>
              <div class="text-[10px] text-slate-500 mt-1">
                カンマ区切り。各要素はそのステップで押下中のキー集合 (<code>+</code> で同時押し連結)。
                例:<br/>
                ・単純順次: <code>DOWN, RIGHT, p</code> (各 press→release)<br/>
                ・<b>同時押しコマンド</b>: <code>DOWN, DOWN+RIGHT, RIGHT, RIGHT+p</code> (差分自動 release/press、波動拳 ↓↘→P 風)
              </div>
            `}
          </div>

          ${ruleMode === 'hold_start_end' ? html`
            <!-- 終了側 別キー (オプション) -->
            <div class="border rounded p-2 bg-orange-50">
              <div class="text-xs font-semibold text-slate-600 mb-1">
                ＋ 終了姿勢時に追加発火するキー (任意、空なら start key の release のみ)
              </div>
              <div class="flex items-center gap-1 mb-1 flex-wrap text-xs">
                <span>修飾:</span>
                <label class="flex items-center gap-0.5"><input type="checkbox" checked=${endModCtrl}  onChange=${(e)=>setEndModCtrl(e.target.checked)} />Ctrl</label>
                <label class="flex items-center gap-0.5"><input type="checkbox" checked=${endModShift} onChange=${(e)=>setEndModShift(e.target.checked)} />Shift</label>
                <label class="flex items-center gap-0.5"><input type="checkbox" checked=${endModAlt}   onChange=${(e)=>setEndModAlt(e.target.checked)} />Alt</label>
                <label class="flex items-center gap-0.5"><input type="checkbox" checked=${endModGui}   onChange=${(e)=>setEndModGui(e.target.checked)} />Win</label>
              </div>
              <div class="flex items-center gap-2">
                <span class="text-xs">+ キー:</span>
                <input type="text" value=${endKey} onInput=${(e) => setEndKey(e.target.value)}
                  maxlength="1" placeholder="(空)" class="border rounded px-2 py-1 w-12 text-center font-mono" />
                <span class="text-xs text-slate-500">
                  ${endKey ? `${endModCtrl?'Ctrl+':''}${endModShift?'Shift+':''}${endModAlt?'Alt+':''}${endModGui?'Win+':''}${endKey} を 30ms FIRE_ONCE` : '— (release のみ)'}
                </span>
              </div>
            </div>
          ` : null}

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

    <!-- Stream Debug -->
    ${streamGaps.length > 0 || streamRate > 0 ? html`
      <div class="mt-4 bg-white rounded-lg shadow-sm border border-slate-200 p-4">
        <div class="flex justify-between items-center mb-2">
          <h2 class="font-semibold">⏱ Stream タイミング (FW timestamp 差)</h2>
          <button onClick=${handleClearStreamStats} class="text-xs text-slate-400 hover:text-slate-700">Clear</button>
        </div>
        <p class="text-xs text-slate-500 mb-2">
          50Hz Stream の理想 gap = 20ms。大きい値が頻発するとフリーズ気味。
          差は <code>sensor.t</code> 同士なので、ブラウザ受信遅延ではなく FW 内部の delay。
        </p>
        ${streamStats.count > 0 ? html`
          <div class="grid grid-cols-5 gap-2 text-sm font-mono mb-2">
            <div class="bg-slate-50 rounded p-2 text-center">
              <div class="text-xs text-slate-500">サンプル数</div>
              <div class="font-bold">${streamStats.count}</div>
            </div>
            <div class="bg-slate-50 rounded p-2 text-center">
              <div class="text-xs text-slate-500">avg</div>
              <div class="font-bold">${streamStats.avg.toFixed(1)} ms</div>
            </div>
            <div class="bg-emerald-50 rounded p-2 text-center">
              <div class="text-xs text-slate-500">min</div>
              <div class="font-bold">${streamStats.min.toFixed(0)} ms</div>
            </div>
            <div class="bg-yellow-50 rounded p-2 text-center">
              <div class="text-xs text-slate-500">p95</div>
              <div class="font-bold ${streamStats.p95 > 50 ? 'text-amber-700' : ''}">${streamStats.p95.toFixed(0)} ms</div>
            </div>
            <div class="bg-red-50 rounded p-2 text-center">
              <div class="text-xs text-slate-500">max</div>
              <div class="font-bold ${streamStats.max > 100 ? 'text-red-700' : ''}">${streamStats.max.toFixed(0)} ms</div>
            </div>
          </div>
          <!-- ヒストグラム (簡易): 直近 100 サンプルを bar chart 風に表示 -->
          <div class="flex items-end gap-px h-12 bg-slate-100 rounded p-1" style="overflow-x:auto;">
            ${streamGaps.slice(-100).map((g) => {
              const v = g.fw;
              const h = Math.min(100, (v / 100) * 100);  // 100ms = 100% bar
              const color = v < 25 ? '#10b981' : v < 50 ? '#fbbf24' : v < 100 ? '#f97316' : '#ef4444';
              return html`<div style="width:4px; height:${h}%; background:${color}; flex-shrink:0;" title=${`${v.toFixed(1)}ms`}></div>`;
            })}
          </div>
          <div class="text-xs text-slate-400 mt-1">
            色: 🟢&lt;25ms (理想 ~20)  🟡 25-50ms  🟠 50-100ms (LCD 干渉?)  🔴 &gt;100ms (停滞)
          </div>
        ` : html`<p class="text-sm text-slate-400">Stream ON にすると統計取得開始</p>`}
      </div>
    ` : null}

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

    <!-- サンプルプロファイル (リポジトリ同梱、Phase 5) -->
    ${samples.length > 0 ? html`
      <div class="mt-4 bg-white rounded-lg shadow-sm border border-slate-200 p-4">
        <h2 class="font-semibold mb-2">🎁 サンプルプロファイル</h2>
        <p class="text-xs text-slate-500 mb-3">
          ワンクリックで既存ルールを上書きし、サンプルを書込みます。動作確認・テンプレートとしてどうぞ。
        </p>
        ${sampleStatus ? html`
          <div class="mb-3 p-2 rounded text-xs font-mono ${sampleStatus.startsWith('❌') ? 'bg-red-100 text-red-700' : sampleStatus.startsWith('✅') ? 'bg-green-100 text-green-700' : 'bg-blue-100 text-blue-700'}">
            ${sampleStatus}
          </div>
        ` : null}
        <div class="grid grid-cols-1 md:grid-cols-2 gap-2">
          ${samples.map((s) => html`
            <div class="border rounded p-2 bg-slate-50 flex flex-col gap-1">
              <div class="font-semibold text-sm">${s.title}</div>
              <div class="text-xs text-slate-600 leading-snug">${s.description}</div>
              <div class="flex items-center justify-between mt-1">
                <span class="text-xs text-slate-400">
                  ${s.rule_count} ルール
                  ${s.tags ? html` · ${s.tags.map((t) => html`<span class="ml-1 px-1 bg-slate-200 rounded">${t}</span>`)}` : null}
                </span>
                <button onClick=${() => handleSampleLoad(s)} disabled=${!connected || sampleLoading !== null}
                  class="px-2 py-0.5 text-xs bg-emerald-200 hover:bg-emerald-300 rounded disabled:opacity-40">
                  ${sampleLoading === s.id ? '⏳ 読込中…' : '📥 適用'}
                </button>
              </div>
            </div>
          `)}
        </div>
      </div>
    ` : null}

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

    <footer class="mt-4 text-center text-xs text-slate-400 space-x-2">
      <span>Burst Motion | USB:115200 / BLE NUS | JSON Lines</span>
      <span class="px-2 py-0.5 bg-slate-100 rounded font-mono">
        Web v${appVersion || 'dev'}${appDeployedAt ? ` (${appDeployedAt})` : ''}
      </span>
      ${deviceInfo ? html`
        <span class="px-2 py-0.5 bg-emerald-100 text-emerald-700 rounded font-mono">
          FW ${deviceInfo.fw}${deviceInfo.fw_phase ? ` Phase ${deviceInfo.fw_phase}` : ''}${deviceInfo.fw_build ? ` (${deviceInfo.fw_build})` : ''}
        </span>
      ` : null}
    </footer>
  </div>
  `;
}

render(html`<${App} />`, document.getElementById('app'));
