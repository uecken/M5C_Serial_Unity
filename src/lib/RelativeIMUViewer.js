// Burst Motion - RelativeIMUViewer (Phase 5.39.2 新規)
//
// 一人称視点 (first-person view) の 3D ビュア。
// - M5C モデル: 中央固定 (default identity)、オプションで q_ref に定着
// - worldGroup: q_current⁻¹ で逆回転 (= ワールドが動いて見える、M5C 視点は固定)
// - 相対モード rule の rel_quat waypoints を worldGroup 内の球面上に配置
// - 軌跡: ハイブリッド (押下前=薄い破線プレビュー、押下後=濃い実線固定)
// - 過去軌跡 trail: 3 秒履歴
//
// 既存 IMUViewer (絶対 3D ビュア) との設計差:
//   ┌─────────────────────────────┬──────────────────────────────┐
//   │ 絶対 IMUViewer              │ 相対 RelativeIMUViewer       │
//   │ M5C を回転 (球は固定)        │ worldGroup を逆回転 (M5C 固定)│
//   │ target.quat = 絶対座標       │ target.euler = q_ref からの相対│
//   └─────────────────────────────┴──────────────────────────────┘

import * as THREE from 'three';

export class RelativeIMUViewer {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x000010);   // 微紫: 一人称視点と分かるよう絶対 (黒) と区別

    const w = canvas.clientWidth || 320;
    const h = canvas.clientHeight || 240;
    this.camera = new THREE.PerspectiveCamera(50, w / h, 0.1, 100);
    this.camera.position.z = -2.5;
    this.camera.lookAt(0, 0, 0);

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setSize(w, h, false);
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.scene.add(new THREE.AmbientLight(0xffffff, 1.0));

    // ---- 球体ワイヤー (scene 固定、絶対ビュアと同じ見た目で基準感を出す) ----
    const sphereGeo = new THREE.SphereGeometry(1, 24, 24);
    const sphereMat = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true, transparent: true, opacity: 0.5 });
    this.sphere = new THREE.Mesh(sphereGeo, sphereMat);
    this.scene.add(this.sphere);

    // ---- worldGroup: M5C を逆回転で動かす対象 (waypoints, trail, 軸ヘルパ) ----
    this.worldGroup = new THREE.Group();
    this.scene.add(this.worldGroup);

    // 軸ヘルパ (worldGroup 内、初期姿勢基準の軸を視認用)
    // ★ 絶対 3D の「World 軸」チェックボックスとは別物、混乱回避のためデフォルト非表示
    this.worldAxes = new THREE.AxesHelper(1.5);
    this.worldAxes.visible = false;
    this.worldGroup.add(this.worldAxes);

    // ---- M5C モデル (中央固定、default identity) ----
    const stickGeo = new THREE.BoxGeometry(0.24, 0.12, 0.48);
    const orange = new THREE.MeshBasicMaterial({ color: 0xffa500 });
    const black  = new THREE.MeshBasicMaterial({ color: 0x000000 });
    const white  = new THREE.MeshBasicMaterial({ color: 0xffffff });
    this.m5StickC = new THREE.Mesh(stickGeo, [orange, orange, black, orange, orange, white]);
    const lcdGeo = new THREE.PlaneGeometry(0.2, 0.05);
    const lcd = new THREE.Mesh(lcdGeo, new THREE.MeshBasicMaterial({ color: 0x111111 }));
    lcd.position.set(0, 0.061, 0.24);
    this.m5StickC.add(lcd);
    const markGeo = new THREE.PlaneGeometry(0.04, 0.02);
    const mark = new THREE.Mesh(markGeo, new THREE.MeshBasicMaterial({ color: 0x33ff33 }));
    mark.position.set(0, -0.061, 0.24);
    this.m5StickC.add(mark);
    this.scene.add(this.m5StickC);

    // ---- 相対 waypoint dots (worldGroup 内に配置) ----
    this.waypointDots = [];     // 紫 dot 配列
    this.waypointLine = null;   // 紫 line (Slerp 接続)

    // ---- 過去軌跡 trail (worldGroup 内、M5C 視点では動いて見える) ----
    this._trail = [];
    this._trailDurationMs = 3000;
    this._initTrailLine();

    // ---- ビュア状態 ----
    this._m5cMode = opts.m5cMode || 'fixed';   // 'fixed' | 'qref_anchor'
    this._trajectoryMode = 'preview';          // 'preview' (破線) | 'fixed' (実線)
    this._qCurrent = new THREE.Quaternion();   // 現在 quat (M5C 軸変換済、Three 系)
    this._qInitial = new THREE.Quaternion();   // 初期姿勢 (基準)、最初の setQuaternion で自動取得
    this._qInitialValid = false;
    this._qRef = new THREE.Quaternion();       // 確定 q_ref (M5C 軸変換済、Three 系)
    this._qRefValid = false;
    this._selectedRule = null;                 // {posture_basis, start_posture, mid_postures, end_posture}
    this.smoothing = opts.smoothing ?? 0.3;

    // resize + RAF
    this._resizeObserver = new ResizeObserver(() => this._onResize());
    this._resizeObserver.observe(canvas);
    this._running = true;
    this._renderEnabled = true;   // タブ非表示時に RAF を止めるためのフラグ
    this._tick = this._tick.bind(this);
    requestAnimationFrame(this._tick);
  }

  _initTrailLine() {
    const MAX = 256;
    const positions = new Float32Array(MAX * 3);
    const colors    = new Float32Array(MAX * 3);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color',    new THREE.BufferAttribute(colors, 3));
    geo.setDrawRange(0, 0);
    const mat = new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 2 });
    this.trailLine = new THREE.Line(geo, mat);
    this.worldGroup.add(this.trailLine);
    this._trailMaxPoints = MAX;
  }

  /** sensor 受信時、worldGroup を q_current⁻¹ で逆回転、M5C 視点で固定する */
  setQuaternion(qw, qx, qy, qz) {
    // 軸変換 (IMUViewer と同じ): M5C(qw,qx,qy,qz) → Three(-qx, qz, qy, qw)
    this._qCurrent.set(-qx, qz, qy, qw);
  }

  /** タブ表示時に RAF を再開、非表示時に停止 (CPU 節約) */
  setRenderEnabled(enabled) {
    const was = this._renderEnabled;
    this._renderEnabled = !!enabled;
    if (!was && this._renderEnabled) requestAnimationFrame(this._tick);
  }

  /** 過去軌跡 trail に新点 (M5C 軸 quat) — worldGroup 内に世界固定で記録 */
  addTrailPoint(qw, qx, qy, qz, timestamp) {
    const t = (typeof timestamp === 'number') ? timestamp : performance.now();
    const q = new THREE.Quaternion(-qx, qz, qy, qw);
    this._trail.push({ q, t });
    this._pruneTrail(t);
    this._updateTrailGeometry(t);
  }

  _pruneTrail(now) {
    const cutoff = now - this._trailDurationMs;
    while (this._trail.length > 0 && this._trail[0].t < cutoff) this._trail.shift();
    if (this._trail.length > this._trailMaxPoints) {
      this._trail.splice(0, this._trail.length - this._trailMaxPoints);
    }
  }

  _updateTrailGeometry(now) {
    if (!this.trailLine) return;
    const geo = this.trailLine.geometry;
    const posAttr = geo.getAttribute('position');
    const colAttr = geo.getAttribute('color');
    const n = this._trail.length;
    for (let i = 0; i < n; i++) {
      const e = this._trail[i];
      const p = this._quatToSpherePoint(e.q).clone().multiplyScalar(1.005);
      posAttr.setXYZ(i, p.x, p.y, p.z);
      const age = (now - e.t) / this._trailDurationMs;
      const alpha = Math.max(0, 1 - age);
      // 赤系 (絶対と同じ)、worldGroup 内なので逆回転して M5C 視点では動いて見える
      colAttr.setXYZ(i, 0.937 * alpha, 0.267 * alpha, 0.267 * alpha);
    }
    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    geo.setDrawRange(0, n);
  }

  clearTrail() {
    this._trail.length = 0;
    if (this.trailLine) this.trailLine.geometry.setDrawRange(0, 0);
  }

  setTrailDurationMs(ms) { this._trailDurationMs = Math.max(200, ms | 0); }

  /** Phase 5.39.2: 選択 rule を相対ビュアに反映
   *  rule = {posture_basis, posture, states[], ...} (FW rule.list 応答形式)
   *
   *  相対モード rule の各 state.posture.euler は「q_ref からの相対 Euler オフセット」
   *  なので、直接 Three quat に変換して waypoint dot として worldGroup に配置する。
   */
  setSelectedRule(rule) {
    this._selectedRule = rule;
    this._rebuildWaypoints();
  }

  /** M5C モデルの動作モード切替 ('fixed' | 'qref_anchor') */
  setM5CMode(mode) {
    this._m5cMode = (mode === 'qref_anchor') ? 'qref_anchor' : 'fixed';
  }

  /** q_ref 確定通知 (trigger.hit phase='enter' で受信)
   *   - valid=true: 実 q_ref を基準姿勢 (q_initial) として設定 → M5C モデル回転開始
   *                 waypoint を固定 (濃い実線)、trajectoryMode='fixed'
   *   - valid=false: q_initial リセット → M5C モデル中央固定、プレビュー軌跡に戻す
   *
   *  ★ ユーザー仕様 (Phase 5.39.2.2):
   *     - ボタン押下前は M5C モデル中央で動かない (q_initial 未設定)
   *     - ボタン押下 (state[0] enter で q_ref 受信) → q_initial = q_ref に設定
   *       以降、M5C モデルは「q_ref 基準でデバイスがどう動いたか」を表示
   *     - ボタン離す → q_initial 解除 → M5C モデル中央に戻る
   */
  setQRef(qrefArr, valid) {
    if (valid && Array.isArray(qrefArr) && qrefArr.length === 4) {
      const [qw, qx, qy, qz] = qrefArr;
      this._qRef.set(-qx, qz, qy, qw);
      this._qRefValid = true;
      // q_ref を初期姿勢として設定 → M5C モデルが q_ref からの相対回転を表示開始
      this._qInitial.copy(this._qRef);
      this._qInitialValid = true;
      this.setTrajectoryMode('fixed');
    } else {
      this._qRef.identity();
      this._qRefValid = false;
      // q_initial も解除 → M5C モデル中央固定に戻る
      this._qInitial.identity();
      this._qInitialValid = false;
      this.setTrajectoryMode('preview');
    }
    // q_ref 変化時は waypoint も描き直し
    this._rebuildWaypoints();
  }

  /** 軌跡表示モード ('preview' = 薄い破線 | 'fixed' = 濃い実線) */
  setTrajectoryMode(mode) {
    this._trajectoryMode = (mode === 'fixed') ? 'fixed' : 'preview';
    this._applyTrajectoryStyle();
  }

  /** rule waypoint を相対 quat として配置
   *  相対モードでは各 state.posture.euler が q_ref からの「相対」オフセット
   *  なので、euler → quat 変換した相対 quat をそのまま球面に配置する。
   */
  _rebuildWaypoints() {
    // 既存削除
    for (const d of this.waypointDots) {
      this.worldGroup.remove(d);
      d.geometry.dispose();
      d.material.dispose();
    }
    this.waypointDots = [];
    if (this.waypointLine) {
      this.worldGroup.remove(this.waypointLine);
      this.waypointLine.geometry.dispose();
      this.waypointLine.material.dispose();
      this.waypointLine = null;
    }
    const rule = this._selectedRule;
    if (!rule) return;

    // states 配列 (Phase 5.39 形式) or 単一 posture (旧形式) から相対 euler 抽出
    const relEulers = this._extractRelEulers(rule);
    if (relEulers.length === 0) return;

    // 各 rel euler を Three quat に変換 (M5C 軸基準で構築 → Three 軸変換)
    const quats = relEulers.map((e) => this._m5cEulerToThreeQuat(e[0], e[1], e[2]));

    // waypoint dots (紫、worldGroup 内、球面より少し外側に)
    for (const q of quats) {
      const d = this._makeDot(0xa855f7, 0.09);
      const p = this._quatToSpherePoint(q).clone().multiplyScalar(1.08);
      d.position.copy(p);
      this.worldGroup.add(d);
      this.waypointDots.push(d);
    }

    // waypoint 接続線 (Slerp、紫、TubeGeometry で太線描画 = WebGL linewidth 仕様回避)
    // 球面より十分外側 (×1.08) に配置 + Tube で 3D 管描画 → 視認性最大化
    if (quats.length >= 2) {
      const SEG = 32;
      const points = [];
      for (let i = 0; i < quats.length - 1; i++) {
        const qa = quats[i], qb = quats[i + 1];
        for (let s = 0; s <= SEG; s++) {
          const t = s / SEG;
          const q = qa.clone().slerp(qb, t);
          points.push(this._quatToSpherePoint(q).clone().multiplyScalar(1.08));
        }
      }
      const curve = new THREE.CatmullRomCurve3(points, false, 'catmullrom', 0.0);
      const tubeGeo = new THREE.TubeGeometry(curve, points.length, 0.02, 8, false);
      const tubeMat = new THREE.MeshBasicMaterial({
        color: 0xa855f7,
        transparent: true,
        opacity: 0.85,
      });
      this.waypointLine = new THREE.Mesh(tubeGeo, tubeMat);
      this.worldGroup.add(this.waypointLine);
    }
    this._applyTrajectoryStyle();
  }

  _extractRelEulers(rule) {
    // 優先: states 配列 (Phase 5.39 hold_with_waypoints)
    if (Array.isArray(rule.states) && rule.states.length > 0) {
      return rule.states
        .filter((s) => s.posture && Array.isArray(s.posture.euler))
        .map((s) => s.posture.euler);
    }
    // フォールバック: 単一 posture
    if (rule.posture && Array.isArray(rule.posture.euler)) {
      return [rule.posture.euler];
    }
    return [];
  }

  /** M5C 軸基準の Euler [Roll, Pitch, Yaw] (deg) を Three.js 軸の Quaternion に変換
   *  M5C 規約 (ZYX intrinsic、Mahony Filter と一致):
   *    quat = Rz(yaw) * Ry(pitch) * Rx(roll)
   *  軸変換: M5C(X,Y,Z) → Three(-X, Z, Y) は IMUViewer と同じ
   */
  _m5cEulerToThreeQuat(rollDeg, pitchDeg, yawDeg) {
    const r = (rollDeg  * Math.PI / 180) / 2;
    const p = (pitchDeg * Math.PI / 180) / 2;
    const y = (yawDeg   * Math.PI / 180) / 2;
    const cr = Math.cos(r), sr = Math.sin(r);
    const cp = Math.cos(p), sp = Math.sin(p);
    const cy = Math.cos(y), sy = Math.sin(y);
    // M5C 系 (qw, qx, qy, qz)
    const qw = cr*cp*cy + sr*sp*sy;
    const qx = sr*cp*cy - cr*sp*sy;
    const qy = cr*sp*cy + sr*cp*sy;
    const qz = cr*cp*sy - sr*sp*cy;
    // Three 軸変換: (-qx, qz, qy, qw)
    return new THREE.Quaternion(-qx, qz, qy, qw);
  }

  _applyTrajectoryStyle() {
    if (!this.waypointLine) return;
    const mat = this.waypointLine.material;
    if (this._trajectoryMode === 'fixed') {
      mat.opacity = 1.0;
      mat.transparent = false;
    } else {
      // preview = 半透明
      mat.opacity = 0.45;
      mat.transparent = true;
    }
    mat.needsUpdate = true;
    // dots の opacity も追随
    for (const d of this.waypointDots) {
      const dm = d.material;
      dm.opacity = (this._trajectoryMode === 'fixed') ? 1.0 : 0.55;
      dm.transparent = true;
      dm.needsUpdate = true;
    }
  }

  _makeDot(color, radius) {
    const g = new THREE.SphereGeometry(radius, 16, 16);
    const m = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 1.0 });
    return new THREE.Mesh(g, m);
  }

  _quatToSpherePoint(q) {
    const dir = new THREE.Vector3(0, 0, 1);
    dir.applyQuaternion(q).normalize();
    return dir;
  }

  getFrameCount() { const n = this._frameCount || 0; this._frameCount = 0; return n; }

  _tick() {
    if (!this._running) return;
    if (!this._renderEnabled) return;   // タブ非表示時は描画停止
    this._frameCount = (this._frameCount || 0) + 1;

    // ★ 相対 3D の正しい仕様 (Phase 5.39.2.2):
    //   - ワールド (球面 / waypoint / 軌跡) は固定 (worldGroup.quaternion = identity)
    //   - M5C モデルは「ボタン押下時の q_ref を基準」とした相対回転で動く
    //     ・q_initial 未設定 (= ボタン押下前) → M5C モデルは中央固定 (identity)
    //     ・q_initial 確定 (= setQRef で q_ref 受信) → M5C モデル = q_initial⁻¹ * q_current
    //   - 起動時に自動取得はしない (= 絶対 3D と同じ見え方になるバグ回避)

    // ワールド固定
    this.worldGroup.quaternion.identity();

    // M5C モデル回転: q_initial 確定済みの場合のみ q_initial⁻¹ * q_current で回転
    if (this._qInitialValid) {
      const qRel = this._qInitial.clone().invert().multiply(this._qCurrent);
      this.m5StickC.quaternion.slerp(qRel, this.smoothing);
    } else {
      // ボタン押下前 = M5C モデル中央で動かない
      this.m5StickC.quaternion.slerp(new THREE.Quaternion(), this.smoothing);
    }

    this.renderer.render(this.scene, this.camera);
    requestAnimationFrame(this._tick);
  }

  /** 現在姿勢を新しい初期基準にする (絶対 3D の Init Yaw / Reset Base 相当) */
  initBase() {
    this._qInitial.copy(this._qCurrent);
    this._qInitialValid = true;
  }
  /** 初期基準を identity に戻す (= Mahony 起動基準と同じ) */
  resetBase() {
    this._qInitial.identity();
    this._qInitialValid = true;
  }

  _onResize() {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    if (w === 0 || h === 0) return;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
  }

  destroy() {
    this._running = false;
    this._resizeObserver.disconnect();
    this.renderer.dispose();
  }
}
