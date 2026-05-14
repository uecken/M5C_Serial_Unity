// Burst Motion - IMUViewer (Phase 4.1 拡張版)
// 旧版 (findradio.jp) の機能を移植:
// - 球体ワイヤーフレーム (基準スケール、★固定★)
// - M5StickC モデル (★球とは独立に回転★、ユーザー仕様: 球は基準として固定)
// - World/Body 座標軸切替
// - 重力ベクトル矢印
// - 球面に登録ルール姿勢の点を表示 (橙)、最近傍 (緑)、現在 (赤)
import * as THREE from 'three';

export class IMUViewer {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x000000);   // 旧版互換、黒背景

    const w = canvas.clientWidth || 320;
    const h = canvas.clientHeight || 240;
    this.camera = new THREE.PerspectiveCamera(50, w / h, 0.1, 100);
    this.camera.position.z = -2.5;
    this.camera.lookAt(0, 0, 0);

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setSize(w, h, false);
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);

    // ライト (mesh は MeshBasicMaterial なので無くても可、念のため)
    this.scene.add(new THREE.AmbientLight(0xffffff, 1.0));

    // ---- 球体ワイヤーフレーム ----
    const sphereGeo = new THREE.SphereGeometry(1, 24, 24);
    const sphereMat = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true });
    this.sphere = new THREE.Mesh(sphereGeo, sphereMat);
    this.scene.add(this.sphere);

    // ---- World 軸 (固定) ----
    this.worldAxes = new THREE.AxesHelper(1.8);
    this.worldAxes.visible = false;
    this.scene.add(this.worldAxes);

    // ---- M5StickC モデル (scene 直下: 球は固定、stickC だけ回転) ----
    const stickGeo = new THREE.BoxGeometry(0.24, 0.12, 0.48);
    const orange = new THREE.MeshBasicMaterial({ color: 0xffa500 });
    const black  = new THREE.MeshBasicMaterial({ color: 0x000000 });
    const white  = new THREE.MeshBasicMaterial({ color: 0xffffff });
    this.m5StickC = new THREE.Mesh(stickGeo, [orange, orange, black, orange, orange, white]);
    // LCD 黒面
    const lcdGeo = new THREE.PlaneGeometry(0.2, 0.05);
    const lcd = new THREE.Mesh(lcdGeo, new THREE.MeshBasicMaterial({ color: 0x111111 }));
    lcd.position.set(0, 0.061, 0.24);
    this.m5StickC.add(lcd);
    // LED マーク
    const markGeo = new THREE.PlaneGeometry(0.04, 0.02);
    const mark = new THREE.Mesh(markGeo, new THREE.MeshBasicMaterial({ color: 0x33ff33 }));
    mark.position.set(0, -0.061, 0.24);
    this.m5StickC.add(mark);
    this.scene.add(this.m5StickC);

    // ---- Body 軸 (M5StickC の子: 一緒に回転) ----
    this.bodyAxes = new THREE.AxesHelper(0.7);
    this.bodyAxes.visible = false;
    this.m5StickC.add(this.bodyAxes);

    // ---- 重力ベクトル矢印 (水色シリンダー + 円錐先端) ----
    this.gravityArrow = new THREE.Group();
    const gShaftGeo = new THREE.CylinderGeometry(0.018, 0.018, 1, 8);
    const gMat = new THREE.MeshBasicMaterial({ color: 0x00aaff });
    this.gShaft = new THREE.Mesh(gShaftGeo, gMat);
    this.gShaft.position.y = 0.5;
    const gTipGeo = new THREE.ConeGeometry(0.05, 0.12, 12);
    this.gTip = new THREE.Mesh(gTipGeo, gMat);
    this.gTip.position.y = 1.06;
    this.gravityArrow.add(this.gShaft);
    this.gravityArrow.add(this.gTip);
    this.gravityArrow.visible = false;
    this.scene.add(this.gravityArrow);

    // ---- 球面ドット (現在姿勢、登録、最近傍、ボタン押下時) ----
    this.dots = {
      current: this._makeDot(0xff0000, 0.06),       // 赤: 現在姿勢
      closest: this._makeDot(0x00ff00, 0.07),       // 緑: 最近傍
      buttonPress: this._makeDot(0xa855f7, 0.08),   // 紫: ボタン押下時の姿勢
    };
    this.dots.current.visible = false;
    this.dots.closest.visible = false;
    this.dots.buttonPress.visible = false;
    this.scene.add(this.dots.current);
    this.scene.add(this.dots.closest);
    this.scene.add(this.dots.buttonPress);
    this._buttonPressTimer = null;

    this.referenceDots = [];   // 登録ルール用、橙
    this.referenceDotIds = []; // referenceDots と並列に rule id を保持 (Phase 5.39.2 選択フォーカス用)
    this.selectedRuleId = -1;  // Phase 5.39.2: -1 = 全表示、>=0 = その rule だけ濃色

    // ---- Phase 5.39.2: 目標軌跡 (waypoint Slerp 接続、紫色 Line) ----
    this.targetTrajectoryLine = null;

    // ---- Phase 5.39.2: 過去軌跡 trail (自分の動き、赤系) ----
    this._trail = [];                         // [{q: THREE.Quaternion, t: ms}, ...]
    this._trailDurationMs = 3000;
    this.trailLine = null;                    // THREE.Line (vertex colors)
    this._initTrailLine();

    // 状態
    this.targetQuat = new THREE.Quaternion();
    this.smoothing = opts.smoothing ?? 0.3;
    this.qRef = new THREE.Quaternion();   // base 姿勢 (Init Yaw 用)

    // resize
    this._resizeObserver = new ResizeObserver(() => this._onResize());
    this._resizeObserver.observe(canvas);
    this._running = true;
    this._tick = this._tick.bind(this);
    requestAnimationFrame(this._tick);
  }

  _makeDot(color, radius) {
    const g = new THREE.SphereGeometry(radius, 16, 16);
    const m = new THREE.MeshBasicMaterial({ color });
    return new THREE.Mesh(g, m);
  }

  // ---- Phase 5.39.2: trail Line 初期化 (最大 3 秒 × 50Hz = 150 vertex 程度を想定) ----
  _initTrailLine() {
    const MAX_TRAIL_POINTS = 256;
    const positions = new Float32Array(MAX_TRAIL_POINTS * 3);
    const colors    = new Float32Array(MAX_TRAIL_POINTS * 3);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color',    new THREE.BufferAttribute(colors, 3));
    geo.setDrawRange(0, 0);
    const mat = new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 2 });
    this.trailLine = new THREE.Line(geo, mat);
    this.scene.add(this.trailLine);
    this._trailMaxPoints = MAX_TRAIL_POINTS;
  }

  /** Phase 5.39.2: 選択 rule の強調表示 (-1 で解除) */
  setSelectedRuleId(id) {
    this.selectedRuleId = (typeof id === 'number') ? id : -1;
    this._applySelectionStyle();
  }

  /** referenceDots の色 / opacity を selectedRuleId に応じて再設定 */
  _applySelectionStyle() {
    const sel = this.selectedRuleId;
    for (let i = 0; i < this.referenceDots.length; i++) {
      const d = this.referenceDots[i];
      const id = this.referenceDotIds[i];
      const isSel = (sel === -1 || id === sel);
      const mat = d.material;
      if (isSel) {
        mat.color.setHex(0xffa500);  // 橙 (通常)
        mat.transparent = false;
        mat.opacity = 1.0;
      } else {
        mat.color.setHex(0x808080);  // 灰
        mat.transparent = true;
        mat.opacity = 0.20;
      }
      mat.needsUpdate = true;
    }
    // 選択解除 (-1) 時は目標軌跡を消す
    if (sel === -1 && this.targetTrajectoryLine) {
      this.targetTrajectoryLine.visible = false;
    } else if (this.targetTrajectoryLine) {
      this.targetTrajectoryLine.visible = true;
    }
  }

  /** Phase 5.39.2: 目標軌跡 (waypoint 間を Slerp 弧で接続)
   *  waypoints = [{qw,qx,qy,qz}, ...] (M5C 軸基準、IMUViewer 内で軸変換)
   */
  setTargetTrajectory(waypoints) {
    // 既存 line を削除
    if (this.targetTrajectoryLine) {
      this.scene.remove(this.targetTrajectoryLine);
      this.targetTrajectoryLine.geometry.dispose();
      this.targetTrajectoryLine.material.dispose();
      this.targetTrajectoryLine = null;
    }
    if (!waypoints || waypoints.length < 2) return;
    // 各 waypoint quat を Three 軸系に変換
    const quats = waypoints.map((w) => new THREE.Quaternion(-w.qx, w.qz, w.qy, w.qw));
    const SEG = 32;
    const points = [];
    for (let i = 0; i < quats.length - 1; i++) {
      const qa = quats[i];
      const qb = quats[i + 1];
      for (let s = 0; s <= SEG; s++) {
        const t = s / SEG;
        const q = qa.clone().slerp(qb, t);
        points.push(this._quatToSpherePoint(q).clone().multiplyScalar(1.01));
      }
    }
    const geo = new THREE.BufferGeometry().setFromPoints(points);
    const mat = new THREE.LineBasicMaterial({ color: 0x8b5cf6, linewidth: 2 });
    this.targetTrajectoryLine = new THREE.Line(geo, mat);
    this.scene.add(this.targetTrajectoryLine);
    // 選択中以外は非表示 (selectedRuleId === -1 で消す)
    if (this.selectedRuleId === -1) this.targetTrajectoryLine.visible = false;
  }

  /** Phase 5.39.2: 過去軌跡 trail に新点を追加 (M5C 軸 quat) */
  addTrailPoint(qw, qx, qy, qz, timestamp) {
    const t = (typeof timestamp === 'number') ? timestamp : performance.now();
    const q = new THREE.Quaternion(-qx, qz, qy, qw);
    this._trail.push({ q, t });
    this._pruneTrail(t);
    this._updateTrailGeometry(t);
  }

  /** 3 秒経過点を削除 */
  _pruneTrail(now) {
    const cutoff = now - this._trailDurationMs;
    while (this._trail.length > 0 && this._trail[0].t < cutoff) {
      this._trail.shift();
    }
    if (this._trail.length > this._trailMaxPoints) {
      this._trail.splice(0, this._trail.length - this._trailMaxPoints);
    }
  }

  /** trail Line geometry を再構築 (古いほど薄く、新しいほど濃く) */
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
      const age = (now - e.t) / this._trailDurationMs;        // 0 (新) → 1 (古)
      const alpha = Math.max(0, 1 - age);                     // 古いほど薄い → 色を白に fade
      // 赤 (0xef4444 = 0.937, 0.267, 0.267) を alpha でフェード (黒へ寄せる)
      const r = 0.937 * alpha;
      const g = 0.267 * alpha;
      const b = 0.267 * alpha;
      colAttr.setXYZ(i, r, g, b);
    }
    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    geo.setDrawRange(0, n);
  }

  /** trail 全消去 (例えば state[0] enter 時 = ボタン押下時) */
  clearTrail() {
    this._trail.length = 0;
    if (this.trailLine) this.trailLine.geometry.setDrawRange(0, 0);
  }

  /** trail 履歴の保持時間 (ms) */
  setTrailDurationMs(ms) {
    this._trailDurationMs = Math.max(200, ms | 0);
  }

  /** Quaternion 設定 (M5StickC IMU 軸 → Three.js 軸の変換含む) */
  setQuaternion(qw, qx, qy, qz) {
    // 旧版互換: THREE.Quaternion(-qx, qz, qy, qw)
    // 軸変換: M5C(X,Y,Z) → Three(-X, Z, Y)
    this.targetQuat.set(-qx, qz, qy, qw);
  }

  /** 軸表示切替 */
  setShowWorldAxes(v) { this.worldAxes.visible = v; }
  setShowBodyAxes(v)  { this.bodyAxes.visible = v; }
  setShowGravity(v)   { this.gravityArrow.visible = v; }

  /** 重力ベクトル更新 (3D 矢印を world 座標系の重力方向に向ける)
   *
   *   Phase 5.34.1 修正:
   *   旧実装は body frame の accel をそのまま world frame として使っていたため、
   *   デバイスが回転すると矢印が反対方向に向く誤動作があった。
   *
   *   正しい計算:
   *     1. accel (body frame, m/s²) を Three の body 軸に remap
   *     2. M5StickC mesh の現在クォータニオンで body → world に rotate
   *     3. 加速度計の出力 = 重力への反作用 (UP 方向) なので negate して重力方向 (DOWN) に
   *     4. gravityArrow は scene 直下に居るので、矢印の +Y を gravity 方向に向けるよう quaternion 設定
   *
   *   結果: 静止状態では矢印は常に world -Y (画面下) を指す。デバイスを振ると
   *         一時的に揺れる (= リニア加速度の影響が見える)。
   */
  setGravityVector(ax, ay, az) {
    if (!this.gravityArrow.visible) return;
    // 1. M5C body 軸 → Three body 軸 remap
    const bodyAccel = new THREE.Vector3(-ax, az, ay);
    const mag = bodyAccel.length();
    if (mag < 1e-6) return;
    bodyAccel.divideScalar(mag);
    // 2. body → world: m5StickC の現在クォータニオンを適用
    const worldAccel = bodyAccel.applyQuaternion(this.m5StickC.quaternion);
    // 3. accel (反作用、UP) を negate → 重力方向 (DOWN)
    const gravityDir = worldAccel.negate();
    // 4. arrow +Y を gravityDir に向ける
    const up = new THREE.Vector3(0, 1, 0);
    const q = new THREE.Quaternion().setFromUnitVectors(up, gravityDir);
    this.gravityArrow.quaternion.copy(q);
  }

  /** 登録ルール姿勢を球面に橙ドットで配置 (旧版互換、見やすい大きさ)
   *  Phase 5.39.2: ids 引数を追加 (selection focus 用、与えなければ全て -1 扱い)
   */
  setReferenceQuaternions(quats, ids) {
    // 既存ドット消去
    for (const d of this.referenceDots) {
      this.scene.remove(d);
      d.geometry.dispose();
      d.material.dispose();
    }
    this.referenceDots = [];
    this.referenceDotIds = [];
    for (let i = 0; i < quats.length; i++) {
      const q = quats[i];
      const d = this._makeDot(0xffa500, 0.07);   // 橙、見やすい大きさ
      // selection focus 用に material.transparent を有効化
      d.material.transparent = true;
      d.material.opacity = 1.0;
      d.position.copy(this._quatToSpherePoint(q));
      this.scene.add(d);
      this.referenceDots.push(d);
      this.referenceDotIds.push(Array.isArray(ids) ? (ids[i] ?? -1) : -1);
    }
    // 現在の selectedRuleId に応じた style を再適用
    this._applySelectionStyle();
  }

  /** ボタン押下時の姿勢を球面に紫ドットで表示 (旧版互換)
   *  durationMs 経過後に消える (デフォルト 2000ms)
   *  M5C 軸からの変換は呼び出し側で行うか、ここで生 quat を受け取る場合は同変換を適用 */
  setButtonPressDot(qw, qx, qy, qz, durationMs = 2000) {
    if (this._buttonPressTimer) clearTimeout(this._buttonPressTimer);
    this.dots.buttonPress.visible = true;
    // 軸変換は他のドットと同じ: M5C(qw,qx,qy,qz) → Three(-qx, qz, qy, qw)
    const q = new THREE.Quaternion(-qx, qz, qy, qw);
    this.dots.buttonPress.position.copy(this._quatToSpherePoint(q));
    this._buttonPressTimer = setTimeout(() => {
      this.dots.buttonPress.visible = false;
      this._buttonPressTimer = null;
    }, durationMs);
  }

  /** 現在姿勢のドット位置 (赤) */
  setCurrentDot(qw, qx, qy, qz) {
    this.dots.current.visible = true;
    const q = new THREE.Quaternion(-qx, qz, qy, qw);
    this.dots.current.position.copy(this._quatToSpherePoint(q));
  }

  /** 最近傍ドット位置 (緑) */
  setClosestDot(qw, qx, qy, qz) {
    if (qw === undefined || qw === null) {
      this.dots.closest.visible = false;
      return;
    }
    this.dots.closest.visible = true;
    const q = new THREE.Quaternion(-qx, qz, qy, qw);
    this.dots.closest.position.copy(this._quatToSpherePoint(q));
  }

  /** quaternion から球面上の点を計算 (z=1 単位ベクトルを quat で回転) */
  _quatToSpherePoint(q) {
    const dir = new THREE.Vector3(0, 0, 1);
    dir.applyQuaternion(q).normalize();
    return dir;
  }

  /** Init Yaw: 現在の姿勢を base にする */
  initBase() {
    this.qRef.copy(this.targetQuat).invert();
  }
  /** base リセット (恒等) */
  resetBase() {
    this.qRef.identity();
  }

  reset() {
    this.targetQuat.identity();
  }

  // Phase 5.35: RAF frame count (perf overlay 用)
  getFrameCount() { const n = this._frameCount || 0; this._frameCount = 0; return n; }

  /** Phase 5.39.2: タブ非表示時は描画停止 (CPU 節約) */
  setRenderEnabled(enabled) {
    const was = this._renderEnabled !== false;
    this._renderEnabled = !!enabled;
    if (!was && this._renderEnabled) requestAnimationFrame(this._tick);
  }

  _tick() {
    if (!this._running) return;
    if (this._renderEnabled === false) return;   // Phase 5.39.2: タブ非表示時は描画停止
    this._frameCount = (this._frameCount || 0) + 1;
    // base からの相対回転を M5StickC モデルに適用 (球体は固定、ユーザー仕様)
    const q = this.qRef.clone().multiply(this.targetQuat);
    this.m5StickC.quaternion.slerp(q, this.smoothing);
    this.renderer.render(this.scene, this.camera);
    requestAnimationFrame(this._tick);
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
