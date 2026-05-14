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

  /** 登録ルール姿勢を球面に橙ドットで配置 (旧版互換、見やすい大きさ) */
  setReferenceQuaternions(quats) {
    // 既存ドット消去
    for (const d of this.referenceDots) {
      this.scene.remove(d);
      d.geometry.dispose();
    }
    this.referenceDots = [];
    for (const q of quats) {
      const d = this._makeDot(0xffa500, 0.07);   // 橙、見やすい大きさ
      d.position.copy(this._quatToSpherePoint(q));
      this.scene.add(d);
      this.referenceDots.push(d);
    }
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

  _tick() {
    if (!this._running) return;
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
