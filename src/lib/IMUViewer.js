// Burst Motion - IMUViewer
// Three.js でクォータニオンに合わせて 3D box を回転させる
import * as THREE from 'three';

export class IMUViewer {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf1f5f9);

    // カメラ
    const w = canvas.clientWidth || 320;
    const h = canvas.clientHeight || 240;
    this.camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 100);
    this.camera.position.set(0, 1.5, 3.5);
    this.camera.lookAt(0, 0, 0);

    // レンダラ
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    this.renderer.setSize(w, h, false);
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);

    // ライト
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const dir = new THREE.DirectionalLight(0xffffff, 0.8);
    dir.position.set(2, 4, 2);
    this.scene.add(dir);

    // 床グリッド
    const grid = new THREE.GridHelper(4, 8, 0x94a3b8, 0xcbd5e1);
    grid.position.y = -0.6;
    this.scene.add(grid);

    // 座標軸 (X=赤, Y=緑, Z=青) ワールド
    this.scene.add(new THREE.AxesHelper(2));

    // M5StickC を模した直方体 (LCD 面 = +Y)
    const m5Group = new THREE.Group();
    // 本体
    const bodyGeo = new THREE.BoxGeometry(1.2, 0.5, 2.4);
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0xff7a00, roughness: 0.4 });
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    m5Group.add(body);
    // LCD 面 (+Y、表面)
    const lcdGeo = new THREE.PlaneGeometry(1.0, 1.6);
    const lcdMat = new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.2 });
    const lcd = new THREE.Mesh(lcdGeo, lcdMat);
    lcd.rotation.x = -Math.PI / 2;
    lcd.position.y = 0.26;
    m5Group.add(lcd);
    // 矢印 (LCD 上方向 = +Z 方向 of body frame)
    const arrowGeo = new THREE.ConeGeometry(0.15, 0.4, 16);
    const arrowMat = new THREE.MeshStandardMaterial({ color: 0x10b981 });
    const arrow = new THREE.Mesh(arrowGeo, arrowMat);
    arrow.position.set(0, 0.26 + 0.001, 1.0);  // LCD 面の上端
    arrow.rotation.x = Math.PI / 2;
    arrow.rotation.z = Math.PI;
    m5Group.add(arrow);

    // ボディ自体のローカル軸表示
    const localAxes = new THREE.AxesHelper(1.5);
    m5Group.add(localAxes);

    this.deviceGroup = m5Group;
    this.scene.add(m5Group);

    // 状態
    this.targetQuat = new THREE.Quaternion();
    this.smoothing = opts.smoothing ?? 0.25;  // 0..1, 1=即時、0=動かない

    // resize 監視
    this._resizeObserver = new ResizeObserver(() => this._onResize());
    this._resizeObserver.observe(canvas);

    // animation loop
    this._running = true;
    this._tick = this._tick.bind(this);
    requestAnimationFrame(this._tick);
  }

  /**
   * sensor.stream から得た quaternion を入力
   * 注意: Mahony の出力は body frame で (qw, qx, qy, qz)
   * Three.js の Quaternion は (x, y, z, w) の順
   */
  setQuaternion(qw, qx, qy, qz) {
    this.targetQuat.set(qx, qy, qz, qw);
  }

  /** Euler 直接指定 (deg) */
  setEuler(roll_deg, pitch_deg, yaw_deg) {
    const e = new THREE.Euler(
      THREE.MathUtils.degToRad(roll_deg),
      THREE.MathUtils.degToRad(pitch_deg),
      THREE.MathUtils.degToRad(yaw_deg),
      'XYZ',
    );
    this.targetQuat.setFromEuler(e);
  }

  reset() {
    this.targetQuat.identity();
  }

  _tick() {
    if (!this._running) return;
    // smoothing で targetQuat に補間
    this.deviceGroup.quaternion.slerp(this.targetQuat, this.smoothing);
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
