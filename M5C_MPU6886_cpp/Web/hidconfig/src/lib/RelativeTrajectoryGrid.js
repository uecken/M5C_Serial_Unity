// Burst Motion - RelativeTrajectoryGrid (Phase 5.39.3b 新規)
//
// 「相対 3D Quat 軌跡」の 2D 投影表示 (ハリポタ Wand Movements chart 風)。
// 投影法: 案 1 = 主軸 forward 正射影
//   q_rel = q_initial⁻¹ * q_current
//   forward = q_rel ⊗ ẑ ⊗ q_rel⁻¹   (M5C 軸の +Z 方向 = LCD 上向き)
//   (x_canvas, y_canvas) = (forward.x * R, -forward.y * R)
//   forward.z > 0 (= 杖先端が前方半球) なら可視、< 0 は薄い半透明 (裏側マーカー)
//
// 描画要素:
//   - 背景: 同心円 (角距離 30° 刻み) + 経線 (45° 刻み)、円周 = 赤道 (forward.z=0)
//   - 中心 (0, 0): 緑十字 + 「init」ラベル = 初期姿勢の先端方向
//   - 過去軌跡 trail: 赤系グラデーション線 (3 秒履歴)
//   - 現在位置: 赤丸
//   - waypoint dots: 紫円 + 番号
//   - waypoint Slerp 接続線: 紫実線 (各補間 quat を投影 → 折れ線)
//   - twist インジケータ (右下): 現在の相対 Roll 値 (青系テキスト)
//
// 設計差 (PitchRollGrid との比較):
//   ┌──────────────────────────┬────────────────────────────────────┐
//   │ PitchRollGrid             │ RelativeTrajectoryGrid              │
//   │ Roll [-180,+180] × Pitch  │ forward.x × forward.y (主軸投影)    │
//   │ 絶対 Euler ベース          │ 相対 Quat ベース (q_initial⁻¹ * q)  │
//   │ 矩形マップ                 │ 円形マップ (半球内)                  │
//   └──────────────────────────┴────────────────────────────────────┘

export class RelativeTrajectoryGrid {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');

    // 状態
    this._qInitial = [1, 0, 0, 0];          // [qw,qx,qy,qz] (M5C 軸、デフォルト identity)
    this._qInitialValid = false;
    this._qCurrent = [1, 0, 0, 0];
    this._qCurrentValid = false;
    this._selectedRule = null;
    this._trail = [];                       // [{q:[qw,qx,qy,qz], t}]
    this._trailDurationMs = 3000;

    this._renderEnabled = true;
    this._running = true;
    this._needsRedraw = true;
    this._lastDrawMs = 0;
    this._drawCount = 0;
    this._frameCount = 0;

    // RAF 駆動 (描画 30Hz cap、tab 非表示時は停止)
    this._tick = this._tick.bind(this);
    requestAnimationFrame(this._tick);

    // resize 監視
    try {
      this._resizeObserver = new ResizeObserver(() => this.resize());
      this._resizeObserver.observe(canvas);
    } catch (_) { /* legacy browsers: ignore */ }
  }

  // ==========================================================
  // 公開 API
  // ==========================================================

  /** Phase 5.39.3a: q_initial 設定 (Init Yaw / Reset Base / posture.init.get で受信) */
  setQInitial(qInitArr, valid) {
    if (valid && Array.isArray(qInitArr) && qInitArr.length === 4) {
      this._qInitial = [qInitArr[0], qInitArr[1], qInitArr[2], qInitArr[3]];
      this._qInitialValid = true;
    } else {
      this._qInitial = [1, 0, 0, 0];
      this._qInitialValid = false;
    }
    this._needsRedraw = true;
  }

  /** 現在 quat (M5C 軸) を保存。setQuaternion 自体は重い描画をしない */
  setQuaternion(qw, qx, qy, qz) {
    this._qCurrent = [qw, qx, qy, qz];
    this._qCurrentValid = true;
    this._needsRedraw = true;
  }

  /** Phase 5.39.2: 選択 rule を描画
   *  相対モード rule (posture_basis='relative') の states[].posture.euler を
   *  「q_initial からの相対 Euler オフセット」として quat 化 → forward 投影。
   *  絶対モード rule は描画しない (null 渡し相当)。
   */
  setSelectedRule(rule) {
    this._selectedRule = rule;
    this._needsRedraw = true;
  }

  addTrailPoint(qw, qx, qy, qz, timestamp) {
    const t = (typeof timestamp === 'number') ? timestamp : performance.now();
    this._trail.push({ q: [qw, qx, qy, qz], t });
    const cutoff = t - this._trailDurationMs;
    while (this._trail.length > 0 && this._trail[0].t < cutoff) this._trail.shift();
    if (this._trail.length > 512) this._trail.splice(0, this._trail.length - 512);
    this._needsRedraw = true;
  }

  clearTrail() {
    this._trail.length = 0;
    this._needsRedraw = true;
  }

  setTrailDurationMs(ms) { this._trailDurationMs = Math.max(200, ms | 0); }

  /** タブ表示時 ON / 非表示時 OFF (CPU 節約) */
  setRenderEnabled(enabled) {
    const was = this._renderEnabled;
    this._renderEnabled = !!enabled;
    if (!was && this._renderEnabled) {
      this._needsRedraw = true;
      requestAnimationFrame(this._tick);
    }
  }

  resize() {
    const w = this.canvas.clientWidth || 320;
    const h = this.canvas.clientHeight || 240;
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
    this._needsRedraw = true;
  }

  getDrawCount() { const n = this._drawCount; this._drawCount = 0; return n; }
  getFrameCount() { const n = this._frameCount; this._frameCount = 0; return n; }

  destroy() {
    this._running = false;
    if (this._resizeObserver) {
      try { this._resizeObserver.disconnect(); } catch (_) {}
    }
  }

  // ==========================================================
  // RAF tick: 30Hz cap で _draw() を呼ぶ
  // ==========================================================
  _tick() {
    if (!this._running) return;
    if (!this._renderEnabled) return;
    this._frameCount++;
    const now = performance.now();
    if (this._needsRedraw && (now - this._lastDrawMs) >= 33) {
      this._lastDrawMs = now;
      this._needsRedraw = false;
      this._draw();
    }
    requestAnimationFrame(this._tick);
  }

  // ==========================================================
  // Quaternion ユーティリティ (M5C 軸、純粋数値配列ベース)
  // ==========================================================

  /** q ⊗ p (Hamilton 積、M5C 系 [w,x,y,z]) */
  static qMul(q, p) {
    const [qw, qx, qy, qz] = q;
    const [pw, px, py, pz] = p;
    return [
      qw*pw - qx*px - qy*py - qz*pz,
      qw*px + qx*pw + qy*pz - qz*py,
      qw*py - qx*pz + qy*pw + qz*px,
      qw*pz + qx*py - qy*px + qz*pw,
    ];
  }

  /** q⁻¹ (= 共役、単位 quat 前提) */
  static qConj(q) {
    return [q[0], -q[1], -q[2], -q[3]];
  }

  /** Slerp (球面線形補間) */
  static qSlerp(qa, qb, t) {
    let dot = qa[0]*qb[0] + qa[1]*qb[1] + qa[2]*qb[2] + qa[3]*qb[3];
    let qb2 = [qb[0], qb[1], qb[2], qb[3]];
    if (dot < 0) {
      qb2 = [-qb[0], -qb[1], -qb[2], -qb[3]];
      dot = -dot;
    }
    if (dot > 0.9995) {
      // 近接時は線形補間 + 正規化
      const r = [
        qa[0] + t*(qb2[0] - qa[0]),
        qa[1] + t*(qb2[1] - qa[1]),
        qa[2] + t*(qb2[2] - qa[2]),
        qa[3] + t*(qb2[3] - qa[3]),
      ];
      const n = Math.sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3]) || 1;
      return [r[0]/n, r[1]/n, r[2]/n, r[3]/n];
    }
    const omega = Math.acos(Math.min(1, Math.max(-1, dot)));
    const sinOm = Math.sin(omega);
    const a = Math.sin((1 - t) * omega) / sinOm;
    const b = Math.sin(t * omega) / sinOm;
    return [
      a*qa[0] + b*qb2[0],
      a*qa[1] + b*qb2[1],
      a*qa[2] + b*qb2[2],
      a*qa[3] + b*qb2[3],
    ];
  }

  /** quat → forward vector (= q ⊗ ẑ ⊗ q⁻¹、ẑ=(0,0,1) を quat で回転)
   *  M5C 軸基準: +Z = LCD 上向き (杖先端)
   */
  static quatToForward(q) {
    const [qw, qx, qy, qz] = q;
    return [
      2 * (qw * qy + qx * qz),                  // fx
      2 * (qy * qz - qw * qx),                  // fy
      qw*qw - qx*qx - qy*qy + qz*qz,            // fz
    ];
  }

  /** M5C 軸 Euler [Roll, Pitch, Yaw] (deg) → quat [w,x,y,z]
   *  Mahony 規約: q = Rz(yaw) * Ry(pitch) * Rx(roll)
   */
  static eulerToQuat(rollDeg, pitchDeg, yawDeg) {
    const r = (rollDeg  * Math.PI / 180) / 2;
    const p = (pitchDeg * Math.PI / 180) / 2;
    const y = (yawDeg   * Math.PI / 180) / 2;
    const cr = Math.cos(r), sr = Math.sin(r);
    const cp = Math.cos(p), sp = Math.sin(p);
    const cy = Math.cos(y), sy = Math.sin(y);
    return [
      cr*cp*cy + sr*sp*sy,
      sr*cp*cy - cr*sp*sy,
      cr*sp*cy + sr*cp*sy,
      cr*cp*sy - sr*sp*cy,
    ];
  }

  /** quat の twist (Z 軸成分) 角を deg で返す (相対 Roll = wand の捻り)
   *  Swing-Twist 分解: twist 成分 = normalize((qw, 0, 0, qz))
   */
  static quatToTwistZDeg(q) {
    const w = q[0], z = q[3];
    const norm = Math.sqrt(w*w + z*z);
    if (norm < 1e-6) return 0;
    const sign = (z >= 0) ? 1 : -1;
    const ang = 2 * Math.atan2(Math.abs(z), Math.abs(w)) * (180 / Math.PI);
    return sign * ang;
  }

  // ==========================================================
  // 投影: 相対 quat → canvas 座標
  // ==========================================================

  _projectCurrent() {
    // q_rel = q_initial⁻¹ ⊗ q_current
    const qRel = RelativeTrajectoryGrid.qMul(
      RelativeTrajectoryGrid.qConj(this._qInitial),
      this._qCurrent
    );
    return RelativeTrajectoryGrid.quatToForward(qRel);
  }

  _projectRelQuat(qM5C) {
    // qM5C は「q_initial からの相対 quat」と解釈 (waypoint 用、setSelectedRule 由来)
    return RelativeTrajectoryGrid.quatToForward(qM5C);
  }

  _projectAbsQuat(qM5C) {
    // 過去軌跡 (sensor.quat) 用: q_initial⁻¹ * qM5C
    const qRel = RelativeTrajectoryGrid.qMul(
      RelativeTrajectoryGrid.qConj(this._qInitial),
      qM5C
    );
    return RelativeTrajectoryGrid.quatToForward(qRel);
  }

  _forwardToCanvas(fwd) {
    const W = this.canvas.width;
    const H = this.canvas.height;
    const cx = W / 2;
    const cy = H / 2;
    const R = Math.min(W, H) * 0.45;
    // Phase 5.39.3a.3: 遠近射影 (perspective projection、「壁との交点」モデル)
    //   ユーザー期待: 杖を「壁」に向けたとき、壁上の交点が 2D 軌跡上の点に
    //   旧 (正射影): canvas = (fwd.x * R, -fwd.y * R)
    //   新 (遠近射影): canvas = (fwd.x / fwd.z * scale, -fwd.y / fwd.z * scale)
    //     杖が真正面 (forward = ẑ) → 中央 (0, 0)
    //     杖が前方 45° に振り → tan(45°) = 1.0 → 半径 R 程度の位置
    //     杖が前方 60° に振り → tan(60°) ≈ 1.73 → 半径外 (clamp)
    //   壁との交点は (fwd.x/fwd.z, fwd.y/fwd.z)、scale で表示範囲調整
    const fx = fwd[0];
    const fy = fwd[1];
    const fz = fwd[2];
    // 杖が後ろ向き / 真横方向 (fz <= 0.1) は表示外
    const visible = fz > 0.1;
    let sx, sy;
    if (visible) {
      // tan(角度) 系の透視投影、scale = R * 1.0 で振り角 45° が画面端付近
      const SCALE = R * 1.0;
      sx = (fx / fz) * SCALE;
      sy = (fy / fz) * SCALE;
      // 画面端を超える場合は clamp (= 振り角 大すぎ、視認外)
      const MAX = R * 2.5;
      sx = Math.max(-MAX, Math.min(MAX, sx));
      sy = Math.max(-MAX, Math.min(MAX, sy));
    } else {
      // 裏半球: 表示外、ただし古い軌跡描画用に座標は返す
      sx = fx * R;
      sy = fy * R;
    }
    return {
      x: cx + sx,
      y: cy - sy,    // 画面 Y 反転
      z: fz,
      visible,
      cx, cy, R,
    };
  }

  // ==========================================================
  // 描画
  // ==========================================================

  _draw() {
    this._drawCount++;
    const { ctx, canvas } = this;
    const W = canvas.width;
    const H = canvas.height;
    const cx = W / 2;
    const cy = H / 2;
    const R = Math.min(W, H) * 0.45;

    // 背景
    ctx.fillStyle = '#f8fafc';
    ctx.fillRect(0, 0, W, H);

    // 描画順: 背景グリッド → waypoint 接続線 → trail → waypoint dot → 現在位置 → labels
    this._drawBackgroundGrid(cx, cy, R);

    if (this._qInitialValid) {
      this._drawWaypoints(cx, cy, R);
      this._drawTrail(cx, cy, R);
      this._drawCurrent(cx, cy, R);
    } else {
      // q_initial 未設定: 案内テキスト
      ctx.fillStyle = '#94a3b8';
      ctx.font = 'bold 12px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Init Yaw ボタンで初期姿勢を設定してください', cx, cy + R + 18);
      ctx.textAlign = 'start';
    }

    this._drawCenterMarker(cx, cy, R);
    this._drawTwistIndicator(W, H);
    this._drawLegend(W, H);
  }

  _drawBackgroundGrid(cx, cy, R) {
    const ctx = this.ctx;
    // 同心円: 30°, 60°, 90° (赤道)
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.45)';
    ctx.lineWidth = 1;
    for (let deg = 30; deg <= 60; deg += 30) {
      const r = R * Math.sin(deg * Math.PI / 180);
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, 2 * Math.PI);
      ctx.stroke();
    }
    // 赤道 = 90°、太め線
    ctx.strokeStyle = 'rgba(71, 85, 105, 0.75)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, 2 * Math.PI);
    ctx.stroke();

    // 経線 (45° 刻みで 8 本)
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
    ctx.lineWidth = 1;
    for (let i = 0; i < 8; i++) {
      const ang = i * (Math.PI / 4);
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + R * Math.cos(ang), cy + R * Math.sin(ang));
      ctx.stroke();
    }

    // 角距離ラベル
    ctx.fillStyle = '#64748b';
    ctx.font = '9px sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText('30°', cx + R * Math.sin(Math.PI / 6) + 2, cy - 2);
    ctx.fillText('60°', cx + R * Math.sin(Math.PI / 3) + 2, cy - 2);
    ctx.fillText('90° (赤道)', cx + R + 4, cy - 2);
  }

  _drawCenterMarker(cx, cy, R) {
    const ctx = this.ctx;
    // 緑十字
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx - 8, cy);
    ctx.lineTo(cx + 8, cy);
    ctx.moveTo(cx, cy - 8);
    ctx.lineTo(cx, cy + 8);
    ctx.stroke();
    // ラベル
    ctx.fillStyle = '#047857';
    ctx.font = 'bold 10px sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText('init (q_initial = 中心)', cx + 10, cy - 6);
  }

  _drawWaypoints(cx, cy, R) {
    const ctx = this.ctx;
    const rule = this._selectedRule;
    if (!rule) return;
    if (rule.posture_basis !== 'relative') {
      // 絶対モード rule は 2D 相対軌跡で意味を持たない → 案内のみ
      ctx.fillStyle = '#94a3b8';
      ctx.font = '10px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('絶対モード rule は相対 2D 軌跡では描画できません', cx, cy + R + 18);
      ctx.textAlign = 'start';
      return;
    }
    // waypoint 抽出 (相対 Euler オフセット → quat)
    const relEulers = this._extractRelEulers(rule);
    if (relEulers.length === 0) return;
    const quats = relEulers.map(e => RelativeTrajectoryGrid.eulerToQuat(e[0], e[1], e[2]));

    // Slerp 接続線
    if (quats.length >= 2) {
      ctx.strokeStyle = '#a855f7';
      ctx.lineWidth = 2;
      ctx.beginPath();
      let started = false;
      const SEG = 24;
      for (let i = 0; i < quats.length - 1; i++) {
        for (let s = 0; s <= SEG; s++) {
          const t = s / SEG;
          const q = (s === 0) ? quats[i] : (s === SEG) ? quats[i + 1] : RelativeTrajectoryGrid.qSlerp(quats[i], quats[i + 1], t);
          const fwd = RelativeTrajectoryGrid.quatToForward(q);
          const p = this._forwardToCanvas(fwd);
          if (!p.visible) { started = false; continue; }
          if (!started) {
            ctx.moveTo(p.x, p.y);
            started = true;
          } else {
            ctx.lineTo(p.x, p.y);
          }
        }
      }
      ctx.stroke();
    }

    // waypoint dot + 番号
    quats.forEach((q, i) => {
      const fwd = RelativeTrajectoryGrid.quatToForward(q);
      const p = this._forwardToCanvas(fwd);
      ctx.save();
      ctx.globalAlpha = p.visible ? 1.0 : 0.35;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 9, 0, 2 * Math.PI);
      ctx.fillStyle = '#a855f7';
      ctx.fill();
      ctx.strokeStyle = '#581c87';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      // 番号 (1-indexed)
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 11px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(`${i + 1}`, p.x, p.y);
      ctx.textAlign = 'start';
      ctx.textBaseline = 'alphabetic';
      ctx.restore();
    });

    // rule 名表示
    if (rule.name) {
      const ctxn = this.ctx;
      ctxn.fillStyle = '#581c87';
      ctxn.font = 'bold 11px sans-serif';
      ctxn.textAlign = 'left';
      ctxn.fillText(`rule: ${rule.name}`, 6, 14);
    }
  }

  _extractRelEulers(rule) {
    if (Array.isArray(rule.states) && rule.states.length > 0) {
      return rule.states
        .filter(s => s.posture && Array.isArray(s.posture.euler))
        .map(s => s.posture.euler);
    }
    if (rule.posture && Array.isArray(rule.posture.euler)) {
      return [rule.posture.euler];
    }
    return [];
  }

  _drawTrail(cx, cy, R) {
    if (this._trail.length < 2) return;
    const ctx = this.ctx;
    const now = performance.now();
    ctx.lineWidth = 2;
    // 古い→新しいで赤系グラデーション (age=1: 透明、age=0: 濃赤)
    // セグメント単位で stroke (Canvas 2D は線分単位の gradient API がないため個別 stroke)
    let prevP = null;
    for (let i = 0; i < this._trail.length; i++) {
      const e = this._trail[i];
      const fwd = this._projectAbsQuat(e.q);
      const p = this._forwardToCanvas(fwd);
      if (!p.visible) { prevP = null; continue; }
      if (prevP) {
        const age = Math.min(1, (now - e.t) / this._trailDurationMs);
        const alpha = Math.max(0.05, 1 - age);
        ctx.strokeStyle = `rgba(239, 68, 68, ${alpha.toFixed(3)})`;
        ctx.beginPath();
        ctx.moveTo(prevP.x, prevP.y);
        ctx.lineTo(p.x, p.y);
        ctx.stroke();
      }
      prevP = p;
    }
  }

  _drawCurrent(cx, cy, R) {
    if (!this._qCurrentValid) return;
    const fwd = this._projectCurrent();
    const p = this._forwardToCanvas(fwd);
    const ctx = this.ctx;
    ctx.save();
    ctx.globalAlpha = p.visible ? 1.0 : 0.4;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 6, 0, 2 * Math.PI);
    ctx.fillStyle = '#ef4444';
    ctx.fill();
    ctx.strokeStyle = '#7f1d1d';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    if (!p.visible) {
      // 裏側マーカー
      ctx.fillStyle = '#7f1d1d';
      ctx.font = '9px sans-serif';
      ctx.fillText('(裏側)', p.x + 8, p.y + 3);
    }
    ctx.restore();
  }

  _drawTwistIndicator(W, H) {
    if (!this._qInitialValid || !this._qCurrentValid) return;
    const ctx = this.ctx;
    // 相対 Roll (twist Z) を表示 (テキストのみ、簡素)
    const qRel = RelativeTrajectoryGrid.qMul(
      RelativeTrajectoryGrid.qConj(this._qInitial),
      this._qCurrent
    );
    const twistDeg = RelativeTrajectoryGrid.quatToTwistZDeg(qRel);
    ctx.fillStyle = 'rgba(30, 64, 175, 0.85)';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'right';
    ctx.fillText(`twist Z: ${twistDeg.toFixed(1)}°`, W - 6, H - 6);
    ctx.textAlign = 'start';
  }

  _drawLegend(W, H) {
    const ctx = this.ctx;
    ctx.fillStyle = '#64748b';
    ctx.font = '9px sans-serif';
    ctx.textAlign = 'left';
    let y = H - 6;
    ctx.fillText('● 紫=waypoint  ● 赤=現在位置 / 軌跡  + 緑=init', 6, y);
  }
}
