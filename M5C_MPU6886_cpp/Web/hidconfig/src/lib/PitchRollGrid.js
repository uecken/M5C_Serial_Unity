// Burst Motion - PitchRollGrid (Phase 4.1)
// Roll [-180, +180] × Pitch [-90, +90] の 2D 平面に
// 現在姿勢 (赤丸)、登録ルール (橙丸)、最近傍 (緑丸)、を可視化
//
// 旧 motion_controller.js の updatePitchRollCanvas + plotPoint + drawAxis を移植

export class PitchRollGrid {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.references = [];   // [{roll, pitch, name?}, ...]
    this.current = null;    // {roll, pitch}
    this.closest = null;    // 最近傍登録 index
    this.firingIdx = -1;    // 発火フラッシュ中のルール index
    this._firingTimer = null;
  }

  // 発火したルールを緑フラッシュ表示。durationMs 経過後に元に戻る
  setFiring(idx, durationMs = 500) {
    if (this._firingTimer) clearTimeout(this._firingTimer);
    this.firingIdx = idx;
    this.draw();
    this._firingTimer = setTimeout(() => {
      this.firingIdx = -1;
      this._firingTimer = null;
      this.draw();
    }, durationMs);
  }

  setCurrent(roll, pitch) {
    this.current = { roll, pitch };
    this.draw();
  }

  setReferences(refs) {
    this.references = refs || [];
    this.draw();
  }

  setClosest(idx) {
    this.closest = idx;
    this.draw();
  }

  resize() {
    const w = this.canvas.clientWidth || 600;
    const h = this.canvas.clientHeight || 200;
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
    this.draw();
  }

  draw() {
    const { ctx, canvas } = this;
    const W = canvas.width;
    const H = canvas.height;

    ctx.fillStyle = '#f1f5f9';
    ctx.fillRect(0, 0, W, H);

    // ジンバルロック領域 (Pitch ≥ +65° / ≤ -65°) を薄赤で可視化
    // ZYX intrinsic Euler は Pitch = asin で ±90° で縮退、±65° 以上は不安定
    const yPitch65pos = this._pitchToY(65);
    const yPitch90pos = this._pitchToY(90);
    const yPitch65neg = this._pitchToY(-65);
    const yPitch90neg = this._pitchToY(-90);
    ctx.fillStyle = 'rgba(239, 68, 68, 0.10)';
    ctx.fillRect(0, yPitch90pos - 2, W, yPitch65pos - yPitch90pos + 2);
    ctx.fillRect(0, yPitch65neg, W, yPitch90neg - yPitch65neg + 2);
    // 境界線 (破線)
    ctx.strokeStyle = 'rgba(220, 38, 38, 0.5)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(0, yPitch65pos); ctx.lineTo(W, yPitch65pos);
    ctx.moveTo(0, yPitch65neg); ctx.lineTo(W, yPitch65neg);
    ctx.stroke();
    ctx.setLineDash([]);
    // 警告ラベル
    ctx.fillStyle = '#dc2626';
    ctx.font = 'bold 9px sans-serif';
    ctx.fillText('⚠ Pitch ≥ +65° ジンバルロック領域 (Euler 判定不安定)', 30, yPitch90pos + 11);
    ctx.fillText('⚠ Pitch ≤ -65° ジンバルロック領域 (Euler 判定不安定)', 30, yPitch90neg - 4);

    this._drawGrid();

    // 登録参照点 (橙)、発火中=緑フラッシュ、最近傍=緑
    // Phase 5.16: tol で囲まれた矩形を半透明で描画 (発火範囲の可視化)
    this.references.forEach((r, idx) => {
      const isFiring = (this.firingIdx === idx);
      const isClosest = (this.closest === idx);
      let color, radius;
      if (isFiring) {
        color = '#22c55e';
        radius = 11;
      } else if (isClosest) {
        color = '#10b981';
        radius = 7;
      } else {
        color = '#f97316';
        radius = 5;
      }
      // tol 矩形を背景に描画 (中央点より先に)
      if (typeof r.rollTol === 'number' && typeof r.pitchTol === 'number') {
        const fillStyle  = isFiring  ? 'rgba(34,197,94,0.20)'
                         : isClosest ? 'rgba(16,185,129,0.18)'
                                     : 'rgba(249,115,22,0.13)';
        const strokeStyle = isFiring  ? 'rgba(21,128,61,0.6)'
                          : isClosest ? 'rgba(5,150,105,0.5)'
                                      : 'rgba(234,88,12,0.4)';
        // Roll tol が極端に広い (≥170) なら描画省略 (= 軸除外、ほぼ全幅で視覚 noise)
        // Pitch tol も同様 (≥85 で全幅相当)
        const rollWide  = r.rollTol  >= 170;
        const pitchWide = r.pitchTol >= 85;
        if (rollWide && pitchWide) {
          // 両軸除外 → 描画省略
        } else if (rollWide) {
          // Roll 任意 → 横帯 (Pitch 範囲のみ)
          const yTop = this._pitchToY(r.pitch + r.pitchTol);
          const yBot = this._pitchToY(r.pitch - r.pitchTol);
          ctx.fillStyle = fillStyle;
          ctx.fillRect(0, yTop, W, yBot - yTop);
          ctx.strokeStyle = strokeStyle;
          ctx.strokeRect(0, yTop, W, yBot - yTop);
        } else if (pitchWide) {
          // Pitch 任意 → 縦帯 (Roll 範囲のみ、wrap 対応)
          this._fillRollRange(ctx, r.roll, r.rollTol, fillStyle, strokeStyle);
        } else {
          // 通常: 矩形 (Roll wrap 対応のため Roll 範囲を 1〜2 個に分割)
          const yTop = this._pitchToY(r.pitch + r.pitchTol);
          const yBot = this._pitchToY(r.pitch - r.pitchTol);
          this._fillRollRangeY(ctx, r.roll, r.rollTol, yTop, yBot, fillStyle, strokeStyle);
        }
      }
      this._plotPoint(r.roll, r.pitch, color, radius);
      if (isFiring) {
        // 発火フラッシュの中央に白丸でアクセント
        const px = this._rollToX(r.roll);
        const py = this._pitchToY(r.pitch);
        ctx.beginPath();
        ctx.arc(px, py, 3, 0, 2 * Math.PI);
        ctx.fillStyle = '#ffffff';
        ctx.fill();
      }
      if (r.name) {
        const px = this._rollToX(r.roll);
        const py = this._pitchToY(r.pitch);
        ctx.fillStyle = isFiring ? '#15803d' : '#475569';
        ctx.font = isFiring ? 'bold 11px sans-serif' : '10px sans-serif';
        ctx.fillText(r.name, px + 8, py - 4);
      }
    });

    // 現在姿勢 (赤、上から描く)
    if (this.current) {
      this._plotPoint(this.current.roll, this.current.pitch, '#ef4444', 6);
    }
  }

  // Roll 範囲 [center - tol, center + tol] を ±180° wrap 考慮で 1〜2 矩形描画
  // (フル高さ = 縦帯バージョン)
  _fillRollRange(ctx, center, tol, fillStyle, strokeStyle) {
    const H = this.canvas.height;
    this._fillRollRangeY(ctx, center, tol, 0, H, fillStyle, strokeStyle);
  }
  _fillRollRangeY(ctx, center, tol, yTop, yBot, fillStyle, strokeStyle) {
    const lo = center - tol;
    const hi = center + tol;
    ctx.fillStyle = fillStyle;
    ctx.strokeStyle = strokeStyle;
    ctx.lineWidth = 1;
    if (lo >= -180 && hi <= 180) {
      // wrap なし、1 矩形
      const xL = this._rollToX(lo);
      const xR = this._rollToX(hi);
      ctx.fillRect(xL, yTop, xR - xL, yBot - yTop);
      ctx.strokeRect(xL, yTop, xR - xL, yBot - yTop);
    } else if (lo < -180) {
      // 左側 wrap: [lo+360, 180] と [-180, hi]
      const xL1 = this._rollToX(lo + 360);
      const xR1 = this._rollToX(180);
      ctx.fillRect(xL1, yTop, xR1 - xL1, yBot - yTop);
      ctx.strokeRect(xL1, yTop, xR1 - xL1, yBot - yTop);
      const xL2 = this._rollToX(-180);
      const xR2 = this._rollToX(hi);
      ctx.fillRect(xL2, yTop, xR2 - xL2, yBot - yTop);
      ctx.strokeRect(xL2, yTop, xR2 - xL2, yBot - yTop);
    } else if (hi > 180) {
      // 右側 wrap: [lo, 180] と [-180, hi-360]
      const xL1 = this._rollToX(lo);
      const xR1 = this._rollToX(180);
      ctx.fillRect(xL1, yTop, xR1 - xL1, yBot - yTop);
      ctx.strokeRect(xL1, yTop, xR1 - xL1, yBot - yTop);
      const xL2 = this._rollToX(-180);
      const xR2 = this._rollToX(hi - 360);
      ctx.fillRect(xL2, yTop, xR2 - xL2, yBot - yTop);
      ctx.strokeRect(xL2, yTop, xR2 - xL2, yBot - yTop);
    }
  }

  _rollToX(roll) {
    // roll [-180, +180] → x [margin, W-margin]
    const margin = 30;
    const W = this.canvas.width;
    return margin + ((roll + 180) / 360) * (W - 2 * margin);
  }

  _pitchToY(pitch) {
    // pitch [-90, +90] → y [margin, H-margin]、上が +pitch
    const margin = 20;
    const H = this.canvas.height;
    return H - margin - ((pitch + 90) / 180) * (H - 2 * margin);
  }

  _plotPoint(roll, pitch, color, radius = 5) {
    const x = this._rollToX(roll);
    const y = this._pitchToY(pitch);
    this.ctx.beginPath();
    this.ctx.arc(x, y, radius, 0, 2 * Math.PI);
    this.ctx.fillStyle = color;
    this.ctx.fill();
    this.ctx.strokeStyle = '#1e293b';
    this.ctx.lineWidth = 1;
    this.ctx.stroke();
  }

  _drawGrid() {
    const { ctx, canvas } = this;
    const W = canvas.width;
    const H = canvas.height;

    // グリッド線 (薄い灰)
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 1;
    ctx.font = '10px monospace';
    ctx.fillStyle = '#94a3b8';

    // 縦線 (Roll、30°毎、ラベル付き)
    for (let r = -180; r <= 180; r += 30) {
      const x = this._rollToX(r);
      ctx.beginPath();
      ctx.moveTo(x, 5);
      ctx.lineTo(x, H - 5);
      ctx.stroke();
      if (r % 60 === 0) {
        ctx.fillText(`${r}`, x - 10, H - 4);
      }
    }
    // 横線 (Pitch、30°毎、ラベル付き)
    for (let p = -90; p <= 90; p += 30) {
      const y = this._pitchToY(p);
      ctx.beginPath();
      ctx.moveTo(5, y);
      ctx.lineTo(W - 5, y);
      ctx.stroke();
      ctx.fillText(`${p}`, 4, y - 2);
    }

    // 軸 (中央線 0)
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    const x0 = this._rollToX(0);
    ctx.moveTo(x0, 5);
    ctx.lineTo(x0, H - 5);
    ctx.stroke();
    ctx.beginPath();
    const y0 = this._pitchToY(0);
    ctx.moveTo(5, y0);
    ctx.lineTo(W - 5, y0);
    ctx.stroke();

    // 軸ラベル
    ctx.fillStyle = '#475569';
    ctx.font = 'bold 11px sans-serif';
    ctx.fillText('Roll →', W - 50, y0 - 6);
    ctx.fillText('↑ Pitch', x0 + 6, 14);
  }
}
