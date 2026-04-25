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
    this.references.forEach((r, idx) => {
      const isFiring = (this.firingIdx === idx);
      const isClosest = (this.closest === idx);
      let color, radius;
      if (isFiring) {
        // 発火フラッシュ: 鮮やかな緑 + 大きめ + 中央白丸でアクセント
        color = '#22c55e';
        radius = 11;
      } else if (isClosest) {
        color = '#10b981';
        radius = 7;
      } else {
        color = '#f97316';
        radius = 5;
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
