// Burst Motion - TimeSeriesChart (Phase 5.5)
// 軽量な時系列折れ線グラフ。Canvas 直接描画、依存なし。
//
// 使い方:
//   const chart = new TimeSeriesChart(canvas, {
//     channels: [
//       { key: 'x',   color: '#ef4444', label: 'X' },
//       { key: 'y',   color: '#10b981', label: 'Y' },
//       { key: 'z',   color: '#3b82f6', label: 'Z' },
//       { key: 'rms', color: '#1e293b', label: 'RMS', width: 2 },
//     ],
//     yMin: -2, yMax: 2,
//     bufferSize: 200,   // 直近 N サンプル
//   });
//   chart.push({ x: 0.1, y: 0.2, z: 0.98, rms: 1.0 });

export class TimeSeriesChart {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.channels = opts.channels || [];
    this.yMin = opts.yMin ?? -1;
    this.yMax = opts.yMax ?? 1;
    this.autoScale = opts.autoScale ?? false;
    this.bufferSize = opts.bufferSize ?? 200;
    this.unit = opts.unit || '';
    // チャンネルごとにリングバッファ
    this.buffers = {};
    for (const ch of this.channels) {
      this.buffers[ch.key] = new Array(this.bufferSize).fill(NaN);
    }
    this.head = 0;  // 次に書込むインデックス
  }

  push(values) {
    for (const ch of this.channels) {
      const v = values[ch.key];
      this.buffers[ch.key][this.head] = (typeof v === 'number') ? v : NaN;
    }
    this.head = (this.head + 1) % this.bufferSize;
    if (this.autoScale) this._updateAutoScale();
    this.draw();
  }

  resize() {
    const w = this.canvas.clientWidth || 400;
    const h = this.canvas.clientHeight || 100;
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
    this.draw();
  }

  setRange(yMin, yMax) {
    this.yMin = yMin;
    this.yMax = yMax;
    this.draw();
  }

  _updateAutoScale() {
    let min = Infinity, max = -Infinity;
    for (const ch of this.channels) {
      const buf = this.buffers[ch.key];
      for (let i = 0; i < buf.length; i++) {
        const v = buf[i];
        if (!isNaN(v)) {
          if (v < min) min = v;
          if (v > max) max = v;
        }
      }
    }
    if (min < Infinity && max > -Infinity) {
      const margin = (max - min) * 0.1 + 0.1;
      this.yMin = min - margin;
      this.yMax = max + margin;
    }
  }

  draw() {
    const { ctx, canvas } = this;
    const W = canvas.width;
    const H = canvas.height;
    if (W === 0 || H === 0) return;

    // 背景
    ctx.fillStyle = '#f8fafc';
    ctx.fillRect(0, 0, W, H);

    // 横軸 (zero line)
    const yToPx = (v) => {
      const range = this.yMax - this.yMin;
      if (range === 0) return H / 2;
      return H - ((v - this.yMin) / range) * H;
    };

    // グリッド
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 3]);
    // 0 line
    if (this.yMin <= 0 && this.yMax >= 0) {
      ctx.beginPath();
      ctx.moveTo(0, yToPx(0));
      ctx.lineTo(W, yToPx(0));
      ctx.stroke();
    }
    ctx.setLineDash([]);

    // 軸ラベル (左側に min/max)
    ctx.fillStyle = '#94a3b8';
    ctx.font = '9px monospace';
    ctx.fillText(this.yMax.toFixed(1), 2, 10);
    ctx.fillText(this.yMin.toFixed(1), 2, H - 2);
    if (this.yMin <= 0 && this.yMax >= 0) {
      ctx.fillText('0', 2, yToPx(0) - 2);
    }

    // チャンネル描画
    const xStep = (W - 25) / (this.bufferSize - 1);
    for (const ch of this.channels) {
      const buf = this.buffers[ch.key];
      ctx.strokeStyle = ch.color;
      ctx.lineWidth = ch.width || 1;
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < this.bufferSize; i++) {
        // ringbuf の論理順 (head から逆順、最古→最新)
        const idx = (this.head + i) % this.bufferSize;
        const v = buf[idx];
        if (isNaN(v)) { started = false; continue; }
        const x = 25 + i * xStep;
        const y = yToPx(v);
        if (!started) { ctx.moveTo(x, y); started = true; }
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    // 凡例 (右上)
    let lx = W - 8;
    ctx.font = 'bold 10px monospace';
    for (let i = this.channels.length - 1; i >= 0; i--) {
      const ch = this.channels[i];
      const last = this._getLast(ch.key);
      const lastTxt = isNaN(last) ? '—' : last.toFixed(2);
      const txt = `${ch.label}:${lastTxt}`;
      const w = ctx.measureText(txt).width;
      ctx.fillStyle = ch.color;
      ctx.fillText(txt, lx - w, 11);
      lx -= w + 8;
    }
  }

  _getLast(key) {
    const buf = this.buffers[key];
    const idx = (this.head - 1 + this.bufferSize) % this.bufferSize;
    return buf[idx];
  }
}
