// Burst Motion - PitchRollGrid (Phase 4.1)
// Roll [-180, +180] × Pitch [-90, +90] の 2D 平面に
// 現在姿勢 (赤丸)、登録ルール (橙丸)、最近傍 (緑丸)、を可視化
//
// 旧 motion_controller.js の updatePitchRollCanvas + plotPoint + drawAxis を移植

export class PitchRollGrid {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.references = [];   // [{roll, pitch, name?, rollTol, pitchTol, id}, ...]
    this.current = null;    // {roll, pitch}
    this.closest = null;    // 最近傍登録 index
    this.firingIdx = -1;
    this._firingTimer = null;
    // Phase 5.28: ドラッグ編集
    this.onRuleEdit = null; // (id, {roll?, pitch?, rollTol?, pitchTol?}) => void
    this._drag = null;      // {idx, mode: 'center'|'edge-l'|'edge-r'|'edge-t'|'edge-b', startX, startY}
    canvas.addEventListener('mousedown', this._onMouseDown.bind(this));
    canvas.addEventListener('mousemove', this._onMouseMove.bind(this));
    canvas.addEventListener('mouseup',   this._onMouseUp.bind(this));
    canvas.addEventListener('mouseleave', this._onMouseUp.bind(this));
    canvas.style.cursor = 'crosshair';
  }

  setEditCallback(cb) { this.onRuleEdit = cb; }

  // hit-test: 中央点 (12px 半径以内) or 矩形境界 (10px 以内) を判定
  _hitTest(mx, my) {
    for (let i = this.references.length - 1; i >= 0; i--) {
      const r = this.references[i];
      const cx = this._rollToX(r.roll);
      const cy = this._pitchToY(r.pitch);
      const dCenter = Math.hypot(mx - cx, my - cy);
      if (dCenter <= 12) return { idx: i, mode: 'center' };
      if (typeof r.rollTol === 'number' && typeof r.pitchTol === 'number'
          && r.rollTol < 170 && r.pitchTol < 85) {
        const xL = this._rollToX(r.roll - r.rollTol);
        const xR = this._rollToX(r.roll + r.rollTol);
        const yT = this._pitchToY(r.pitch + r.pitchTol);
        const yB = this._pitchToY(r.pitch - r.pitchTol);
        if (Math.abs(my - yT) < 6 && mx >= xL - 6 && mx <= xR + 6) return { idx: i, mode: 'edge-t' };
        if (Math.abs(my - yB) < 6 && mx >= xL - 6 && mx <= xR + 6) return { idx: i, mode: 'edge-b' };
        if (Math.abs(mx - xL) < 6 && my >= yT - 6 && my <= yB + 6) return { idx: i, mode: 'edge-l' };
        if (Math.abs(mx - xR) < 6 && my >= yT - 6 && my <= yB + 6) return { idx: i, mode: 'edge-r' };
      }
    }
    return null;
  }

  _eventToCanvas(e) {
    const rect = this.canvas.getBoundingClientRect();
    const sx = this.canvas.width / rect.width;
    const sy = this.canvas.height / rect.height;
    return { x: (e.clientX - rect.left) * sx, y: (e.clientY - rect.top) * sy };
  }

  _onMouseDown(e) {
    const { x, y } = this._eventToCanvas(e);
    const hit = this._hitTest(x, y);
    if (hit) {
      this._drag = hit;
      e.preventDefault();
    }
  }

  _onMouseMove(e) {
    const { x, y } = this._eventToCanvas(e);
    if (!this._drag) {
      const hit = this._hitTest(x, y);
      this.canvas.style.cursor = hit
        ? (hit.mode === 'center' ? 'move'
          : (hit.mode.startsWith('edge-l') || hit.mode.startsWith('edge-r')) ? 'ew-resize'
          : 'ns-resize')
        : 'crosshair';
      return;
    }
    const r = this.references[this._drag.idx];
    if (!r) return;
    // canvas 座標 → Roll/Pitch 角度に逆変換
    const margin = 30;
    const W = this.canvas.width;
    const newRoll  = ((x - margin) / (W - 2 * margin)) * 360 - 180;
    const margin2 = 20;
    const H = this.canvas.height;
    const newPitch = -(((y - (H - margin2)) / (H - 2 * margin2)) * 180) - 90;
    if (this._drag.mode === 'center') {
      r.roll = Math.max(-180, Math.min(180, Math.round(newRoll)));
      r.pitch = Math.max(-90, Math.min(90, Math.round(newPitch)));
    } else if (this._drag.mode === 'edge-l' || this._drag.mode === 'edge-r') {
      const newTol = Math.max(5, Math.min(180, Math.round(Math.abs(newRoll - r.roll))));
      r.rollTol = newTol;
    } else if (this._drag.mode === 'edge-t' || this._drag.mode === 'edge-b') {
      const newTol = Math.max(5, Math.min(90, Math.round(Math.abs(newPitch - r.pitch))));
      r.pitchTol = newTol;
    }
    this.draw();
  }

  _onMouseUp(e) {
    if (this._drag && this.onRuleEdit) {
      const r = this.references[this._drag.idx];
      if (r && r.id !== undefined) {
        this.onRuleEdit(r.id, {
          roll: r.roll, pitch: r.pitch, rollTol: r.rollTol, pitchTol: r.pitchTol,
        });
      }
    }
    this._drag = null;
    this.canvas.style.cursor = 'crosshair';
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
    // Phase 5.36: 高頻度呼出 (sensor stream 50Hz 等) に対し draw を 30Hz cap
    //   sensor は毎回 current 更新するが、canvas redraw コストが大きいので throttle。
    //   setReferences / setSequences / setFiring 等の event-driven 呼出は cap 対象外で即時 draw。
    const now = performance.now();
    if (!this._lastCurrentDrawMs || (now - this._lastCurrentDrawMs) >= 33) {
      this._lastCurrentDrawMs = now;
      this.draw();
    }
  }

  setReferences(refs) {
    this.references = refs || [];
    this.draw();
  }

  // Phase 5.34: 複数 waypoint の SEQUENCE rule を別レイヤで描画
  // seqs = [{id, name, currentState, waypoints: [{roll, pitch, rollTol, pitchTol}, ...]}, ...]
  setSequences(seqs) {
    this.sequences = seqs || [];
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

  // Phase 5.35: draw() 呼出回数をカウント (perf overlay 用)
  getDrawCount() { const n = this._drawCount || 0; this._drawCount = 0; return n; }

  draw() {
    this._drawCount = (this._drawCount || 0) + 1;
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

    // Phase 5.34: SEQUENCE 描画 (waypoint 連結 + 番号 + 矢印)
    //   既存 references の上に描画。色: 紫系 (橙単点と区別)。
    //   currentState 強調: 滞在中 = 緑塗り、未到達 = 紫薄塗り
    if (this.sequences && this.sequences.length > 0) {
      this._drawSequences();
    }

    // 現在姿勢 (赤、上から描く)
    if (this.current) {
      this._plotPoint(this.current.roll, this.current.pitch, '#ef4444', 6);
    }
  }

  _drawSequences() {
    const { ctx } = this;
    this.sequences.forEach((seq) => {
      const wps = seq.waypoints || [];
      if (wps.length < 1) return;
      const cur = seq.currentState >= 0 ? seq.currentState : -1;
      // 各 waypoint 矩形 (tol) を薄く描画
      wps.forEach((wp, i) => {
        const isActive = (i === cur);
        const isPassed = (cur >= 0 && i < cur);
        const fillStyle = isActive
          ? 'rgba(34,197,94,0.32)'           // 緑 (state 滞在中)
          : isPassed
          ? 'rgba(139,92,246,0.10)'          // 紫薄 (通過済)
          : 'rgba(139,92,246,0.22)';         // 紫 (未到達)
        const strokeStyle = isActive
          ? 'rgba(21,128,61,0.9)'
          : 'rgba(109,40,217,0.55)';
        if (typeof wp.rollTol === 'number' && typeof wp.pitchTol === 'number'
            && wp.rollTol < 170 && wp.pitchTol < 85) {
          const yTop = this._pitchToY(wp.pitch + wp.pitchTol);
          const yBot = this._pitchToY(wp.pitch - wp.pitchTol);
          this._fillRollRangeY(ctx, wp.roll, wp.rollTol, yTop, yBot, fillStyle, strokeStyle);
        }
      });
      // waypoint 間の矢印
      for (let i = 0; i < wps.length - 1; i++) {
        const x1 = this._rollToX(wps[i].roll);
        const y1 = this._pitchToY(wps[i].pitch);
        const x2 = this._rollToX(wps[i + 1].roll);
        const y2 = this._pitchToY(wps[i + 1].pitch);
        ctx.strokeStyle = (cur >= 0 && i < cur) ? 'rgba(139,92,246,0.4)' : 'rgba(109,40,217,0.8)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash((cur >= 0 && i < cur) ? [3, 3] : []);
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
        ctx.setLineDash([]);
        // 矢じり
        const angle = Math.atan2(y2 - y1, x2 - x1);
        const headSize = 7;
        ctx.beginPath();
        ctx.moveTo(x2, y2);
        ctx.lineTo(x2 - headSize * Math.cos(angle - Math.PI / 6),
                   y2 - headSize * Math.sin(angle - Math.PI / 6));
        ctx.lineTo(x2 - headSize * Math.cos(angle + Math.PI / 6),
                   y2 - headSize * Math.sin(angle + Math.PI / 6));
        ctx.closePath();
        ctx.fillStyle = (cur >= 0 && i < cur) ? 'rgba(139,92,246,0.4)' : 'rgba(109,40,217,0.85)';
        ctx.fill();
      }
      // 各 waypoint に番号 + 中心点を描画
      wps.forEach((wp, i) => {
        const x = this._rollToX(wp.roll);
        const y = this._pitchToY(wp.pitch);
        const isActive = (i === cur);
        const isPassed = (cur >= 0 && i < cur);
        const radius = isActive ? 9 : 7;
        const fill = isActive ? '#22c55e' : isPassed ? '#a78bfa' : '#8b5cf6';
        ctx.beginPath();
        ctx.arc(x, y, radius, 0, 2 * Math.PI);
        ctx.fillStyle = fill;
        ctx.fill();
        ctx.strokeStyle = '#1e293b';
        ctx.lineWidth = 1;
        ctx.stroke();
        // 番号 (1-indexed で表示)
        ctx.fillStyle = '#ffffff';
        ctx.font = `bold ${isActive ? 11 : 9}px sans-serif`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(`${i + 1}`, x, y);
        ctx.textAlign = 'start';
        ctx.textBaseline = 'alphabetic';
      });
      // 名前ラベルを waypoint[0] 近くに
      if (seq.name && wps[0]) {
        const x = this._rollToX(wps[0].roll);
        const y = this._pitchToY(wps[0].pitch);
        ctx.fillStyle = '#581c87';
        ctx.font = 'bold 10px sans-serif';
        ctx.fillText(seq.name, x + 12, y + 4);
      }
    });
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
