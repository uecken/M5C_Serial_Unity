# 2026-04-25 Phase 4.3: Closest-only モード (TriggerEngine 発火制限)

## 背景
Phase 4.2 で `rule.closest` コマンド + Web UI 表示を実装したが、これは **表示のみ**で、
実際のトリガー判定は依然として「複数ルールの開始条件が同時マッチすると全部発火」していた。

旧 motion_controller.js の `getClosestPK3` は「**登録姿勢に最も近いルールだけを実行**」する
仕様だったので、同じ動作を新 FW にも持ち込む。

## 設計

### モード切替式 (default OFF)
- `engine.closest_only` コマンドで実行時切替
- OFF (default): 従来通り全マッチ並列発火 — Phase 4.2 までの動作と互換
- ON: 姿勢 enabled な idle ルールのうち、現在 quat と最近傍 1 件だけ評価

### TriggerEngine::tick() の二相処理 (closest-only ON 時)

**Phase A: 最近傍探索**
- `current_state >= 0` (= 既に状態にいる) ルールはスキップ → release/transition 評価のため後段で処理
- 姿勢 enabled な idle ルールのみ対象
- 各ルール `states[0].match_condition.posture.quat` と `s.quat` の内積から角度差 `2*acos(|dot|)` を計算
- 最小角度のインデックス `closest_idx` を確定

**Phase B: 評価**
- `current_state >= 0` ルール → 通常評価 (release を取りこぼさない)
- 姿勢 disabled な idle ルール (button/accel/gyro のみ) → 通常評価
- 姿勢 enabled な idle ルール → `closest_idx` 一致のみ `evaluateRule()`

### コード抜粋
```cpp
void TriggerEngine::tick(const SensorState& s) {
    if (!closest_only_mode_) {
        for (auto& rule : rules_) evaluateRule(rule, s);
        return;
    }
    int closest_idx = -1;
    float min_angle = 4.0f;
    for (size_t i = 0; i < rules_.size(); i++) {
        const auto& r = rules_[i];
        if (r.current_state >= 0) continue;
        if (r.states_count == 0) continue;
        const auto& pc = r.states[0].match_condition.posture;
        if (!pc.enabled) continue;
        float dot = s.quat[0]*pc.quat[0] + s.quat[1]*pc.quat[1] +
                    s.quat[2]*pc.quat[2] + s.quat[3]*pc.quat[3];
        if (dot < 0) dot = -dot;
        if (dot > 1.0f) dot = 1.0f;
        float angle = 2.0f * acosf(dot);
        if (angle < min_angle) { min_angle = angle; closest_idx = (int)i; }
    }
    for (size_t i = 0; i < rules_.size(); i++) {
        auto& r = rules_[i];
        if (r.current_state >= 0)                            { evaluateRule(r, s); continue; }
        if (r.states_count == 0)                             continue;
        const auto& pc = r.states[0].match_condition.posture;
        if (!pc.enabled)                                     { evaluateRule(r, s); continue; }
        if ((int)i == closest_idx)                            evaluateRule(r, s);
    }
}
```

### CPU 負荷
- 1 ルールあたり 1 回の dot + acos = 数 µs
- 全ルール走査でも N=50 ルールで <1 ms
- 100Hz tick (10ms) に対して余裕

## FW 変更

### TriggerEngine
- `closest_only_mode_` フラグ追加 (default false)
- `setClosestOnlyMode(bool)` / `isClosestOnlyMode()`
- `tick()` 内分岐実装

### main_v2.cpp
- 新コマンド `engine.closest_only`:
  ```jsonc
  → {"cmd":"engine.closest_only","enabled":true}
  ← {"type":"ack","cmd":"engine.closest_only","ok":true,"enabled":true}

  → {"cmd":"engine.closest_only"}                    // 引数なし = 現在値取得
  ← {"type":"ack","cmd":"engine.closest_only","ok":true,"enabled":false}
  ```
- `device.info` 応答に `closest_only` フィールド追加

## Web 変更

### app.js
- `closestOnlyMode` state 追加
- `device.info` 応答受信時に FW 状態を反映する useEffect
- ack 受信時の同期処理 (`engine.closest_only` cmd の応答を捕捉)
- Roll/Pitch 2D グリッド下に **Closest-only モード** チェックボックス追加
- ON 時はバッジ表示 "ON: 旧 getClosestPK3 互換"
- OFF 時はバッジ表示 "OFF: 全マッチ並列発火"

## ビルド結果
```
RAM:   11.9% (39136 / 327680 bytes)
Flash: 54.4% (713389 / 1310720 bytes)
```
変化なし (FW サイズに有意差なし、関数追加のみ)。

## 公開
https://uecken.github.io/M5C_Serial_Unity/

## 残課題
- **HOLD_START_END で開始/終了で別キー指定** (現状同じキーの press/release のみ)
- 6 点キャリブ結果の NVS 永続化 (現状 RAM のみ)
- 重力比較グラフ移植 (旧 page4、優先度低)
- 傾斜インジケータ移植 (旧 page5、優先度低)

## ファイル変更
- 改訂: `src/core/TriggerEngine.hpp` (`closest_only_mode_` + accessors)
- 改訂: `src/core/TriggerEngine.cpp` (tick() の二相分岐、math.h include)
- 改訂: `src/main_v2.cpp` (`engine.closest_only` コマンド、`device.info` 拡張)
- 改訂: `Web/hidconfig/src/app.js` (state + UI トグル + ack 同期)
- 新規: `docs/changelog/2026-04-25-phase4-3-closest-only-mode.md`
