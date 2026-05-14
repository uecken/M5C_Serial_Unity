// Burst Motion - core/TriggerEngine.cpp
#include "TriggerEngine.hpp"
#include "../hid/IHidSink.hpp"
#include <Arduino.h>
#include <math.h>

namespace BurstMotion {

// Forward declarations for helpers (defined later in this file)
static int evalButton(const ButtonCond& c, const SensorState& s);
static int evalPosture(const PostureCond& c, const SensorState& s);
// Phase 5.39.3a: 第 3 引数は「デバイス単位の q_initial」(以前は rule 単位 q_ref)。動作は同じ。
static int evalPostureRelative(const PostureCond& c, const SensorState& s, const float q_initial[4]);
static int evalAccel(const AccelCond& c, const SensorState& s);
static int evalGyro(const GyroCond& c, const SensorState& s);
static int evalStillness(const Condition& c, const SensorState& s, ActionRule* rule);

// Phase 5.39: Quaternion 演算ヘルパー (相対 quat 判定用)
//   - quatMultiply  : out = a ⊗ b  (Hamilton 積、w,x,y,z 順)
//   - quatConjugate : out = q* (w, -x, -y, -z)
//   - quatToZYXEuler: q → (roll, pitch, yaw) [rad]、Mahony と同じ ZYX 順序
static void quatMultiply(const float* a, const float* b, float* out) {
    // (w1, x1, y1, z1) ⊗ (w2, x2, y2, z2)
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void quatConjugate(const float* q, float* out) {
    out[0] =  q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

// ZYX (roll-pitch-yaw) Euler 抽出 — Mahony / IMU 系で標準的な定義
//   roll  = atan2(2(wx + yz), 1 - 2(x^2 + y^2))
//   pitch = asin(2(wy - zx))    [-π/2, +π/2]、ジンバルロックは ±π/2
//   yaw   = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
static void quatToZYXEuler(const float* q, float* roll, float* pitch, float* yaw) {
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    *roll  = atan2f(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));
    float sp = 2.0f * (w*y - z*x);
    if (sp >  1.0f) sp =  1.0f;
    if (sp < -1.0f) sp = -1.0f;
    *pitch = asinf(sp);
    *yaw   = atan2f(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));
}

TriggerEngine::TriggerEngine()
    : hid_sink_(nullptr), watch_enabled_(false), closest_only_mode_(false) {}

void TriggerEngine::addRule(const ActionRule& rule) {
    ActionRule r = rule;
    r.current_state = -1;
    r.state_enter_ms = 0;
    r.last_fire_ms = 0;
    // Phase 5.39 runtime 初期化 (Phase 5.39.3a 以降 q_ref は未使用、フィールド残置のみ)
    r.q_ref[0] = 1.0f; r.q_ref[1] = 0.0f; r.q_ref[2] = 0.0f; r.q_ref[3] = 0.0f;
    r.q_ref_valid = false;
    r.stillness_since_ms = 0;
    rules_.push_back(r);
}

// Phase 5.39.3a: デバイス単位 「初期姿勢」 アクセサ
void TriggerEngine::setInitialPosture(const float q[4]) {
    q_initial_[0] = q[0];
    q_initial_[1] = q[1];
    q_initial_[2] = q[2];
    q_initial_[3] = q[3];
    q_initial_valid_ = true;
}
void TriggerEngine::getInitialPosture(float q[4]) const {
    q[0] = q_initial_[0];
    q[1] = q_initial_[1];
    q[2] = q_initial_[2];
    q[3] = q_initial_[3];
}
void TriggerEngine::clearInitialPosture() {
    q_initial_[0] = 1.0f;
    q_initial_[1] = 0.0f;
    q_initial_[2] = 0.0f;
    q_initial_[3] = 0.0f;
    q_initial_valid_ = false;
}

void TriggerEngine::clearRules() {
    rules_.clear();
}

bool TriggerEngine::removeRule(uint16_t id) {
    for (auto it = rules_.begin(); it != rules_.end(); ++it) {
        if (it->id == id) {
            rules_.erase(it);
            return true;
        }
    }
    return false;
}

void TriggerEngine::tick(const SensorState& s) {
    uint32_t now = s.timestamp_ms;
    current_tick_ms_ = now;
    // 注: グローバル cooldown は採用しない。
    //     ユーザー要件「姿勢のみで HOLD_START_ONLY (WASD 等) の連続入力を阻害したくない」
    //     ため、cooldown は Closest-only モード ON 時の button-edge lock 内で完結させる。

    if (!closest_only_mode_) {
        // 通常モード: 全ルール並列評価
        for (auto& rule : rules_) {
            evaluateRule(rule, s);
        }
        prev_buttons_bitmap_ = s.buttons_bitmap;
        return;
    }

    // Closest-only モード + Button-edge lock:
    //   ユーザー要件: ボタン押下した瞬間の姿勢でルールを確定し、その後 lock_window_ms 内に
    //   他のトリガー (accel/gyro) を満たせば HID 入力。発火後は lock_cooldown_ms 経過まで再 lock 不可。
    //   理由: ボタン押下後にモーションすると姿勢が変化し、別ルールに切り替わってしまう誤発火を防ぐ。
    //
    // フロー:
    //   1. ボタン立ち上がり検出 → cooldown 過ぎ + 未 lock なら closest 計算 → lock 確立
    //   2. lock 中: 確定ルールの accel/gyro 条件を評価、満たせば発火 + cooldown 設定 + lock 解除
    //   3. lock 期限切れ or ボタン全解放 → lock 解除 (発火なし)
    //   4. 既に state にいるルール (HOLD 中) は常に通常評価 (release/transition のため)
    //   5. 姿勢 disabled なルール (button/accel/gyro のみ) は通常評価

    uint16_t btn_now = s.buttons_bitmap;
    uint16_t btn_edge = btn_now & ~prev_buttons_bitmap_;  // 立ち上がりエッジ
    prev_buttons_bitmap_ = btn_now;

    // === Cooldown 期間中: 全ルール評価を停止 (HOLD release のみ許可) ===
    // ユーザー要件: 「cooldown 中は他のルールも適用させない」
    // 連発防止のため、姿勢なしルール (hard_attack 等) や lock 対象外も含めて
    // 新規発火を完全に阻止する。HOLD で既に状態にいるルールだけは
    // release/transition のため評価する必要がある。
    if (now < lock_cooldown_until_ms_) {
        // lock 中だった場合は強制解除 (発火後 lock_active_=false にしているが念のため)
        if (lock_active_) {
            lock_active_ = false;
            lock_rule_idx_ = -1;
        }
        for (auto& r : rules_) {
            if (r.current_state >= 0) {
                evaluateRule(r, s);
            }
        }
        return;
    }

    // === Lock 解除条件チェック ===
    if (lock_active_) {
        bool window_expired = (now - lock_started_ms_) > lock_window_ms_;
        // lock 確立時に押下されていたボタン群が一つでも残っているか
        bool button_still_pressed = (btn_now & lock_button_bitmap_) != 0;
        if (window_expired || !button_still_pressed) {
            fireLockEvent("lock.expired", lock_locked_rule_id_, now);
            lock_active_ = false;
            lock_rule_idx_ = -1;
        }
    }

    // === 新規 lock 確立 ===
    if (btn_edge != 0 && !lock_active_ && now >= lock_cooldown_until_ms_) {
        // ボタン立ち上がり + cooldown 過ぎ + 未 lock → 現在の姿勢で closest を確定
        // 距離計算: Roll/Pitch の二乗距離 (yaw 無視) — ユーザー設計方針
        // クォータニオンは 3D 表示専用、判定は Euler ベース。
        // ルールの judge_by が BY_QUAT のときのみ quat 内積を使う (個別オプトイン)。
        int closest_idx = -1;
        float min_dist = 1e30f;
        for (size_t i = 0; i < rules_.size(); i++) {
            const auto& r = rules_[i];
            if (r.current_state >= 0) continue;
            if (r.states_count == 0) continue;
            const Condition& cond = r.states[0].match_condition;
            if (!cond.posture.enabled) continue;
            // ボタン条件あり → 現在 bitmap と一致するルールのみ候補
            if (cond.button.enabled) {
                if (evalButton(cond.button, s) == 0) continue;
            }
            // 姿勢距離計算
            const PostureCond& pc = cond.posture;
            float dist;
            if (pc.judge_by == PostureJudge::BY_QUAT) {
                // Quaternion 内積から角度差 (rad)、二乗してスケール合わせ (deg^2 相当)
                float dot = s.quat[0]*pc.quat[0] + s.quat[1]*pc.quat[1] +
                            s.quat[2]*pc.quat[2] + s.quat[3]*pc.quat[3];
                if (dot < 0) dot = -dot;
                if (dot > 1.0f) dot = 1.0f;
                float angle_deg = 2.0f * acosf(dot) * 57.29578f;
                dist = angle_deg * angle_deg;
            } else {
                // BY_EULER (default): Roll/Pitch の tol-正規化二乗距離 (yaw は環境依存性高いので無視)
                // Roll は ±180° wrap、Pitch は asin で ±90° 範囲 (wrap 不要)
                //
                // 改良 (Phase 5.15): euler_tol を「Closest 計算の軸重み (逆数)」としても活用。
                //   各軸の寄与 = (diff / tol)^2
                //   → tol = 15 (厳密) → 寄与大、tol = 180 (任意) → 寄与小
                //   ユーザーは「判定で使いたくない軸の tol を 180 に設定」するだけで、
                //   その軸が closest 選定に影響しなくなる (例: 昇竜拳の Roll を除外)
                float dr = s.euler[0] - pc.euler[0];
                while (dr > 180.0f)  dr -= 360.0f;
                while (dr < -180.0f) dr += 360.0f;
                float dp = s.euler[1] - pc.euler[1];
                float tr = pc.euler_tol[0] > 0.001f ? pc.euler_tol[0] : 180.0f;
                float tp = pc.euler_tol[1] > 0.001f ? pc.euler_tol[1] : 90.0f;
                float dr_n = dr / tr;
                float dp_n = dp / tp;
                dist = dr_n*dr_n + dp_n*dp_n;
            }
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = (int)i;
            }
        }
        if (closest_idx >= 0) {
            lock_active_ = true;
            lock_started_ms_ = now;
            lock_rule_idx_ = closest_idx;
            lock_locked_rule_id_ = rules_[closest_idx].id;
            lock_button_bitmap_ = btn_now;
            fireLockEvent("lock.acquired", lock_locked_rule_id_, now);
        }
    }

    // === Lock 中ルールの発火判定 (accel/gyro のみ評価) ===
    if (lock_active_ && lock_rule_idx_ >= 0 && (size_t)lock_rule_idx_ < rules_.size()) {
        ActionRule& r = rules_[lock_rule_idx_];
        if (r.current_state >= 0) {
            // HOLD 等で既に状態にいる → 通常評価
            evaluateRule(r, s);
        } else {
            // 姿勢 + ボタンを「成立」確定として、accel/gyro のみ AND 評価
            Condition mock = r.states[0].match_condition;
            mock.posture.enabled = false;
            mock.button.enabled = false;
            if (matchCondition(mock, s)) {
                // 発火: state[0] 入場
                r.current_state = 0;
                r.state_enter_ms = now;
                executeAction(r.states[0].on_enter);
                fireWatchEvent(r, "enter", &r.states[0].on_enter);
                if (r.states_count == 1 && !r.loop) {
                    executeAction(r.states[0].on_exit);
                    r.last_fire_ms = now;
                    r.current_state = -1;
                }
                // 発火後の lock 解除 + cooldown
                fireLockEvent("lock.fired", lock_locked_rule_id_, now);
                lock_cooldown_until_ms_ = now + lock_cooldown_ms_;
                lock_active_ = false;
                lock_rule_idx_ = -1;
            }
        }
    }

    // === lock 対象外のルールを通常評価 ===
    //   - 既に state にいるルール (release/transition のため必須)
    //   - 姿勢 disabled なルール (button/accel のみ)
    //   - HOLD_START_ONLY (loop=true, 1 state) は姿勢ありでも通常評価 (Phase 5.27)
    //     → 移動系 HOLD ルールが Closest-only モードで永遠に発火しない問題対策
    //   - lock 中のルール本体は上で処理済みなので skip
    bool in_cooldown = (now < lock_cooldown_until_ms_);
    for (size_t i = 0; i < rules_.size(); i++) {
        if ((int)i == lock_rule_idx_) continue;
        auto& r = rules_[i];
        if (r.current_state >= 0) {
            evaluateRule(r, s);
            continue;
        }
        if (in_cooldown) continue;
        if (r.states_count == 0) continue;
        const auto& cond = r.states[0].match_condition;
        if (!cond.posture.enabled) {
            // 姿勢なしルールは通常評価
            evaluateRule(r, s);
            continue;
        }
        // HOLD_START_ONLY (loop=true, states_count=1) も姿勢ありで通常評価
        // (Btn release で発火する移動系ルール用、lock 経路を経由しない)
        if (r.loop && r.states_count == 1) {
            evaluateRule(r, s);
            continue;
        }
        // それ以外の姿勢ありルール (ONESHOT / HOLD_START_END) は lock 経由のみ
    }
}

// lock 状態変化の watch event 出力
void TriggerEngine::fireLockEvent(const char* phase, uint16_t rule_id, uint32_t now_ms) {
    if (!watch_enabled_) return;
    JsonDocument doc;
    doc["type"] = "lock";
    doc["t"] = now_ms;
    doc["id"] = rule_id;
    doc["phase"] = phase;
    if (event_output_fn_) {
        event_output_fn_(doc);
    } else {
        serializeJson(doc, Serial);
        Serial.print("\n");
    }
}

// Closest-only モード用: 姿勢は最近傍判定で選ばれた前提で、
// posture を pass させて他の条件 (button/accel/gyro) のみで発火判定する。
// button 条件は絞り込みフェーズで既に成立しているので、ここで再評価しても問題ない。
void TriggerEngine::evaluateRulePostureSelected(ActionRule& rule, const SensorState& s) {
    if (rule.current_state >= 0) return;
    uint32_t now = s.timestamp_ms;
    // cooldown
    if (rule.cooldown_ms > 0 && rule.last_fire_ms != 0 &&
        (now - rule.last_fire_ms) < rule.cooldown_ms) {
        return;
    }
    // posture を一時的に disable した条件で AND 評価
    Condition mock = rule.states[0].match_condition;
    mock.posture.enabled = false;
    if (!matchCondition(mock, s)) return;

    // 発火: state[0] 入場
    rule.current_state = 0;
    rule.state_enter_ms = now;
    executeAction(rule.states[0].on_enter);
    fireWatchEvent(rule, "enter", &rule.states[0].on_enter);

    // ONESHOT (1 状態 + ループなし) なら即 idle 復帰
    if (rule.states_count == 1 && !rule.loop) {
        executeAction(rule.states[0].on_exit);
        rule.last_fire_ms = now;
        rule.current_state = -1;
    }
}

void TriggerEngine::evaluateRule(ActionRule& rule, const SensorState& s) {
    uint32_t now = s.timestamp_ms;

    if (rule.current_state < 0) {
        // idle → 最初の state (states[0]) の条件チェック
        // cooldown 中は無視
        if (rule.cooldown_ms > 0 &&
            rule.last_fire_ms != 0 &&
            (now - rule.last_fire_ms) < rule.cooldown_ms) {
            return;
        }
        if (rule.states_count > 0 &&
            matchCondition(rule.states[0].match_condition, s, &rule)) {
            rule.current_state = 0;
            rule.state_enter_ms = now;
            // Phase 5.39.3a: state[0] enter での q_ref スナップショットは撤去。
            //   判定は g_engine.q_initial (デバイス単位、posture.init で明示設定) を使用する。
            //   下記は旧仕様の保留コードであり、将来 rule 単位 ref に戻す可能性のため残す:
            // if (rule.posture_basis == PB_RELATIVE_QUAT) {
            //     rule.q_ref[0] = s.quat[0];
            //     rule.q_ref[1] = s.quat[1];
            //     rule.q_ref[2] = s.quat[2];
            //     rule.q_ref[3] = s.quat[3];
            //     rule.q_ref_valid = true;
            // }
            executeAction(rule.states[0].on_enter);
            fireWatchEvent(rule, "enter", &rule.states[0].on_enter);

            // 単一状態 + ループなし = ONESHOT、即座に idle に戻す
            if (rule.states_count == 1 && !rule.loop) {
                executeAction(rule.states[0].on_exit);
                rule.last_fire_ms = now;
                rule.current_state = -1;
                rule.q_ref_valid = false;
                rule.stillness_since_ms = 0;
            }
        }
        return;
    }

    // 現在状態の処理
    State& cur = rule.states[rule.current_state];

    // タイムアウト
    if (cur.max_dwell_ms > 0 &&
        (now - rule.state_enter_ms) > cur.max_dwell_ms) {
        executeAction(cur.on_exit);
        fireWatchEvent(rule, "timeout");
        rule.current_state = -1;
        rule.q_ref_valid = false;
        rule.stillness_since_ms = 0;
        return;
    }

    // 次状態を検討
    int next = rule.current_state + 1;
    if (next >= rule.states_count) {
        if (rule.loop) {
            next = 0;   // 循環 (HOLD_*)
        } else {
            next = -1;  // 完了 → idle
        }
    }

    if (next < 0) {
        // 全状態通過後の idle 復帰 (実際は上の分岐で到達しない)
        executeAction(cur.on_exit);
        rule.last_fire_ms = now;
        rule.current_state = -1;
        rule.q_ref_valid = false;
        rule.stillness_since_ms = 0;
        return;
    }

    // HOLD_START_ONLY: 次状態 = 現状態 (ループ, 1 state)。
    // 判定は「現状態の条件が False になった」= 離脱
    if (rule.states_count == 1 && rule.loop) {
        if (!matchCondition(cur.match_condition, s, &rule)) {
            executeAction(cur.on_exit);
            fireWatchEvent(rule, "release");
            rule.current_state = -1;
            rule.q_ref_valid = false;
            rule.stillness_since_ms = 0;
        }
        return;
    }

    // Phase 5.39.2.10: hold_with_waypoints (loop=false, states_count>=2) で、
    // rule 代表 button (state[0].button) が成立しなくなったら release event 発火 + state リセット。
    // 「Btn3 押下中だけジェスチャ評価、離したら中断」UX を実現。
    // 現在 state がどこ (state[N-1] = end_posture で button=None でも) にいても
    // rule の代表 button condition を見るので確実に release 発火。
    if (!rule.loop && rule.states_count >= 2) {
        const auto& btn0 = rule.states[0].match_condition.button;
        if (btn0.enabled && evalButton(btn0, s) == 0) {
            executeAction(cur.on_exit);
            fireWatchEvent(rule, "release");
            rule.current_state = -1;
            rule.q_ref_valid = false;
            rule.stillness_since_ms = 0;
            return;
        }
    }

    // 次状態の条件一致で遷移
    if (matchCondition(rule.states[next].match_condition, s, &rule)) {
        executeAction(cur.on_exit);
        rule.current_state = next;
        rule.state_enter_ms = now;
        // stillness は state ごとに測り直す (next state が stillness 要求でも別途累積開始)
        rule.stillness_since_ms = 0;
        executeAction(rule.states[next].on_enter);
        fireWatchEvent(rule, "transition", &rule.states[next].on_enter);

        // 非ループ (SEQUENCE) の最終状態 = 発火完了
        if (!rule.loop && next == rule.states_count - 1) {
            executeAction(rule.states[next].on_exit);
            rule.last_fire_ms = now;
            rule.current_state = -1;
            rule.q_ref_valid = false;
            rule.stillness_since_ms = 0;
        }
    }
}

// Helper: 単一サブ条件を評価、-1=disabled/0=false/1=true
static int evalButton(const ButtonCond& c, const SensorState& s) {
    if (!c.enabled) return -1;
    uint8_t idx = c.idx;
    if (idx == 0 || idx > 15) return 0;
    bool pressed = (s.buttons_bitmap >> (idx - 1)) & 1;
    if (c.state == 0) return pressed ? 1 : 0;
    if (c.state == 1) return pressed ? 0 : 1;
    return 1;  // any
}

static int evalPosture(const PostureCond& c, const SensorState& s) {
    if (!c.enabled) return -1;
    if (c.judge_by == PostureJudge::BY_QUAT) {
        float dot = s.quat[0]*c.quat[0] + s.quat[1]*c.quat[1] +
                    s.quat[2]*c.quat[2] + s.quat[3]*c.quat[3];
        if (dot < 0) dot = -dot;
        return (dot >= c.quat_dot_min) ? 1 : 0;
    }
    for (int i = 0; i < 3; i++) {
        float diff = s.euler[i] - c.euler[i];
        // Roll (i=0) と Yaw (i=2) は ±180° で循環するので wrap、Pitch (i=1) は wrap 不要 (±90° 範囲)
        if (i == 0 || i == 2) {
            while (diff > 180.0f) diff -= 360.0f;
            while (diff < -180.0f) diff += 360.0f;
        }
        if (fabsf(diff) > c.euler_tol[i]) return 0;
    }
    return 1;
}

// Phase 5.39: 相対 Quaternion ベースの姿勢判定
//   q_rel = quat_conj(q_ref) ⊗ q_current
//   q_rel から ZYX Euler を抽出 (deg) → posture.euler との差を tol 比較
//   posture.euler は「ref からの相対 Euler オフセット」として解釈される。
static int evalPostureRelative(const PostureCond& c, const SensorState& s, const float q_ref[4]) {
    if (!c.enabled) return -1;
    float q_conj[4];
    quatConjugate(q_ref, q_conj);
    float q_rel[4];
    quatMultiply(q_conj, s.quat, q_rel);

    if (c.judge_by == PostureJudge::BY_QUAT) {
        // 相対 quat と target quat (posture.quat) の内積で判定 (target も ref 基準)
        float dot = q_rel[0]*c.quat[0] + q_rel[1]*c.quat[1] +
                    q_rel[2]*c.quat[2] + q_rel[3]*c.quat[3];
        if (dot < 0) dot = -dot;
        return (dot >= c.quat_dot_min) ? 1 : 0;
    }

    float rel_roll_rad, rel_pitch_rad, rel_yaw_rad;
    quatToZYXEuler(q_rel, &rel_roll_rad, &rel_pitch_rad, &rel_yaw_rad);
    const float RAD2DEG_F = 57.29577951f;
    float rel_euler[3];
    rel_euler[0] = rel_roll_rad  * RAD2DEG_F;
    rel_euler[1] = rel_pitch_rad * RAD2DEG_F;
    rel_euler[2] = rel_yaw_rad   * RAD2DEG_F;
    for (int i = 0; i < 3; i++) {
        float diff = rel_euler[i] - c.euler[i];
        if (i == 0 || i == 2) {
            while (diff > 180.0f) diff -= 360.0f;
            while (diff < -180.0f) diff += 360.0f;
        }
        if (fabsf(diff) > c.euler_tol[i]) return 0;
    }
    return 1;
}

// Phase 5.39: 静止検出 (WB Magic Caster Wand 方式)
//   |sqrt(ax^2+ay^2+az^2) - 9.81| < accel_th_mg * 0.00981 m/s^2  (mg → m/s^2)
//   sqrt(gx^2+gy^2+gz^2) < gyro_th_dps
//   両者を window_ms 連続で満たす場合に 1 を返す。動きが入った瞬間に rule->stillness_since_ms をリセット。
//   rule == nullptr の場合は持続時刻を持てないので、瞬間値の判定のみ (連続条件はパスとみなす最低限実装)。
static int evalStillness(const Condition& c, const SensorState& s, ActionRule* rule) {
    if (!c.stillness_required) return -1;
    float a_mag = sqrtf(s.accel[0]*s.accel[0] + s.accel[1]*s.accel[1] + s.accel[2]*s.accel[2]);
    float a_dev = fabsf(a_mag - 9.81f);
    float g_mag = sqrtf(s.gyro[0]*s.gyro[0] + s.gyro[1]*s.gyro[1] + s.gyro[2]*s.gyro[2]);
    float a_th = c.stillness_accel_th_mg * 0.00981f;  // mg → m/s^2
    float g_th = (float)c.stillness_gyro_th_dps;
    bool still_now = (a_dev < a_th) && (g_mag < g_th);

    if (rule == nullptr) {
        // 連続時刻を保持できないので瞬間値のみ
        return still_now ? 1 : 0;
    }

    uint32_t now = s.timestamp_ms;
    uint16_t window = c.stillness_window_ms > 0 ? c.stillness_window_ms : 200;
    if (still_now) {
        if (rule->stillness_since_ms == 0) {
            rule->stillness_since_ms = now;
        }
        if ((now - rule->stillness_since_ms) >= window) {
            return 1;
        }
        return 0;
    } else {
        rule->stillness_since_ms = 0;
        return 0;
    }
}

static int evalAccel(const AccelCond& c, const SensorState& s) {
    if (!c.enabled) return -1;
    if (c.use_per_axis) {
        for (int i = 0; i < 3; i++) {
            float v = fabsf(s.accel[i]);
            bool ok = (c.comparison == Comparison::CMP_GTE)
                        ? (v >= c.per_axis[i])
                        : (v <= c.per_axis[i]);
            if (!ok) return 0;
        }
        return 1;
    }
    bool ok = (c.comparison == Comparison::CMP_GTE)
                ? (s.accel_abs >= c.abs_threshold)
                : (s.accel_abs <= c.abs_threshold);
    return ok ? 1 : 0;
}

static int evalGyro(const GyroCond& c, const SensorState& s) {
    if (!c.enabled) return -1;
    if (c.use_per_axis) {
        for (int i = 0; i < 3; i++) {
            float v = fabsf(s.gyro[i]);
            bool ok = (c.comparison == Comparison::CMP_GTE)
                        ? (v >= c.per_axis[i])
                        : (v <= c.per_axis[i]);
            if (!ok) return 0;
        }
        return 1;
    }
    bool ok = (c.comparison == Comparison::CMP_GTE)
                ? (s.gyro_abs >= c.abs_threshold)
                : (s.gyro_abs <= c.abs_threshold);
    return ok ? 1 : 0;
}

bool TriggerEngine::matchCondition(const Condition& cond, const SensorState& s,
                                   ActionRule* rule) const {
    // Phase 5.39: 5 つのサブ条件 (button / posture / accel / gyro / stillness) を logic_op で結合
    int results[5];
    int count = 0;

    int r;
    r = evalButton(cond.button, s);   if (r >= 0) results[count++] = r;
    // Phase 5.39.3a: rule の posture_basis を見て absolute / relative を切替
    //   relative モードかつ デバイス単位 q_initial_valid_ のときだけ相対判定、
    //   それ以外 (= q_initial 未設定など) は絶対判定にフォールバック。
    //   q_initial は posture.init コマンドで設定され、TriggerEngine メンバとして保持される。
    if (rule != nullptr && rule->posture_basis == PB_RELATIVE_QUAT && q_initial_valid_) {
        r = evalPostureRelative(cond.posture, s, q_initial_);
    } else {
        r = evalPosture(cond.posture, s);
    }
    if (r >= 0) results[count++] = r;
    r = evalAccel(cond.accel, s);     if (r >= 0) results[count++] = r;
    r = evalGyro(cond.gyro, s);       if (r >= 0) results[count++] = r;
    // Phase 5.39: 静止検出 (Condition の AND/OR ロジックに参加)
    r = evalStillness(cond, s, rule); if (r >= 0) results[count++] = r;

    if (count == 0) return true;  // 全部 disabled = 常に true
    if (cond.logic_op == LogicOp::OP_AND) {
        for (int i = 0; i < count; i++) if (!results[i]) return false;
        return true;
    }
    for (int i = 0; i < count; i++) if (results[i]) return true;
    return false;
}

void TriggerEngine::executeAction(const Action& action) {
    if (!hid_sink_ || action.type == ActionType::AT_NONE) return;

    switch (action.type) {
        case ActionType::AT_PRESS:
            for (uint8_t i = 0; i < action.keys_len; i++) {
                hid_sink_->pressKey(action.keys[i]);
            }
            break;
        case ActionType::AT_RELEASE:
            for (uint8_t i = 0; i < action.keys_len; i++) {
                hid_sink_->releaseKey(action.keys[i]);
            }
            break;
        case ActionType::AT_FIRE_ONCE:
            for (uint8_t i = 0; i < action.keys_len; i++) {
                hid_sink_->pressKey(action.keys[i]);
            }
            delay(action.duration_ms > 0 ? action.duration_ms : 20);
            for (uint8_t i = 0; i < action.keys_len; i++) {
                hid_sink_->releaseKey(action.keys[i]);
            }
            break;
        case ActionType::AT_FIRE_MACRO: {
            // Phase 5.14: key_modes による同時押し対応
            //   FIRE        : press → wait → release (デフォルト)
            //   PRESS       : press のみ (release しない、追加押下)
            //   RELEASE     : release のみ
            //   RELEASE_ALL : 全押下キー解放
            uint16_t step_ms = action.interval_ms > 0 ? action.interval_ms : 30;
            for (uint8_t i = 0; i < action.keys_len; i++) {
                uint8_t code = action.keys[i];
                uint8_t mode = action.key_modes[i];
                switch (mode) {
                    case KeyMacroMode::PRESS:
                        hid_sink_->pressKey(code);
                        break;
                    case KeyMacroMode::RELEASE:
                        hid_sink_->releaseKey(code);
                        break;
                    case KeyMacroMode::RELEASE_ALL:
                        hid_sink_->releaseAll();
                        break;
                    case KeyMacroMode::FIRE:
                    default:
                        hid_sink_->pressKey(code);
                        delay(step_ms);
                        hid_sink_->releaseKey(code);
                        break;
                }
                delay(step_ms);
            }
            break;
        }
        case ActionType::AT_MOUSE_MOVE:
            hid_sink_->moveMouse(action.mouse_dx, action.mouse_dy, action.mouse_wheel);
            break;
        case ActionType::AT_MOUSE_CLICK:
            hid_sink_->clickMouse(action.mouse_buttons);
            break;
        default:
            break;
    }
}

void TriggerEngine::fireWatchEvent(const ActionRule& rule, const char* phase, const Action* action) {
    if (!watch_enabled_) return;
    // Phase 5.39.2.7: JsonDocument に組立て、callback 経由で USB + BLE NUS 両方に出力
    JsonDocument doc;
    doc["type"] = "trigger.hit";
    doc["t"] = rule.state_enter_ms;
    doc["id"] = rule.id;
    doc["rule_name"] = rule.name;
    doc["phase"] = phase;
    if (action && action->keys_len > 0) {
        doc["action_type"] = (unsigned)action->type;
        JsonArray keys = doc["keys"].to<JsonArray>();
        for (uint8_t i = 0; i < action->keys_len; i++) {
            keys.add(action->keys[i]);
        }
        if (action->modifiers) doc["modifiers"] = action->modifiers;
        if (action->duration_ms) doc["duration_ms"] = action->duration_ms;
        if (action->interval_ms) doc["interval_ms"] = action->interval_ms;
    }
    // Phase 5.39.3a: enter phase + 相対モード時に「デバイス単位 q_initial」を出力。
    //   旧 Phase 5.39.2 では rule.q_ref (state[0] enter snapshot) を出力していたが、
    //   Phase 5.39.3a 以降は g_engine.q_initial_ (posture.init で明示設定) を出力する。
    //   Web 側は q_ref キーで受信し、軌跡固定描画 / 相対 3D ビュアに使う (キー名は互換性のため維持)。
    if (phase && strcmp(phase, "enter") == 0 && rule.posture_basis == PB_RELATIVE_QUAT && q_initial_valid_) {
        JsonArray qref = doc["q_ref"].to<JsonArray>();
        qref.add(q_initial_[0]);
        qref.add(q_initial_[1]);
        qref.add(q_initial_[2]);
        qref.add(q_initial_[3]);
    }
    // callback 経由で routing (両方経由)、未設定時は Serial 直接出力 (後方互換)
    if (event_output_fn_) {
        event_output_fn_(doc);
    } else {
        serializeJson(doc, Serial);
        Serial.print("\n");
    }
}

}  // namespace BurstMotion
