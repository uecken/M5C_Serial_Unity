// Burst Motion - core/TriggerEngine.cpp
#include "TriggerEngine.hpp"
#include "../hid/IHidSink.hpp"
#include <Arduino.h>
#include <math.h>

namespace BurstMotion {

// Forward declarations for helpers (defined later in this file)
static int evalButton(const ButtonCond& c, const SensorState& s);
static int evalPosture(const PostureCond& c, const SensorState& s);
static int evalAccel(const AccelCond& c, const SensorState& s);
static int evalGyro(const GyroCond& c, const SensorState& s);

TriggerEngine::TriggerEngine()
    : hid_sink_(nullptr), watch_enabled_(false), closest_only_mode_(false) {}

void TriggerEngine::addRule(const ActionRule& rule) {
    ActionRule r = rule;
    r.current_state = -1;
    r.state_enter_ms = 0;
    r.last_fire_ms = 0;
    rules_.push_back(r);
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
    //   - lock 中のルール本体は上で処理済みなので skip
    //
    // 重要: 同 tick で lock-fire されて cooldown が設定された場合、
    //       hard_attack 等の姿勢なしルールが同 tick で発火すると 'p' + 'h' のような
    //       同時入力になってしまう (ユーザー指摘の連続入力問題)。
    //       → cooldown 期間中は新規発火を阻止 (HOLD release のみ許可)
    bool in_cooldown = (now < lock_cooldown_until_ms_);
    for (size_t i = 0; i < rules_.size(); i++) {
        if ((int)i == lock_rule_idx_) continue;
        auto& r = rules_[i];
        if (r.current_state >= 0) {
            evaluateRule(r, s);
            continue;
        }
        if (in_cooldown) continue;  // 同 tick lock-fire 後 / 直前 cooldown は新規発火停止
        if (r.states_count == 0) continue;
        const auto& cond = r.states[0].match_condition;
        if (!cond.posture.enabled) {
            // 姿勢なしルールは通常評価 (例: SF の hard_attack = button + accel のみ)
            evaluateRule(r, s);
        }
        // 姿勢ありで lock 対象外のルールは何もしない (誤発火抑止)
    }
}

// lock 状態変化の watch event 出力
void TriggerEngine::fireLockEvent(const char* phase, uint16_t rule_id, uint32_t now_ms) {
    if (!watch_enabled_) return;
    Serial.printf("{\"type\":\"lock\",\"t\":%u,\"id\":%u,\"phase\":\"%s\"}\n",
                  now_ms, rule_id, phase);
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
            matchCondition(rule.states[0].match_condition, s)) {
            rule.current_state = 0;
            rule.state_enter_ms = now;
            executeAction(rule.states[0].on_enter);
            fireWatchEvent(rule, "enter", &rule.states[0].on_enter);

            // 単一状態 + ループなし = ONESHOT、即座に idle に戻す
            if (rule.states_count == 1 && !rule.loop) {
                executeAction(rule.states[0].on_exit);
                rule.last_fire_ms = now;
                rule.current_state = -1;
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
        return;
    }

    // HOLD_START_ONLY: 次状態 = 現状態 (ループ, 1 state)。
    // 判定は「現状態の条件が False になった」= 離脱
    if (rule.states_count == 1 && rule.loop) {
        if (!matchCondition(cur.match_condition, s)) {
            executeAction(cur.on_exit);
            fireWatchEvent(rule, "release");
            rule.current_state = -1;
        }
        return;
    }

    // 次状態の条件一致で遷移
    if (matchCondition(rule.states[next].match_condition, s)) {
        executeAction(cur.on_exit);
        rule.current_state = next;
        rule.state_enter_ms = now;
        executeAction(rule.states[next].on_enter);
        fireWatchEvent(rule, "transition", &rule.states[next].on_enter);

        // 非ループ (SEQUENCE) の最終状態 = 発火完了
        if (!rule.loop && next == rule.states_count - 1) {
            executeAction(rule.states[next].on_exit);
            rule.last_fire_ms = now;
            rule.current_state = -1;
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

bool TriggerEngine::matchCondition(const Condition& cond, const SensorState& s) const {
    int results[4];
    int count = 0;

    int r;
    r = evalButton(cond.button, s);   if (r >= 0) results[count++] = r;
    r = evalPosture(cond.posture, s); if (r >= 0) results[count++] = r;
    r = evalAccel(cond.accel, s);     if (r >= 0) results[count++] = r;
    r = evalGyro(cond.gyro, s);       if (r >= 0) results[count++] = r;

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
    Serial.printf("{\"type\":\"trigger.hit\",\"t\":%u,\"id\":%u,\"rule_name\":\"%s\",\"phase\":\"%s\"",
                  rule.state_enter_ms, rule.id, rule.name, phase);
    if (action && action->keys_len > 0) {
        Serial.printf(",\"action_type\":%u,\"keys\":[", (unsigned)action->type);
        for (uint8_t i = 0; i < action->keys_len; i++) {
            if (i > 0) Serial.print(',');
            Serial.print(action->keys[i]);
        }
        Serial.print(']');
        if (action->modifiers) Serial.printf(",\"modifiers\":%u", action->modifiers);
        if (action->duration_ms) Serial.printf(",\"duration_ms\":%u", action->duration_ms);
        if (action->interval_ms) Serial.printf(",\"interval_ms\":%u", action->interval_ms);
    }
    Serial.print("}\n");
}

}  // namespace BurstMotion
