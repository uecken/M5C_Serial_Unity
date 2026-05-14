// Burst Motion - core/TriggerEngine.hpp
// 統一状態機械 evaluator。複数 ActionRule を独立並列評価。
// if-else のみ、ML/DTW 不使用、遅延ゼロ。
#pragma once
#include "types.hpp"
#include <vector>
#include <ArduinoJson.h>

namespace BurstMotion {

class IHidSink;  // forward declaration

// Phase 5.39.2.7: watch event の出力先 callback
// 主目的: trigger.hit / lock event を USB Serial + BLE NUS 両方に routing
// (旧実装は Serial.printf 直接呼出で BLE NUS 接続中の Web に届かない問題対策)
using EventOutputFn = void(*)(JsonDocument& doc);

class TriggerEngine {
public:
    TriggerEngine();

    // ActionRule を追加 (runtime 用)
    void addRule(const ActionRule& rule);

    // 全ルール削除
    void clearRules();

    // ID 一致のルールを削除 (Phase 5.10)
    bool removeRule(uint16_t id);

    // HID 出力先設定
    void setHidSink(IHidSink* sink) { hid_sink_ = sink; }

    // 毎 tick 呼出 (IMU loop 内から)
    void tick(const SensorState& s);

    // watch (trigger.hit イベント通知)
    void setWatchEnabled(bool enabled) { watch_enabled_ = enabled; }

    // Phase 5.39.2.7: watch event の出力 routing 設定 (USB Serial + BLE NUS 両方経由用)
    void setEventOutputFn(EventOutputFn fn) { event_output_fn_ = fn; }

    // Closest-only モード: 姿勢条件が enabled な idle ルール群のうち、
    // 現在 quat と最も近い 1 つだけを発火対象にする (旧 getClosestPK3 互換)
    // 選定後の発火判定は通常通り (button/accel/gyro の AND 評価) — 「選択」と「発火」を分離
    void setClosestOnlyMode(bool enabled) { closest_only_mode_ = enabled; }
    bool isClosestOnlyMode() const { return closest_only_mode_; }

    // Button-edge lock パラメータ:
    //   ボタン立ち上がり時の姿勢を「凍結」して closest ルールを 1 件確定 (lock)。
    //   その後 lock window (ms) 内で他条件 (accel/gyro) を満たせば発火。
    //   発火後 cooldown (ms) 経過まで新規 lock 不可。
    // ユーザー要件: 「ボタン押下後にモーションすると姿勢が変化して別ルールに切り替わる誤発火を防ぐ」
    void setLockWindowMs(uint16_t v)   { lock_window_ms_ = v; }
    void setLockCooldownMs(uint16_t v) { lock_cooldown_ms_ = v; }
    uint16_t lockWindowMs()   const { return lock_window_ms_; }
    uint16_t lockCooldownMs() const { return lock_cooldown_ms_; }
    // 現在 lock 中のルール ID (-1 = 未 lock)、UI 可視化用
    int     currentLockedRuleId()  const { return lock_active_ ? (int)lock_locked_rule_id_ : -1; }
    uint32_t currentLockElapsedMs(uint32_t now_ms) const {
        return lock_active_ ? (now_ms - lock_started_ms_) : 0;
    }

    // 登録済みルール数
    size_t ruleCount() const { return rules_.size(); }

    // ルール取得 (iterate 用)
    const std::vector<ActionRule>& rules() const { return rules_; }

    // ============================================================
    // Phase 5.39.3a: デバイス単位 「初期姿勢」 q_initial
    //   posture_basis == PB_RELATIVE_QUAT な rule の判定基準として全 rule 共通で使用。
    //   設定タイミング: posture.init コマンド (Web UI の Init Yaw / Reset Base)、
    //                    または起動時に NVS から復元。
    //   未設定時 (= identity quaternion) は q_initial_valid_=false で、判定は絶対モードへフォールバック。
    // ============================================================
    void setInitialPosture(const float q[4]);
    void getInitialPosture(float q[4]) const;
    bool isInitialPostureValid() const { return q_initial_valid_; }
    void clearInitialPosture();   // identity に戻す (q_initial_valid_=false)

private:
    std::vector<ActionRule> rules_;
    IHidSink* hid_sink_;
    bool watch_enabled_;
    bool closest_only_mode_;
    EventOutputFn event_output_fn_ = nullptr;

    // Button-edge lock 状態
    uint16_t lock_window_ms_   = 500;   // lock 維持時間 (default 500ms)
    uint16_t lock_cooldown_ms_ = 300;   // 発火後の待機時間 (default 300ms)
    bool     lock_active_      = false;
    uint32_t lock_started_ms_  = 0;
    int      lock_rule_idx_    = -1;     // ルールの vector index
    uint16_t lock_locked_rule_id_ = 0;   // ルールの id (Web UI 用)
    uint16_t lock_button_bitmap_ = 0;    // lock 確立時のボタン状態
    uint32_t lock_cooldown_until_ms_ = 0;
    uint16_t prev_buttons_bitmap_ = 0;
    // グローバル発火 cooldown: closest_only モード問わず、ルール発火後 lock_cooldown_ms_ 期間
    // 全ルール (HOLD release 以外) の新規発火を阻止する。
    // ユーザー要件: 「cooldown 中は他のルールを適用させない」
    uint32_t last_global_fire_ms_ = 0;
    uint32_t current_tick_ms_ = 0;   // executeAction から参照する用

    // Phase 5.39.3a: デバイス単位 「初期姿勢」
    //   q_initial_       : 相対モード rule 判定用の参照クォータニオン (w,x,y,z)
    //   q_initial_valid_ : posture.init で明示設定された場合のみ true、起動直後は false
    float q_initial_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    bool  q_initial_valid_ = false;

    // 状態機械 1 ルール評価
    void evaluateRule(ActionRule& rule, const SensorState& s);

    // Closest-only モード用: 姿勢条件を pass させて他の条件のみで発火判定
    void evaluateRulePostureSelected(ActionRule& rule, const SensorState& s);

    // Condition 判定 (全サブ条件を logic_op で結合)
    // Phase 5.39: rule を非 const で取り、stillness_since_ms / q_ref を読書する。
    //             rule == nullptr の呼び出しは stillness/相対 quat 機能を無効化する後方互換パス。
    bool matchCondition(const Condition& cond, const SensorState& s,
                        ActionRule* rule = nullptr) const;

    // Action 実行 (HID 送信)
    void executeAction(const Action& action);

    // watch event 送信 (action != nullptr なら HID キー情報も含める)
    void fireWatchEvent(const ActionRule& rule, const char* phase, const Action* action = nullptr);
    // Button-edge lock の状態変化通知 (acquired / expired / fired)
    void fireLockEvent(const char* phase, uint16_t rule_id, uint32_t now_ms);
};

}  // namespace BurstMotion
