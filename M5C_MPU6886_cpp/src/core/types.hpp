// Burst Motion - core/types.hpp
// ボード/IMU 非依存のデータ型。ホスト側 (PC) でも単体テスト可能な設計。
//
// 重要: enum 値は Arduino.h / ESP-IDF のマクロ (EULER, NONE, PRESS, RELEASE 等) と
//       衝突しないよう、プレフィックスを付けている。
#pragma once
#include <stdint.h>

namespace BurstMotion {

// IMU センサ状態 (ボード非依存)
struct SensorState {
    float accel[3];            // 加速度 [m/s^2] (user frame)
    float gyro[3];             // ジャイロ [deg/s] (user frame)
    float euler[3];            // Euler [deg] roll/pitch/yaw (user frame)
    float quat[4];             // Quaternion w,x,y,z (user frame)
    float accel_abs;           // 合成加速度 [g]
    float gyro_abs;            // 合成ジャイロ [deg/s]
    uint16_t buttons_bitmap;   // ボタン状態 (bit 0..15)
    uint32_t timestamp_ms;     // millis() 相当
};

// アクション種別 (AT_ プレフィックスは Arduino.h の PRESS/RELEASE 等衝突回避)
enum class ActionType : uint8_t {
    AT_NONE       = 0,
    AT_PRESS      = 1,   // HID キー押下
    AT_RELEASE    = 2,   // HID キー解放
    AT_FIRE_ONCE  = 3,   // press → duration_ms 後 release（ONESHOT）
    AT_FIRE_MACRO = 4,   // keys[] を interval_ms 間隔で順次送信
    AT_MOUSE_MOVE = 5,
    AT_MOUSE_CLICK= 6,
    AT_GAMEPAD    = 7,
    AT_CONSUMER   = 8
};

// アクション定義
// key_modes (FIRE_MACRO 専用、Phase 5.14):
//   0 = FIRE   (press → wait → release、デフォルト、独立発火)
//   1 = PRESS  (press のみ、release しない = 追加押下、波動拳の ↓+→ 同時押し用)
//   2 = RELEASE (release のみ)
//   3 = RELEASE_ALL (keys[i] 無視、すべての押下キーを離す = 掃除)
struct Action {
    ActionType type;
    uint8_t keys[8];           // キーコード (HID usage or 内部 enum)
    uint8_t key_modes[8];      // 各 keys[i] のモード (FIRE_MACRO 用、上記 enum)
    uint8_t keys_len;
    uint8_t modifiers;         // Ctrl/Shift/Alt ビットマップ
    uint16_t duration_ms;      // FIRE_ONCE 用
    uint16_t interval_ms;      // FIRE_MACRO 用
    int16_t mouse_dx;
    int16_t mouse_dy;
    int8_t mouse_wheel;
    uint8_t mouse_buttons;
};

// Macro key mode 定数 (Phase 5.14)
namespace KeyMacroMode {
    constexpr uint8_t FIRE = 0;
    constexpr uint8_t PRESS = 1;
    constexpr uint8_t RELEASE = 2;
    constexpr uint8_t RELEASE_ALL = 3;
}

// 論理演算 (OP_ プレフィックスは Windows AND/OR マクロ衝突回避)
enum class LogicOp : uint8_t { OP_AND = 0, OP_OR = 1 };

// 姿勢判定方法 (BY_ プレフィックスは Arduino.h EULER マクロ衝突回避)
enum class PostureJudge : uint8_t { BY_EULER = 0, BY_QUAT = 1 };

// 比較演算
enum class Comparison : uint8_t { CMP_GTE = 0, CMP_LTE = 1 };

// 条件サブフィールド
struct PostureCond {
    bool enabled;
    float euler[3];            // 中央値 (roll, pitch, yaw) [deg]
    float euler_tol[3];        // 許容幅
    float quat[4];             // 中央値 (w,x,y,z)
    float quat_dot_min;        // 内積閾値 (cos 角度差)
    PostureJudge judge_by;
};

struct AccelCond {
    bool enabled;
    bool use_per_axis;         // false: 合成値のみ、true: 軸別
    float abs_threshold;       // 合成加速度閾値 [g]
    float per_axis[3];         // 軸別閾値
    Comparison comparison;
};

struct GyroCond {
    bool enabled;
    bool use_per_axis;
    float abs_threshold;
    float per_axis[3];
    Comparison comparison;
};

struct ButtonCond {
    bool enabled;
    uint8_t idx;               // 0-15
    uint8_t state;             // 0=pressed, 1=released, 2=any
};

// 複合条件（1 状態の判定に使う）
struct Condition {
    LogicOp logic_op;          // 内部要素の結合 AND/OR
    ButtonCond button;
    PostureCond posture;
    AccelCond accel;
    GyroCond gyro;
};

// 状態機械の 1 状態
struct State {
    Condition match_condition;
    uint16_t min_dwell_ms;     // 最小滞在時間
    uint16_t max_dwell_ms;     // タイムアウト (0=無制限)
    Action on_enter;
    Action on_exit;
};

// アクションルール (状態機械)
// 状態数 N で各トリガーモードを表現:
//   N=1, loop=false       : ONESHOT
//   N=1, loop=true        : HOLD_START_ONLY
//   N=2, loop=true        : HOLD_START_END
//   N>=2, loop=false      : SEQUENCE
struct ActionRule {
    uint16_t id;
    char name[32];
    State states[4];           // 最大 4 状態 (MVP)、SEQUENCE 長も含む
    uint8_t states_count;
    bool loop;
    int8_t priority;
    uint16_t cooldown_ms;

    // Runtime state (RAM only、JSON serialize 対象外)
    int8_t current_state;      // -1=idle
    uint32_t state_enter_ms;
    uint32_t last_fire_ms;
};

// HID 出力先 (OUT_ プレフィックスは NONE マクロ衝突回避)
enum class HidOutputMode : uint8_t {
    OUT_NONE = 0,
    OUT_BLE  = 1,
    OUT_USB  = 2,
    OUT_BOTH = 3
};

// FW 情報
struct DeviceInfo {
    const char* fw_version;
    const char* board;
    const char* imu_type;
    uint32_t uptime_ms;
    uint8_t battery_percent;
    bool ble_connected;
    bool usb_connected;
    HidOutputMode output_mode;
};

}  // namespace BurstMotion
