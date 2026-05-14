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
//
// Phase 5.33: 呪文名テキスト (Wingardium Leviosa = 18 文字) 対応のため keys[] を 24 に拡張
//   "wingardium leviosa\n" = 19 chars が最長想定。24 で余裕
struct Action {
    ActionType type;
    uint8_t keys[24];          // キーコード (HID usage or 内部 enum、Phase 5.33: 8→24)
    uint8_t key_modes[24];     // 各 keys[i] のモード (FIRE_MACRO 用、上記 enum)
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

// Phase 5.39: 姿勢判定の基準
//   PB_ABSOLUTE_EULER : Mahony 起動基準の絶対 Euler 判定 (既存挙動、後方互換)
//   PB_RELATIVE_QUAT  : Phase 5.39.3a で意味が変更:
//                       「デバイス単位の g_engine.q_initial を ref として、
//                        q_rel = q_initial* ⊗ q_current の ZYX Euler を tol 比較」
//                       (旧 Phase 5.39 の rule 単位 q_ref snapshot は撤去)
//   注: enum class ではなく uint8_t enum (シリアライズ簡略化、PostureBasis = 0/1)
enum PostureBasis : uint8_t {
    PB_ABSOLUTE_EULER = 0,
    PB_RELATIVE_QUAT  = 1,
};

// Phase 5.39.3a: waypoint 通過順序の方針
//   WO_SEQUENTIAL : state[0] → state[1] → ... → state[N-1] の固定順 (既定、既存挙動)
//   WO_UNORDERED  : 任意順 (Phase 5.39.3d.1 で実装予定、現状は受付のみで動作は SEQUENTIAL と同じ)
//   WO_DTW        : DTW マッチング (Phase 5.39.5+ で実装予定、現状は受付のみ)
enum WaypointOrder : uint8_t {
    WO_SEQUENTIAL = 0,
    WO_UNORDERED  = 1,
    WO_DTW        = 2,
};

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
    // Phase 5.39: 静止検出 (stillness) — WB Magic Caster Wand 方式
    // stillness_required=true なら、|accel - 1g| が accel_th_mg 以下 AND
    // |gyro| が gyro_th_dps 以下を window_ms 連続で満たすまで条件不成立。
    // 連続時刻は Condition 側に持てない (static 不可) ので Rule::stillness_since_ms を使用。
    bool     stillness_required;
    uint16_t stillness_window_ms;     // default 200
    uint8_t  stillness_accel_th_mg;   // default 100 (= 0.1g)
    uint8_t  stillness_gyro_th_dps;   // default 5
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
    // Phase 5.39: 姿勢判定基準 (0=ABS_EULER default、1=REL_QUAT)
    // PostureBasis enum を uint8_t で格納 (serialize 簡略化)
    uint8_t posture_basis;
    // Phase 5.39.3a: waypoint 順序方針 (WaypointOrder enum、default 0 = WO_SEQUENTIAL)
    //   現状は受付のみ (UNORDERED/DTW は将来 Phase で実装)
    uint8_t waypoint_order;

    // Runtime state (RAM only、JSON serialize 対象外)
    int8_t current_state;      // -1=idle
    uint32_t state_enter_ms;
    uint32_t last_fire_ms;

    // Phase 5.39 runtime (RAM only) — Phase 5.39.3a で q_ref の用途は変化:
    //   q_ref          : (旧 Phase 5.39.2 仕様)
    //                    Phase 5.39.3a 以降は state[0] enter での snapshot を停止。
    //                    判定は g_engine.q_initial (デバイス単位) を使用。
    //                    フィールドは将来 A 案 (rule 単位 ref) に戻す可能性のため残置。
    //   q_ref_valid    : 旧仕様の有効性フラグ。Phase 5.39.3a では常に false で運用。
    //                    判定では q_initial_valid_ (TriggerEngine メンバ) を参照する。
    //   stillness_since_ms : 静止判定で「いつから静止していたか」(0 = 未開始)
    float    q_ref[4];
    bool     q_ref_valid;
    uint32_t stillness_since_ms;
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
