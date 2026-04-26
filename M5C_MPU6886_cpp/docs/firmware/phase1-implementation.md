# Phase 1 実装ガイド — 新 FW コア層

## 目標
ボード非依存の core 層を新規作成し、既存 FW と並行運用可能な状態にする。

## スコープ

### 新規作成
```
src/core/
├── types.hpp              # ActionRule, Condition, State, Action
├── TriggerEngine.hpp/cpp  # 統一状態機械 evaluator
├── MahonyFilter.hpp/cpp   # 姿勢推定
├── QuatOffset.hpp/cpp     # q_ref 管理
├── Profile.hpp/cpp        # LittleFS JSON SerDe
└── AxisRemap.hpp          # 機種別リマップ (構造体定義のみ)
```

### 変更しない (Phase 2 以降)
- 既存 `src/MotionController.hpp` (旧 FW、並存)
- 既存 `src/IMU_BLEorSerial_tester.cpp` (旧 main、並存)

### 新規 PlatformIO env
```ini
[env:m5stick-c-v2]
platform = espressif32
board = m5stick-c
framework = arduino
platform_packages = framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32.git#2.0.14
monitor_speed = 115200
lib_deps =
    h2zero/NimBLE-Arduino @ ~1.4.1
    bblanchon/ArduinoJson @ ^7.0.0
build_flags =
    -D USE_NIMBLE
    -D BOARD_M5STICKC
    -D IMU_TYPE=MPU6886
    -D FW_VERSION=\"2.0.0-dev\"
build_src_filter = +<core/*> +<hal/esp32/*> +<hid/*> +<transport/*> +<main_v2.cpp>
```

新 main は `src/main_v2.cpp` に作成、既存 `IMU_BLEorSerial_tester.cpp` と共存。

## ステップバイステップ

### Step 1: platformio.ini に `[env:m5stick-c-v2]` 追加
- 既存 `[env:m5stick-c]` は保持
- `build_src_filter` で新旧を分離

### Step 2: types.hpp 作成
```cpp
// src/core/types.hpp
#pragma once
#include <cstdint>
#include <array>
#include <string>
#include <vector>

namespace BurstMotion {

enum class OnEnterActionType : uint8_t {
    NONE, PRESS, RELEASE, FIRE_ONCE, FIRE_MACRO,
    MOUSE_MOVE, MOUSE_CLICK, GAMEPAD_PRESS, CONSUMER
};

struct Action {
    OnEnterActionType type;
    std::array<uint8_t, 20> payload;   // 種別別にパース
    uint16_t duration_ms;               // fire_once 用
    uint16_t interval_ms;               // fire_macro 用
    uint8_t payload_len;
};

struct PostureCondition {
    bool enabled;
    float euler[3];
    float euler_tol[3];
    float quat[4];
    float quat_dot_min;
    bool use_quat;                      // false=Euler 判定、true=Quat 判定
};

struct AccelCondition {
    bool enabled;
    float abs_threshold;                // 合成加速度 [g]
    bool per_axis_enabled;
    float per_axis[3];
    uint8_t comparison;                 // 0=gte, 1=lte
};

struct GyroCondition {
    bool enabled;
    float abs_threshold;
    float per_axis[3];
};

struct ButtonCondition {
    bool enabled;
    uint8_t idx;                        // 0=未指定
    uint8_t state;                      // 0=pressed, 1=released, 2=any
};

enum class LogicOp : uint8_t { AND, OR };

struct Condition {
    LogicOp logic_op;
    ButtonCondition button;
    PostureCondition posture;
    AccelCondition accel;
    GyroCondition gyro;
};

struct State {
    Condition match_condition;
    uint16_t min_dwell_ms;
    uint16_t max_dwell_ms;
    Action on_enter;
    Action on_exit;
};

struct ActionRule {
    uint16_t id;
    std::string name;
    std::vector<State> states;
    bool loop;
    int8_t priority;
    uint16_t cooldown_ms;

    // runtime state (RAM only、JSON シリアライズ対象外)
    int current_state;                  // -1=idle
    uint32_t state_enter_ms;
    uint32_t last_fire_ms;              // cooldown 計算用
};

struct SensorState {
    float accel[3];
    float gyro[3];
    float euler[3];                     // user frame
    float quat[4];                      // user frame
    float accel_abs;
    float gyro_abs;
    uint16_t buttons_bitmap;            // 現ボタン状態
    uint32_t timestamp_ms;
};

}  // namespace BurstMotion
```

### Step 3: TriggerEngine.hpp/cpp 作成
状態機械 evaluator。複数ルールを毎 tick で並列評価。

### Step 4: MahonyFilter.hpp/cpp 作成
姿勢推定の統一実装。[xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) の Mahony を参考に実装。

### Step 5: Profile.hpp/cpp 作成
LittleFS `/profiles/*.json` の load/save (ArduinoJson v7)。

### Step 6: main_v2.cpp スケルトン
```cpp
// src/main_v2.cpp
#include <Arduino.h>
#include "core/types.hpp"
#include "core/TriggerEngine.hpp"
// ...

void setup() {
    Serial.begin(115200);
    Serial.println("{\"type\":\"boot\",\"fw\":\"2.0.0-dev\",\"board\":\"m5stickc\"}");
    // TODO: LittleFS init, BLE init, IMU init
}

void loop() {
    // TODO: SerialJsonLine process, TriggerEngine tick
    delay(10);
}
```

### Step 7: ビルド確認
```bash
cd C:/Users/thefu/Documents/M5C_Serial_Unity/M5C_MPU6886_cpp
pio run -e m5stick-c-v2
```

### Step 8: 既存 FW が壊れていないこと確認
```bash
pio run -e m5stick-c  # 旧 env、これも成功すること
```

## 完了条件

- [ ] `pio run -e m5stick-c-v2` が成功
- [ ] `pio run -e m5stick-c` が従来通り成功（既存 FW 保全）
- [ ] `pio run -e m5stick-c-v2 -t upload --upload-port COM8` でアップロード成功
- [ ] Serial Monitor で `{"type":"boot",...}` が見える
- [ ] docs/firmware/phase1-complete.md にサマリ記載

## 関連

- [アーキテクチャ概要](../architecture/overview.md)
- [JSON Lines プロトコル](../protocol/json-lines.md)
- [Phase 2 実装ガイド](phase2-implementation.md) (TBD)
