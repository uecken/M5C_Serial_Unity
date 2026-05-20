#pragma once
// ============================================================
// デバイス固有設定 (M5StickC + MPU6886)
//   新しい機体 (XIAO nRF52840 Sense, ESP32+外付 IMU 等) に移植する時は
//   このファイルだけを差し替える。コード本体は #define を参照する。
// ============================================================

#define DEVICE_NAME          "M5StickC+MPU6886"

// --- IMU (MPU6886, I2C 直読み) ---
#define IMU_I2C_ADDR         0x68
#define IMU_I2C_SDA          21
#define IMU_I2C_SCL          22
#define IMU_ACC_LSB_PER_G    4096.0f   // ±8g レンジ: 4096 LSB/g

// --- 機体軸マウント設定 (杖に実装した時の方向 → センサ軸) ---
//   杖の Forward(先端) / Right(右) / Up(上) が、センサの ±X/±Y/±Z の
//   どれに当たるかを直接指定する。3 方向を明示するので handedness 設定は不要
//   (左右の符号曖昧さが原理的に発生しない)。
//
//   符号付き軸コード:  +X=1 +Y=2 +Z=3,  負で反転 (-X=-1 -Y=-2 -Z=-3)
//   ※ 3 つは互いに異なる軸 (有効な置換) であること
//   ユーザー環境 (M5StickC 杖持ち): 先端=+Y
#define AXIS_PX  (+1)
#define AXIS_NX  (-1)
#define AXIS_PY  (+2)
#define AXIS_NY  (-2)
#define AXIS_PZ  (+3)
#define AXIS_NZ  (-3)

#define WAND_FORWARD   AXIS_PY   // 杖先端 (前方向) = +Y
#define WAND_RIGHT     AXIS_PX   // 右方向        = +X
#define WAND_UP        AXIS_PZ   // 上方向        = +Z (実測重力の整合チェック/初期値用)
//   ※ 上下フリック判定は WAND_UP ではなく実測重力を使う

// --- 内蔵 LED / ボタン (M5StickC) ---
#define BUILTIN_LED_PIN      10        // 内蔵赤 LED (active-low)
#define BUTTON_A_PIN         37        // 前面 A ボタン (active-low, input-only)
