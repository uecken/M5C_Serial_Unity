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

// --- 軸割当 (body frame) ---
//   tip   = 杖先端 (前方向)、right = 右方向、上方向は重力から実測
//   INDEX: 0=X, 1=Y, 2=Z   SIGN: +1 / -1
//   ※ 実機テストで確定すること (ユーザー環境: 前突き=Y)
#define AXIS_TIP_INDEX       1         // Y
#define AXIS_TIP_SIGN        (+1)
#define AXIS_RIGHT_INDEX     0         // X
#define AXIS_RIGHT_SIGN      (+1)

// --- 座標系 handedness ---
//   +1 = 右手系 (X×Y=+Z),  -1 = 左手系
//   ※ 断定せず実機検証で設定する値。MPU6886 チップは右手系だが、
//      基板実装・軸読みで変わりうるため必ず回転テストで確認:
//      +Z まわり反時計回り(上から見て) → gz>0 なら右手系(+1)
#define IMU_HANDEDNESS       (+1)      // ← 要実機検証 (暫定値)

// --- 内蔵 LED / ボタン (M5StickC) ---
#define BUILTIN_LED_PIN      10        // 内蔵赤 LED (active-low)
#define BUTTON_A_PIN         37        // 前面 A ボタン (active-low, input-only)
