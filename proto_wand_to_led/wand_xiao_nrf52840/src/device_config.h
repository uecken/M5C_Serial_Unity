#pragma once
// ============================================================
// デバイス固有設定 (XIAO nRF52840 Sense + 内蔵 LSM6DS3TR-C)
//   M5StickC 版 (wand_m5stickc/src/device_config.h) と同じ役割。
//   ジェスチャ判定コード本体は ../../shared/wand_gesture.h を共用し、
//   このファイルの #define で機体差 (IMU/LED/ボタン/軸) を吸収する。
//
//   ※ Arduino.h (variant 定義) の後に include すること。
//     PIN_LSM6DS3TR_C_*, LED_BUILTIN, D0 は variant マクロ。
// ============================================================

#define DEVICE_NAME          "XIAO-nRF52840-Sense+LSM6DS3"

// --- IMU (LSM6DS3TR-C, Wire1 直読み) ---
//   IMU は Wire1 にぶら下がる (variant: PIN_WIRE1_SDA=D17/P0.07, SCL=D16/P0.27)。
//   Wire1.begin() が自動でこのピンを使う。電源は IMU_POWER_PIN を HIGH 駆動して供給。
#define IMU_I2C_ADDR         0x6A
#define IMU_POWER_PIN        PIN_LSM6DS3TR_C_POWER   // D15/P1.08: HIGH で IMU 給電
#define IMU_INT1_PIN         PIN_LSM6DS3TR_C_INT1    // D18/P0.11: 今回未使用 (将来 WOM wake)
#define IMU_ACC_LSB_PER_G    4098.0f                 // ±8g レンジ: 0.244 mg/LSB → 1g ≈ 4098 LSB

// --- 機体軸マウント設定 (杖に実装した時の方向 → センサ軸) ---
//   符号付き軸コード: +X=1 +Y=2 +Z=3, 負で反転 (-X=-1 -Y=-2 -Z=-3)
//   ※ 杖への XIAO 実装向きが決まり次第ブリングアップで実測確定。
//     暫定は M5StickC と同じ (先端=+Y)。serial の |a|= ダンプで各軸を確認して調整する。
#define AXIS_PX  (+1)
#define AXIS_NX  (-1)
#define AXIS_PY  (+2)
#define AXIS_NY  (-2)
#define AXIS_PZ  (+3)
#define AXIS_NZ  (-3)

#define WAND_FORWARD   AXIS_PY   // 杖先端 (前方向) = +Y (暫定、要実測)
#define WAND_RIGHT     AXIS_PX   // 右方向        = +X (暫定)
#define WAND_UP        AXIS_PZ   // 上方向        = +Z (暫定)

// --- 内蔵 LED / 外付ボタン ---
//   LED_BUILTIN = 内蔵赤 LED (=LED_RED, P0.26, active-low)。M5 の GPIO10 と同じ active-low 扱い。
//   外付ボタン: タクトスイッチを BUTTON_B_PIN と GND の間に配線 (INPUT_PULLUP, 押下=LOW)。
//     M5 の B ボタン相当 (長押し=モード切替 / 一瞬押し=手動 LUMOS/NOX)。
#define BUILTIN_LED_PIN      LED_BUILTIN   // 内蔵赤 LED (active-low)
#define BUTTON_B_PIN         D0            // 外付タクト (GND 間, INPUT_PULLUP, active-low)
