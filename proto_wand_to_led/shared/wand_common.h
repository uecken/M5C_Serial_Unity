#pragma once
#include <stdint.h>

// ============================================================
// 杖プラットフォーム共通の挙動定数
//   M5StickC (ESP32) / XIAO nRF52840 Sense など全ての杖で共通の値。
//   sleep の「しきい値」は共通、sleep の「手段」と「wake 源」は
//   各機の device_config.h / プラットフォーム実装側で定義する。
// ============================================================
namespace wand_common {

// 一定時間静止が続いたら sleep に入る (両機共通のしきい値)
//   M5StickC      : ESP32 deep sleep, wake = Button A (ext0)
//   XIAO nRF52840 : System OFF,       wake = LSM6DS3 wake-on-motion (INT)
constexpr uint32_t SLEEP_AFTER_SEC = 30;

}  // namespace wand_common
