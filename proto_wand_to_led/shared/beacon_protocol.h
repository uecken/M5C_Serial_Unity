#pragma once
#include <stdint.h>

namespace wand_beacon {

// BLE Manufacturer Specific Data payload (杖 → LED 全受信機 broadcast)
// 配置: company_id (2B, LE) + seq + trigger_id + strength + target_id (2B, LE)
// 合計 7 バイト (BLE adv 31 バイト制限の余裕内)
struct __attribute__((packed)) Payload {
  uint16_t company_id;   // 0xFFFF (test ID, Bluetooth SIG 未登録)
  uint8_t  seq;          // 連番 (振り 1 回 = +1)、受信側で重複抑止
  uint8_t  trigger_id;   // TRIG_* 定数のいずれか
  uint8_t  strength;     // 振り強度 0-255 (将来 LED 輝度反映)
  uint16_t target_id;    // 宛先 DEVICE_ID (2B、最大 65534 台)。TARGET_ALL=全機
};

static_assert(sizeof(Payload) == 7, "Payload must be 7 bytes");

constexpr uint16_t COMPANY_ID = 0xFFFF;
constexpr uint16_t TARGET_ALL = 0xFFFF;  // 全受信機が反応 (DEVICE_ID は 1-65534)

// Trigger ID 体系
// 0x00-0x0F: 基本ジェスチャ
// 0x10-0x1F: Phase 1 呪文 (Lumos / Nox)
// 0x20-0x2F: Phase 2 呪文 (Incendio / Aguamenti 等)
// 0x30+:     Phase 3+ 軌跡認識
constexpr uint8_t TRIG_SHAKE     = 0x01;  // Phase 0 動作確認 (任意方向)
constexpr uint8_t TRIG_LUMOS     = 0x10;  // Phase 1 上振り → 点灯
constexpr uint8_t TRIG_NOX       = 0x11;  // Phase 1 下振り → 消灯
constexpr uint8_t TRIG_INCENDIO  = 0x20;  // Phase 2 前突き → オレンジ flicker
constexpr uint8_t TRIG_AGUAMENTI = 0x21;  // Phase 2 下流し → 青 fade
// 0x22-0x2F は将来拡張用予約

// LED 点灯時間 (受信側で trigger_id ごとに分岐)
constexpr uint32_t LED_DURATION_SHAKE_MS     = 5000;
constexpr uint32_t LED_DURATION_LUMOS_MS     = 5000;
constexpr uint32_t LED_DURATION_INCENDIO_MS  = 3000;
constexpr uint32_t LED_DURATION_AGUAMENTI_MS = 5000;
// NOX は持続時間ではなく即時消灯 (タイマ無効化)

}  // namespace wand_beacon
