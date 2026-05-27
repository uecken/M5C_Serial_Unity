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
constexpr uint8_t TRIG_INCENDIO  = 0x20;  // 横振り(±X) → オレンジ flicker (前突きから移動)
constexpr uint8_t TRIG_AGUAMENTI = 0x21;  // Phase 2 下流し → 青 fade
constexpr uint8_t TRIG_WINGARDIUM = 0x22; // Wingardium Leviosa 浮遊 (連続制御)
constexpr uint8_t TRIG_EXPECTO_PATRONUM = 0x23; // 前突き(杖先端 +Y) → 守護霊 (明るい持続光)
// 0x24-0x2F は将来拡張用予約

// --- Wingardium Leviosa (浮遊、連続制御) のプロトコル ---
//   活性化: 杖を上向きに保持 (pitch 高 + 静止) を一定時間継続
//   活性化後「浮遊モード」中、杖は trigger_id=TRIG_WINGARDIUM の adv を ~50ms 間隔(20Hz)で連続送信
//   (adv interval は 20ms 固定。スマホ画面を最速更新するため)。
//   その strength バイトに「ピッチ(上下の傾き)」を載せる:
//     strength = 0   → 杖を真下向き (羽を最下部へ)
//     strength = 128 → 水平        (羽を中央へ)
//     strength = 255 → 真上向き    (羽を最上部へ)
//   受信側 (スマホ等) の扱い:
//     - TRIG_WINGARDIUM 受信で羽を表示し浮遊モードに入る
//     - 後続パケットの strength を羽の Y 位置にマップ。**seq の重複排除はせず毎パケット反映**
//       (連続制御のため。他の単発呪文は seq dedup する)
//     - スロー・イージング (y += (target-y)*0.05 等) でゆっくり浮遊させる
//     - ~1 秒 TRIG_WINGARDIUM が来なければ浮遊モード終了 (羽フェードアウト)
constexpr uint8_t WINGARDIUM_PITCH_MID = 128;  // 水平に対応する strength 値

// LED 点灯時間 (受信側で trigger_id ごとに分岐)
constexpr uint32_t LED_DURATION_SHAKE_MS     = 250;   // SHAKE=魔法失敗 → 一瞬だけ点灯
constexpr uint32_t LED_DURATION_INCENDIO_MS  = 3000;  // 横振り → オレンジ風 3 秒
constexpr uint32_t LED_DURATION_AGUAMENTI_MS = 5000;  // 下流し → 青風 5 秒
constexpr uint32_t LED_DURATION_PATRONUM_MS  = 3000;  // Expecto Patronum 前突き → 光の波動アニメ 3 秒 (点滅しながら外へ拡散)
// LUMOS は時限ではなく all_on() で永続点灯 (NOX まで)。NOX は all_off() で即時消灯。
// → LED_DURATION_LUMOS_MS は不要 (受信側で時限制御しないため定義しない)

}  // namespace wand_beacon
