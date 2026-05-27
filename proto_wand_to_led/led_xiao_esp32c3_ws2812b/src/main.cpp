// proto_wand_to_led / led_xiao_esp32c3_ws2812b / src/main.cpp
// LED 受信機 (空中の魔法演出): XIAO ESP32-C3 + NimBLE scan + WS2812B (NeoPixel)
//
//   杖の BLE Advertising (Manufacturer Data, company_id=0xFFFF) を連続スキャンで受信し、
//   trigger_id ごとに WS2812B の演出を切り替える。ケース外・外部 5V 給電前提。
//
//   配線: C3 の LED_DATA_PIN ─[330Ω]─▶ WS2812B DIN / 外部5V ─ WS2812B 5V,GND
//        ⚠ 外部5V電源の GND と C3 の GND を必ず接続 (共通GND)
//        ⚠ DIN は 3.3V ロジック。確実性のため 74AHCT125 等で 3.3V→5V レベルシフト推奨
//
//   ★ 調整は下記 #define (画素数 / データピン / 上限輝度) だけ変えればよい。

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>
#include "../../shared/beacon_protocol.h"

// ============================================================
// ★ ハードウェア設定 (ここだけ変えればよい)
// ============================================================
#define NUM_PIXELS     30          // WS2812B の画素数
#define LED_DATA_PIN   10          // データピン (XIAO ESP32-C3: D10 = GPIO10)
#define MAX_BRIGHTNESS 160         // 0-255 上限輝度 (外部5Vだが眩しさ/発熱の抑制)
#define DEVICE_ID      1           // この受信機の宛先ID (TARGET_ALL or 一致で反応)

Adafruit_NeoPixel strip(NUM_PIXELS, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================
// 色定義 (Web 版 wand_receiver と合わせた配色)
// ============================================================
static inline uint32_t C(uint8_t r, uint8_t g, uint8_t b) { return strip.Color(r, g, b); }
static const uint32_t COL_LUMOS    = 0;  // 実際は下で warm white を生成
static const uint8_t  WARM_R = 255, WARM_G = 240, WARM_B = 200;   // Lumos 暖色白
static const uint8_t  INC_R  = 255, INC_G  = 70,  INC_B  = 0;     // Incendio 橙
static const uint8_t  AGU_R  = 0,   AGU_G  = 120, AGU_B  = 255;   // Aguamenti 青
static const uint8_t  PAT_R  = 190, PAT_G  = 225, PAT_B  = 255;   // Patronum/Wingardium 銀青

// ============================================================
// 演出ステートマシン (BLE コールバックは要求だけ積み、描画は loop で行う)
// ============================================================
enum class FX { OFF, LUMOS, FLASH, INCENDIO, AGUAMENTI, PATRONUM, WINGARDIUM };

FX        fx           = FX::OFF;
uint32_t  fx_start_ms  = 0;
uint32_t  fx_dur_ms    = 0;       // 0 = 持続 (Lumos)
bool      base_on      = false;   // Lumos の持続状態 (一時演出後の戻り先)
float     wing_pos     = 0.5f;    // Wingardium: 現在の光位置 0..1
float     wing_target  = 0.5f;    // 〃 目標 (strength から)
uint32_t  wing_last_ms = 0;       // 最終 Wingardium 受信時刻 (タイムアウト用)
int8_t    last_seq     = -1;      // 単発呪文の重複抑止 (Wingardium は除外)

// BLE タスク → loop へ渡す共有 (latest-wins)
static portMUX_TYPE rx_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool     rx_pending = false;
volatile uint8_t  rx_seq, rx_trig, rx_strength;
volatile uint16_t rx_target;

// ---- 描画ヘルパ ----
void fill(uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, r, g, b);
}
void render_base() {  // 一時演出の終了後に戻る状態
  if (base_on) fill(WARM_R, WARM_G, WARM_B);
  else         strip.clear();
  strip.show();
}

void start_fx(FX f, uint32_t dur_ms) { fx = f; fx_start_ms = millis(); fx_dur_ms = dur_ms; }

// ============================================================
// トリガ処理 (loop 側で実行)
// ============================================================
void handle_trigger(uint8_t seq, uint8_t trig, uint8_t strength, uint16_t target) {
  if (target != wand_beacon::TARGET_ALL && target != DEVICE_ID) return;

  // Wingardium(0x22) は連続制御 → dedup せず毎回反映
  if (trig == wand_beacon::TRIG_WINGARDIUM) {
    wing_target  = (float)strength / 255.0f;      // 0=下 .. 1=上
    wing_last_ms = millis();
    if (fx != FX::WINGARDIUM) { fx = FX::WINGARDIUM; wing_pos = wing_target; }
    return;
  }

  if ((int8_t)seq == last_seq) return;            // 単発呪文の重複抑止
  last_seq = (int8_t)seq;

  switch (trig) {
    case wand_beacon::TRIG_LUMOS:      base_on = true;  fx = FX::LUMOS; render_base(); break;
    case wand_beacon::TRIG_NOX:        base_on = false; fx = FX::OFF;   render_base(); break;
    case wand_beacon::TRIG_SHAKE:      start_fx(FX::FLASH,     250);  break;
    case wand_beacon::TRIG_INCENDIO:   start_fx(FX::INCENDIO,  3000); break;
    case wand_beacon::TRIG_AGUAMENTI:  start_fx(FX::AGUAMENTI, 5000); break;
    case wand_beacon::TRIG_EXPECTO_PATRONUM: start_fx(FX::PATRONUM, 6000); break;
    default: break;
  }
  Serial.printf("[RX] seq=%u trig=0x%02X str=%u tgt=%u\n", seq, trig, strength, (unsigned)target);
}

// ============================================================
// BLE スキャン (NimBLE, passive, 重複あり)
// ============================================================
class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* dev) override {
    if (!dev->haveManufacturerData()) return;
    std::string md = dev->getManufacturerData();
    if (md.size() < sizeof(wand_beacon::Payload)) return;
    wand_beacon::Payload p;
    memcpy(&p, md.data(), sizeof(p));
    if (p.company_id != wand_beacon::COMPANY_ID) return;
    portENTER_CRITICAL(&rx_mux);
    rx_seq = p.seq; rx_trig = p.trigger_id; rx_strength = p.strength; rx_target = p.target_id;
    rx_pending = true;
    portEXIT_CRITICAL(&rx_mux);
  }
};

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== WS2812B aerial receiver (XIAO ESP32-C3) ===");
  Serial.printf("pixels=%d data=GPIO%d brightness<=%d id=%d\n",
                NUM_PIXELS, LED_DATA_PIN, MAX_BRIGHTNESS, DEVICE_ID);

  strip.begin();
  strip.setBrightness(MAX_BRIGHTNESS);
  strip.clear(); strip.show();

  NimBLEDevice::init("");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new ScanCB(), /*wantDuplicates=*/true);
  scan->setActiveScan(false);     // passive: adv だけ受ければよい
  scan->setInterval(80);          // 0.625ms 単位
  scan->setWindow(80);            // window==interval = 連続スキャン (低レイテンシ)
  scan->setMaxResults(0);         // 蓄積せずコールバックのみ
  scan->start(0, nullptr, false); // 0 = 無期限
  Serial.println("[BLE] scanning (company_id=0xFFFF) ...");
}

// ============================================================
// 演出レンダリング (非ブロッキング、~50fps)
// ============================================================
void render() {
  uint32_t now = millis();
  uint32_t t   = now - fx_start_ms;

  // 一時演出の終了 → base へ
  if (fx_dur_ms && t >= fx_dur_ms &&
      (fx == FX::FLASH || fx == FX::INCENDIO || fx == FX::AGUAMENTI || fx == FX::PATRONUM)) {
    fx = FX::OFF; render_base(); return;
  }

  switch (fx) {
    case FX::OFF:   strip.clear(); strip.show(); break;
    case FX::LUMOS: fill(WARM_R, WARM_G, WARM_B); strip.show(); break;

    case FX::FLASH: {  // 一瞬の白フラッシュ (フェードアウト)
      float k = 1.0f - (float)t / fx_dur_ms;
      fill((uint8_t)(255*k), (uint8_t)(255*k), (uint8_t)(255*k)); strip.show();
      break;
    }
    case FX::INCENDIO: {  // 橙の炎ゆらぎ
      for (uint16_t i = 0; i < NUM_PIXELS; i++) {
        float f = 0.55f + 0.45f * (float)random(0, 100) / 100.0f;
        strip.setPixelColor(i, (uint8_t)(INC_R*f), (uint8_t)(INC_G*f), (uint8_t)(INC_B*f));
      }
      strip.show();
      break;
    }
    case FX::AGUAMENTI: {  // 青が流れる波
      float phase = (float)t / 350.0f;
      for (uint16_t i = 0; i < NUM_PIXELS; i++) {
        float w = 0.4f + 0.6f * (0.5f + 0.5f * sinf(phase - (float)i * 0.5f));
        strip.setPixelColor(i, (uint8_t)(AGU_R*w), (uint8_t)(AGU_G*w), (uint8_t)(AGU_B*w));
      }
      strip.show();
      break;
    }
    case FX::PATRONUM: {  // 銀青が中心から放射する波
      float radius = (float)t / fx_dur_ms * (NUM_PIXELS * 0.6f);
      float center = (NUM_PIXELS - 1) / 2.0f;
      for (uint16_t i = 0; i < NUM_PIXELS; i++) {
        float d = fabsf((float)i - center);
        float w = (d <= radius) ? (0.3f + 0.7f * (1.0f - d / (NUM_PIXELS * 0.6f))) : 0.0f;
        if (w < 0) w = 0;
        strip.setPixelColor(i, (uint8_t)(PAT_R*w), (uint8_t)(PAT_G*w), (uint8_t)(PAT_B*w));
      }
      strip.show();
      break;
    }
    case FX::WINGARDIUM: {  // ピッチ(strength)で光の塊を上下に浮遊
      if (now - wing_last_ms > 1500) { fx = FX::OFF; render_base(); break; }  // 連続途絶で解除
      wing_pos += 0.25f * (wing_target - wing_pos);          // ゆっくり追従
      float center = wing_pos * (NUM_PIXELS - 1);
      for (uint16_t i = 0; i < NUM_PIXELS; i++) {
        float d = fabsf((float)i - center);
        float w = (d < 3.0f) ? (1.0f - d / 3.0f) : 0.0f;     // 幅 ~3px の塊
        strip.setPixelColor(i, (uint8_t)(PAT_R*w), (uint8_t)(PAT_G*w), (uint8_t)(PAT_B*w));
      }
      strip.show();
      break;
    }
  }
}

void loop() {
  // BLE タスクからの受信要求を取り込む
  if (rx_pending) {
    uint8_t seq, trig, strn; uint16_t tgt;
    portENTER_CRITICAL(&rx_mux);
    seq = rx_seq; trig = rx_trig; strn = rx_strength; tgt = rx_target; rx_pending = false;
    portEXIT_CRITICAL(&rx_mux);
    handle_trigger(seq, trig, strn, tgt);
  }

  static uint32_t last_render = 0;
  uint32_t now = millis();
  if (now - last_render >= 20) {   // ~50fps
    last_render = now;
    render();
  }
}
