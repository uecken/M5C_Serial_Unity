// proto_wand_to_led / proto_mag-let-wand_to_candle / src/main.cpp
// ろうそく(受信側): フォトTRで「杖 ADMGLW の可視光」を検出 → wake → D0/D1 を HIGH で LED 点灯
//   → 30秒(今は)で自動消灯 → deep sleep(nRF=System OFF / ESP32=esp_deep_sleep) に戻り再武装。
//
// ★同一コードで 3 ボード対応 (XIAO):
//   - XIAO nRF52840      : env seeed_xiao_nrf52840 / _lowpower    (Adafruit nRF52 framework)
//   - XIAO ESP32-C3      : env seeed_xiao_esp32c3  / _lowpower    (arduino-esp32 framework)
//   - XIAO ESP32-C6      : env seeed_xiao_esp32c6  / _lowpower    (arduino-esp32 framework)
//
// ★配線 (D2 で wake / 外付け 470kΩ プルアップ):
//   3V3 ─[470kΩ]─ D2 ─[フォトTR C], [フォトTR E]─ GND   → 暗=High / 光でフォトTR導通 → D2=LOW
//     → 「LOW」で検出 & deep sleep から wake。 検出距離/外乱光は 470k 値で調整(小さく=鈍く)。
//   LED: D0 と D1 を 点灯(wake起動)時に HIGH で駆動(2ピンでソース能力UP)。起動時/sleep中は OFF。
//        各ピン ─[直列R ~100〜220Ω]─ LED ─ GND。 USE_FLICKER=1 で PWM ゆらぎ。
//   電源: 3V3 / BAT(LIR2032H)。
//
// 設計根拠(§5): フォトTR直列LEDは増幅無し+逆二乗則で ~5mm しか届かない → フォトTRは検出のみ、LEDは MCU 駆動。
//
// ⚠ ESP32-C3/C6 の deep sleep GPIO wake は対象ピンが限られる:
//     C3 = GPIO0..5 (XIAO の D2=GPIO4 → OK) / C6 = LP_GPIO0..7 (XIAO の D2=GPIO2 → OK)。
//   別ピンに変える時は wake 可否を要確認。まず SIMPLE(LOWPOWER_SYSTEMOFF=0) で検出を確認するのが安全。
//   deep sleep 待機電流は nRF52840(~1.5µA) < C3(~5µA) < C6(~7µA) ※チップ値。基板実装で増える。

#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
  #include "esp_sleep.h"
  #define PLAT_ESP32 1
#else
  #include <Adafruit_TinyUSB.h>   // Serial(USB CDC)。led_xiao と同様 begin 順序に注意
  #include <nrf.h>
  #include <nrf_gpio.h>            // nrf_gpio_cfg_sense_input (System OFF からの GPIO wake)
  #define PLAT_NRF52 1
#endif

// ---- ビルドモード ----
#ifndef LOWPOWER_SYSTEMOFF
#define LOWPOWER_SYSTEMOFF 0   // 0=簡易ポーリング(USB観測) / 1=deep sleep + D2(LOW) wake
#endif
#ifndef USE_FLICKER
#define USE_FLICKER 0          // 0=点灯時 D0/D1 solid HIGH / 1=PWM ゆらぎ
#endif

// ---- ピン (XIAO シルク。各ボードの variant が正しい GPIO にマップ) ----
static const int kSensePin = D2;   // 光検出/wake (外付け470kプルアップ, 光=LOW)
static const int kLedPinA  = D0;   // LED 駆動 (点灯時 HIGH)
static const int kLedPinB  = D1;   // LED 駆動 (点灯時 HIGH)
static const int kReedPin  = D3;   // 任意リセット (INPUT_PULLUP)
#if PLAT_NRF52
static const uint32_t kSenseNrfPin = 28;  // nRF: D2=P0.28 の絶対 GPIO 番号 (sense 設定用)
#endif

static const int      LIGHT_ACTIVE = LOW;
static const int      CONFIRM_MS   = 100;
static const uint32_t AUTO_OFF_MS  = 30UL * 1000UL;   // 今は30秒で消灯→sleep (本番は 3*60*1000)
static const int      FLICKER_MIN  = 60;
static const int      FLICKER_MAX  = 255;

inline bool lightNow() { return digitalRead(kSensePin) == LIGHT_ACTIVE; }   // 光検出 = D2 LOW
void ledOn()  { digitalWrite(kLedPinA, HIGH); digitalWrite(kLedPinB, HIGH); }
void ledOff() { digitalWrite(kLedPinA, LOW);  digitalWrite(kLedPinB, LOW); }

#if USE_FLICKER
// ろうそく炎ゆらぎ (D0/D1 両方 PWM)
void flickerStep() {
  static int level = FLICKER_MIN, target = (FLICKER_MIN + FLICKER_MAX) / 2;
  static uint32_t tNext = 0;
  uint32_t now = millis();
  if (now >= tNext) { target = random(FLICKER_MIN, FLICKER_MAX); tNext = now + random(40, 120); }
  level += (target - level) / 4;
  level += random(-8, 9);
  level = constrain(level, FLICKER_MIN, FLICKER_MAX);
  analogWrite(kLedPinA, level);
  analogWrite(kLedPinB, level);
}
#endif

// D2 LOW が CONFIRM_MS 継続したら「杖の光」と確定
bool detectWandLight() {
  uint32_t over = 0, t0 = millis();
  while (millis() - t0 < CONFIRM_MS + 50) {
    if (lightNow()) { if (!over) over = millis(); else if (millis() - over >= CONFIRM_MS) return true; }
    else over = 0;
    delay(5);
  }
  return false;
}

// 点灯ショー: D0/D1 HIGH → 30秒(今は) or リードSW → 消灯
void runCandleShow() {
  Serial.println("[candle] LIT (D0/D1=HIGH)");
  ledOn();
  uint32_t t0 = millis();
  while (millis() - t0 < AUTO_OFF_MS) {
#if USE_FLICKER
    flickerStep();
#endif
    if (digitalRead(kReedPin) == LOW) { Serial.println("[candle] reed reset"); break; }
    delay(20);
  }
  ledOff();
  Serial.println("[candle] OFF");
}

void initPins() {
  analogReadResolution(12);
  pinMode(kLedPinA, OUTPUT); pinMode(kLedPinB, OUTPUT); ledOff();   // 起動時は消灯
  pinMode(kReedPin, INPUT_PULLUP);
  pinMode(kSensePin, INPUT);     // 外付け470kプルアップ前提 → 内部プルなし
}

// ====================== プラットフォーム層 (wake/sleep) ======================
void platSerialBegin() {
#if PLAT_NRF52
  TinyUSBDevice.begin(0);        // Serial より先に (led_xiao と同様)
#endif
  Serial.begin(115200);
  delay(50);
}

// deep sleep からの wake が「光(D2 LOW)」によるものか
bool platWokeByLight() {
#if PLAT_ESP32
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO;
#else
  uint32_t rr = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = rr;                 // 1書き込みでクリア
  return rr & POWER_RESETREAS_OFF_Msk;       // System OFF からの GPIO DETECT wake
#endif
}

// 光が消える(D2 High)のを待って D2(LOW) を wake 源に武装し deep sleep へ (戻らない=wakeはリセット)
void platArmLowWakeAndSleep() {
  ledOff();                                  // ★sleep中は D0/D1 OFF
  uint32_t t0 = millis();
  while (lightNow() && millis() - t0 < 5000) delay(20);   // LOW のまま武装すると即wakeループ防止
#if PLAT_ESP32
  Serial.println("[candle] arm D2(LOW) -> esp_deep_sleep"); Serial.flush();
  // C3/C6(RISC-V): GPIO の LOW レベルで deep sleep から wake。XIAO は kSensePin が GPIO 番号と一致。
  esp_deep_sleep_enable_gpio_wakeup(1ULL << (uint32_t)kSensePin, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();                    // 戻らない
#else
  Serial.println("[candle] arm D2(LOW) -> System OFF"); Serial.flush();
  // 外付け470kプルアップ前提で内部プルなし。LOW レベルで GPIO DETECT → wake。
  nrf_gpio_cfg_sense_input(kSenseNrfPin, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
  NRF_POWER->SYSTEMOFF = 1;
  while (1) { __WFE(); }                      // 戻らない (リセットで setup() から再開)
#endif
}

// ===========================================================================
#if LOWPOWER_SYSTEMOFF
// --- 低消費・本番: D2(LOW) で deep sleep から wake ---
void setup() {
  platSerialBegin();
  initPins();
  bool wokeByLight = platWokeByLight();
  Serial.printf("[candle] boot wokeByLight=%d D2=%d\n", wokeByLight, digitalRead(kSensePin));

  // 起動時 D0/D1 OFF (initPins済)。光で起動 or 光があれば点灯、無ければ無点灯のまま。
  if (wokeByLight || (lightNow() && detectWandLight())) {
    runCandleShow();              // D0/D1 HIGH で 30秒(今は)点灯 → 消灯
  } else {
    delay(AUTO_OFF_MS);           // 非点灯でも今は30秒起きてから寝る (USB再書込みの窓)
  }
  platArmLowWakeAndSleep();       // D0/D1 OFF → D2(LOW) 武装 → deep sleep (戻らない)
}
void loop() {}

#else
// --- 簡易・デモ: D2 をポーリング。USB給電で観測しながらしきい値を詰める ---
void setup() {
  platSerialBegin();
  initPins();
  Serial.println("[candle] SIMPLE_POLL (光=D2 LOW / 点灯=D0,D1 HIGH / 外付け470kプルアップ)");
}
void loop() {
  static uint32_t tLog = 0;
  if (millis() - tLog > 1000) {  // キャリブレーション補助
    Serial.printf("[candle] D2=%d light=%d adc=%d\n",
                  digitalRead(kSensePin), lightNow(), analogRead(kSensePin));
    tLog = millis();
  }
  if (detectWandLight()) runCandleShow();
  delay(10);
}
#endif
