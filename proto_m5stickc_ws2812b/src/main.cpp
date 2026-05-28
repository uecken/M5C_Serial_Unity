// proto_m5stickc_ws2812b / src/main.cpp
// M5StickC (ESP32) で WS2812B を駆動する「魔法受信 LED」。
//
//   パラメータ源 (command sources) — すべて同じ cfg:: / fx:: API に流し込む:
//     1. Serial   : on/off, n=, b=, p=, c=r,g,b, 色プリセット, spell <x>, show/save/load/default, id=, ble=
//     2. BLE 受信 : proto_wand_to_led の 杖 が出す Advertising ビーコン(魔法)を SCAN し、
//                   trigger_id (spell) ごとに色/エフェクトを切替 (shared/beacon_protocol.h を共用)
//     3. WiFi     : 将来用フック (#ifdef ENABLE_WIFI)。今は stub (net 名前空間)。
//
//   ★ 電流: WS2812B 1個=黄フル ~40mA。60個フル ≈ 2.4A は M5StickC の 5V では賄えない。
//     FastLED.setMaxPowerInVoltsAndMilliamps() で上限をかけ超過時は輝度を自動低減。
//     多数個フル輝度には外部5V電源 + GND共通が必須 (README/docs/wiring.md)。

#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "../../proto_wand_to_led/shared/beacon_protocol.h"  // 杖と共用のワイヤフォーマット

// ============================================================
// ハード設定 (M5StickC)
// ============================================================
#define DATA_PIN        26   // WS2812B DIN ← 底面ヘッダ G26 (330Ω 任意。Grove G32/G33 も可)
#define NUM_LEDS_MAX    128  // 配列確保上限 (実点灯数は cfg::num_leds で可変)
#define BTN_A_PIN       37   // 前面 M5 ボタン (active-low, input-only)
#define BTN_B_PIN       39   // 側面ボタン   (active-low, input-only)
#define BUILTIN_LED_PIN 10   // 内蔵赤 LED   (active-low)

CRGB leds[NUM_LEDS_MAX];

// spell 用の色定義
static const CRGB COL_WARMWHITE = CRGB(255, 180, 80);   // LUMOS
static const CRGB COL_ORANGE    = CRGB(255, 80, 0);     // INCENDIO
static const CRGB COL_BLUE      = CRGB(0, 80, 255);     // AGUAMENTI
static const CRGB COL_PATRONUM  = CRGB(180, 220, 255);  // EXPECTO PATRONUM (淡い水色白)

// ============================================================
// パラメータ + NVS 永続化 (Preferences)  ※杖の gcfg パターンを踏襲
// ============================================================
namespace cfg {
constexpr const char* NS = "ledcfg";

// --- デフォルト ---
constexpr int      DEF_NUM_LEDS   = 60;
constexpr uint8_t  DEF_BRIGHTNESS = 255;
constexpr int      DEF_MAX_MA     = 450;     // 単一USBの安全側。外部5Vなら p= で上げる
constexpr uint16_t DEF_DEVICE_ID  = 1;       // BLE target_id フィルタ用 (1-65534)

// --- 実行時変数 ---
int      num_leds   = DEF_NUM_LEDS;
uint8_t  brightness = DEF_BRIGHTNESS;
CRGB     color      = CRGB::Yellow;          // 手動 SOLID の色 (spell の色とは別)
int      max_ma     = DEF_MAX_MA;
uint16_t device_id  = DEF_DEVICE_ID;
bool     ble_enabled = true;

// 電流上限プリセット [mA] (Button B で巡回)。2500 は外部5V電源時のみ。
const int MA_PRESETS[] = {150, 300, 450, 1200, 2500};
int       ma_index     = 2;                  // 既定 450mA

Preferences prefs;

void set_defaults() {
  num_leds = DEF_NUM_LEDS; brightness = DEF_BRIGHTNESS; color = CRGB::Yellow;
  max_ma = DEF_MAX_MA; device_id = DEF_DEVICE_ID; ble_enabled = true; ma_index = 2;
}

void load() {
  prefs.begin(NS, true);                     // read-only
  num_leds   = prefs.getInt  ("n",   DEF_NUM_LEDS);
  brightness = prefs.getUChar("bri", DEF_BRIGHTNESS);
  color.r    = prefs.getUChar("cr",  255);
  color.g    = prefs.getUChar("cg",  255);
  color.b    = prefs.getUChar("cb",  0);
  max_ma     = prefs.getInt  ("ma",  DEF_MAX_MA);
  device_id  = prefs.getUShort("id", DEF_DEVICE_ID);
  ble_enabled= prefs.getUChar("ble", 1) != 0;
  prefs.end();
  num_leds = constrain(num_leds, 1, NUM_LEDS_MAX);
}

void save() {
  prefs.begin(NS, false);                    // read-write
  prefs.putInt  ("n",   num_leds);
  prefs.putUChar("bri", brightness);
  prefs.putUChar("cr",  color.r);
  prefs.putUChar("cg",  color.g);
  prefs.putUChar("cb",  color.b);
  prefs.putInt  ("ma",  max_ma);
  prefs.putUShort("id", device_id);
  prefs.putUChar("ble", ble_enabled ? 1 : 0);
  prefs.end();
}

void print() {
  Serial.printf("[CFG] n=%d bri=%u color=#%02X%02X%02X cap=%dmA id=%u ble=%d (DATA=GPIO%d)\n",
                num_leds, brightness, color.r, color.g, color.b, max_ma,
                (unsigned)device_id, ble_enabled, DATA_PIN);
}
}  // namespace cfg

// 黄フル輝度の理論電流概算 (実機寄り 40mA/個)。情報表示用。
static int estimate_full_ma() {
  return (int)((long)cfg::num_leds * 40L * cfg::brightness / 255L);
}

// ============================================================
// エフェクトエンジン (非ブロッキング)
//   base: 手動の持続状態 (OFF / SOLID)。LUMOS=持続 warm white、NOX=OFF。
//   transient: spell の時限/連続エフェクト。終了で base に戻る。
//   描画は commit() の FastLED.show() に一本化し、電流上限をここで適用。
// ============================================================
namespace fx {
enum Trans { NONE, FLASH, INCENDIO, AGUAMENTI, PATRONUM, WINGARDIUM };

bool     base_on    = false;
CRGB     base_color = CRGB::Yellow;
Trans    trans      = NONE;
uint32_t trans_end  = 0;
uint32_t trans_total= 0;
uint32_t trans_tick = 0;
int      wave_pos   = 0;
constexpr int WAVE_WIDTH = 6;

void fill_active(const CRGB& c) {
  for (int i = 0; i < NUM_LEDS_MAX; i++) leds[i] = (i < cfg::num_leds) ? c : CRGB::Black;
}

void commit() {
  FastLED.setBrightness(cfg::brightness);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, cfg::max_ma);   // ★電流を物理キャップ
  FastLED.show();
  bool active = base_on || trans != NONE;
  digitalWrite(BUILTIN_LED_PIN, active ? LOW : HIGH);        // 内蔵LEDで状態表示
}

void draw_base() { fill_active(base_on ? base_color : CRGB::Black); commit(); }

// --- 手動 (Serial/Button) ---
void set_base(bool onv) { base_on = onv; base_color = cfg::color; trans = NONE; draw_base(); }
void refresh_color()    { if (trans == NONE && base_on) { base_color = cfg::color; draw_base(); } }

// --- spell ---
void lumos() { base_on = true;  base_color = COL_WARMWHITE; trans = NONE; draw_base(); }
void nox()   { base_on = false;                            trans = NONE; draw_base(); }
void flash(const CRGB& c, uint32_t dur) { trans = FLASH; trans_end = millis() + dur; fill_active(c); commit(); }
void incendio(uint32_t dur)  { trans = INCENDIO;  trans_end = millis() + dur; trans_tick = 0; }
void aguamenti(uint32_t dur) { trans = AGUAMENTI; trans_total = dur; trans_end = millis() + dur; trans_tick = 0; }
void patronum(uint32_t dur)  { trans = PATRONUM;  trans_end = millis() + dur; wave_pos = 0; trans_tick = 0; }
void wingardium(uint8_t strength) {
  trans = WINGARDIUM; trans_end = millis() + 1000;          // ~1s 無音で終了
  CRGB c = COL_PATRONUM; c.nscale8(strength);
  fill_active(c); commit();
}

void draw_patronum() {
  for (int i = 0; i < NUM_LEDS_MAX; i++) {
    if (i >= cfg::num_leds) { leds[i] = CRGB::Black; continue; }
    int d = abs(i - wave_pos);
    if (d <= WAVE_WIDTH) {
      uint8_t v = 255 - (uint8_t)(255 * d / (WAVE_WIDTH + 1));
      CRGB c = COL_PATRONUM; c.nscale8(v); leds[i] = c;
    } else leds[i] = CRGB::Black;
  }
}

void poll() {
  if (trans == NONE) return;
  uint32_t now = millis();
  if ((int32_t)(now - trans_end) >= 0) { trans = NONE; draw_base(); return; }  // 時限終了→base復帰
  switch (trans) {
    case INCENDIO:
      if (now >= trans_tick) { trans_tick = now + 50; CRGB c = COL_ORANGE; c.nscale8(random8(140, 255)); fill_active(c); commit(); }
      break;
    case AGUAMENTI:
      if (now >= trans_tick) { trans_tick = now + 33; uint32_t rem = trans_end - now;
        uint8_t lvl = (uint8_t)(255UL * rem / trans_total); CRGB c = COL_BLUE; c.nscale8(lvl); fill_active(c); commit(); }
      break;
    case PATRONUM:
      if (now >= trans_tick) { trans_tick = now + 40; draw_patronum(); commit();
        if (++wave_pos > cfg::num_leds + WAVE_WIDTH) wave_pos = 0; }
      break;
    default: break;   // FLASH / WINGARDIUM は静止 (発火時に描画済み)
  }
}
}  // namespace fx

// ============================================================
// spell ディスパッチ (BLE受信 / Serial test / 将来WiFi が共通で呼ぶ)
//   重複抑止 (seq) と宛先フィルタ (target_id) は led_xiao_nrf52840 を踏襲。
// ============================================================
namespace dispatch {
uint8_t last_seq   = 0;
bool    have_first = false;

void spell(uint8_t seq, uint8_t trig, uint8_t strength, uint16_t target, int rssi) {
  // 宛先フィルタ: 全機宛て or 自機宛て のみ
  if (target != wand_beacon::TARGET_ALL && target != cfg::device_id) return;
  // 重複抑止 (連続制御の WINGARDIUM は毎パケット反映)
  if (trig != wand_beacon::TRIG_WINGARDIUM && have_first && seq == last_seq) return;
  last_seq = seq; have_first = true;

  Serial.printf("[SPELL] seq=%u trig=0x%02X strength=%u target=%u rssi=%d => ",
                seq, trig, strength, (unsigned)target, rssi);
  switch (trig) {
    case wand_beacon::TRIG_LUMOS:            Serial.println("LUMOS (warm white, until NOX)"); fx::lumos(); break;
    case wand_beacon::TRIG_NOX:              Serial.println("NOX (off)");                      fx::nox();   break;
    case wand_beacon::TRIG_INCENDIO:         Serial.println("INCENDIO (orange flicker 3s)");   fx::incendio(wand_beacon::LED_DURATION_INCENDIO_MS);  break;
    case wand_beacon::TRIG_AGUAMENTI:        Serial.println("AGUAMENTI (blue fade 5s)");       fx::aguamenti(wand_beacon::LED_DURATION_AGUAMENTI_MS); break;
    case wand_beacon::TRIG_EXPECTO_PATRONUM: Serial.println("PATRONUM (wave 3s)");             fx::patronum(wand_beacon::LED_DURATION_PATRONUM_MS);  break;
    case wand_beacon::TRIG_SHAKE:            Serial.println("SHAKE (brief flash 250ms)");      fx::flash(CRGB::White, wand_beacon::LED_DURATION_SHAKE_MS); break;
    case wand_beacon::TRIG_WINGARDIUM:       Serial.println("WINGARDIUM (strength->brightness)"); fx::wingardium(strength); break;
    default:                                 Serial.println("unknown trigger, ignored");       break;
  }
}
}  // namespace dispatch

// ============================================================
// BLE 受信 (NimBLE scan)  ※ NimBLE-Arduino 1.4.3 API
//   コールバックは BLE host タスク文脈で走るため、ここでは FastLED を触らず
//   イベントを FreeRTOS キューに積むだけ。loop() で取り出して dispatch する。
// ============================================================
namespace ble_rx {
struct Evt { uint8_t seq, trig, strength; uint16_t target; int rssi; };
QueueHandle_t q = nullptr;

class Cb : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* dev) override {
    if (!dev->haveManufacturerData()) return;
    std::string md = dev->getManufacturerData();
    if (md.size() < sizeof(wand_beacon::Payload)) return;
    wand_beacon::Payload p;
    memcpy(&p, md.data(), sizeof(p));                 // ARM/未アライン安全コピー
    if (p.company_id != wand_beacon::COMPANY_ID) return;
    Evt e{p.seq, p.trigger_id, p.strength, p.target_id, dev->getRSSI()};
    if (q) xQueueSend(q, &e, 0);
  }
};
Cb cb;

void begin() {
  q = xQueueCreate(8, sizeof(Evt));
  NimBLEDevice::init("LED-RX");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&cb, /*wantDuplicates=*/true);  // 杖は同一パケットを連送するため重複も受ける
  scan->setActiveScan(false);                         // passive: データは adv に載っている
  scan->setInterval(100);                             // ms (1.4.3 は ms 単位。Bluefruit の 0.625ms 単位と違う)
  scan->setWindow(100);                               // window==interval → ~100% duty
  if (cfg::ble_enabled) scan->start(0, nullptr, false);
}

void set_enabled(bool en) {
  cfg::ble_enabled = en;
  NimBLEScan* scan = NimBLEDevice::getScan();
  if (en) scan->start(0, nullptr, false);
  else    scan->stop();
}

void poll() {                                         // loop() から呼ぶ: キューを排出
  if (!q) return;
  Evt e;
  while (xQueueReceive(q, &e, 0) == pdTRUE)
    dispatch::spell(e.seq, e.trig, e.strength, e.target, e.rssi);
}
}  // namespace ble_rx

// ============================================================
// WiFi (将来用フック)  ※今は stub。env:m5stick-c-wifi で -D ENABLE_WIFI。
//   実装時は HTTP/MQTT 等を受けて cfg::set* と dispatch::spell() を呼ぶだけ
//   (BLE/Serial と同じ command-source 抽象に合流させる)。
// ============================================================
#ifdef ENABLE_WIFI
#include <WiFi.h>
namespace net {
void begin() {
  Serial.println("[NET] WiFi hook present (ENABLE_WIFI) but not yet implemented.");
  Serial.println("[NET] TODO: STA/AP + HTTP/MQTT -> cfg setters + dispatch::spell()");
}
void poll() {}
}  // namespace net
#else
namespace net { inline void begin() {} inline void poll() {} }
#endif

// ============================================================
// ボタン (簡易デバウンス: 立下りエッジ検出)
// ============================================================
struct Button {
  int  pin;
  bool down = false;
  uint32_t t_change = 0;
  bool pressed() {
    bool d = (digitalRead(pin) == LOW);               // active-low
    uint32_t now = millis();
    if (d != down && (now - t_change) > 30) {
      down = d; t_change = now;
      if (d) return true;
    }
    return false;
  }
};
Button btnA{BTN_A_PIN}, btnB{BTN_B_PIN};

// ============================================================
// Serial コマンド
// ============================================================
namespace cmd {
void print_help() {
  Serial.println();
  Serial.println("=== M5StickC -> WS2812B (spell receiver) ===");
  cfg::print();
  Serial.println("LED  : on/off | n=<個数> | b=<0-255 輝度> | p=<mA上限> | c=r,g,b | y/w/r/g/b");
  Serial.println("CFG  : show | save | load | default | id=<1-65534> | ble=0|1");
  Serial.println("SPELL: spell l(lumos) | spell nox | spell i(incendio) | spell a(aguamenti)");
  Serial.println("       spell e(patronum) | spell t(shake) | spell w <0-255>(wingardium)");
  Serial.println("BLE  : 杖(proto_wand_to_led)の魔法ビーコンを scan 受信 -> spell ごとに点灯");
}

void do_spell(const char* a) {
  static uint8_t seq = 0; seq++;
  while (*a == ' ') a++;
  if      (!strncmp(a, "nox", 3)) dispatch::spell(seq, wand_beacon::TRIG_NOX, 0, wand_beacon::TARGET_ALL, 0);
  else if (a[0] == 'l')           dispatch::spell(seq, wand_beacon::TRIG_LUMOS, 0, wand_beacon::TARGET_ALL, 0);
  else if (a[0] == 'i')           dispatch::spell(seq, wand_beacon::TRIG_INCENDIO, 0, wand_beacon::TARGET_ALL, 0);
  else if (a[0] == 'a')           dispatch::spell(seq, wand_beacon::TRIG_AGUAMENTI, 0, wand_beacon::TARGET_ALL, 0);
  else if (a[0] == 'e')           dispatch::spell(seq, wand_beacon::TRIG_EXPECTO_PATRONUM, 0, wand_beacon::TARGET_ALL, 0);
  else if (a[0] == 't')           dispatch::spell(seq, wand_beacon::TRIG_SHAKE, 0, wand_beacon::TARGET_ALL, 0);
  else if (a[0] == 'w')           { int v = atoi(a + 1); dispatch::spell(seq, wand_beacon::TRIG_WINGARDIUM, (uint8_t)constrain(v,0,255), wand_beacon::TARGET_ALL, 0); }
  else Serial.println("[?] spell l|nox|i|a|e|t|w <0-255>");
}

void handle_line(char* s) {
  while (*s == ' ') s++;
  int n = strlen(s);
  while (n > 0 && (s[n-1] == '\r' || s[n-1] == '\n' || s[n-1] == ' ')) s[--n] = 0;
  if (n == 0) return;

  if      (!strcmp(s, "on"))      fx::set_base(true);
  else if (!strcmp(s, "off"))     fx::set_base(false);
  else if (!strcmp(s, "help") || !strcmp(s, "?")) { print_help(); return; }
  else if (!strcmp(s, "show"))    { cfg::print(); return; }
  else if (!strcmp(s, "save"))    { cfg::save(); Serial.println("[CFG] saved"); return; }
  else if (!strcmp(s, "load"))    { cfg::load(); fx::draw_base(); cfg::print(); return; }
  else if (!strcmp(s, "default")) { cfg::set_defaults(); Serial.println("[CFG] defaults (save で永続化)"); cfg::print(); fx::draw_base(); return; }
  else if (!strncmp(s, "spell", 5)) { do_spell(s + 5); return; }
  else if (!strcmp(s, "y"))       { cfg::color = CRGB::Yellow; fx::refresh_color(); }
  else if (!strcmp(s, "w"))       { cfg::color = CRGB::White;  fx::refresh_color(); }
  else if (!strcmp(s, "r"))       { cfg::color = CRGB::Red;    fx::refresh_color(); }
  else if (!strcmp(s, "g"))       { cfg::color = CRGB::Green;  fx::refresh_color(); }
  else if (!strcmp(s, "b"))       { cfg::color = CRGB::Blue;   fx::refresh_color(); }
  else if (!strncmp(s, "n=", 2))  { cfg::num_leds = constrain(atoi(s+2), 1, NUM_LEDS_MAX); fx::draw_base(); }
  else if (!strncmp(s, "b=", 2))  { cfg::brightness = (uint8_t)constrain(atoi(s+2), 0, 255); fx::commit(); }
  else if (!strncmp(s, "p=", 2))  { cfg::max_ma = max(10, atoi(s+2)); fx::commit(); }
  else if (!strncmp(s, "id=", 3)) { int v = atoi(s+3); if (v >= 1 && v <= 65534) cfg::device_id = (uint16_t)v; cfg::print(); return; }
  else if (!strncmp(s, "ble=", 4)){ ble_rx::set_enabled(atoi(s+4) != 0); Serial.printf("[BLE] scan %s\n", cfg::ble_enabled ? "ON" : "OFF"); return; }
  else if (!strncmp(s, "c=", 2))  {
    int r=0,gg=0,bb=0;
    if (sscanf(s+2, "%d,%d,%d", &r, &gg, &bb) == 3) { cfg::color = CRGB((uint8_t)r,(uint8_t)gg,(uint8_t)bb); fx::refresh_color(); }
  } else { Serial.println("[?] unknown. type 'help'"); return; }

  // 状態を一行表示 (電流の見積りも)
  uint8_t eff = calculate_max_brightness_for_power_vmA(leds, NUM_LEDS_MAX, cfg::brightness, 5, cfg::max_ma);
  Serial.printf("[LED] base=%s n=%d bri=%u cap=%dmA est=%dmA(@full) eff_bri=%u%s\n",
                fx::base_on ? "ON" : "off", cfg::num_leds, cfg::brightness, cfg::max_ma,
                estimate_full_ma(), eff, (eff < cfg::brightness) ? " (capped)" : "");
}

void handle_serial() {
  static char buf[64];
  static int  len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (len) { buf[len]=0; handle_line(buf); len=0; } }
    else if (len < (int)sizeof(buf)-1) buf[len++] = c;
  }
}
}  // namespace cmd

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BUILTIN_LED_PIN, OUTPUT);
  digitalWrite(BUILTIN_LED_PIN, HIGH);                // OFF
  pinMode(BTN_A_PIN, INPUT);                          // 外部プルアップ
  pinMode(BTN_B_PIN, INPUT);

  cfg::load();                                        // NVS からパラメータ
  // ma_index を保存済み max_ma に合わせる (Button B 巡回の起点)
  for (int i = 0; i < (int)(sizeof(cfg::MA_PRESETS)/sizeof(cfg::MA_PRESETS[0])); i++)
    if (cfg::MA_PRESETS[i] == cfg::max_ma) cfg::ma_index = i;

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS_MAX);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, cfg::max_ma);
  FastLED.clear(true);                                // 起動時は消灯

  ble_rx::begin();                                    // BLE scan 受信開始
  net::begin();                                       // WiFi (stub or 将来)

  cmd::print_help();
  fx::draw_base();
}

void loop() {
  if (btnA.pressed()) fx::set_base(!fx::base_on);     // A: 点灯/消灯トグル
  if (btnB.pressed()) {                               // B: 電流上限プリセット巡回
    cfg::ma_index = (cfg::ma_index + 1) % (int)(sizeof(cfg::MA_PRESETS)/sizeof(cfg::MA_PRESETS[0]));
    cfg::max_ma = cfg::MA_PRESETS[cfg::ma_index];
    Serial.printf("[CAP] max current -> %dmA\n", cfg::max_ma);
    fx::commit();
  }
  ble_rx::poll();                                     // 受信キュー → spell dispatch
  fx::poll();                                         // エフェクトのアニメ進行
  net::poll();                                        // WiFi (stub)
  cmd::handle_serial();
  delay(5);
}
