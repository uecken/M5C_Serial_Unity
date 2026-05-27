// proto_wand_to_led / led_xiao_nrf52840 / src/main.cpp
// LED (受信側): XIAO nRF52840 + Bluefruit + 内蔵 LED + 外部 LED (D0~D10)
// Phase 0: 連続スキャンで杖の adv 受信、LED 点灯
//
// 複数の受信機を運用する想定。各機ごとに下記「機体設定」を変えてビルドする。
//
// 重要な実装メモ (Phase 0 デバッグで判明):
// - TinyUSBDevice.begin(0) を Serial.begin() より先に明示呼び出しが必須
// - ARM Cortex-M4 unaligned access 回避のため payload は memcpy でコピー
// - PowerShell SerialPort 経由で読む場合 DtrEnable=$true 必須

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <nrf_gpio.h>
#include "../../shared/beacon_protocol.h"

using namespace Adafruit_LittleFS_Namespace;

// ============================================================
// ピン記法ヘルパ — 全て「絶対 nRF GPIO 番号 (port*32 + pin)」に統一
//   外部 LED は nrf_gpio_* で直接駆動 (Arduino digitalWrite のピンマップを介さない)
//   - XIAO シルク D0~D10  → XIAO_D_TO_NRF[] で物理 nRF ピンに変換
//   - 生 GPIO P<port>.<pin> → G(port, pin)
//     例: P1.11 → G(1, 11) = 43  /  P0.02 → G(0, 2) = 2
// ============================================================
#define G(port, pin) ((port) * 32 + (pin))

// XIAO nRF52840 の D0~D10 → 物理 nRF GPIO 番号 (Seeed wiki 準拠)
//   D0=P0.02 D1=P0.03 D2=P0.28 D3=P0.29 D4=P0.04 D5=P0.05
//   D6=P1.11 D7=P1.12 D8=P1.13 D9=P1.14 D10=P1.15
static const int XIAO_D_TO_NRF[] = {
  G(0,2), G(0,3), G(0,28), G(0,29), G(0,4), G(0,5),
  G(1,11), G(1,12), G(1,13), G(1,14), G(1,15)
};

// ============================================================
// 設定 (InternalFS に保存。USB シリアルで書き換え可能)
//   全機に同じ FW を焼いて、各機を Serial コマンドで個別設定する。
//   コマンド: show / id=<n> / pins=D0,D2,G1.11 / save / default
// ============================================================
namespace cfg {
constexpr uint32_t MAGIC      = 0x57414E44;  // "WAND"
constexpr int      MAX_PINS   = 12;
const char* const  CFG_PATH   = "/wandcfg.bin";

struct Config {
  uint32_t magic;
  uint16_t device_id;          // 1-65534
  uint8_t  pin_count;
  int16_t  pins[MAX_PINS];     // 解決済み「絶対 nRF GPIO 番号」
};

Config current;

void set_defaults() {
  current.magic     = MAGIC;
  current.device_id = 1;
  current.pin_count = 2;
  current.pins[0]   = XIAO_D_TO_NRF[0];  // D0 = P0.02
  current.pins[1]   = XIAO_D_TO_NRF[1];  // D1 = P0.03
}

bool load() {
  InternalFS.begin();
  File f(InternalFS);
  if (!f.open(CFG_PATH, FILE_O_READ)) return false;
  Config tmp;
  int n = f.read((uint8_t*)&tmp, sizeof(tmp));
  f.close();
  if (n != (int)sizeof(tmp) || tmp.magic != MAGIC) return false;
  current = tmp;
  return true;
}

bool save() {
  InternalFS.begin();
  InternalFS.remove(CFG_PATH);
  File f(InternalFS);
  if (!f.open(CFG_PATH, FILE_O_WRITE)) return false;
  current.magic = MAGIC;
  f.write((const uint8_t*)&current, sizeof(current));
  f.close();
  return true;
}

void print() {
  Serial.printf("[CFG] device_id=%u, pin_count=%u, pins=[",
                (unsigned)current.device_id, current.pin_count);
  for (int i = 0; i < current.pin_count; i++) {
    Serial.printf("%d%s", current.pins[i], (i + 1 < current.pin_count) ? "," : "");
  }
  Serial.println("]");
}

// "D3" / "G1.11" を「絶対 nRF GPIO 番号」に解決。失敗時 -1
//   D0~D10 → XIAO_D_TO_NRF[] で物理ピンに変換
//   G<port>.<pin> → port*32+pin
int parse_pin_token(const char* tok) {
  while (*tok == ' ') tok++;
  if ((tok[0] == 'D' || tok[0] == 'd')) {
    int n = atoi(tok + 1);
    if (n >= 0 && n <= 10) return XIAO_D_TO_NRF[n];
    return -1;
  }
  if ((tok[0] == 'G' || tok[0] == 'g')) {
    // G<port>.<pin> 形式
    const char* dot = strchr(tok, '.');
    if (!dot) return -1;
    int port = atoi(tok + 1);
    int pin  = atoi(dot + 1);
    if (port < 0 || port > 1 || pin < 0 || pin > 31) return -1;
    return port * 32 + pin;
  }
  return -1;  // D/G 以外は受け付けない (生番号は誤用しやすいので不可)
}

// "pins=D0,D2,G1.11" の右辺をパースして current.pins に設定
void set_pins_from_csv(char* csv) {
  int count = 0;
  char* p = strtok(csv, ",");
  while (p && count < MAX_PINS) {
    int pin = parse_pin_token(p);
    if (pin >= 0) current.pins[count++] = (int16_t)pin;
    else Serial.printf("[CFG] skip invalid pin token: %s\n", p);
    p = strtok(nullptr, ",");
  }
  current.pin_count = count;
  Serial.printf("[CFG] pins set: %d 本\n", count);
}
}  // namespace cfg

namespace led {
// 内蔵 赤 LED  = トリガ受信時の点灯表示 (5 秒など)、active-low
// 内蔵 青 LED  = 起動後ハートビート (5 秒に 1 回 100ms 点滅)、active-low
// 外部 LED群   = EXT_LED_PINS[]、active-HIGH。内蔵赤と連動
constexpr uint32_t BLUE_HEARTBEAT_PERIOD_MS = 5000;
constexpr uint32_t BLUE_HEARTBEAT_FLASH_MS  = 100;

uint32_t red_off_at_ms  = 0;
bool     red_is_on      = false;
uint32_t blue_next_ms   = 0;
uint32_t blue_off_at_ms = 0;
bool     blue_is_on     = false;
bool     lumos_mode     = false;  // LUMOS で true (全 LED 持続)、NOX で false
// --- Expecto Patronum: 光の波動が外へ広がるアニメ (3s、点滅を繰り返す) ---
bool     patronum_active = false; // 守護霊アニメ実行中
uint32_t patronum_end_ms = 0;     // アニメ終了時刻
uint32_t patronum_tick_ms= 0;     // 次ステップ時刻
int      patronum_k      = 0;     // 点灯している外部 LED 本数 (0=波の合間)
constexpr uint32_t PATRONUM_TICK_MS = 70;  // 1 ステップ (波の進む速さ)

// 外部 LED は nrf_gpio_* で直接駆動 (絶対 GPIO 番号、Arduino ピンマップを介さない)
void ext_write(bool on) {
  for (int i = 0; i < cfg::current.pin_count; i++) {
    if (on) nrf_gpio_pin_set((uint32_t)cfg::current.pins[i]);
    else    nrf_gpio_pin_clear((uint32_t)cfg::current.pins[i]);
  }
}

// pins 設定が変わった時に再 init するヘルパ (Serial 設定後にも呼ぶ)
void reinit_ext_pins() {
  for (int i = 0; i < cfg::current.pin_count; i++) {
    nrf_gpio_cfg_output((uint32_t)cfg::current.pins[i]);
    nrf_gpio_pin_clear((uint32_t)cfg::current.pins[i]);  // OFF (active-high)
  }
}

void init_pins() {
  pinMode(LED_RED,   OUTPUT); digitalWrite(LED_RED,   HIGH);  // 内蔵赤 OFF
  pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, HIGH);  // 内蔵緑 OFF
  pinMode(LED_BLUE,  OUTPUT); digitalWrite(LED_BLUE,  HIGH);  // 内蔵青 OFF
  reinit_ext_pins();
  blue_next_ms = millis();
}

// 内蔵赤 + 外部 LED群を時限点灯 (SHAKE 等。ハートビート青は別途継続)
void red_on_for(uint32_t duration_ms) {
  digitalWrite(LED_RED, LOW);
  ext_write(true);
  red_is_on       = true;
  patronum_active = false;
  red_off_at_ms   = millis() + duration_ms;
}

// Expecto Patronum: 「光の波動が外へ広がる」アニメを duration_ms 実行 (非ブロッキング)。
//   poll() が PATRONUM_TICK_MS ごとに外部 LED を 0→1→2…本と増やして点灯 (波の拡散) し、
//   上限に達したら全消灯→再拡散を繰り返す。内蔵 青/緑 も波に同期して点滅。
//   INCENDIO(赤点灯) と見た目で区別。
void patronum_start(uint32_t duration_ms) {
  patronum_active = true;
  patronum_end_ms = millis() + duration_ms;
  patronum_tick_ms= millis();
  patronum_k      = 0;
  lumos_mode      = false;
  red_is_on       = false;
  red_off_at_ms   = 0;
  blue_is_on      = true;               // アニメ中はハートビート抑止
}

void red_off_now() {
  digitalWrite(LED_RED, HIGH);
  ext_write(false);
  red_is_on     = false;
  red_off_at_ms = 0;
}

// LUMOS: 全 LED 点灯 (NOX まで持続、ハートビート抑止)
void all_on() {
  digitalWrite(LED_RED,   LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE,  LOW);
  ext_write(true);
  lumos_mode    = true;
  red_is_on     = false;
  red_off_at_ms = 0;
  blue_is_on    = true;
}

// NOX: 全 LED 消灯 (ハートビート再開)
void all_off() {
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE,  HIGH);
  ext_write(false);
  lumos_mode      = false;
  patronum_active = false;
  red_is_on       = false;
  red_off_at_ms   = 0;
  blue_is_on      = false;
  blue_next_ms    = millis() + BLUE_HEARTBEAT_PERIOD_MS;
}

// loop() から定期 poll: 時限消灯 + ハートビート (LUMOS 中は抑止)
void poll() {
  uint32_t now = millis();

  // Expecto Patronum: 光の波動アニメ (最優先、3s 点滅しながら外へ拡散)
  if (patronum_active) {
    if ((int32_t)(now - patronum_end_ms) >= 0) {
      digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_BLUE, HIGH);
      ext_write(false);
      patronum_active = false;
      blue_is_on      = false;
      blue_next_ms    = now + BLUE_HEARTBEAT_PERIOD_MS;  // ハートビート再開
      Serial.println("[LED] EXPECTO PATRONUM done");
      return;
    }
    if ((int32_t)(now - patronum_tick_ms) >= 0) {
      patronum_tick_ms = now + PATRONUM_TICK_MS;
      int n = cfg::current.pin_count;
      if (n < 1) n = 1;
      patronum_k++;
      if (patronum_k > n) patronum_k = 0;             // 0 = 波の合間 (全消灯)
      // 外部 LED: 内側(pins[0])から外側へ点灯本数を増やす = 波が広がる
      for (int i = 0; i < cfg::current.pin_count; i++) {
        if (i < patronum_k) nrf_gpio_pin_set((uint32_t)cfg::current.pins[i]);
        else                nrf_gpio_pin_clear((uint32_t)cfg::current.pins[i]);
      }
      // 内蔵: 波に同期して青点滅、最大拡散の瞬間だけ緑も足して明るくフラッシュ
      digitalWrite(LED_BLUE,  (patronum_k > 0)  ? LOW : HIGH);
      digitalWrite(LED_GREEN, (patronum_k >= n) ? LOW : HIGH);
    }
    return;  // アニメ中は他の LED 制御をしない
  }

  if (lumos_mode) return;

  if (red_is_on && red_off_at_ms != 0 && (int32_t)(now - red_off_at_ms) >= 0) {
    digitalWrite(LED_RED, HIGH);
    ext_write(false);
    red_is_on     = false;
    red_off_at_ms = 0;
    Serial.println("[LED] RED + EXT auto OFF");
  }

  if (!blue_is_on && (int32_t)(now - blue_next_ms) >= 0) {
    digitalWrite(LED_BLUE, LOW);
    blue_is_on     = true;
    blue_off_at_ms = now + BLUE_HEARTBEAT_FLASH_MS;
    blue_next_ms   = now + BLUE_HEARTBEAT_PERIOD_MS;
  }
  if (blue_is_on && (int32_t)(now - blue_off_at_ms) >= 0) {
    digitalWrite(LED_BLUE, HIGH);
    blue_is_on = false;
  }
}
}  // namespace led

namespace receiver {
volatile uint8_t last_seq    = 0xFF;
volatile bool    have_first  = false;

void handle_trigger(uint8_t seq, uint8_t trigger_id, uint8_t strength,
                    uint16_t target_id, int8_t rssi) {
  // 宛先フィルタ: 全機宛て or 自機宛て のみ反応
  if (target_id != wand_beacon::TARGET_ALL && target_id != cfg::current.device_id) return;
  // 重複抑止
  if (have_first && seq == last_seq) return;
  last_seq   = seq;
  have_first = true;

  Serial.printf("[RX] seq=%u trig=0x%02X strength=%u target=%u rssi=%d ",
                seq, trigger_id, strength, (unsigned)target_id, rssi);

  switch (trigger_id) {
    case wand_beacon::TRIG_SHAKE:
      Serial.println("=> RED+EXT ON 250ms (SHAKE = 魔法失敗の一瞬点灯)");
      led::red_on_for(wand_beacon::LED_DURATION_SHAKE_MS);
      break;
    case wand_beacon::TRIG_LUMOS:
      Serial.println("=> ALL LED ON (LUMOS, until NOX)");
      led::all_on();
      break;
    case wand_beacon::TRIG_NOX:
      Serial.println("=> ALL LED OFF (NOX)");
      led::all_off();
      break;
    case wand_beacon::TRIG_INCENDIO:
      Serial.println("=> RED+EXT ON 3s (INCENDIO)");
      led::red_on_for(wand_beacon::LED_DURATION_INCENDIO_MS);
      break;
    case wand_beacon::TRIG_AGUAMENTI:
      Serial.println("=> RED+EXT ON 5s (AGUAMENTI)");
      led::red_on_for(wand_beacon::LED_DURATION_AGUAMENTI_MS);
      break;
    case wand_beacon::TRIG_EXPECTO_PATRONUM:
      Serial.println("=> WAVE pulse 3s (EXPECTO PATRONUM)");
      led::patronum_start(wand_beacon::LED_DURATION_PATRONUM_MS);
      break;
    default:
      Serial.println("=> unknown trigger, ignored");
      break;
  }
}
}  // namespace receiver

void scan_callback(ble_gap_evt_adv_report_t* report) {
  uint8_t buf[31];
  uint8_t len = Bluefruit.Scanner.parseReportByType(
      report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buf, sizeof(buf));

  if (len >= sizeof(wand_beacon::Payload)) {
    // ARM Cortex-M4 で unaligned access fault を避けるため memcpy 経由
    wand_beacon::Payload payload;
    memcpy(&payload, buf, sizeof(payload));
    if (payload.company_id == wand_beacon::COMPANY_ID) {
      receiver::handle_trigger(payload.seq, payload.trigger_id, payload.strength,
                               payload.target_id, report->rssi);
    }
  }
  Bluefruit.Scanner.resume();
}

// ============================================================
// Serial 設定コマンド処理
//   show                : 現在の設定を表示
//   id=<n>              : DEVICE_ID を設定 (1-65534)
//   pins=D0,D2,G1.11    : 外部 LED ピンを設定 (D/G/生番号 混在可)
//   save                : 設定を InternalFS に保存 (再起動後も有効)
//   default             : デフォルトに戻す (保存は別途 save)
//   help                : コマンド一覧
// ============================================================
namespace serialcmd {
char buf[96];
int  len = 0;

void handle_line(char* line) {
  // 前後空白除去
  while (*line == ' ') line++;
  char* end = line + strlen(line) - 1;
  while (end > line && (*end == ' ' || *end == '\r')) *end-- = '\0';

  if (strcmp(line, "show") == 0) {
    cfg::print();
  } else if (strncmp(line, "id=", 3) == 0) {
    int v = atoi(line + 3);
    if (v >= 1 && v <= 65534) {
      cfg::current.device_id = (uint16_t)v;
      Serial.printf("[CFG] device_id=%d (save で永続化)\n", v);
    } else {
      Serial.println("[CFG] id は 1-65534");
    }
  } else if (strncmp(line, "pins=", 5) == 0) {
    cfg::set_pins_from_csv(line + 5);
    led::reinit_ext_pins();
    Serial.println("[CFG] pins 適用 (save で永続化)");
  } else if (strcmp(line, "save") == 0) {
    Serial.println(cfg::save() ? "[CFG] saved" : "[CFG] save FAILED");
  } else if (strcmp(line, "default") == 0) {
    cfg::set_defaults();
    led::reinit_ext_pins();
    Serial.println("[CFG] defaults loaded (save で永続化)");
  } else if (strcmp(line, "test") == 0) {
    // 配線診断: 設定ピンを 1 本ずつ 1 秒点灯 → 最後に全部 2 秒点灯
    Serial.println("[TEST] each pin 1s, then all 2s");
    for (int i = 0; i < cfg::current.pin_count; i++) {
      Serial.printf("[TEST] pin[%d] = nRF %d ON\n", i, cfg::current.pins[i]);
      nrf_gpio_pin_set((uint32_t)cfg::current.pins[i]);
      delay(1000);
      nrf_gpio_pin_clear((uint32_t)cfg::current.pins[i]);
    }
    Serial.println("[TEST] ALL ON 2s");
    led::ext_write(true);
    delay(2000);
    led::ext_write(false);
    Serial.println("[TEST] done");
  } else if (strcmp(line, "help") == 0 || line[0] == '?') {
    Serial.println("cmds: show / id=<n> / pins=D0,D2,G1.11 / save / default / test");
  } else if (line[0] != '\0') {
    Serial.printf("[CFG] unknown: %s (help で一覧)\n", line);
  }
}

void poll() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = '\0'; handle_line(buf); len = 0; }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}
}  // namespace serialcmd

void setup() {
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  // 設定読み込み (フラッシュに無ければデフォルト)
  cfg::set_defaults();
  bool loaded = cfg::load();

  led::init_pins();

  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && (millis() - t) < 5000) delay(10);

  Serial.println();
  Serial.println("=== LED Receiver (Phase 0, 100% wake scan) ===");
  Serial.printf("config source: %s\n", loaded ? "InternalFS" : "DEFAULT");
  cfg::print();
  Serial.println("BLUE=heartbeat / RED+EXT=trigger / LUMOS=all on / NOX=all off");
  Serial.println("Serial cmds: show / id=<n> / pins=D0,D2,G1.11 / save / default");
  Serial.flush();

  Bluefruit.begin(0, 1);
  Bluefruit.setName("LED-RX");

  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.setInterval(160, 160);  // window=interval=100ms (100% duty)
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);

  Serial.println("[BLE] scanning, waiting for wand (company_id=0xFFFF) ...");
  Serial.flush();
}

void loop() {
  led::poll();
  serialcmd::poll();
  delay(20);
}
