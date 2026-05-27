// proto_wand_to_led / wand_xiao_nrf52840 / src/main.cpp
// 杖 (送信側): XIAO nRF52840 Sense + 内蔵 LSM6DS3TR-C + Bluefruit Advertising Beacon
// Phase 0 相当: 常時 ON でジェスチャ検出 → BLE adv 500ms burst (sleep / WOM はスコープ外)
//
// ジェスチャ判定は ../../shared/wand_gesture.h を M5StickC 版と共用。
// ワイヤフォーマットは ../../shared/beacon_protocol.h (受信側 led_xiao_nrf52840 と相互運用)。
//
// 実装メモ (led_xiao_nrf52840 で判明した nRF52840 の作法):
//   - TinyUSBDevice.begin(0) を Serial.begin() より先に呼ぶ (USB CDC 列挙の安定化)
//   - PowerShell SerialPort 経由で読む場合 DtrEnable=$true 必須
//   - 内蔵 IMU は Wire1 (D17/D16)、電源 EN ピンを HIGH 駆動してから初期化

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <Wire.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "device_config.h"
#include "../../shared/beacon_protocol.h"
#include "../../shared/wand_common.h"
#include "../../shared/wand_gesture.h"

using namespace Adafruit_LittleFS_Namespace;

// ジェスチャ判定コア。BLE 送信(ble::emit_beacon) と LED フラッシュ(wled::flash) は setup() で注入。
// 閾値は gcfg が det.cfg を読み書きして LittleFS に永続化 (M5 の NVS 相当)。
wand_gesture::Detector det;

// ============================================================
// ジェスチャ閾値の永続化 (LittleFS /wandg.bin)。値の実体は det.cfg。
//   Serial: gshow / gth= / gratio= / gcool= / galpha= / gband= / gsave / gdefault
// ============================================================
namespace gcfg {
constexpr float    DEF_FLICK_THRESHOLD_G = 1.5f;
constexpr float    DEF_UPDOWN_RATIO      = 0.75f;
constexpr uint32_t DEF_COOLDOWN_MS       = 1000;
constexpr float    DEF_GRAV_ALPHA        = 0.02f;
constexpr float    DEF_STILL_BAND        = 0.15f;

constexpr uint32_t MAGIC = 0x57414E47;   // "WANG"
const char* const  PATH  = "/wandg.bin";

struct Stored {
  uint32_t magic;
  float    flick_threshold_g;
  float    updown_ratio;
  uint32_t cooldown_ms;
  float    grav_alpha;
  float    still_band;
};

void set_defaults() {
  det.cfg.flick_threshold_g = DEF_FLICK_THRESHOLD_G;
  det.cfg.updown_ratio      = DEF_UPDOWN_RATIO;
  det.cfg.cooldown_ms       = DEF_COOLDOWN_MS;
  det.cfg.grav_alpha        = DEF_GRAV_ALPHA;
  det.cfg.still_band        = DEF_STILL_BAND;
}

bool load() {
  InternalFS.begin();
  File f(InternalFS);
  if (!f.open(PATH, FILE_O_READ)) return false;
  Stored s;
  int n = f.read((uint8_t*)&s, sizeof(s));
  f.close();
  if (n != (int)sizeof(s) || s.magic != MAGIC) return false;
  det.cfg.flick_threshold_g = s.flick_threshold_g;
  det.cfg.updown_ratio      = s.updown_ratio;
  det.cfg.cooldown_ms       = s.cooldown_ms;
  det.cfg.grav_alpha        = s.grav_alpha;
  det.cfg.still_band        = s.still_band;
  return true;
}

void save() {
  InternalFS.begin();
  InternalFS.remove(PATH);
  File f(InternalFS);
  if (!f.open(PATH, FILE_O_WRITE)) { Serial.println("[GCFG] save open failed"); return; }
  Stored s { MAGIC, det.cfg.flick_threshold_g, det.cfg.updown_ratio,
             det.cfg.cooldown_ms, det.cfg.grav_alpha, det.cfg.still_band };
  f.write((const uint8_t*)&s, sizeof(s));
  f.close();
  Serial.println("[GCFG] saved");
}

void print() {
  Serial.printf("[GCFG] th=%.2fg ratio=%.2f cool=%lums alpha=%.3f band=%.2fg\n",
                det.cfg.flick_threshold_g, det.cfg.updown_ratio,
                (unsigned long)det.cfg.cooldown_ms, det.cfg.grav_alpha, det.cfg.still_band);
}
}  // namespace gcfg

// ============================================================
// IMU (LSM6DS3TR-C, 内蔵, Wire1 直読み, I2C 0x6A)
//   ※ MPU6886 と違い出力はリトルエンディアン (low byte first), OUTX_L_XL=0x28 起点
// ============================================================
namespace imu {
constexpr uint8_t I2C_ADDR     = IMU_I2C_ADDR;             // 0x6A
constexpr float   ACC_LSB_TO_G = 1.0f / IMU_ACC_LSB_PER_G; // device_config.h

// --- bit-bang I2C (nRF52 ハードウェア TWIM は無応答時にタイムアウト無くハングするため、
//     IMU アクセスは全て soft I2C で行う。Wire1 ピン SDA=P0.07/D17, SCL=P0.27/D16 を手動駆動)。
namespace bb {
constexpr int SDA_PIN = PIN_WIRE1_SDA;   // 17 (P0.07)
constexpr int SCL_PIN = PIN_WIRE1_SCL;   // 16 (P0.27)
inline void dly()    { delayMicroseconds(5); }
inline void sda_hi() { pinMode(SDA_PIN, INPUT_PULLUP); }                       // 解放 (pull-up で High)
inline void sda_lo() { pinMode(SDA_PIN, OUTPUT); digitalWrite(SDA_PIN, LOW); }
inline void scl_hi() { pinMode(SCL_PIN, INPUT_PULLUP); }
inline void scl_lo() { pinMode(SCL_PIN, OUTPUT); digitalWrite(SCL_PIN, LOW); }
inline int  sda_rd() { pinMode(SDA_PIN, INPUT_PULLUP); return digitalRead(SDA_PIN); }
void start() { sda_hi(); scl_hi(); dly(); sda_lo(); dly(); scl_lo(); dly(); }
void stop()  { sda_lo(); dly(); scl_hi(); dly(); sda_hi(); dly(); }
bool wr(uint8_t b) {  // true=ACK
  for (int i = 0; i < 8; i++) { if (b & 0x80) sda_hi(); else sda_lo(); dly(); scl_hi(); dly(); scl_lo(); b <<= 1; }
  sda_hi(); dly(); scl_hi(); dly(); int ack = sda_rd(); scl_lo(); dly(); return ack == 0;
}
uint8_t rd(bool ack) {
  uint8_t b = 0; sda_hi();
  for (int i = 0; i < 8; i++) { dly(); scl_hi(); dly(); b = (b << 1) | (sda_rd() & 1); scl_lo(); }
  if (ack) sda_lo(); else sda_hi(); dly(); scl_hi(); dly(); scl_lo(); sda_hi();
  return b;
}
int read_reg(uint8_t addr7, uint8_t reg) {  // -1=addr NACK, -2=read-addr NACK
  start(); if (!wr(addr7 << 1)) { stop(); return -1; }
  wr(reg);
  start(); if (!wr((addr7 << 1) | 1)) { stop(); return -2; }
  uint8_t v = rd(false); stop(); return v;
}
bool write_reg(uint8_t addr7, uint8_t reg, uint8_t val) {
  start(); bool ok = wr(addr7 << 1); wr(reg); wr(val); stop(); return ok;
}
bool read_burst(uint8_t addr7, uint8_t reg, uint8_t* buf, uint8_t len) {
  start(); if (!wr(addr7 << 1)) { stop(); return false; }
  wr(reg);
  start(); if (!wr((addr7 << 1) | 1)) { stop(); return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = rd(i < len - 1);   // 最終バイトのみ NACK
  stop(); return true;
}
}  // namespace bb

// 上位 API は全て bit-bang に委譲 (Wire1/TWIM は未使用 = ハングしない)
bool found      = false;   // WHO_AM_I が取れたか
bool power_high = true;    // IMU が応答した電源極性
bool    write_reg(uint8_t reg, uint8_t val) { return bb::write_reg(I2C_ADDR, reg, val); }
uint8_t read_reg(uint8_t reg)               { int v = bb::read_reg(I2C_ADDR, reg); return v < 0 ? 0 : (uint8_t)v; }
bool    read_burst(uint8_t start_reg, uint8_t* buf, size_t len) {
  return bb::read_burst(I2C_ADDR, start_reg, buf, (uint8_t)len);
}

// バスのアイドル電位を読む (プロトコルなし。pull-up で High になるか = バスが生きてるか)
static void idle_levels(const char* tag) {
  pinMode(bb::SDA_PIN, INPUT_PULLUP);
  pinMode(bb::SCL_PIN, INPUT_PULLUP);
  delay(2);
  Serial.printf("[IMU] idle bus %s: SDA=%d SCL=%d (1=High/pull-up効く, 0=Low に張付き)\n",
                tag, digitalRead(bb::SDA_PIN), digitalRead(bb::SCL_PIN));
}

bool begin() {
  pinMode(IMU_POWER_PIN, OUTPUT);
  pinMode(IMU_INT1_PIN, INPUT);   // INT1 を入力に (出力で干渉しないよう)
  // WHO_AM_I を電源 HIGH/LOW 両極性で確認し、IMU が応答する極性に給電を固定する (全て bit-bang)
  digitalWrite(IMU_POWER_PIN, HIGH); delay(250);
  idle_levels("power=HIGH");
  int hi = bb::read_reg(I2C_ADDR, 0x0F);
  digitalWrite(IMU_POWER_PIN, LOW);  delay(250);
  idle_levels("power=LOW");
  int lo = bb::read_reg(I2C_ADDR, 0x0F);
  bool ok_hi = (hi == 0x6A || hi == 0x69);
  bool ok_lo = (lo == 0x6A || lo == 0x69);
  found      = ok_hi || ok_lo;
  power_high = ok_hi || !ok_lo;   // 応答した極性に固定 (どちらも不可なら HIGH)
  digitalWrite(IMU_POWER_PIN, power_high ? HIGH : LOW); delay(200);
  Serial.printf("[IMU] bitbang WHO_AM_I: power_HIGH=0x%02X(%d) power_LOW=0x%02X(%d) -> use=%s found=%d (addr=0x%02X SDA=D%d SCL=D%d)\n",
                hi & 0xFF, hi, lo & 0xFF, lo, power_high ? "HIGH" : "LOW", found,
                I2C_ADDR, bb::SDA_PIN, bb::SCL_PIN);
  if (!found) return false;
  bb::write_reg(I2C_ADDR, 0x10, 0x4C);   // CTRL1_XL: ODR=104Hz, FS=±8g
  bb::write_reg(I2C_ADDR, 0x12, 0x44);   // CTRL3_C: BDU=1 + IF_INC=1
  delay(10);
  return true;
}

// 3 軸加速度を g 単位で読む (リトルエンディアン)。成功 true。
bool read_accel_g(float& ax_g, float& ay_g, float& az_g) {
  uint8_t raw[6];
  if (!read_burst(0x28, raw, 6)) return false;   // OUTX_L_XL..OUTZ_H_XL
  int16_t ax = (int16_t)((raw[1] << 8) | raw[0]);
  int16_t ay = (int16_t)((raw[3] << 8) | raw[2]);
  int16_t az = (int16_t)((raw[5] << 8) | raw[4]);
  ax_g = ax * ACC_LSB_TO_G;
  ay_g = ay * ACC_LSB_TO_G;
  az_g = az * ACC_LSB_TO_G;
  return true;
}
}  // namespace imu

// ============================================================
// BLE Advertising Beacon (Bluefruit, 非接続 broadcast)
//   M5 の NimBLE 版と同じ挙動: 検出時に Payload を載せて 20ms 間隔で 500ms burst。
//   Bluefruit は adv データのインプレース更新ができないため、毎回
//   stop → clearData → addManufacturerData → start で更新する (Wingardium 20Hz も同様)。
// ============================================================
namespace ble {
uint32_t           adv_stop_at_ms = 0;
constexpr uint32_t ADV_BURST_MS   = 500;

void begin() {
  Bluefruit.begin(1, 0);                 // peripheral=1, central=0 (advertiser)
  Bluefruit.setName("Wand-Proto");
  Bluefruit.setTxPower(8);               // +8dBm (最大に近い)
  Bluefruit.autoConnLed(false);          // Bluefruit の自動 LED を無効 (赤 LED を自前制御するため)
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED);
  Bluefruit.Advertising.setInterval(32, 32);   // 32 * 0.625ms = 20ms 固定 (BLE 仕様最小)
  Bluefruit.Advertising.setFastTimeout(0);     // fast/slow 切替なし
}

void emit_beacon(uint8_t trigger_id, uint8_t strength, uint16_t target_id) {
  static uint8_t seq_counter = 0;
  seq_counter++;

  wand_beacon::Payload payload;
  payload.company_id = wand_beacon::COMPANY_ID;
  payload.seq        = seq_counter;
  payload.trigger_id = trigger_id;
  payload.strength   = strength;
  payload.target_id  = target_id;

  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  // payload 先頭 2B = company_id(0xFFFF) がそのまま manufacturer ID になり、
  // 受信側 (led) の parseReportByType(MANUFACTURER_SPECIFIC_DATA) + memcpy と一致する。
  Bluefruit.Advertising.addManufacturerData(&payload, sizeof(payload));
  Bluefruit.Advertising.start(0);        // 0 = タイムアウト無し (poll_stop で止める)
  adv_stop_at_ms = millis() + ADV_BURST_MS;

  Serial.printf("[BLE] adv start seq=%u trig=0x%02X strength=%u target=%u\n",
                seq_counter, trigger_id, strength, (unsigned)target_id);
}

// loop() から定期呼び出し: burst 時間が過ぎたら adv 停止
void poll_stop() {
  if (adv_stop_at_ms != 0 && millis() >= adv_stop_at_ms) {
    Bluefruit.Advertising.stop();
    adv_stop_at_ms = 0;
    Serial.println("[BLE] adv stopped");
  }
}
}  // namespace ble

// ============================================================
// 内蔵赤 LED (LED_BUILTIN = LED_RED, active-low) で状態表示
//   フィードバック表示の ON/OFF は ctrl 名前空間 (外付 B ボタン) が管理。デフォルト OFF。
//   有効時: 静止(ready)=消灯 / 収束中=点滅 / ジェスチャ検出時=一瞬フラッシュ(75ms)
// ============================================================
namespace wled {
constexpr int      PIN      = BUILTIN_LED_PIN;  // 内蔵赤 LED (active-low)
constexpr uint32_t FLASH_MS = 75;

bool     enabled        = false;    // デフォルト OFF (起動時は光らない)
uint32_t flash_until_ms = 0;

void init() {
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);          // OFF (active-low)
}

// 起動表示: enabled に関係なく 2 回点滅 (「起動した」だけを示す。魔法=beacon は出さない)
void boot_blink() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN, LOW);  delay(80);   // 点灯
    digitalWrite(PIN, HIGH); delay(120);  // 消灯
  }
}

void flash() { if (enabled) flash_until_ms = millis() + FLASH_MS; }  // 検出時に呼ぶ

void update(bool ready) {
  if (!enabled) { digitalWrite(PIN, HIGH); return; }  // 無効時は常時消灯
  uint32_t now = millis();
  if (now < flash_until_ms) {
    digitalWrite(PIN, LOW);            // フラッシュ中 = 点灯
  } else if (ready) {
    digitalWrite(PIN, HIGH);           // 静止 = 消灯
  } else {
    bool on = (now / 100) % 2 == 0;    // 収束中 = 5Hz 点滅
    digitalWrite(PIN, on ? LOW : HIGH);
  }
}
}  // namespace wled

// ============================================================
// 操作モード制御 (外付 B ボタン: BUTTON_B_PIN, INPUT_PULLUP, active-low)
//   B 長押し (>=1s)  → 手動操作モード / モーションモード をトグル
//                      切替確認に赤 LED 点滅 (モーション=2回 / 手動=3回)
//   B 一瞬押し:
//     モーションモード → 内蔵 LED フィードバック表示 ON/OFF
//     手動操作モード   → LUMOS / NOX を交互に beacon 送信
//   手動操作モード中はジェスチャ判定 (check / wingardium) を停止する。
//   ※ XIAO は sleep スコープ外のため、起動時は常にモーションモード (モード永続化なし)。
// ============================================================
namespace ctrl {
constexpr int      BTN_B         = BUTTON_B_PIN;
constexpr uint32_t LONG_PRESS_MS = 1000;   // これ以上の押下で長押し = モード切替
constexpr uint32_t DEBOUNCE_MS   = 30;     // 一瞬押しの最小時間 (チャタリング除去)

bool     manual_mode   = false;  // false=モーションモード(既定) / true=手動操作モード
bool     next_is_lumos = true;   // 手動モードで次に送るのが LUMOS か (一瞬押しごとに反転)

bool     btn_down    = false;    // 押下中か
uint32_t press_start = 0;        // 押下開始時刻
bool     long_fired  = false;    // 今回の押下で長押しが既に発火したか

void init() {
  pinMode(BTN_B, INPUT_PULLUP);   // 外付タクト (GND 間)。押下で LOW
}

// モード切替の確認点滅 (enabled に関係なく必ず光らせて操作受付を伝える)
//   モーションモード = 2 回 / 手動操作モード = 3 回
void blink_mode(bool to_manual) {
  int n = to_manual ? 3 : 2;
  for (int i = 0; i < n; i++) {
    digitalWrite(wled::PIN, LOW);  delay(120);   // 点灯
    digitalWrite(wled::PIN, HIGH); delay(150);   // 消灯
  }
}

void on_long_press() {   // モード切替
  manual_mode = !manual_mode;
  Serial.printf("[MODE] %s\n", manual_mode ? "MANUAL (B short = LUMOS/NOX)"
                                           : "MOTION (gesture)");
  if (manual_mode) {
    next_is_lumos = true;            // 手動モードに入ったら次は LUMOS から
    det.levitation_until = 0;        // 浮遊モードが残っていれば解除
  }
  blink_mode(manual_mode);
}

void on_short_press() {
  if (manual_mode) {
    // 手動操作: LUMOS / NOX を交互送信 (全機宛て)
    uint8_t trig = next_is_lumos ? wand_beacon::TRIG_LUMOS : wand_beacon::TRIG_NOX;
    ble::emit_beacon(trig, 200, wand_beacon::TARGET_ALL);
    det.last_trigger_ms = millis();
    Serial.printf("[MANUAL] %s\n", next_is_lumos ? "LUMOS" : "NOX");
    next_is_lumos = !next_is_lumos;
    wled::flash();                       // 手元ランプが有効なら一瞬光らせる
  } else {
    // モーションモード: 手元 LED フィードバック ON/OFF
    wled::enabled = !wled::enabled;
    Serial.printf("[LED] feedback %s\n", wled::enabled ? "ON" : "OFF");
    if (!wled::enabled) digitalWrite(wled::PIN, HIGH);  // 無効化したら即消灯
  }
}

// 毎ループ呼ぶ: 押下を計測して長押し/一瞬押しを振り分ける
//   長押し中 (1s 到達まで) は何も発火せず、離す瞬間にのみ一瞬押しが発火。
//   1s 到達で長押し成立後は離しても一瞬押しは出ない (排他)。
void poll() {
  bool     pressed = (digitalRead(BTN_B) == 0);   // active-low: LOW=押下
  uint32_t now     = millis();

  if (pressed && !btn_down) {                 // 立下り = 押下開始
    btn_down    = true;
    press_start = now;
    long_fired  = false;
    det.last_motion_ms = now;                 // ボタン操作中は (将来 sleep 時に) 起こしておく
  } else if (pressed && btn_down) {           // 押下継続
    if (!long_fired && (now - press_start) >= LONG_PRESS_MS) {
      long_fired = true;                      // 離す前に長押し成立 → モード切替
      on_long_press();
      det.last_motion_ms = millis();          // blink の delay 分を補正
    }
  } else if (!pressed && btn_down) {          // 立上り = 離した
    btn_down = false;
    uint32_t dur = now - press_start;
    if (!long_fired && dur >= DEBOUNCE_MS) {  // 長押し未発火 & チャタリングでない
      on_short_press();
    }
  }
}
}  // namespace ctrl

// ============================================================
// Serial コマンド (行単位パーサ)
//   トリガ: t/l/n/i/a/e/w 単独 → 全機宛て / "<id> <cmd>" → 特定機宛て (例 "2 l")
//   設定:   gshow / gth=<f> / gratio=<f> / gcool=<ms> / galpha=<f> / gband=<f> / gsave / gdefault
// ============================================================
void emit_named(uint8_t trig, uint16_t target) {
  Serial.printf("[SERIAL_CMD] trigger 0x%02X target=%u\n", trig, (unsigned)target);
  ble::emit_beacon(trig, 200, target);
  det.last_trigger_ms = millis();
}

void handle_line(char* line) {
  while (*line == ' ') line++;
  int n = strlen(line);
  while (n > 0 && (line[n-1] == ' ' || line[n-1] == '\r' || line[n-1] == '\n')) line[--n] = 0;
  if (n == 0) return;

  // --- 設定コマンド ---
  if (strcmp(line, "gshow") == 0)       { gcfg::print(); return; }
  if (strcmp(line, "gsave") == 0)       { gcfg::save(); return; }
  if (strcmp(line, "gdefault") == 0)    { gcfg::set_defaults(); Serial.println("[GCFG] defaults (gsave で永続化)"); gcfg::print(); return; }
  if (strncmp(line, "gth=", 4) == 0)    { det.cfg.flick_threshold_g = atof(line+4); gcfg::print(); return; }
  if (strncmp(line, "gratio=", 7) == 0) { det.cfg.updown_ratio = atof(line+7); gcfg::print(); return; }
  if (strncmp(line, "gcool=", 6) == 0)  { det.cfg.cooldown_ms = (uint32_t)atol(line+6); gcfg::print(); return; }
  if (strncmp(line, "galpha=", 7) == 0) { det.cfg.grav_alpha = atof(line+7); gcfg::print(); return; }
  if (strncmp(line, "gband=", 6) == 0)  { det.cfg.still_band = atof(line+6); gcfg::print(); return; }

  // --- トリガコマンド ("<id> <cmd>" or "<cmd>") ---
  uint16_t target = wand_beacon::TARGET_ALL;
  char* cmd = line;
  char* sp = strchr(line, ' ');
  if (sp) {                       // "2 l" 形式: 前半を target_id とみなす
    *sp = 0;
    long id = atol(line);
    if (id >= 1 && id <= 65534) target = (uint16_t)id;
    cmd = sp + 1;
    while (*cmd == ' ') cmd++;
  }
  switch (cmd[0]) {
    case 't': emit_named(wand_beacon::TRIG_SHAKE, target);     break;
    case 'l': emit_named(wand_beacon::TRIG_LUMOS, target);     break;
    case 'n': emit_named(wand_beacon::TRIG_NOX, target);       break;
    case 'i': emit_named(wand_beacon::TRIG_INCENDIO, target);  break;  // 横振り
    case 'a': emit_named(wand_beacon::TRIG_AGUAMENTI, target); break;
    case 'e': emit_named(wand_beacon::TRIG_EXPECTO_PATRONUM, target); break;  // 前突き = 守護霊
    case 'w': det.force_levitation(); break;  // Wingardium 浮遊モードを強制開始
    default:
      Serial.println("[CMD] t/l/n/i/a/e/w | <id> <cmd> | gshow/gth=/gratio=/gcool=/galpha=/gband=/gsave/gdefault");
      break;
  }
}

void handle_serial() {
  static char buf[64];
  static int  len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; handle_line(buf); len = 0; }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// ============================================================
// Arduino setup / loop
// ============================================================
void setup() {
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);          // Serial.begin() より先に (USB CDC 列挙の安定化)
  }

  wled::init();        // 内蔵赤 LED (OFF)
  ctrl::init();        // 外付 B ボタン (INPUT_PULLUP)

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 20000) delay(10);  // 診断: ホスト接続を最大 20s 待つ
  delay(100);

  Serial.println();
  Serial.println("=== Wand Beacon (XIAO nRF52840 Sense / LSM6DS3) ===");
  Serial.println("Up-flick=LUMOS / Down-flick=NOX / Thrust-fwd=EXPECTO / Side-swing=INCENDIO");
  Serial.println("Hold-up(0.8s)=WINGARDIUM levitation (pitch stream)");
  Serial.println("Btn B long(>=1s)=MOTION<->MANUAL (blink x2/x3) / short: MANUAL=LUMOS/NOX, MOTION=LED");
  Serial.println("Adv burst: 500ms, Cooldown: 1s, always-on (no sleep)");
  Serial.flush();

  wled::boot_blink();  // 起動表示 (2 回点滅、魔法は出さない)

  Serial.println("[setup] before imu::begin"); Serial.flush();
  bool imu_ok = imu::begin();
  Serial.printf("[setup] after imu::begin ok=%d\n", imu_ok); Serial.flush();
  if (imu_ok) Serial.println("[IMU] LSM6DS3TR-C OK (Wire1, 0x6A)");
  else        Serial.println("[WARN] IMU init failed - 診断のため継続 (loop で WHO_AM_I を周期表示)");

  // ジェスチャ検出器に BLE 送信 / LED フラッシュを注入し、軸マウントを設定 (共通コア)
  det.begin(&ble::emit_beacon, &wled::flash);
  det.cfg.forward_axis = WAND_FORWARD;
  det.cfg.right_axis   = WAND_RIGHT;

  gcfg::set_defaults();
  bool loaded = gcfg::load();   // LittleFS から閾値を det.cfg に読み込み (無ければデフォルト)
  Serial.printf("[GCFG] source: %s\n", loaded ? "LittleFS" : "DEFAULT");
  gcfg::print();
  Serial.println("[CMD] t/l/n/i/a/e/w | <id> <cmd> | gshow/gth=/gratio=/gcool=/galpha=/gband=/gsave");

  // 重力推定を初期化 (起動直後の静止姿勢で 1 回読む)
  {
    float ax, ay, az;
    if (imu::read_accel_g(ax, ay, az)) det.set_gravity(ax, ay, az);
  }

  Serial.println("[setup] before ble::begin"); Serial.flush();
  ble::begin();
  Serial.println("[BLE] init OK, waiting for gesture...");
  Serial.println("[MODE] boot in MOTION (gesture)");

  det.last_motion_ms = millis();   // (将来 sleep 時の起点)
}

void loop() {
  static uint32_t last_print_ms = 0;
  float ax, ay, az;
  bool ok = imu::read_accel_g(ax, ay, az);

  if (ok) {
    det.update_gravity(ax, ay, az);   // 重力推定はモードに関係なく継続 (復帰時に即 ready)
    // 手動操作モード中はジェスチャ判定を全停止 (B ボタン操作だけで魔法を出す)
    if (!ctrl::manual_mode) {
      if (!det.is_levitating()) det.check(ax, ay, az);
      det.poll_wingardium(ax, ay, az);  // 上向き保持で浮遊モード → ピッチ連続送信
    }
  }
  ble::poll_stop();
  ctrl::poll();                   // B ボタン: 長押し=モード切替 / 一瞬押し=LED toggle or LUMOS/NOX
  wled::update(det.ready);

  handle_serial();

  // 200ms 間隔でシリアル出力 (デバッグ / 軸マウント確認用)。
  // IMU 読み失敗時は WHO_AM_I を出して診断できるようにする。
  uint32_t now = millis();
  if (now - last_print_ms > 200) {
    last_print_ms = now;
    if (ok) Serial.printf("|a|=%.2fg (ax=%.2f ay=%.2f az=%.2f)\n",
                          sqrtf(ax*ax + ay*ay + az*az), ax, ay, az);
    else    Serial.printf("[IMU] read fail, WHO_AM_I=0x%02X\n", imu::read_reg(0x0F));
  }

  delay(10);  // 100Hz サンプリング
}
