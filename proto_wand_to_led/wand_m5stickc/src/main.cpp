// proto_wand_to_led / wand_m5stickc / src/main.cpp
// 杖 (送信側): M5StickC + MPU6886 + NimBLE Advertising Beacon
// Phase 0: 振り検出 (|a| > 2.5g) → BLE adv 500ms burst → クールダウン 1s

#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "../../shared/beacon_protocol.h"

// ============================================================
// ジェスチャ判定パラメータ (実行時可変 + NVS 保存)
//   重力基準フリック検出 (Gravity-Referenced Flick Detection):
//     重力補償 (linear accel 抽出) + 鉛直射影 + 閾値判定
//   Serial コマンド (gshow/gth=/gratio=/gcool=/galpha=/gsave) で調整。
//   コンパイル時デフォルトは下記、NVS に保存があればそれを優先。
// ============================================================
namespace gcfg {
// --- デフォルト値 (#define 相当のチューニング基準) ---
constexpr float    DEF_FLICK_THRESHOLD_G = 1.2f;  // linear accel 閾値 [g]
constexpr float    DEF_UPDOWN_RATIO      = 0.6f;  // |鉛直成分|/|動き| の閾値 (0-1)
constexpr uint32_t DEF_COOLDOWN_MS       = 1000;  // トリガ間隔 [ms]
constexpr float    DEF_GRAV_ALPHA        = 0.02f; // 重力推定 EMA 係数 (小=ゆっくり)
constexpr float    DEF_STILL_BAND        = 0.15f; // 静止判定: ||accel|-1g| がこれ未満なら静止 [g]

// --- 実行時変数 (これを書き換えて挙動を変える) ---
float    flick_threshold_g = DEF_FLICK_THRESHOLD_G;
float    updown_ratio      = DEF_UPDOWN_RATIO;
uint32_t cooldown_ms       = DEF_COOLDOWN_MS;
float    grav_alpha        = DEF_GRAV_ALPHA;
float    still_band        = DEF_STILL_BAND;

Preferences prefs;
constexpr const char* NS = "wandg";  // NVS 名前空間

void set_defaults() {
  flick_threshold_g = DEF_FLICK_THRESHOLD_G;
  updown_ratio      = DEF_UPDOWN_RATIO;
  cooldown_ms       = DEF_COOLDOWN_MS;
  grav_alpha        = DEF_GRAV_ALPHA;
  still_band        = DEF_STILL_BAND;
}

void load() {
  prefs.begin(NS, true);  // read-only
  flick_threshold_g = prefs.getFloat("th",    DEF_FLICK_THRESHOLD_G);
  updown_ratio      = prefs.getFloat("ratio", DEF_UPDOWN_RATIO);
  cooldown_ms       = prefs.getULong("cool",  DEF_COOLDOWN_MS);
  grav_alpha        = prefs.getFloat("alpha", DEF_GRAV_ALPHA);
  still_band        = prefs.getFloat("band",  DEF_STILL_BAND);
  prefs.end();
}

void save() {
  prefs.begin(NS, false);  // read-write
  prefs.putFloat("th",    flick_threshold_g);
  prefs.putFloat("ratio", updown_ratio);
  prefs.putULong("cool",  cooldown_ms);
  prefs.putFloat("alpha", grav_alpha);
  prefs.putFloat("band",  still_band);
  prefs.end();
}

void print() {
  Serial.printf("[GCFG] th=%.2fg ratio=%.2f cool=%lums alpha=%.3f band=%.2fg\n",
                flick_threshold_g, updown_ratio,
                (unsigned long)cooldown_ms, grav_alpha, still_band);
}
}  // namespace gcfg

// ============================================================
// MPU6886 (M5StickC 内蔵 IMU, I2C: SDA=21, SCL=22)
// ============================================================
namespace imu {
constexpr uint8_t I2C_ADDR     = 0x68;
constexpr int     SDA_PIN      = 21;
constexpr int     SCL_PIN      = 22;
constexpr float   ACC_LSB_TO_G = 1.0f / 4096.0f;  // ±8g スケール

bool write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t read_reg(uint8_t reg) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool read_accel_burst(uint8_t* buf, size_t len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)len) != len) return false;
  for (size_t i = 0; i < len; i++) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }
  return true;
}

bool begin() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(10);
  if (read_reg(0x75) != 0x19) return false;  // WHO_AM_I
  write_reg(0x6B, 0x80);                     // Reset
  delay(100);
  write_reg(0x6B, 0x01);                     // Wake, auto clock
  delay(10);
  write_reg(0x1C, 0x10);                     // Accel FS = ±8g
  write_reg(0x1A, 0x03);                     // DLPF = 41Hz
  delay(10);
  return true;
}

// 3 軸加速度を g 単位で読む。成功 true。失敗 false
bool read_accel_g(float& ax_g, float& ay_g, float& az_g) {
  uint8_t raw[6];
  if (!read_accel_burst(raw, 6)) return false;
  int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
  ax_g = ax * ACC_LSB_TO_G;
  ay_g = ay * ACC_LSB_TO_G;
  az_g = az * ACC_LSB_TO_G;
  return true;
}
}  // namespace imu

// ============================================================
// BLE Advertising Beacon
// ============================================================
namespace ble {
NimBLEAdvertising* adv = nullptr;
uint32_t           adv_stop_at_ms = 0;
constexpr uint32_t ADV_BURST_MS   = 500;

void begin() {
  NimBLEDevice::init("Wand-Proto");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // +9dBm (最大に近い)
  adv = NimBLEDevice::getAdvertising();
  adv->setMinInterval(0x20);   // 20ms (BLE 仕様最小)
  adv->setMaxInterval(0x30);   // 30ms
  adv->setScanResponse(false);
  adv->setAdvertisementType(BLE_GAP_CONN_MODE_NON);  // 非接続 broadcast
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

  NimBLEAdvertisementData data;
  std::string mfg((const char*)&payload, sizeof(payload));
  data.setManufacturerData(mfg);
  adv->setAdvertisementData(data);

  if (adv_stop_at_ms != 0) adv->stop();
  adv->start();
  adv_stop_at_ms = millis() + ADV_BURST_MS;

  Serial.printf("[BLE] adv start seq=%u trig=0x%02X strength=%u target=%u\n",
                seq_counter, trigger_id, strength, (unsigned)target_id);
}

// loop() から定期呼び出し: burst 時間が過ぎたら adv 停止
void poll_stop() {
  if (adv_stop_at_ms != 0 && millis() >= adv_stop_at_ms) {
    adv->stop();
    adv_stop_at_ms = 0;
    Serial.println("[BLE] adv stopped");
  }
}
}  // namespace ble

// ============================================================
// ジェスチャ検出
//   重力ベクトルを「上」基準として、振り上げ/振り下げを判定:
//   - 上振り (linear accel が上向き) → LUMOS
//   - 下振り (linear accel が下向き) → NOX
//   - 上下成分が小さい強い振り       → SHAKE
// ============================================================
namespace wled { void flash(); }  // 前方宣言 (検出時フラッシュ用)

namespace detector {
uint32_t last_trigger_ms = 0;
// 重力推定 (低域フィルタ EMA)。初期値は汎用に z=+1g
float grav_x = 0.0f, grav_y = 0.0f, grav_z = 1.0f;

// 静止ゲート + ready 状態
//   |accel| が 1g 付近 = 静止 → この時だけ重力更新 (フリック混入を防ぐ)
//   静止が一定数続いたら ready=true (= 振ってよい合図、LED 点灯)
int  still_count = 0;
bool ready       = false;
constexpr int   READY_STILL_SAMPLES = 25;     // 約 0.25s (100Hz)
// 静止判定幅 gcfg::still_band を使用 (NVS 可変)

void update_gravity(float ax, float ay, float az) {
  float a_mag = sqrtf(ax*ax + ay*ay + az*az);
  if (fabsf(a_mag - 1.0f) < gcfg::still_band) {
    // ほぼ静止 → 重力を更新 (クリーンな重力が取れる)
    grav_x += gcfg::grav_alpha * (ax - grav_x);
    grav_y += gcfg::grav_alpha * (ay - grav_y);
    grav_z += gcfg::grav_alpha * (az - grav_z);
    if (still_count < READY_STILL_SAMPLES) still_count++;
    if (still_count >= READY_STILL_SAMPLES) ready = true;
  }
  // 動き中 (フリック等) → 重力は凍結。ready は維持
}

void check(float ax, float ay, float az) {
  uint32_t now = millis();

  // 重力方向 (= 上方向の単位ベクトル)。加速度計は静止時、上向き軸が +1g
  float gmag = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
  if (gmag < 0.1f) gmag = 0.1f;
  float ux = grav_x / gmag, uy = grav_y / gmag, uz = grav_z / gmag;

  // linear accel = 生 − 重力
  float lx = ax - grav_x, ly = ay - grav_y, lz = az - grav_z;
  float lmag = sqrtf(lx*lx + ly*ly + lz*lz);

  // linear を上方向に投影 (正=上振り, 負=下振り)
  float up_proj = lx*ux + ly*uy + lz*uz;

  if (lmag < gcfg::flick_threshold_g || (now - last_trigger_ms) <= gcfg::cooldown_ms) {
    return;  // 動きが弱い or クールダウン中
  }

  // strength: linear accel の強さを 0-255 にマップ (閾値→0, +2.8g→255)
  int s = (int)((lmag - gcfg::flick_threshold_g) * 91.0f);
  if (s < 0) s = 0; if (s > 255) s = 255;

  float ratio = fabsf(up_proj) / lmag;  // 上下成分の割合
  uint8_t trig;
  const char* name;
  if (ratio >= gcfg::updown_ratio && up_proj > 0) {
    trig = wand_beacon::TRIG_LUMOS; name = "LUMOS (up)";
  } else if (ratio >= gcfg::updown_ratio && up_proj < 0) {
    trig = wand_beacon::TRIG_NOX;   name = "NOX (down)";
  } else {
    trig = wand_beacon::TRIG_SHAKE; name = "SHAKE (other)";
  }

  // 物理ジェスチャは全機宛て (TARGET_ALL)
  ble::emit_beacon(trig, (uint8_t)s, wand_beacon::TARGET_ALL);
  last_trigger_ms = now;
  wled::flash();  // 内蔵 LED を一瞬光らせて検出をフィードバック
  Serial.printf("*** %s  lmag=%.2fg up_proj=%.2f ratio=%.2f strength=%d ***\n",
                name, lmag, up_proj, ratio, s);
}
}  // namespace detector

// ============================================================
// M5StickC 内蔵赤 LED (GPIO 10, active-low) で状態表示
//   A ボタン (GPIO 37) で LED フィードバックの ON/OFF をトグル。デフォルト OFF。
//   有効時:
//     静止 (ready)        → 消灯
//     収束中 (起動/動作中) → 点滅
//     ジェスチャ検出時      → 一瞬フラッシュ (75ms)
//   無効時: 常時消灯
// ============================================================
namespace wled {
constexpr int      PIN      = 10;   // 内蔵赤 LED (active-low)
constexpr int      BTN_A    = 37;   // M5StickC 前面 A ボタン (active-low, 外部プルアップ)
constexpr uint32_t FLASH_MS = 75;

bool     enabled        = false;    // デフォルト OFF (起動時は光らない)
uint32_t flash_until_ms = 0;
bool     last_btn       = true;     // HIGH = 非押下
uint32_t last_btn_ms    = 0;

void init() {
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);          // OFF
  pinMode(BTN_A, INPUT);            // GPIO37 は input-only、M5StickC 外部プルアップ
}

void flash() { if (enabled) flash_until_ms = millis() + FLASH_MS; }  // 検出時に呼ぶ

// A ボタン押下 (立下りエッジ + デバウンス) で enabled トグル
void poll_button() {
  bool b = (digitalRead(BTN_A) != 0);
  uint32_t now = millis();
  if (last_btn && !b && (now - last_btn_ms) > 250) {  // HIGH→LOW = 押下
    enabled = !enabled;
    last_btn_ms = now;
    Serial.printf("[LED] feedback %s\n", enabled ? "ON" : "OFF");
    if (!enabled) digitalWrite(PIN, HIGH);  // 無効化したら即消灯
  }
  last_btn = b;
}

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
// Serial コマンド (行単位パーサ)
//   トリガ: t/l/n/i/a 単独 → 全機宛て / "<id> <cmd>" → 特定機宛て (例 "2 l")
//   設定:   gshow / gth=<f> / gratio=<f> / gcool=<ms> / galpha=<f> / gband=<f> / gsave / gdefault
// ============================================================
void emit_named(uint8_t trig, uint16_t target) {
  Serial.printf("[SERIAL_CMD] trigger 0x%02X target=%u\n", trig, (unsigned)target);
  ble::emit_beacon(trig, 200, target);
  detector::last_trigger_ms = millis();
}

void handle_line(char* line) {
  while (*line == ' ') line++;
  // 末尾空白/CR 除去
  int n = strlen(line);
  while (n > 0 && (line[n-1] == ' ' || line[n-1] == '\r' || line[n-1] == '\n')) line[--n] = 0;
  if (n == 0) return;

  // --- 設定コマンド ---
  if (strcmp(line, "gshow") == 0)       { gcfg::print(); return; }
  if (strcmp(line, "gsave") == 0)       { gcfg::save(); Serial.println("[GCFG] saved"); return; }
  if (strcmp(line, "gdefault") == 0)    { gcfg::set_defaults(); Serial.println("[GCFG] defaults (gsave で永続化)"); gcfg::print(); return; }
  if (strncmp(line, "gth=", 4) == 0)    { gcfg::flick_threshold_g = atof(line+4); gcfg::print(); return; }
  if (strncmp(line, "gratio=", 7) == 0) { gcfg::updown_ratio = atof(line+7); gcfg::print(); return; }
  if (strncmp(line, "gcool=", 6) == 0)  { gcfg::cooldown_ms = (uint32_t)atol(line+6); gcfg::print(); return; }
  if (strncmp(line, "galpha=", 7) == 0) { gcfg::grav_alpha = atof(line+7); gcfg::print(); return; }
  if (strncmp(line, "gband=", 6) == 0)  { gcfg::still_band = atof(line+6); gcfg::print(); return; }

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
    case 'i': emit_named(wand_beacon::TRIG_INCENDIO, target);  break;
    case 'a': emit_named(wand_beacon::TRIG_AGUAMENTI, target); break;
    default:
      Serial.println("[CMD] t/l/n/i/a | <id> <cmd> | gshow/gth=/gratio=/gcool=/galpha=/gsave/gdefault");
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
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Wand Beacon (gesture) ===");
  Serial.println("Up-flick=LUMOS / Down-flick=NOX / Other-strong=SHAKE");
  Serial.println("Adv burst: 500ms, Cooldown: 1s");

  wled::init();   // M5StickC 内蔵 LED (ready 表示用)

  if (!imu::begin()) {
    Serial.println("[FATAL] MPU6886 init failed");
    while (1) delay(1000);
  }
  Serial.println("[IMU] MPU6886 OK");

  gcfg::load();   // NVS からジェスチャ閾値を読み込み (無ければデフォルト)
  gcfg::print();
  Serial.println("[CMD] t/l/n/i/a | <id> <cmd> | gshow/gth=/gratio=/gcool=/galpha=/gsave");

  // 重力推定を初期化 (起動直後の静止姿勢で 1 回読む)
  {
    float ax, ay, az;
    if (imu::read_accel_g(ax, ay, az)) {
      detector::grav_x = ax; detector::grav_y = ay; detector::grav_z = az;
    }
  }

  ble::begin();
  Serial.println("[BLE] init OK, waiting for gesture...");
}

void loop() {
  static uint32_t last_print_ms = 0;
  float ax, ay, az;
  if (!imu::read_accel_g(ax, ay, az)) {
    delay(10);
    return;
  }
  float a = sqrtf(ax*ax + ay*ay + az*az);

  detector::update_gravity(ax, ay, az);
  detector::check(ax, ay, az);
  ble::poll_stop();
  wled::poll_button();            // A ボタンで LED フィードバック ON/OFF
  wled::update(detector::ready);

  // Serial コマンド処理 (行単位)
  //   トリガ:   t/l/n/i/a (全機宛て)、宛先指定は "<id> <cmd>" 例 "2 l"
  //   設定:     gshow / gth=<f> / gratio=<f> / gcool=<ms> / galpha=<f> / gsave / gdefault
  handle_serial();

  // 200ms 間隔で IMU 値をシリアルに出力 (デバッグ)
  uint32_t now = millis();
  if (now - last_print_ms > 200) {
    last_print_ms = now;
    Serial.printf("|a|=%.2fg\n", a);
  }

  delay(10);  // 100Hz サンプリング
}
