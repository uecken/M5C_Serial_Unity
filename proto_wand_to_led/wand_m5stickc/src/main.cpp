// proto_wand_to_led / wand_m5stickc / src/main.cpp
// 杖 (送信側): M5StickC + MPU6886 + NimBLE Advertising Beacon
// Phase 0: 振り検出 (|a| > 2.5g) → BLE adv 500ms burst → クールダウン 1s

#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include "device_config.h"
#include "../../shared/beacon_protocol.h"
#include "../../shared/wand_common.h"
#include "../../shared/wand_gesture.h"

// ジェスチャ判定コア (重力基準フリック検出 + Wingardium) は shared/wand_gesture.h に集約し
// M5StickC / XIAO nRF52840 Sense で共用する。BLE 送信(ble::emit_beacon) と LED フラッシュ
// (wled::flash) は setup() で関数ポインタ注入。閾値は gcfg が det.cfg を読み書きして永続化。
wand_gesture::Detector det;

// ============================================================
// ジェスチャ判定パラメータ (実行時可変 + NVS 保存)
//   重力基準フリック検出 (Gravity-Referenced Flick Detection):
//     重力補償 (linear accel 抽出) + 鉛直射影 + 閾値判定
//   Serial コマンド (gshow/gth=/gratio=/gcool=/galpha=/gsave) で調整。
//   コンパイル時デフォルトは下記、NVS に保存があればそれを優先。
// ============================================================
namespace gcfg {
// --- デフォルト値 (#define 相当のチューニング基準) ---
constexpr float    DEF_FLICK_THRESHOLD_G = 1.5f;  // linear accel 閾値 [g] (シビア化: 1.2→1.5)
constexpr float    DEF_UPDOWN_RATIO      = 0.75f; // |鉛直成分|/|動き| の閾値 (シビア化: 0.6→0.75)
constexpr uint32_t DEF_COOLDOWN_MS       = 1000;  // トリガ間隔 [ms]
constexpr float    DEF_GRAV_ALPHA        = 0.02f; // 重力推定 EMA 係数 (小=ゆっくり)
constexpr float    DEF_STILL_BAND        = 0.15f; // 静止判定: ||accel|-1g| がこれ未満なら静止 [g]
constexpr uint32_t DEF_WOM_THR_MG        = IMU_WOM_THRESHOLD_MG; // device_config.h を一元ソースに

// --- 実行時変数。ジェスチャ閾値は共通コアの det.cfg を直接読み書きする ---
uint32_t wom_thr_mg = DEF_WOM_THR_MG;  // WOM wake 閾値 (M5 固有、Config 外。deep sleep 時に適用)

Preferences prefs;
constexpr const char* NS = "wandg";  // NVS 名前空間

void set_defaults() {
  det.cfg.flick_threshold_g = DEF_FLICK_THRESHOLD_G;
  det.cfg.updown_ratio      = DEF_UPDOWN_RATIO;
  det.cfg.cooldown_ms       = DEF_COOLDOWN_MS;
  det.cfg.grav_alpha        = DEF_GRAV_ALPHA;
  det.cfg.still_band        = DEF_STILL_BAND;
  wom_thr_mg                = DEF_WOM_THR_MG;
}

void load() {
  prefs.begin(NS, true);  // read-only
  det.cfg.flick_threshold_g = prefs.getFloat("th",    DEF_FLICK_THRESHOLD_G);
  det.cfg.updown_ratio      = prefs.getFloat("ratio", DEF_UPDOWN_RATIO);
  det.cfg.cooldown_ms       = prefs.getULong("cool",  DEF_COOLDOWN_MS);
  det.cfg.grav_alpha        = prefs.getFloat("alpha", DEF_GRAV_ALPHA);
  det.cfg.still_band        = prefs.getFloat("band",  DEF_STILL_BAND);
  wom_thr_mg                = prefs.getULong("womthr",DEF_WOM_THR_MG);
  prefs.end();
}

void save() {
  prefs.begin(NS, false);  // read-write
  prefs.putFloat("th",    det.cfg.flick_threshold_g);
  prefs.putFloat("ratio", det.cfg.updown_ratio);
  prefs.putULong("cool",  det.cfg.cooldown_ms);
  prefs.putFloat("alpha", det.cfg.grav_alpha);
  prefs.putFloat("band",  det.cfg.still_band);
  prefs.putULong("womthr",wom_thr_mg);
  prefs.end();
}

void print() {
  Serial.printf("[GCFG] th=%.2fg ratio=%.2f cool=%lums alpha=%.3f band=%.2fg womthr=%lumg\n",
                det.cfg.flick_threshold_g, det.cfg.updown_ratio,
                (unsigned long)det.cfg.cooldown_ms, det.cfg.grav_alpha, det.cfg.still_band,
                (unsigned long)wom_thr_mg);
}
}  // namespace gcfg

// ============================================================
// MPU6886 (M5StickC 内蔵 IMU, I2C: SDA=21, SCL=22)
// ============================================================
namespace imu {
constexpr uint8_t I2C_ADDR     = IMU_I2C_ADDR;
constexpr int     SDA_PIN      = IMU_I2C_SDA;
constexpr int     SCL_PIN      = IMU_I2C_SCL;
constexpr float   ACC_LSB_TO_G = 1.0f / IMU_ACC_LSB_PER_G;  // device_config.h

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

#if ENABLE_IMU_WOM_WAKE
// MPU6886 を Wake-on-Motion (WOM) モードに設定する (deep sleep 前に呼ぶ)
//   INT は active-low + latch で出力 → ESP32 ext0 level=0 で wake
//   threshold_mg: 動き検出閾値 [mg] (1 LSB ≈ 3.9mg)
void enable_wom(int threshold_mg) {
  uint8_t thr = (uint8_t)(threshold_mg / 3.9f);   // mg → LSB
  if (thr < 1) thr = 1;

  pinMode(IMU_INT_PIN, INPUT);   // GPIO35 (INT 監視用、診断 + ext0 wake)

  write_reg(0x6B, 0x00);   // PWR_MGMT_1: sleep 解除, 内部クロック
  delay(10);
  write_reg(0x6C, 0x07);   // PWR_MGMT_2: gyro 無効, accel 有効
  write_reg(0x1D, 0x01);   // ACCEL_CONFIG2: accel DLPF 有効
  write_reg(0x20, thr);    // ACCEL_WOM_X_THR
  write_reg(0x21, thr);    // ACCEL_WOM_Y_THR
  write_reg(0x22, thr);    // ACCEL_WOM_Z_THR
  write_reg(0x69, 0xC0);   // ACCEL_INTEL_CTRL: EN=1, MODE=1(前サンプル比較), OR
  write_reg(0x38, 0xE0);   // INT_ENABLE: WOM_X/Y/Z_INT_EN
  write_reg(0x37, 0xA0);   // INT_PIN_CFG: active-low + latch
  // ※ SMPLRT_DIV(0x19) で ODR を落とすと WOM 振り検出が効かなくなった (fires 4→1) ため
  //    デフォルト ODR のまま使う (fires=4 で振り検出が動いていた構成)
  // 初回の誤 WOM (前サンプル無し) をクリアして INT を idle(HIGH) に戻す (CYCLE 前)
  delay(100);
  (void)read_reg(0x3A);
  delay(20);
  (void)read_reg(0x3A);
  write_reg(0x6B, 0x20);   // PWR_MGMT_1: CYCLE=1 (低電力 accel cycle) を最後に
  Serial.printf("[WOM] cfg done thr=%dmg(%dLSB), INT(GPIO%d)=%d (1=idle/0=asserted)\n",
                threshold_mg, thr, IMU_INT_PIN, digitalRead(IMU_INT_PIN));
}
#endif
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
  adv->setMaxInterval(0x20);   // 20ms 固定 (浮遊モードのピッチ更新を最速で電波に乗せる)
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
// M5StickC 内蔵赤 LED (GPIO 10, active-low) で状態表示
//   フィードバック表示の ON/OFF は ctrl 名前空間 (B ボタン) が管理。デフォルト OFF。
//   (A ボタンは deep sleep からの wake 専用)
//   有効時:
//     静止 (ready)        → 消灯
//     収束中 (起動/動作中) → 点滅
//     ジェスチャ検出時      → 一瞬フラッシュ (75ms)
//   無効時: 常時消灯
// ============================================================
namespace wled {
constexpr int      PIN      = BUILTIN_LED_PIN;  // 内蔵赤 LED (active-low)
constexpr uint32_t FLASH_MS = 75;

bool     enabled        = false;    // デフォルト OFF (起動時は光らない)
uint32_t flash_until_ms = 0;

void init() {
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);          // OFF
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
// 操作モード制御 (B ボタン: GPIO39, active-low, input-only)
//   B 長押し (>=1s)  → 手動操作モード / モーションモード をトグル
//                      切替確認に赤 LED 点滅 (モーション=2回 / 手動=3回)
//   B 一瞬押し:
//     モーションモード → 内蔵 LED フィードバック表示 ON/OFF (従来機能)
//     手動操作モード   → LUMOS / NOX を交互に beacon 送信
//   手動操作モード中はジェスチャ判定 (check / wingardium) を停止する。
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

// deep sleep を跨いでモードを保持する RTC メモリ (電源 OFF/リセットでは失われる)。
//   wake cause で deep sleep 復帰か再起動かを判定するため、この値は
//   「deep sleep 復帰時のみ」採用する (再起動時はモーションモードに固定)。
RTC_DATA_ATTR bool rtc_manual_mode = false;

void init() {
  pinMode(BTN_B, INPUT);   // GPIO39 は input-only、M5StickC 外部プルアップ
  // deep sleep から Button B で wake した場合、その押下が起動時にまだ続いていることがある。
  // これを「一瞬押し」として誤検出すると LED フィードバックが勝手に ON になり、以後 Motion で
  // 光ってしまう。起動時に既に押されていれば消化済み(long_fired)扱いにして、離しても何も起こさない。
  if (digitalRead(BTN_B) == 0) {   // LOW = 押下中 (= wake 押下の継続)
    btn_down    = true;
    press_start = millis();
    long_fired  = true;            // 短押し/長押しのどちらも発火させない
  }
}

// deep sleep に入る直前に呼ぶ: 現在のモードを RTC メモリへ退避
void save_mode_for_sleep() { rtc_manual_mode = manual_mode; }

// 起動時に呼ぶ: deep sleep 復帰なら退避モードを復元、それ以外(再起動)はモーションモード
void restore_mode(bool from_deep_sleep) {
  manual_mode = from_deep_sleep ? rtc_manual_mode : false;
  if (manual_mode) next_is_lumos = true;   // 手動復帰時は次の一瞬押しを LUMOS から
  Serial.printf("[MODE] boot in %s (%s)\n",
                manual_mode ? "MANUAL" : "MOTION",
                from_deep_sleep ? "restored from deep sleep" : "reboot/power-on default");
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
    next_is_lumos = true;                // 手動モードに入ったら次は LUMOS から
    det.levitation_until = 0;      // 浮遊モードが残っていれば解除
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
    // モーションモード: 従来どおり手元 LED フィードバック ON/OFF
    wled::enabled = !wled::enabled;
    Serial.printf("[LED] feedback %s\n", wled::enabled ? "ON" : "OFF");
    if (!wled::enabled) digitalWrite(wled::PIN, HIGH);  // 無効化したら即消灯
  }
}

// 毎ループ呼ぶ: 押下を計測して長押し/一瞬押しを振り分ける
void poll() {
  bool     pressed = (digitalRead(BTN_B) == 0);   // active-low: LOW=押下
  uint32_t now     = millis();

  if (pressed && !btn_down) {                 // 立下り = 押下開始
    btn_down    = true;
    press_start = now;
    long_fired  = false;
    det.last_motion_ms = now;           // ボタン操作中は sleep させない
  } else if (pressed && btn_down) {           // 押下継続
    if (!long_fired && (now - press_start) >= LONG_PRESS_MS) {
      long_fired = true;                      // 離す前に長押し成立 → モード切替
      on_long_press();
      det.last_motion_ms = millis();    // blink の delay 分を補正
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
// 省電力 (ESP32 deep sleep)
//   静止が SLEEP_AFTER_SEC (共通 30s) 続いたら deep sleep。
//   wake = Button A (ext0, LOW)。wake は実質リブート → setup() から再開。
// ============================================================
namespace pm {
bool enabled = true;   // sleep 有効 (テスト中は serial "sleep=0" で無効化可)

void enter_deep_sleep() {
  ctrl::save_mode_for_sleep();               // 現在のモードを RTC メモリへ退避 (wake 時に復元)
  digitalWrite(wled::PIN, HIGH);             // LED 消灯
  ble::adv->stop();                          // adv 停止
#if ENABLE_IMU_WOM_WAKE
  // 実験的: MPU6886 WOM で wake (IMU INT → GPIO35, active-low) + Button A fallback
  Serial.printf("[PM] %lus 静止 → deep sleep. wake=motion(WOM GPIO%d) or Button A(GPIO%d)\n",
                (unsigned long)wand_common::SLEEP_AFTER_SEC, IMU_INT_PIN, SLEEP_WAKE_BUTTON);
  Serial.flush();
  imu::enable_wom((int)gcfg::wom_thr_mg);   // NVS 可変の WOM 閾値を適用
  // WOM の「初回サンプル誤発火」(前サンプル0 vs 現在1g で閾値超え) を消化:
  //   INT が安定して idle(HIGH) になるまでクリア。これをしないと sleep 直後に
  //   ext0 が即トリガして即 wake してしまう。
  {
    uint32_t t0 = millis(), stableSince = millis();
    while (millis() - t0 < 1500) {                 // 最大 1.5s
      if (digitalRead(IMU_INT_PIN) == 0) {          // 誤発火 (INT LOW)
        (void)imu::read_reg(0x3A);                  // クリア → INT idle へ
        stableSince = millis();
      }
      if (millis() - stableSince > 250) break;      // 250ms 連続 idle → 安定
      delay(10);
    }
    Serial.printf("[PM] WOM settled, INT=%d, sleeping\n", digitalRead(IMU_INT_PIN));
    Serial.flush();
  }
  esp_sleep_enable_ext0_wakeup((gpio_num_t)IMU_INT_PIN, 0);  // ext0: WOM INT (motion) active-low
  // + Button A を fallback wake に (WOM 取りこぼし対策)。
  //   ※ ESP32(PICO-D4) は active-low の独立 level wake が ext0+ext1 の計 2 本まで
  //     (ext1 は ALL_LOW/ANY_HIGH のみで active-low の OR 不可)。WOM(ext0)+Button A(ext1) で
  //     2 本使用済 → Button B は WOM build では wake 不可。両ボタン wake が要るなら
  //     WOM 無効の env:m5stick-c を使う (そちらは Button A+B 両対応)。
  esp_sleep_enable_ext1_wakeup(1ULL << SLEEP_WAKE_BUTTON, ESP_EXT1_WAKEUP_ALL_LOW);
#else
  // 確実: Button A (ext0) + Button B (ext1) のどちらでも wake
  Serial.printf("[PM] %lus 静止 → deep sleep. wake=Button A(GPIO%d) or Button B(GPIO%d)\n",
                (unsigned long)wand_common::SLEEP_AFTER_SEC, BUTTON_A_PIN, BUTTON_B_PIN);
  Serial.flush();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_A_PIN, 0);                    // ext0: Button A
  esp_sleep_enable_ext1_wakeup(1ULL << BUTTON_B_PIN, ESP_EXT1_WAKEUP_ALL_LOW);  // ext1: Button B
#endif
  esp_deep_sleep_start();                    // 復帰しない (次回 wake = リセット)
}

void poll() {
  if (!enabled) return;
  // 起動直後やトリガ直後は last_motion_ms が新しいので即 sleep しない
  uint32_t idle = millis() - det.last_motion_ms;
  if (idle > wand_common::SLEEP_AFTER_SEC * 1000UL) {
    enter_deep_sleep();
  }
}
}  // namespace pm

// ============================================================
// Serial コマンド (行単位パーサ)
//   トリガ: t/l/n/i/a 単独 → 全機宛て / "<id> <cmd>" → 特定機宛て (例 "2 l")
//   設定:   gshow / gth=<f> / gratio=<f> / gcool=<ms> / galpha=<f> / gband=<f> / gsave / gdefault
// ============================================================
void emit_named(uint8_t trig, uint16_t target) {
  Serial.printf("[SERIAL_CMD] trigger 0x%02X target=%u\n", trig, (unsigned)target);
  ble::emit_beacon(trig, 200, target);
  det.last_trigger_ms = millis();
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
  if (strncmp(line, "gth=", 4) == 0)    { det.cfg.flick_threshold_g = atof(line+4); gcfg::print(); return; }
  if (strncmp(line, "gratio=", 7) == 0) { det.cfg.updown_ratio = atof(line+7); gcfg::print(); return; }
  if (strncmp(line, "gcool=", 6) == 0)  { det.cfg.cooldown_ms = (uint32_t)atol(line+6); gcfg::print(); return; }
  if (strncmp(line, "galpha=", 7) == 0) { det.cfg.grav_alpha = atof(line+7); gcfg::print(); return; }
  if (strncmp(line, "gband=", 6) == 0)  { det.cfg.still_band = atof(line+6); gcfg::print(); return; }
  if (strncmp(line, "wom=", 4) == 0)    { gcfg::wom_thr_mg = (uint32_t)atol(line+4); gcfg::print(); return; }
  if (strncmp(line, "sleep=", 6) == 0)  { pm::enabled = (atoi(line+6) != 0); Serial.printf("[PM] sleep %s\n", pm::enabled ? "ON" : "OFF"); return; }
  if (strcmp(line, "dsleep") == 0)      { Serial.println("[PM] forced deep sleep (test)"); pm::enter_deep_sleep(); return; }  // 即 deep sleep (WOM テスト用)
#if ENABLE_IMU_WOM_WAKE
  if (strcmp(line, "womtest") == 0) {
    // 眠らずに WOM を設定し INT(GPIO35) を 15s 監視。振って INT が 0(asserted) に落ちるか確認
    Serial.println("[WOMTEST] WOM 設定 → INT(GPIO35) を 15s 監視。振って INT=0 になれば WOM 発火 OK");
    imu::enable_wom((int)gcfg::wom_thr_mg);
    uint32_t tEnd = millis() + 15000;
    int last = -1, fires = 0;
    while (millis() < tEnd) {
      int v = digitalRead(IMU_INT_PIN);
      if (v != last) {
        Serial.printf("[WOMTEST] INT=%d (t=%lu)\n", v, (unsigned long)millis());
        if (v == 0) { fires++; (void)imu::read_reg(0x3A); }  // asserted を数えてクリア(再発火可能に)
        last = v;
      }
      delay(5);
    }
    Serial.printf("[WOMTEST] 終了 fires=%d。通常モードに復帰\n", fires);
    imu::begin();  // 通常 accel モードへ戻す
    return;
  }
#endif

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
    case 'w': det.force_levitation(); break;  // Wingardium 浮遊モードを強制開始 (8s ピッチ連続送信)
    default:
      Serial.println("[CMD] t/l/n/i/a/e/w | <id> <cmd> | gshow/gth=/gratio=/gcool=/galpha=/gsave/gdefault");
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
  Serial.println("Up-flick=LUMOS / Down-flick=NOX / Thrust+Y=EXPECTO / Side-swing=INCENDIO");
  Serial.println("Hold-up(0.8s)=WINGARDIUM levitation (pitch stream)");
  Serial.println("Btn B long(>=1s)=MOTION<->MANUAL toggle (blink x2/x3) / short: MANUAL=LUMOS/NOX, MOTION=LED");
  Serial.println("Adv burst: 500ms, Cooldown: 1s");

  // wake 要因を先に判定 (deep sleep 復帰では起動点滅しない = wake/Motion で光らせない)
  esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
  bool from_deep_sleep = (wake_cause != ESP_SLEEP_WAKEUP_UNDEFINED);

  wled::init();        // M5StickC 内蔵 LED (ready 表示用)
  ctrl::init();        // B ボタン (モード切替 + 手動 LUMOS/NOX)
  if (!from_deep_sleep) wled::boot_blink();  // 起動点滅は電源 ON/リセット時のみ (wake 時は無点灯)

  if (!imu::begin()) {
    Serial.println("[FATAL] MPU6886 init failed");
    while (1) delay(1000);
  }
  Serial.println("[IMU] MPU6886 OK");

  // ジェスチャ検出器に BLE 送信 / LED フラッシュを注入し、軸マウントを設定 (共通コア)
  det.begin(&ble::emit_beacon, &wled::flash);
  det.cfg.forward_axis = WAND_FORWARD;
  det.cfg.right_axis   = WAND_RIGHT;

  gcfg::load();   // NVS からジェスチャ閾値を det.cfg に読み込み (無ければデフォルト)
  gcfg::print();
  Serial.println("[CMD] t/l/n/i/a/e/w | <id> <cmd> | gshow/gth=/gratio=/gcool=/galpha=/gsave");

  // 重力推定を初期化 (起動直後の静止姿勢で 1 回読む)
  {
    float ax, ay, az;
    if (imu::read_accel_g(ax, ay, az)) {
      det.grav_x = ax; det.grav_y = ay; det.grav_z = az;
    }
  }

  ble::begin();
  Serial.println("[BLE] init OK, waiting for gesture...");

  // sleep タイマ起点を現在に (起動直後に即 sleep しないように)
  det.last_motion_ms = millis();
  {
#if ENABLE_IMU_WOM_WAKE
    if      (wake_cause == ESP_SLEEP_WAKEUP_EXT0) Serial.println("[PM] woke by motion (WOM)");
    else if (wake_cause == ESP_SLEEP_WAKEUP_EXT1) Serial.println("[PM] woke by Button A (WOM fallback)");
#else
    if      (wake_cause == ESP_SLEEP_WAKEUP_EXT0) Serial.println("[PM] woke by Button A");
    else if (wake_cause == ESP_SLEEP_WAKEUP_EXT1) Serial.println("[PM] woke by Button B");
#endif
    // deep sleep 復帰なら直前のモードを復元、通常の再起動/電源 ON はモーションモードから起動。
    ctrl::restore_mode(from_deep_sleep);
  }
}

void loop() {
  static uint32_t last_print_ms = 0;
  float ax, ay, az;
  if (!imu::read_accel_g(ax, ay, az)) {
    delay(10);
    return;
  }
  float a = sqrtf(ax*ax + ay*ay + az*az);

  det.update_gravity(ax, ay, az);   // 重力推定はモードに関係なく継続 (復帰時に即 ready)
  // 手動操作モード中はジェスチャ判定を全停止 (B ボタン操作だけで魔法を出す)
  if (!ctrl::manual_mode) {
    // 浮遊モード中は通常ジェスチャ判定を止める (上下の浮遊操作で LUMOS/NOX が誤発火しないよう)
    if (!det.is_levitating()) det.check(ax, ay, az);
    det.poll_wingardium(ax, ay, az);  // 上向き保持で浮遊モード → ピッチ連続送信
  }
  ble::poll_stop();
  ctrl::poll();                   // B ボタン: 長押し=モード切替 / 一瞬押し=LED toggle or LUMOS/NOX
  wled::update(det.ready);
  pm::poll();                     // 30s 静止で deep sleep (wake=Button A)

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
