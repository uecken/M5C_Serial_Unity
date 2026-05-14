// Burst Motion - main_v2.cpp
// Phase 1 MVP: IMU 読取 → Mahony → JSON Lines で sensor stream
// BLE HID 接続は begin だけ行い、actual HID 出力は Phase 2 で統合
//
// エントリーポイント: [env:m5stick-c-v2] / [env:m5atom-s3-v2]

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <math.h>

#include "core/types.hpp"
#include "core/MahonyFilter.hpp"
#include "core/TriggerEngine.hpp"
#include "core/Profile.hpp"
#include "hid/BleHidSink.hpp"
#include "transport/SerialJsonLine.hpp"
#include "transport/BleNusServer.hpp"

#if defined(BOARD_M5STICKC)
  #include "hal/esp32/ImuMpu6886.hpp"
  #include "hal/esp32/Axp192.hpp"
  #include "hal/esp32/M5StickCDisplay.hpp"
  #include "hal/esp32/ButtonsGpio.hpp"
#elif defined(BOARD_M5ATOM_S3)
  // TODO: Phase 4 で M5Atom S3 の IMU ドライバ追加
  #error "BOARD_M5ATOM_S3 IMU driver not yet implemented"
#endif

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 921600
#endif

#ifndef FW_VERSION
#define FW_VERSION "2.0.0-dev"
#endif
#ifndef FW_PHASE
#define FW_PHASE "5.39"
#endif
// __DATE__ / __TIME__ はビルド時に自動埋め込まれる (例: "Apr 26 2026" "00:24:36")
#define FW_BUILD __DATE__ " " __TIME__

using namespace BurstMotion;

// =========================================================
// グローバル状態
// =========================================================
static SerialJsonLine g_serial;
static BleNusServer g_ble_nus;
static MahonyFilter g_mahony(2.0f, 0.0f);
static TriggerEngine g_engine;
static BleHidSink g_ble_hid("Burst Motion");
static bool g_ble_nus_started = false;

#if defined(BOARD_M5STICKC)
static ImuMpu6886 g_imu;
static Axp192 g_axp;
static M5StickCDisplay g_lcd;
static ButtonsGpio g_buttons;
static bool g_axp_ok = false;
static bool g_lcd_ok = false;
static uint32_t g_last_lcd_update_ms = 0;
static uint8_t g_battery_percent = 0;
static float g_battery_voltage = 0;
static bool g_battery_charging = false;
#endif

static SensorState g_sensor_state = {};
static uint32_t g_last_imu_ms = 0;
static uint32_t g_last_stream_ms = 0;
static uint16_t g_stream_rate_hz = 0;  // 0 = OFF
static uint32_t g_boot_ms = 0;

// Phase 5.39.2.6: Button sim override (自動テスト用、btn.sim コマンドで制御)
//   g_btn_sim_mask の bit が 1 の桁は g_btn_sim_value で上書き、0 の桁は物理 GPIO 値を維持
//   起動時は全て 0 (= sim 無効、物理値そのまま)
static uint16_t g_btn_sim_mask  = 0;
static uint16_t g_btn_sim_value = 0;
static HidOutputMode g_output_mode = HidOutputMode::OUT_BLE;
static bool g_ble_hid_started = false;
static bool g_imu_ok = false;

// Phase 5.32: Built-in 動作モード切替
// ENGINE = 既存 TriggerEngine (rule ベース、profile 適用)
// MOUSE  = ハードコードのエアマウス (Btn3=左/Btn2=右/同時押し=ホイール、IMU→cursor)
enum class DeviceMode : uint8_t { ENGINE = 0, MOUSE = 1 };
static DeviceMode g_device_mode = DeviceMode::ENGINE;
struct MouseModeRuntime {
    bool left_held = false;
    bool right_held = false;
    bool wheel_active = false;
    float wheel_accum = 0.0f;
    uint32_t last_ms = 0;
};
static MouseModeRuntime g_mouse_rt;

// Gyro bias (起動時自動キャリブレーション、deg/s 単位の rad/s 換算)
static float g_gyro_bias_rad[3] = {0, 0, 0};
static bool g_gyro_calibrated = false;

// 6 点 accel キャリブレーション (各軸 ±方向の重力測定値、bias + scale 補正)
// orientations: 0=+Z (LCD up), 1=-Z (LCD down), 2=+X (right side up), 3=-X (left), 4=+Y (top up), 5=-Y (bottom up)
static struct {
    bool active;
    int step;          // 現在のステップ (0-5)
    int samples_taken;
    float sum[3];      // 累積加速度 [m/s^2]
    float face_avg[6][3];  // 各 face 平均値
    bool face_done[6];
} g_calib6 = {};

// 加速度個体差補正 (6 点キャリブ後、Phase 5.9 で NVS 永続化対応)
static float g_accel_bias[3] = {0, 0, 0};
static float g_accel_scale[3] = {1, 1, 1};
static bool g_accel_calibrated = false;

// NVS 永続化ヘルパ (Preferences、bm_calib namespace)
static void saveCalibToNvs() {
    Preferences p;
    if (!p.begin("bm_calib", false)) return;
    p.putBytes("bias",  g_accel_bias,  sizeof(g_accel_bias));
    p.putBytes("scale", g_accel_scale, sizeof(g_accel_scale));
    p.putBool("done", true);
    p.end();
}
static void loadCalibFromNvs() {
    Preferences p;
    if (!p.begin("bm_calib", true)) return;
    if (p.getBool("done", false)) {
        if (p.getBytesLength("bias")  == sizeof(g_accel_bias))  p.getBytes("bias",  g_accel_bias,  sizeof(g_accel_bias));
        if (p.getBytesLength("scale") == sizeof(g_accel_scale)) p.getBytes("scale", g_accel_scale, sizeof(g_accel_scale));
        g_accel_calibrated = true;
    }
    p.end();
}

// =========================================================
// Phase 5.32: Device mode (engine / mouse) の NVS 永続化
// =========================================================
static void saveDeviceModeNvs() {
    Preferences p;
    if (p.begin("bm_mode", false)) {
        p.putUChar("mode", (uint8_t)g_device_mode);
        p.end();
    }
}
static void loadDeviceModeNvs() {
    Preferences p;
    if (p.begin("bm_mode", true)) {
        if (p.isKey("mode")) {
            g_device_mode = (DeviceMode)p.getUChar("mode", 0);
        }
        p.end();
    }
}

// =========================================================
// Phase 5.32: ハードコード Mouse モード tick
// 入力: SensorState (IMU + buttons_bitmap)
// 出力: BLE HID Mouse (move, press/release, wheel)
//
// マッピング (M5StickC、Phase 5.30 の Btn 命名に従う):
//   Btn3 (G26、一番手前) のみ → 左クリック HOLD
//   Btn2 (G36) のみ          → 右クリック HOLD
//   Btn3 + Btn2 同時押し     → ホイール (Pitch ジャイロ → 上下スクロール)
// 連続カーソル: ジャイロ Yaw → dx、Pitch → dy (deadzone + sensitivity)
//   ボタン無関係に常時動く (PC 一般のマウスのデフォルト動作)
// =========================================================
static void runMouseModeTick(const SensorState& s) {
    if (!g_ble_hid.isConnected() || !g_ble_hid.isEnabled()) {
        // 未接続/disabled 中は内部状態をリセットし何もしない
        g_mouse_rt.left_held = false;
        g_mouse_rt.right_held = false;
        g_mouse_rt.wheel_active = false;
        g_mouse_rt.wheel_accum = 0.0f;
        return;
    }
    // ボタン状態 (M5StickC 想定: bitmap bit0=Btn1, bit1=Btn2, bit2=Btn3)
    bool btn3 = (s.buttons_bitmap >> 2) & 1;
    bool btn2 = (s.buttons_bitmap >> 1) & 1;
    bool both = btn3 && btn2;

    uint32_t now = s.timestamp_ms;
    float dt = (g_mouse_rt.last_ms == 0) ? 0.01f : ((now - g_mouse_rt.last_ms) * 0.001f);
    if (dt > 0.5f || dt < 0.0f) dt = 0.01f;
    g_mouse_rt.last_ms = now;

    if (both) {
        // 両押し: 左右クリックを離してホイールモード
        if (g_mouse_rt.left_held)  { g_ble_hid.releaseMouseButton(MOUSE_LEFT);  g_mouse_rt.left_held  = false; }
        if (g_mouse_rt.right_held) { g_ble_hid.releaseMouseButton(MOUSE_RIGHT); g_mouse_rt.right_held = false; }
        // Pitch ジャイロ (deg/s) を accumulate して 30 deg ごとにホイール ±1 ノッチ
        constexpr float WHEEL_DEG_PER_NOTCH = 30.0f;
        g_mouse_rt.wheel_accum += s.gyro[1] * dt / WHEEL_DEG_PER_NOTCH;
        int8_t notches = 0;
        while (g_mouse_rt.wheel_accum >= 1.0f) { notches++; g_mouse_rt.wheel_accum -= 1.0f; }
        while (g_mouse_rt.wheel_accum <= -1.0f) { notches--; g_mouse_rt.wheel_accum += 1.0f; }
        if (notches != 0) g_ble_hid.moveMouse(0, 0, notches);
        g_mouse_rt.wheel_active = true;
        return;
    }
    // wheel モードを抜けた瞬間 accumulator リセット
    if (g_mouse_rt.wheel_active) {
        g_mouse_rt.wheel_accum = 0.0f;
        g_mouse_rt.wheel_active = false;
    }
    // Btn3 → 左クリック HOLD
    if (btn3 && !g_mouse_rt.left_held) {
        g_ble_hid.pressMouseButton(MOUSE_LEFT);
        g_mouse_rt.left_held = true;
    } else if (!btn3 && g_mouse_rt.left_held) {
        g_ble_hid.releaseMouseButton(MOUSE_LEFT);
        g_mouse_rt.left_held = false;
    }
    // Btn2 → 右クリック HOLD
    if (btn2 && !g_mouse_rt.right_held) {
        g_ble_hid.pressMouseButton(MOUSE_RIGHT);
        g_mouse_rt.right_held = true;
    } else if (!btn2 && g_mouse_rt.right_held) {
        g_ble_hid.releaseMouseButton(MOUSE_RIGHT);
        g_mouse_rt.right_held = false;
    }
    // 連続カーソル: Yaw ジャイロ → dx、Pitch ジャイロ → dy
    constexpr float SENSITIVITY = 8.0f;   // px / deg
    constexpr float DEADZONE = 3.0f;      // deg/s
    float gx = s.gyro[2];  // yaw deg/s
    float gy = s.gyro[1];  // pitch deg/s
    if (fabsf(gx) < DEADZONE) gx = 0;
    if (fabsf(gy) < DEADZONE) gy = 0;
    int16_t dx = (int16_t)(gx * dt * SENSITIVITY);
    int16_t dy = (int16_t)(gy * dt * SENSITIVITY);
    if (dx != 0 || dy != 0) g_ble_hid.moveMouse(dx, dy, 0);
}

// =========================================================
// BLE 接続安定性のチューニング (Phase 5.31)
// - ATT MTU を 247 に拡張して大きい JSON のチャンク数を減らす
// - 接続パラメータを Min 15ms / Max 30ms / Latency 0 / Timeout 4s に設定
//   (NimBLE デフォルトは Timeout 720ms と短く、Windows BLE スタックで
//    notify 集中時に切れやすかった)
// =========================================================
static void tuneBleConnection() {
    NimBLEDevice::setMTU(247);
    NimBLEServer* server = NimBLEDevice::getServer();
    if (!server) return;
    // min, max は 1.25ms 単位、latency は events、timeout は 10ms 単位
    //   min=12 (15ms), max=24 (30ms), latency=0, timeout=400 (4000ms)
    server->setDataLen(0xFFFF, 251);  // PDU 拡張 (LE Data Length Extension)
    // 接続中の全ピアにパラメータ更新を要求
    size_t cnt = server->getConnectedCount();
    for (size_t i = 0; i < cnt; i++) {
        NimBLEConnInfo info = server->getPeerInfo(i);
        server->updateConnParams(info.getConnHandle(), 12, 24, 0, 400);
    }
}

// =========================================================
// rule.add / remove / clear 後に active_profile へ自動保存 (Phase 5.12)
// active_profile が未設定なら "default" を使用
// =========================================================
static void autoSaveActiveProfile() {
    String active = Profile::getActive();
    if (active.length() == 0) {
        active = "default";
        Profile::setActive("default");
    }
    std::vector<ActionRule> rules;
    for (const auto& r : g_engine.rules()) rules.push_back(r);
    JsonDocument errOut;
    Profile::save(active.c_str(), rules, errOut);
}

// =========================================================
// Phase 5.33: 方向名 → posture Condition マッピング (ハリーポッターワンド用)
//
// 8 方向シーケンス (Kano 流) を内部の Pitch/Roll 領域に展開:
//   M5StickC 基本姿勢 R+90, P0 (LCD 左向き縦持ち、Phase 5.30 規約) からの flick
//   "U"  上振り = Pitch +30°, Roll +90° (base)
//   "D"  下振り = Pitch -30°, Roll +90°
//   "L"  左振り = Pitch  0°, Roll +60°  (LCD 上向きへ回転)
//   "R"  右振り = Pitch  0°, Roll +120° (LCD 下向きへ回転)
//   "UL" "UR" "DL" "DR" = 上記の組合せ
// 各方向の許容: Pitch ±15°, Roll ±20°, Yaw 無視 (180)
//
// 戻り値: true = 方向名解析成功 / false = 不明な方向名
// =========================================================
static bool directionToCondition(const char* dir, Condition& out) {
    if (!dir || !dir[0]) return false;
    // 中央 (Pitch, Roll) を方向別に決定
    float p = 0.0f, r = 90.0f;
    if      (strcmp(dir, "U")  == 0) { p = +30; r =  90; }
    else if (strcmp(dir, "D")  == 0) { p = -30; r =  90; }
    else if (strcmp(dir, "L")  == 0) { p =   0; r =  60; }
    else if (strcmp(dir, "R")  == 0) { p =   0; r = 120; }
    else if (strcmp(dir, "UL") == 0) { p = +30; r =  60; }
    else if (strcmp(dir, "UR") == 0) { p = +30; r = 120; }
    else if (strcmp(dir, "DL") == 0) { p = -30; r =  60; }
    else if (strcmp(dir, "DR") == 0) { p = -30; r = 120; }
    else return false;
    out.logic_op = LogicOp::OP_AND;
    out.posture.enabled = true;
    out.posture.judge_by = PostureJudge::BY_EULER;
    out.posture.euler[0] = r;
    out.posture.euler[1] = p;
    out.posture.euler[2] = 0;
    out.posture.euler_tol[0] = 20.0f;   // Roll ±20°
    out.posture.euler_tol[1] = 15.0f;   // Pitch ±15°
    out.posture.euler_tol[2] = 180.0f;  // Yaw 無視
    // quat はダミー (BY_EULER 判定なので未使用、ただし closest 計算用に保持)
    out.posture.quat[0] = 1.0f;
    out.posture.quat[1] = out.posture.quat[2] = out.posture.quat[3] = 0.0f;
    out.posture.quat_dot_min = 0.95f;
    return true;
}

// =========================================================
// キー名 → BleCombo HID code 変換 (rule.add の "key" / "keys" で使用)
// 1 文字 ASCII はそのまま、それ以外は名前付きで解決。
// 戻り値 0 は不明 (無視される)
// =========================================================
static uint8_t parseKeyName(const char* name) {
    if (!name || !name[0]) return 0;
    // 1 文字 ASCII (printable)
    if (name[1] == 0) return (uint8_t)name[0];
    // 大文字化して比較 (case-insensitive)
    auto eq = [](const char* a, const char* b) -> bool {
        for (size_t i = 0; ; i++) {
            char ca = a[i]; char cb = b[i];
            if (ca >= 'a' && ca <= 'z') ca -= 32;
            if (cb >= 'a' && cb <= 'z') cb -= 32;
            if (ca != cb) return false;
            if (ca == 0) return true;
        }
    };
    // 矢印
    if (eq(name, "ARROW_RIGHT") || eq(name, "RIGHT")) return 0xD7;  // KEY_RIGHT_ARROW
    if (eq(name, "ARROW_LEFT")  || eq(name, "LEFT"))  return 0xD8;  // KEY_LEFT_ARROW
    if (eq(name, "ARROW_DOWN")  || eq(name, "DOWN"))  return 0xD9;  // KEY_DOWN_ARROW
    if (eq(name, "ARROW_UP")    || eq(name, "UP"))    return 0xDA;  // KEY_UP_ARROW
    // 制御
    if (eq(name, "ENTER")     || eq(name, "RETURN")) return 0xB0;
    if (eq(name, "ESC")       || eq(name, "ESCAPE")) return 0xB1;
    if (eq(name, "BACKSPACE") || eq(name, "BS"))     return 0xB2;
    if (eq(name, "TAB"))                              return 0xB3;
    if (eq(name, "SPACE"))                            return 0x20;
    if (eq(name, "DELETE")    || eq(name, "DEL"))    return 0xD4;
    if (eq(name, "INSERT")    || eq(name, "INS"))    return 0xD1;
    if (eq(name, "HOME"))                             return 0xD2;
    if (eq(name, "END"))                              return 0xD5;
    if (eq(name, "PAGE_UP")   || eq(name, "PGUP"))   return 0xD3;
    if (eq(name, "PAGE_DOWN") || eq(name, "PGDN"))   return 0xD6;
    if (eq(name, "CAPS_LOCK") || eq(name, "CAPS"))   return 0xC1;
    if (eq(name, "PRTSC")     || eq(name, "PRINT"))  return 0xCE;
    // Function キー (F1〜F24)
    if ((name[0] == 'F' || name[0] == 'f') && name[1] >= '1' && name[1] <= '9') {
        int n = atoi(name + 1);
        if (n >= 1 && n <= 12)  return 0xC2 + (n - 1);   // F1..F12 = 0xC2..0xCD
        if (n >= 13 && n <= 24) return 0xF0 + (n - 13);  // F13..F24 = 0xF0..0xFB
    }
    // 修飾キー単体 (通常 modifiers ビットマスクで使うが手動指定も可能に)
    if (eq(name, "CTRL")  || eq(name, "LCTRL"))   return 0x80;
    if (eq(name, "SHIFT") || eq(name, "LSHIFT"))  return 0x81;
    if (eq(name, "ALT")   || eq(name, "LALT"))    return 0x82;
    if (eq(name, "GUI")   || eq(name, "WIN") || eq(name, "CMD")) return 0x83;
    return 0;
}

// =========================================================
// コマンドハンドラ (Phase 1 最小セット)
// =========================================================
static void handleCommand(JsonDocument& in, JsonDocument& out) {
    const char* cmd = in["cmd"] | "";

    if (strcmp(cmd, "ping") == 0) {
        out["type"] = "pong";
        out["fw"] = FW_VERSION;
        out["fw_phase"] = FW_PHASE;
        out["fw_build"] = FW_BUILD;
        out["board"] =
            #if defined(BOARD_M5STICKC)
                "m5stickc"
            #elif defined(BOARD_M5ATOM_S3)
                "m5atom_s3"
            #else
                "unknown"
            #endif
        ;
        out["imu"] =
            #if defined(BOARD_M5STICKC)
                "mpu6886"
            #else
                "unknown"
            #endif
        ;
        out["uptime"] = (uint32_t)(millis() - g_boot_ms);
    }
    else if (strcmp(cmd, "device.info") == 0) {
        out["type"] = "device.info";
        out["fw"] = FW_VERSION;
        out["fw_phase"] = FW_PHASE;
        out["fw_build"] = FW_BUILD;
        out["board"] =
            #if defined(BOARD_M5STICKC)
                "m5stickc"
            #else
                "unknown"
            #endif
        ;
        out["imu"] =
            #if defined(BOARD_M5STICKC)
                "mpu6886"
            #else
                "unknown"
            #endif
        ;
        out["uptime"] = (uint32_t)(millis() - g_boot_ms);
        out["ble_connected"] = g_ble_hid.isConnected();
        out["ble_nus_started"] = g_ble_nus_started;
        out["ble_nus_connected"] = g_ble_nus.isConnected();
        out["output"] =
            g_output_mode == HidOutputMode::OUT_BLE  ? "ble"  :
            g_output_mode == HidOutputMode::OUT_USB  ? "usb"  :
            g_output_mode == HidOutputMode::OUT_BOTH ? "both" : "none";
        #if defined(BOARD_M5STICKC)
        if (g_axp_ok) {
            JsonObject bat = out["battery"].to<JsonObject>();
            bat["percent"] = g_battery_percent;
            bat["voltage"] = g_battery_voltage;
            bat["charging"] = g_battery_charging;
        }
        out["lcd_ok"] = g_lcd_ok;
        out["axp_ok"] = g_axp_ok;
        #endif
        out["rule_count"] = (uint32_t)g_engine.ruleCount();
        out["active_profile"] = Profile::getActive();
        out["closest_only"] = g_engine.isClosestOnlyMode();
        out["lock_window_ms"] = g_engine.lockWindowMs();
        out["lock_cooldown_ms"] = g_engine.lockCooldownMs();
        // Phase 5.32: 動作モード
        out["device_mode"] = (g_device_mode == DeviceMode::MOUSE) ? "mouse" : "engine";
    }
    else if (strcmp(cmd, "sensor.stream") == 0) {
        g_stream_rate_hz = in["rate_hz"] | 0;
        out["type"] = "ack";
        out["cmd"] = "sensor.stream";
        out["ok"] = true;
    }
    else if (strcmp(cmd, "mode.set") == 0) {
        // Phase 5.32: 動作モード切替 (engine / mouse)
        const char* m = in["mode"] | "engine";
        DeviceMode prev = g_device_mode;
        if (strcmp(m, "mouse") == 0) g_device_mode = DeviceMode::MOUSE;
        else                          g_device_mode = DeviceMode::ENGINE;
        // mode を切替えた瞬間、HID 出力中のキー/ボタンをすべてリリース
        if (prev != g_device_mode) {
            g_ble_hid.releaseAll();
            if (g_mouse_rt.left_held)  { g_ble_hid.releaseMouseButton(MOUSE_LEFT);  g_mouse_rt.left_held  = false; }
            if (g_mouse_rt.right_held) { g_ble_hid.releaseMouseButton(MOUSE_RIGHT); g_mouse_rt.right_held = false; }
            g_mouse_rt.wheel_accum = 0.0f;
            g_mouse_rt.wheel_active = false;
        }
        saveDeviceModeNvs();
        out["type"] = "ack";
        out["cmd"] = "mode.set";
        out["ok"] = true;
        out["device_mode"] = (g_device_mode == DeviceMode::MOUSE) ? "mouse" : "engine";
    }
    else if (strcmp(cmd, "mode.get") == 0) {
        out["type"] = "mode";
        out["device_mode"] = (g_device_mode == DeviceMode::MOUSE) ? "mouse" : "engine";
    }
    else if (strcmp(cmd, "output.set") == 0) {
        const char* target = in["target"] | "none";
        if      (strcmp(target, "ble")  == 0) g_output_mode = HidOutputMode::OUT_BLE;
        else if (strcmp(target, "usb")  == 0) g_output_mode = HidOutputMode::OUT_USB;
        else if (strcmp(target, "both") == 0) g_output_mode = HidOutputMode::OUT_BOTH;
        else                                  g_output_mode = HidOutputMode::OUT_NONE;
        bool ble_on = (g_output_mode == HidOutputMode::OUT_BLE || g_output_mode == HidOutputMode::OUT_BOTH);
        g_ble_hid.setEnabled(ble_on);
        out["type"] = "ack";
        out["cmd"] = "output.set";
        out["ok"] = true;
    }
    else if (strcmp(cmd, "ble.stop") == 0) {
        // BLE HID advertising 停止 + 切断
        bool stopped = false;
        if (g_ble_hid_started) {
            stopped = g_ble_hid.stop();
            g_ble_hid_started = false;
        }
        out["type"] = "ack";
        out["cmd"] = "ble.stop";
        out["ok"] = true;
        out["stopped"] = stopped;
        out["ble_hid"] = g_ble_hid_started;
    }
    else if (strcmp(cmd, "ble.start") == 0) {
        // BLE HID + BLE NUS を同時開始 (NimBLE 単一サーバー上に同居)
        if (!g_ble_hid_started) {
            g_ble_hid.begin();
            g_ble_hid_started = true;
            // HID begin の直後に NUS service を追加
            // (BleHidSink::begin() で NimBLEDevice::init + advertising が走る、
            //  その上に NUS service を載せる)
            if (!g_ble_nus_started) {
                if (g_ble_nus.begin()) {
                    g_ble_nus.setHandler(handleCommand);
                    g_ble_nus_started = true;
                }
            }
            tuneBleConnection();  // Phase 5.31
        }
        out["type"] = "ack";
        out["cmd"] = "ble.start";
        out["ok"] = true;
        out["ble_hid"] = g_ble_hid_started;
        out["ble_nus"] = g_ble_nus_started;
    }
    else if (strcmp(cmd, "watch.set") == 0) {
        g_engine.setWatchEnabled(in["enabled"] | false);
        out["type"] = "ack";
        out["cmd"] = "watch.set";
        out["ok"] = true;
    }
    // ================ Phase 5.39.2.6: Button シミュレーション (自動テスト用) ================
    // {"cmd":"btn.sim", "idx":3, "state":1}      → Btn3 を「押下中」として override
    // {"cmd":"btn.sim", "idx":3, "state":0}      → Btn3 を「離している」として override
    // {"cmd":"btn.sim", "clear":true}            → 全 sim 解除 (物理 GPIO 値に戻す)
    // {"cmd":"btn.sim", "mask":0x07, "value":0x04}  → 直接 bitmap 指定 (上級)
    // sim 中は g_buttons.update() 後に sim 値で上書き → TriggerEngine 評価で sim が反映される
    else if (strcmp(cmd, "btn.sim") == 0) {
        out["type"] = "ack";
        out["cmd"] = "btn.sim";
        if (in["clear"] | false) {
            g_btn_sim_mask = 0;
            g_btn_sim_value = 0;
            out["ok"] = true;
            out["mask"] = (uint32_t)g_btn_sim_mask;
            out["value"] = (uint32_t)g_btn_sim_value;
        } else if (in["mask"].is<unsigned int>() && in["value"].is<unsigned int>()) {
            // 直接 bitmap 指定モード
            g_btn_sim_mask = in["mask"];
            g_btn_sim_value = in["value"];
            out["ok"] = true;
            out["mask"] = (uint32_t)g_btn_sim_mask;
            out["value"] = (uint32_t)g_btn_sim_value;
        } else {
            int idx = in["idx"] | 0;
            int state = in["state"] | -1;
            if (idx < 1 || idx > 16 || state < 0 || state > 1) {
                out["type"] = "err";
                out["ok"] = false;
                out["err"] = "invalid_idx_or_state (idx 1-16, state 0/1)";
            } else {
                uint16_t bit = 1u << (idx - 1);
                g_btn_sim_mask |= bit;
                if (state == 1) {
                    g_btn_sim_value |= bit;
                } else {
                    g_btn_sim_value &= ~bit;
                }
                out["ok"] = true;
                out["mask"] = (uint32_t)g_btn_sim_mask;
                out["value"] = (uint32_t)g_btn_sim_value;
            }
        }
    }
    // ================ HID 直接テスト ================
    else if (strcmp(cmd, "test.hid") == 0) {
        // {"cmd":"test.hid", "action":"press"|"release"|"fire"|"text"|"mouse_move"|"mouse_click",
        //  "key":"a", "text":"Hello", "dx":10, "dy":0, "button":"left"}
        const char* action = in["action"] | "fire";
        bool ok = true;
        if (!g_ble_hid_started) {
            out["type"] = "err"; out["cmd"] = "test.hid"; out["err"] = "ble_not_started";
            return;
        }
        if (strcmp(action, "press") == 0) {
            const char* k = in["key"] | "";
            uint8_t code = parseKeyName(k);  // Phase 5.27: ARROW_LEFT 等の特殊キー対応
            if (code) g_ble_hid.pressKey(code);
        }
        else if (strcmp(action, "release") == 0) {
            const char* k = in["key"] | "";
            uint8_t code = parseKeyName(k);
            if (code) g_ble_hid.releaseKey(code);
            else g_ble_hid.releaseAll();
        }
        else if (strcmp(action, "fire") == 0) {
            const char* k = in["key"] | "a";
            uint8_t code = parseKeyName(k);
            uint16_t dur = in["duration_ms"] | 30;
            if (code) {
                g_ble_hid.pressKey(code);
                delay(dur);
                g_ble_hid.releaseKey(code);
            }
        }
        else if (strcmp(action, "text") == 0) {
            const char* text = in["text"] | "";
            for (size_t i = 0; text[i]; i++) {
                g_ble_hid.pressKey((uint8_t)text[i]);
                delay(20);
                g_ble_hid.releaseKey((uint8_t)text[i]);
                delay(20);
            }
        }
        else if (strcmp(action, "mouse_move") == 0) {
            int16_t dx = in["dx"] | 0;
            int16_t dy = in["dy"] | 0;
            int8_t wheel = in["wheel"] | 0;
            g_ble_hid.moveMouse(dx, dy, wheel);
            // デバッグ: Phase 5.21 - マウス動作不良切り分け用
            out["dx"] = dx;
            out["dy"] = dy;
            out["enabled"] = g_ble_hid.isEnabled();
            out["connected"] = g_ble_hid.isConnected();
        }
        else if (strcmp(action, "mouse_click") == 0) {
            const char* btn = in["button"] | "left";
            uint8_t b = strcmp(btn, "right") == 0 ? 2 : strcmp(btn, "middle") == 0 ? 4 : 1;
            g_ble_hid.clickMouse(b);
            out["btn"] = b;
            out["enabled"] = g_ble_hid.isEnabled();
            out["connected"] = g_ble_hid.isConnected();
        }
        else if (strcmp(action, "release_all") == 0) {
            // Phase 5.22: 緊急 全キー解放 (HOLD ルールが暴走したとき用)
            g_ble_hid.releaseAll();
            // ルールの状態もリセット (HOLD 中だったルールを idle に戻す)
            g_engine.clearRules();
            // active_profile から再ロード
            String active = Profile::getActive();
            if (active.length() > 0) {
                std::vector<ActionRule> rules;
                JsonDocument errOut;
                if (Profile::load(active.c_str(), rules, errOut)) {
                    for (auto& r : rules) g_engine.addRule(r);
                }
            }
            out["released"] = true;
        }
        else {
            ok = false;
        }
        out["type"] = "ack";
        out["cmd"] = "test.hid";
        out["action"] = action;
        out["ok"] = ok;
        out["ble_connected"] = g_ble_hid.isConnected();
    }
    // ================ Gyro キャリブレーション ================
    else if (strcmp(cmd, "calibrate.simple") == 0) {
        // 1秒間 (50 サンプル) gyro 平均を bias として使用
        uint16_t dur = in["duration_ms"] | 1000;
        uint16_t samples = dur / 10;
        if (samples < 10) samples = 10;
        if (samples > 200) samples = 200;
        double sx=0, sy=0, sz=0;
        uint16_t got = 0;
        for (uint16_t i = 0; i < samples; i++) {
            float a[3], g[3];
            if (g_imu.read(a, g)) {
                sx += g[0]; sy += g[1]; sz += g[2];
                got++;
            }
            delay(10);
        }
        if (got > 0) {
            g_gyro_bias_rad[0] = (float)(sx / got);
            g_gyro_bias_rad[1] = (float)(sy / got);
            g_gyro_bias_rad[2] = (float)(sz / got);
            g_gyro_calibrated = true;
            // Mahony を reset (姿勢を identity に戻す)
            g_mahony.reset();
            out["type"] = "ack";
            out["cmd"] = "calibrate.simple";
            out["ok"] = true;
            out["samples"] = got;
            constexpr float RAD2DEG = 57.29577951308232f;
            JsonArray bias = out["gyro_bias_dps"].to<JsonArray>();
            bias.add(g_gyro_bias_rad[0] * RAD2DEG);
            bias.add(g_gyro_bias_rad[1] * RAD2DEG);
            bias.add(g_gyro_bias_rad[2] * RAD2DEG);
        } else {
            out["type"] = "err";
            out["cmd"] = "calibrate.simple";
            out["err"] = "no_imu_data";
        }
    }
    // ================ 6 点 Accel キャリブレーション ================
    else if (strcmp(cmd, "calibrate.full.start") == 0) {
        memset(&g_calib6, 0, sizeof(g_calib6));
        g_calib6.active = true;
        g_calib6.step = 0;
        out["type"] = "calibration.step";
        out["step"] = 0;
        out["total"] = 6;
        out["instruction"] = "LCD を上向き (+Z) に静置";
    }
    else if (strcmp(cmd, "calibrate.full.capture") == 0) {
        if (!g_calib6.active) {
            out["type"] = "err"; out["cmd"] = "calibrate.full.capture"; out["err"] = "not_started";
            return;
        }
        int step = g_calib6.step;
        if (step < 0 || step > 5) {
            out["type"] = "err"; out["err"] = "invalid_step";
            return;
        }
        // 50 サンプル取って平均
        double sum[3] = {0, 0, 0};
        int got = 0;
        for (int i = 0; i < 50; i++) {
            float a[3], gv[3];
            if (g_imu.read(a, gv)) {
                for (int j = 0; j < 3; j++) sum[j] += a[j];
                got++;
            }
            delay(10);
        }
        if (got < 30) {
            out["type"] = "err"; out["err"] = "imu_read_failed";
            return;
        }
        for (int j = 0; j < 3; j++) g_calib6.face_avg[step][j] = (float)(sum[j] / got);
        g_calib6.face_done[step] = true;

        const char* instructions[7] = {
            "LCD を上向き (+Z) に静置",
            "LCD を下向き (-Z) に静置",
            "右側面を上 (+X) に立てる",
            "左側面を上 (-X) に立てる",
            "上端 (USB-C 反対側、+Y) を上に立てる",
            "下端 (USB-C 側、-Y) を上に立てる",
            "完了"
        };

        g_calib6.step++;
        out["type"] = "calibration.step";
        out["step"] = g_calib6.step;
        out["total"] = 6;
        out["captured"] = step;
        JsonArray g = out["face_avg_g"].to<JsonArray>();
        g.add(g_calib6.face_avg[step][0] / 9.80665f);
        g.add(g_calib6.face_avg[step][1] / 9.80665f);
        g.add(g_calib6.face_avg[step][2] / 9.80665f);
        if (g_calib6.step >= 6) {
            out["instruction"] = "全 6 面取得完了。calibrate.full.finish で計算";
            out["all_done"] = true;
        } else {
            out["instruction"] = instructions[g_calib6.step];
        }
    }
    else if (strcmp(cmd, "calibrate.full.finish") == 0) {
        if (!g_calib6.active) {
            out["type"] = "err"; out["cmd"] = "calibrate.full.finish"; out["err"] = "not_started";
            return;
        }
        // 全 6 面取得済みか確認
        for (int i = 0; i < 6; i++) {
            if (!g_calib6.face_done[i]) {
                out["type"] = "err"; out["err"] = "face_missing"; out["face"] = i;
                return;
            }
        }
        // bias = (positive + negative) / 2
        // scale = G / ((positive - negative) / 2)
        const float G = 9.80665f;
        // X 軸 face 2 (+X) と face 3 (-X)
        g_accel_bias[0]  = (g_calib6.face_avg[2][0] + g_calib6.face_avg[3][0]) / 2;
        g_accel_scale[0] = G / ((g_calib6.face_avg[2][0] - g_calib6.face_avg[3][0]) / 2);
        // Y 軸 face 4 (+Y) と face 5 (-Y)
        g_accel_bias[1]  = (g_calib6.face_avg[4][1] + g_calib6.face_avg[5][1]) / 2;
        g_accel_scale[1] = G / ((g_calib6.face_avg[4][1] - g_calib6.face_avg[5][1]) / 2);
        // Z 軸 face 0 (+Z) と face 1 (-Z)
        g_accel_bias[2]  = (g_calib6.face_avg[0][2] + g_calib6.face_avg[1][2]) / 2;
        g_accel_scale[2] = G / ((g_calib6.face_avg[0][2] - g_calib6.face_avg[1][2]) / 2);
        g_accel_calibrated = true;
        g_calib6.active = false;
        // NVS に永続化 (起動時自動ロードされる、Phase 5.9)
        saveCalibToNvs();

        out["type"] = "ack";
        out["cmd"] = "calibrate.full.finish";
        out["ok"] = true;
        JsonArray b = out["accel_bias_ms2"].to<JsonArray>();
        for (int i = 0; i < 3; i++) b.add(g_accel_bias[i]);
        JsonArray s = out["accel_scale"].to<JsonArray>();
        for (int i = 0; i < 3; i++) s.add(g_accel_scale[i]);
    }
    else if (strcmp(cmd, "calibrate.full.cancel") == 0) {
        g_calib6.active = false;
        out["type"] = "ack"; out["cmd"] = "calibrate.full.cancel"; out["ok"] = true;
    }
    // ================ ActionRule 登録 (簡易版) ================
    else if (strcmp(cmd, "rule.clear") == 0) {
        g_engine.clearRules();
        autoSaveActiveProfile();
        out["type"] = "ack"; out["cmd"] = "rule.clear"; out["ok"] = true;
    }
    else if (strcmp(cmd, "rule.remove") == 0) {
        // {"cmd":"rule.remove","id":N}
        uint16_t id = in["id"] | 0;
        bool ok = g_engine.removeRule(id);
        if (ok) autoSaveActiveProfile();
        out["type"] = ok ? "ack" : "err";
        out["cmd"] = "rule.remove";
        out["ok"] = ok;
        out["id"] = id;
        out["rule_count"] = (uint32_t)g_engine.ruleCount();
        if (!ok) out["err"] = "not_found";
    }
    else if (strcmp(cmd, "rule.list") == 0) {
        out["type"] = "rule.list";
        JsonArray arr = out["rules"].to<JsonArray>();
        for (const auto& r : g_engine.rules()) {
            JsonObject o = arr.add<JsonObject>();
            o["id"] = r.id;
            o["name"] = r.name;
            o["states_count"] = r.states_count;
            o["loop"] = r.loop;
            o["current_state"] = r.current_state;  // -1=idle、>=0 = state 滞在中
            // Phase 5.39: posture_basis (0=absolute, 1=relative) を文字列で返す
            o["posture_basis"] = (r.posture_basis == PB_RELATIVE_QUAT) ? "relative" : "absolute";
            // states[0] の posture を返す (ある場合)
            if (r.states_count > 0 && r.states[0].match_condition.posture.enabled) {
                JsonObject p = o["posture"].to<JsonObject>();
                JsonArray e = p["euler"].to<JsonArray>();
                JsonArray et = p["euler_tol"].to<JsonArray>();
                for (int i = 0; i < 3; i++) {
                    e.add(r.states[0].match_condition.posture.euler[i]);
                    et.add(r.states[0].match_condition.posture.euler_tol[i]);
                }
                // 3D 球面表示用に quat も返す (旧版互換)
                JsonArray q = p["quat"].to<JsonArray>();
                for (int i = 0; i < 4; i++) {
                    q.add(r.states[0].match_condition.posture.quat[i]);
                }
                p["judge_by"] =
                    (r.states[0].match_condition.posture.judge_by == PostureJudge::BY_QUAT) ? "quat" : "euler";
            }
            // ボタン条件 (デバッグ用)
            if (r.states_count > 0 && r.states[0].match_condition.button.enabled) {
                JsonObject b = o["button"].to<JsonObject>();
                b["idx"] = r.states[0].match_condition.button.idx;
                b["state"] = r.states[0].match_condition.button.state;
            }
            // 加速度条件 (デバッグ用)
            if (r.states_count > 0 && r.states[0].match_condition.accel.enabled) {
                JsonObject a = o["accel"].to<JsonObject>();
                a["abs_threshold"] = r.states[0].match_condition.accel.abs_threshold;
                a["use_per_axis"] = r.states[0].match_condition.accel.use_per_axis;
            }
            // 出力アクション (どのキーが登録されているか確認用、Phase 5.9.1)
            if (r.states_count > 0) {
                JsonObject a = o["action"].to<JsonObject>();
                a["type"] = (uint8_t)r.states[0].on_enter.type;
                const char* atName = "?";
                switch (r.states[0].on_enter.type) {
                    case ActionType::AT_NONE:        atName = "none"; break;
                    case ActionType::AT_PRESS:       atName = "press"; break;
                    case ActionType::AT_RELEASE:     atName = "release"; break;
                    case ActionType::AT_FIRE_ONCE:   atName = "fire_once"; break;
                    case ActionType::AT_FIRE_MACRO:  atName = "fire_macro"; break;
                    case ActionType::AT_MOUSE_MOVE:  atName = "mouse_move"; break;
                    case ActionType::AT_MOUSE_CLICK: atName = "mouse_click"; break;
                    default: break;
                }
                a["type_name"] = atName;
                JsonArray ks = a["keys"].to<JsonArray>();
                for (uint8_t i = 0; i < r.states[0].on_enter.keys_len; i++) {
                    ks.add(r.states[0].on_enter.keys[i]);
                }
                // Phase 5.14: key_modes も返す (UI で同時押し表示用)
                JsonArray kms = a["key_modes"].to<JsonArray>();
                for (uint8_t i = 0; i < r.states[0].on_enter.keys_len; i++) {
                    kms.add(r.states[0].on_enter.key_modes[i]);
                }
                a["modifiers"] = r.states[0].on_enter.modifiers;
                a["interval_ms"] = r.states[0].on_enter.interval_ms;
                a["duration_ms"] = r.states[0].on_enter.duration_ms;
            }
            // Phase 5.34: SEQUENCE/ハリポタワンドの可視化のため全 states を配列で返す
            //   各エントリは waypoint の posture (euler, tol) と on_enter サマリ
            JsonArray statesArr = o["states"].to<JsonArray>();
            for (uint8_t si = 0; si < r.states_count && si < 4; si++) {
                JsonObject so = statesArr.add<JsonObject>();
                const State& st = r.states[si];
                if (st.match_condition.posture.enabled) {
                    JsonObject sp = so["posture"].to<JsonObject>();
                    JsonArray se = sp["euler"].to<JsonArray>();
                    JsonArray set = sp["euler_tol"].to<JsonArray>();
                    for (int i = 0; i < 3; i++) {
                        se.add(st.match_condition.posture.euler[i]);
                        set.add(st.match_condition.posture.euler_tol[i]);
                    }
                }
                if (st.match_condition.button.enabled) {
                    JsonObject sb = so["button"].to<JsonObject>();
                    sb["idx"] = st.match_condition.button.idx;
                    sb["state"] = st.match_condition.button.state;
                }
                // Phase 5.39: stillness 情報も返す (required=true の state のみ)
                if (st.match_condition.stillness_required) {
                    JsonObject sst = so["stillness"].to<JsonObject>();
                    sst["required"] = true;
                    sst["window_ms"] = st.match_condition.stillness_window_ms;
                    sst["accel_th_mg"] = st.match_condition.stillness_accel_th_mg;
                    sst["gyro_th_dps"] = st.match_condition.stillness_gyro_th_dps;
                }
                so["max_dwell_ms"] = st.max_dwell_ms;
                // 発火する waypoint (最終状態) のキーストローク数だけ示す (text 復元用)
                if (st.on_enter.keys_len > 0) {
                    so["fire_keys_len"] = st.on_enter.keys_len;
                    // 短い場合は中身も (text 復元用)
                    if (st.on_enter.keys_len <= 24) {
                        JsonArray fk = so["fire_keys"].to<JsonArray>();
                        for (uint8_t i = 0; i < st.on_enter.keys_len; i++) {
                            fk.add(st.on_enter.keys[i]);
                        }
                    }
                }
            }
        }
        out["sensor_btn"] = g_sensor_state.buttons_bitmap;  // 即時参照用
    }
    // ================ Closest Rule (Quaternion 角度差) ================
    else if (strcmp(cmd, "rule.closest") == 0) {
        // 現在 quat と各ルールの登録 quat の angleTo 最小を返す
        // 旧 motion_controller.js と互換のロジック
        const auto& rules = g_engine.rules();
        if (rules.empty()) {
            out["type"] = "rule.closest"; out["id"] = -1;
            return;
        }
        float min_angle = 4.0f;  // > π
        int closest_id = -1;
        const float* cq = g_sensor_state.quat;
        for (const auto& r : rules) {
            if (r.states_count == 0) continue;
            const float* rq = r.states[0].match_condition.posture.quat;
            // 内積 → 2*acos(|dot|) で 0..π 範囲の角度差
            float dot = cq[0]*rq[0] + cq[1]*rq[1] + cq[2]*rq[2] + cq[3]*rq[3];
            if (dot < 0) dot = -dot;
            if (dot > 1) dot = 1;
            float angle = 2.0f * acosf(dot);
            if (angle < min_angle) {
                min_angle = angle;
                closest_id = r.id;
            }
        }
        out["type"] = "rule.closest";
        out["id"] = closest_id;
        out["angle_rad"] = min_angle;
    }
    // ================ Closest-only モード (TriggerEngine 発火制限) ================
    else if (strcmp(cmd, "engine.closest_only") == 0) {
        // {"cmd":"engine.closest_only","enabled":true|false}
        // enabled 省略時は現在値を返す
        if (in["enabled"].is<bool>()) {
            g_engine.setClosestOnlyMode(in["enabled"].as<bool>());
        }
        out["type"] = "ack";
        out["cmd"] = "engine.closest_only";
        out["ok"] = true;
        out["enabled"] = g_engine.isClosestOnlyMode();
    }
    // ================ Button-edge lock パラメータ ================
    else if (strcmp(cmd, "engine.lock.set") == 0) {
        // {"cmd":"engine.lock.set", "window_ms":500, "cooldown_ms":300}
        // 個別省略可、指定された値だけ更新
        if (in["window_ms"].is<int>() || in["window_ms"].is<unsigned int>()) {
            uint32_t v = in["window_ms"];
            if (v > 0 && v <= 10000) g_engine.setLockWindowMs((uint16_t)v);
        }
        if (in["cooldown_ms"].is<int>() || in["cooldown_ms"].is<unsigned int>()) {
            uint32_t v = in["cooldown_ms"];
            if (v <= 10000) g_engine.setLockCooldownMs((uint16_t)v);
        }
        out["type"] = "ack";
        out["cmd"] = "engine.lock.set";
        out["ok"] = true;
        out["window_ms"] = g_engine.lockWindowMs();
        out["cooldown_ms"] = g_engine.lockCooldownMs();
    }
    else if (strcmp(cmd, "engine.lock.get") == 0) {
        out["type"] = "engine.lock";
        out["window_ms"] = g_engine.lockWindowMs();
        out["cooldown_ms"] = g_engine.lockCooldownMs();
        out["locked_rule_id"] = g_engine.currentLockedRuleId();
        if (g_engine.currentLockedRuleId() >= 0) {
            out["elapsed_ms"] = g_engine.currentLockElapsedMs(millis());
        }
    }
    // ================ Profile (LittleFS 永続化) ================
    else if (strcmp(cmd, "profile.list") == 0) {
        auto names = Profile::list();
        out["type"] = "profile.list";
        JsonArray arr = out["profiles"].to<JsonArray>();
        for (const auto& n : names) arr.add(n);
        out["active"] = Profile::getActive();
    }
    else if (strcmp(cmd, "profile.save") == 0) {
        const char* name = in["name"] | "";
        if (!name[0]) {
            out["type"] = "err"; out["cmd"] = "profile.save"; out["err"] = "missing_name";
            return;
        }
        JsonDocument errOut;
        // 現在登録中のルールを全て保存
        std::vector<ActionRule> rules;
        for (const auto& r : g_engine.rules()) rules.push_back(r);
        if (Profile::save(name, rules, errOut)) {
            Profile::setActive(name);
            out["type"] = "ack"; out["cmd"] = "profile.save"; out["ok"] = true;
            out["name"] = name;
            out["rule_count"] = (uint32_t)rules.size();
        } else {
            out["type"] = "err"; out["cmd"] = "profile.save";
            out["err"] = errOut["err"] | "unknown";
        }
    }
    else if (strcmp(cmd, "profile.load") == 0) {
        const char* name = in["name"] | "";
        if (!name[0]) {
            out["type"] = "err"; out["cmd"] = "profile.load"; out["err"] = "missing_name";
            return;
        }
        std::vector<ActionRule> rules;
        JsonDocument errOut;
        if (Profile::load(name, rules, errOut)) {
            // 既存ルールクリア → 新ルール追加
            g_engine.clearRules();
            for (auto& r : rules) g_engine.addRule(r);
            Profile::setActive(name);
            out["type"] = "ack"; out["cmd"] = "profile.load"; out["ok"] = true;
            out["name"] = name;
            out["rule_count"] = (uint32_t)rules.size();
        } else {
            out["type"] = "err"; out["cmd"] = "profile.load";
            out["err"] = errOut["err"] | "unknown";
        }
    }
    else if (strcmp(cmd, "profile.delete") == 0) {
        const char* name = in["name"] | "";
        JsonDocument errOut;
        if (Profile::remove(name, errOut)) {
            out["type"] = "ack"; out["cmd"] = "profile.delete"; out["ok"] = true;
        } else {
            out["type"] = "err"; out["cmd"] = "profile.delete";
            out["err"] = errOut["err"] | "unknown";
        }
    }
    else if (strcmp(cmd, "profile.active") == 0) {
        // active プロファイル名取得
        out["type"] = "profile.active";
        out["name"] = Profile::getActive();
    }
    else if (strcmp(cmd, "rule.add") == 0) {
        // 簡易スキーマ: {"cmd":"rule.add","r":{"id":N,"name":"","ui_mode":"oneshot|hold_start_only|hold_start_end",
        //   "accel_abs_threshold":3.0,"key":"a","interval_ms":30,"cooldown_ms":500}}
        // 完全 JSON スキーマは Phase 2 後半で実装、ここは MVP 用のショートカット
        JsonObject r = in["r"];
        if (r.isNull()) {
            out["type"] = "err"; out["cmd"] = "rule.add"; out["err"] = "missing_r";
            return;
        }
        ActionRule rule = {};
        rule.id = r["id"] | (uint16_t)g_engine.ruleCount();
        const char* name = r["name"] | "rule";
        strncpy(rule.name, name, sizeof(rule.name) - 1);
        rule.priority = r["priority"] | 0;
        rule.cooldown_ms = r["cooldown_ms"] | 500;

        // =====================================================
        // Phase 5.33: directions[] ショートハンド (ハリポタワンド)
        // 例: {"directions":["DR","R","UR","D"], "type_text":"wingardium leviosa\n"}
        //   → 4 状態の SEQUENCE rule に展開、最終状態の on_enter で type_text を打鍵
        // 既存 posture/ui_mode は無視 (ショートハンドが優先)
        // =====================================================
        if (r["directions"].is<JsonArray>()) {
            JsonArray dirs = r["directions"].as<JsonArray>();
            uint8_t n = dirs.size();
            if (n < 2) n = 2;
            if (n > 4) n = 4;
            rule.states_count = n;
            rule.loop = false;  // SEQUENCE (一方通行、最後で発火→idle)
            const char* type_text = r["type_text"] | "";
            // 各状態の max_dwell_ms (デフォルト 800ms): 連続 waypoint 間の最大待ち時間
            uint16_t max_dwell = r["direction_max_ms"] | 800;
            // 各状態の min_dwell_ms (デフォルト 0): 短すぎる通過を弾く
            uint16_t min_dwell = r["direction_min_ms"] | 0;
            // Phase 5.33: オプションのボタンゲート
            //   button_idx 指定時は「ボタン押下中だけシーケンス進行」(Kano ワンド方式)
            //   無指定なら純ジェスチャ判定 (誤発動リスクと引き換えに魔法らしさ)
            uint8_t gate_btn_idx = 0;
            uint8_t gate_btn_state = 0;  // 0=pressed
            if (r["button_idx"].is<int>() || r["button_idx"].is<uint8_t>()) {
                uint8_t bidx = r["button_idx"];
                if (bidx > 0 && bidx <= 15) {
                    gate_btn_idx = bidx;
                    gate_btn_state = r["button_state"] | 0;
                }
            }
            bool ok = true;
            for (uint8_t i = 0; i < n; i++) {
                const char* d = dirs[i].as<const char*>();
                if (!directionToCondition(d, rule.states[i].match_condition)) {
                    ok = false;
                    break;
                }
                // ボタンゲート: 全 waypoint に同じボタン条件を適用 (押している間だけ進行)
                if (gate_btn_idx > 0) {
                    rule.states[i].match_condition.button.enabled = true;
                    rule.states[i].match_condition.button.idx = gate_btn_idx;
                    rule.states[i].match_condition.button.state = gate_btn_state;
                }
                rule.states[i].max_dwell_ms = max_dwell;
                rule.states[i].min_dwell_ms = min_dwell;
            }
            if (!ok) {
                out["type"] = "err";
                out["cmd"] = "rule.add";
                out["err"] = "bad_direction";
                return;
            }
            // 最終状態の on_enter で type_text を FIRE_MACRO (各文字を順次キー入力)
            State& sf = rule.states[n - 1];
            sf.on_enter.type = ActionType::AT_FIRE_MACRO;
            sf.on_enter.interval_ms = r["interval_ms"] | 15;
            uint8_t kn = 0;
            for (size_t i = 0; type_text[i] && kn < 24; i++, kn++) {
                sf.on_enter.keys[kn] = (uint8_t)type_text[i];
                sf.on_enter.key_modes[kn] = KeyMacroMode::FIRE;
            }
            sf.on_enter.keys_len = kn;
            g_engine.addRule(rule);
            autoSaveActiveProfile();
            out["type"] = "ack";
            out["cmd"] = "rule.add";
            out["ok"] = true;
            out["id"] = rule.id;
            out["mode"] = "sequence_directions";
            out["states"] = (uint32_t)n;
            out["rule_count"] = (uint32_t)g_engine.ruleCount();
            return;
        }

        // =====================================================
        // Phase 5.39: ui_mode == "hold_with_waypoints" ショートハンド
        //   start_posture (1) + mid_postures[] (0-2) + end_posture (1) を states[] に展開
        //   state[0]      : AT_PRESS、start_posture + button + stillness 任意
        //   state[1..N-2] : AT_NONE、mid_postures[i]
        //   state[N-1]    : AT_RELEASE、end_posture + stillness 任意 (end_stillness)
        //   posture_basis: "absolute" (default) / "relative"
        //   ボタン条件 / 加速度 / stillness は rule 全体で共通 (rule あたり 1 セット)
        // =====================================================
        {
            const char* uim = r["ui_mode"] | "";
            if (strcmp(uim, "hold_with_waypoints") == 0) {
                // posture_basis 解釈
                const char* pb = r["posture_basis"] | "absolute";
                rule.posture_basis = (strcmp(pb, "relative") == 0)
                                       ? PB_RELATIVE_QUAT : PB_ABSOLUTE_EULER;
                rule.loop = false;  // SEQUENCE (一方通行)

                // mid_postures 個数 (0-2)
                int mid_count = 0;
                if (r["mid_postures"].is<JsonArray>()) {
                    mid_count = (int)r["mid_postures"].as<JsonArray>().size();
                    if (mid_count > 2) mid_count = 2;
                    if (mid_count < 0) mid_count = 0;
                }
                rule.states_count = (uint8_t)(2 + mid_count);  // 開始(1) + 中間(0-2) + 終了(1)

                // ボタン条件 (rule 共通)
                bool btn_enabled = false;
                uint8_t btn_idx = 0;
                uint8_t btn_state = 0;
                if (r["button_idx"].is<int>() || r["button_idx"].is<uint8_t>()) {
                    uint8_t bidx = r["button_idx"];
                    if (bidx > 0 && bidx <= 15) {
                        btn_enabled = true;
                        btn_idx = bidx;
                        btn_state = r["button_state"] | 0;
                    }
                }

                // stillness 条件 (state[0] / state[N-1] のみ)
                bool start_still = r["stillness_required"] | false;
                bool end_still   = r["end_stillness_required"] | false;
                uint16_t still_win   = r["stillness_window_ms"] | 200;
                uint8_t  still_amg   = (uint8_t)(r["stillness_accel_th_mg"] | 100);
                uint8_t  still_gdps  = (uint8_t)(r["stillness_gyro_th_dps"] | 5);

                // 開始キー解決 (state[0] AT_PRESS、state[N-1] AT_RELEASE で同じ keys を使う)
                const char* key = r["key"] | "";
                uint8_t modifiers = r["modifiers"] | 0;
                uint8_t shared_keys[8];
                uint8_t shared_keys_len = 0;
                if (key[0]) {
                    uint8_t key_code = parseKeyName(key);
                    if (key_code != 0) {
                        if (modifiers & 0x01) shared_keys[shared_keys_len++] = 0x80;
                        if (modifiers & 0x02) shared_keys[shared_keys_len++] = 0x81;
                        if (modifiers & 0x04) shared_keys[shared_keys_len++] = 0x82;
                        if (modifiers & 0x08) shared_keys[shared_keys_len++] = 0x83;
                        shared_keys[shared_keys_len++] = key_code;
                    }
                }

                // Helper: posture (euler + tol) を State.match_condition.posture に設定
                auto applyPosture = [&](State& st, JsonObject po) {
                    st.match_condition.logic_op = LogicOp::OP_AND;
                    if (po.isNull()) return;
                    st.match_condition.posture.enabled = true;
                    st.match_condition.posture.judge_by = PostureJudge::BY_EULER;
                    JsonArray e  = po["euler"];
                    JsonArray et = po["euler_tol"];
                    for (int i = 0; i < 3; i++) {
                        st.match_condition.posture.euler[i]     = e  ? e[i].as<float>()  : 0.0f;
                        st.match_condition.posture.euler_tol[i] = et ? et[i].as<float>() : 180.0f;
                    }
                    if (po["quat"].is<JsonArray>()) {
                        JsonArray q = po["quat"];
                        for (int i = 0; i < 4; i++) {
                            st.match_condition.posture.quat[i] = q[i].as<float>();
                        }
                    } else {
                        st.match_condition.posture.quat[0] = 1.0f;
                        st.match_condition.posture.quat[1] = 0.0f;
                        st.match_condition.posture.quat[2] = 0.0f;
                        st.match_condition.posture.quat[3] = 0.0f;
                    }
                    st.match_condition.posture.quat_dot_min = po["quat_dot_min"] | 0.95f;
                };

                // state[0]: 開始
                {
                    State& st = rule.states[0];
                    JsonObject po = r["start_posture"];
                    applyPosture(st, po);
                    if (btn_enabled) {
                        st.match_condition.button.enabled = true;
                        st.match_condition.button.idx = btn_idx;
                        st.match_condition.button.state = btn_state;
                    }
                    if (start_still) {
                        st.match_condition.stillness_required = true;
                        st.match_condition.stillness_window_ms = still_win;
                        st.match_condition.stillness_accel_th_mg = still_amg;
                        st.match_condition.stillness_gyro_th_dps = still_gdps;
                    }
                    // on_enter = AT_PRESS (shared_keys)
                    if (shared_keys_len > 0) {
                        st.on_enter.type = ActionType::AT_PRESS;
                        for (uint8_t i = 0; i < shared_keys_len; i++) {
                            st.on_enter.keys[i] = shared_keys[i];
                        }
                        st.on_enter.keys_len = shared_keys_len;
                        st.on_enter.modifiers = modifiers;
                    }
                    st.max_dwell_ms = r["start_max_dwell_ms"] | 0;
                }

                // state[1..mid_count]: 中間
                JsonArray mids = r["mid_postures"].as<JsonArray>();
                for (int i = 0; i < mid_count; i++) {
                    State& st = rule.states[1 + i];
                    JsonObject po = mids[i].as<JsonObject>();
                    applyPosture(st, po);
                    // 中間 state はボタン継続が常識的: btn が pressed なら継続要求、released なら無視
                    if (btn_enabled) {
                        st.match_condition.button.enabled = true;
                        st.match_condition.button.idx = btn_idx;
                        st.match_condition.button.state = btn_state;
                    }
                    st.on_enter.type = ActionType::AT_NONE;
                    st.max_dwell_ms = r["mid_max_dwell_ms"] | 0;
                }

                // state[N-1]: 終了
                {
                    uint8_t last_idx = (uint8_t)(1 + mid_count);
                    State& st = rule.states[last_idx];
                    JsonObject po = r["end_posture"];
                    applyPosture(st, po);
                    // 終了側はボタン release を待つ運用が多いが、UI 仕様により button_idx を維持
                    // (release-on-button-release 構成は hold_start_end と整合)
                    // ここではボタン条件を付与せず、姿勢一致のみで遷移 (シンプル MVP)
                    if (end_still) {
                        st.match_condition.stillness_required = true;
                        st.match_condition.stillness_window_ms = still_win;
                        st.match_condition.stillness_accel_th_mg = still_amg;
                        st.match_condition.stillness_gyro_th_dps = still_gdps;
                    }
                    // on_enter = AT_RELEASE (shared_keys)
                    if (shared_keys_len > 0) {
                        st.on_enter.type = ActionType::AT_RELEASE;
                        for (uint8_t i = 0; i < shared_keys_len; i++) {
                            st.on_enter.keys[i] = shared_keys[i];
                        }
                        st.on_enter.keys_len = shared_keys_len;
                        st.on_enter.modifiers = modifiers;
                    }
                    st.max_dwell_ms = r["end_max_dwell_ms"] | 0;
                }

                g_engine.addRule(rule);
                autoSaveActiveProfile();
                out["type"] = "ack";
                out["cmd"] = "rule.add";
                out["ok"] = true;
                out["id"] = rule.id;
                out["mode"] = "hold_with_waypoints";
                out["states"] = (uint32_t)rule.states_count;
                out["posture_basis"] = (rule.posture_basis == PB_RELATIVE_QUAT) ? "relative" : "absolute";
                out["rule_count"] = (uint32_t)g_engine.ruleCount();
                return;
            }
        }

        const char* mode = r["ui_mode"] | "oneshot";
        bool is_hold_only = strcmp(mode, "hold_start_only") == 0;
        bool is_hold_end  = strcmp(mode, "hold_start_end") == 0;
        rule.loop = is_hold_only || is_hold_end;
        rule.states_count = is_hold_end ? 2 : 1;

        // states[0] 開始条件
        State& s0 = rule.states[0];
        s0.match_condition.logic_op = LogicOp::OP_AND;
        // accel ABS 閾値
        if (r["accel_abs_threshold"].is<float>() || r["accel_abs_threshold"].is<int>()) {
            s0.match_condition.accel.enabled = true;
            s0.match_condition.accel.use_per_axis = false;
            s0.match_condition.accel.abs_threshold = r["accel_abs_threshold"];
            s0.match_condition.accel.comparison = Comparison::CMP_GTE;
        }
        // 姿勢 (Euler) 閾値: roll/pitch/yaw 中央値+許容
        if (r["posture"].is<JsonObject>()) {
            JsonObject p = r["posture"];
            s0.match_condition.posture.enabled = true;
            // judge_by: "euler" (default) or "quat"
            const char* jb = p["judge_by"] | "euler";
            s0.match_condition.posture.judge_by =
                (strcmp(jb, "quat") == 0) ? PostureJudge::BY_QUAT : PostureJudge::BY_EULER;
            JsonArray e = p["euler"];
            JsonArray t = p["euler_tol"];
            for (int i = 0; i < 3; i++) {
                // float 値もちゃんと取得 (Web から 90.0 / -1.0 等が送られる)
                s0.match_condition.posture.euler[i]     = e ? e[i].as<float>() : 0.0f;
                s0.match_condition.posture.euler_tol[i] = t ? t[i].as<float>() : 180.0f;
            }
            // quat_dot_min (BY_QUAT 用)
            s0.match_condition.posture.quat_dot_min = p["quat_dot_min"] | 0.95f;
            // quat も保存 (rule.closest で使う、Web から指定または現在値)
            if (p["quat"].is<JsonArray>()) {
                JsonArray q = p["quat"];
                for (int i = 0; i < 4; i++) {
                    s0.match_condition.posture.quat[i] = q ? (float)(q[i] | 0) : (i == 0 ? 1.0f : 0.0f);
                }
            } else {
                // posture 指定だけで quat なしなら現在の sensor quat を使う
                for (int i = 0; i < 4; i++) {
                    s0.match_condition.posture.quat[i] = g_sensor_state.quat[i];
                }
            }
        }
        // ボタン
        if (r["button_idx"].is<uint8_t>() || r["button_idx"].is<int>()) {
            uint8_t bidx = r["button_idx"];
            if (bidx > 0 && bidx <= 15) {
                s0.match_condition.button.enabled = true;
                s0.match_condition.button.idx = bidx;
                // 0=pressed (default), 1=released, 2=any
                s0.match_condition.button.state = r["button_state"] | 0;
            }
        }

        // on_enter Action
        // 入力指定の優先順位:
        //   1. r["keys"] = ["DOWN","RIGHT","P"] のような配列 → FIRE_MACRO で順次送信
        //   2. r["text"] = "Hello" → ASCII 各文字を順次送信
        //   3. r["key"]  = "ARROW_RIGHT" or "a" → 単一キー (FIRE_ONCE / HOLD)
        const char* key = r["key"] | "";
        const char* text = r["text"] | "";
        uint8_t modifiers = r["modifiers"] | 0;   // Ctrl=1, Shift=2, Alt=4, GUI=8
        s0.on_enter.modifiers = modifiers;
        s0.on_enter.interval_ms = r["interval_ms"] | 30;
        bool keys_handled = false;
        if (r["keys"].is<JsonArray>()) {
            // 連続入力 (macro): 各要素を parseKeyName で HID code に変換
            // Phase 5.14: prefix 対応
            //   "+KEY"  = press のみ (key_mode=PRESS、追加押下)
            //   "-KEY"  = release のみ (key_mode=RELEASE)
            //   "!"     = 全 release (key_mode=RELEASE_ALL、key 無視)
            //   "KEY"   = press → wait → release (key_mode=FIRE、デフォルト)
            JsonArray ks = r["keys"];
            uint8_t n = 0;
            for (JsonVariant v : ks) {
                if (n >= 24) break;
                const char* kn = v.as<const char*>();
                if (!kn || !kn[0]) continue;
                uint8_t mode = KeyMacroMode::FIRE;
                const char* keyStart = kn;
                if (kn[0] == '!') {
                    s0.on_enter.keys[n] = 0;
                    s0.on_enter.key_modes[n] = KeyMacroMode::RELEASE_ALL;
                    n++;
                    continue;
                }
                if (kn[0] == '+') { mode = KeyMacroMode::PRESS; keyStart = kn + 1; }
                else if (kn[0] == '-') { mode = KeyMacroMode::RELEASE; keyStart = kn + 1; }
                uint8_t code = parseKeyName(keyStart);
                if (code != 0) {
                    s0.on_enter.keys[n] = code;
                    s0.on_enter.key_modes[n] = mode;
                    n++;
                }
            }
            if (n > 0) {
                s0.on_enter.type = ActionType::AT_FIRE_MACRO;
                s0.on_enter.keys_len = n;
                keys_handled = true;
            }
        }
        if (!keys_handled && text[0]) {
            // FIRE_MACRO で text の各文字を順次送信
            s0.on_enter.type = ActionType::AT_FIRE_MACRO;
            uint8_t n = 0;
            for (size_t i = 0; text[i] && n < 24; i++, n++) {
                s0.on_enter.keys[n] = (uint8_t)text[i];
            }
            s0.on_enter.keys_len = n;
            keys_handled = true;
        }
        if (!keys_handled && key[0]) {
            // 単一キー (名前付き or ASCII 1 文字)
            uint8_t key_code = parseKeyName(key);
            if (key_code != 0) {
                uint8_t key_count = 0;
                if (modifiers & 0x01) s0.on_enter.keys[key_count++] = 0x80;  // KEY_LEFT_CTRL
                if (modifiers & 0x02) s0.on_enter.keys[key_count++] = 0x81;  // KEY_LEFT_SHIFT
                if (modifiers & 0x04) s0.on_enter.keys[key_count++] = 0x82;  // KEY_LEFT_ALT
                if (modifiers & 0x08) s0.on_enter.keys[key_count++] = 0x83;  // KEY_LEFT_GUI
                s0.on_enter.keys[key_count++] = key_code;
                s0.on_enter.keys_len = key_count;
                if (is_hold_only) {
                    s0.on_enter.type = ActionType::AT_PRESS;
                    s0.on_exit.type = ActionType::AT_RELEASE;
                    memcpy(s0.on_exit.keys, s0.on_enter.keys, key_count);
                    s0.on_exit.keys_len = key_count;
                } else {
                    s0.on_enter.type = ActionType::AT_FIRE_ONCE;
                    s0.on_enter.duration_ms = r["duration_ms"] | 30;
                }
            }
        }

        // HOLD_START_END: 開始姿勢でキー press、終了姿勢で release
        // end_key 指定時は s2 を追加して別キーを終了姿勢時に FIRE_ONCE する
        if (is_hold_end) {
            // s0 (開始) を press タイプに変更
            s0.on_enter.type = ActionType::AT_PRESS;
            // s1 (終了)
            State& s1 = rule.states[1];
            s1.match_condition.logic_op = LogicOp::OP_AND;
            if (r["end_posture"].is<JsonObject>()) {
                s1.match_condition.posture.enabled = true;
                s1.match_condition.posture.judge_by = PostureJudge::BY_EULER;
                JsonObject p = r["end_posture"];
                JsonArray e = p["euler"];
                JsonArray t = p["euler_tol"];
                for (int i = 0; i < 3; i++) {
                    s1.match_condition.posture.euler[i] = e ? (float)(e[i] | 0) : 0;
                    s1.match_condition.posture.euler_tol[i] = t ? (float)(t[i] | 180) : 180;
                }
                // end_posture も quat 保存 (closest 計算 / inspection 用)
                if (p["quat"].is<JsonArray>()) {
                    JsonArray q = p["quat"];
                    for (int i = 0; i < 4; i++) {
                        s1.match_condition.posture.quat[i] =
                            q ? (float)(q[i] | 0) : (i == 0 ? 1.0f : 0.0f);
                    }
                }
            }
            // 終了状態 入場 → release (s0 と同じ keys、modifiers 含む)
            s1.on_enter.type = ActionType::AT_RELEASE;
            memcpy(s1.on_enter.keys, s0.on_enter.keys, s0.on_enter.keys_len);
            s1.on_enter.keys_len = s0.on_enter.keys_len;

            // end_key が指定されていれば s2 を追加: 即遷移して FIRE_ONCE
            // s2.match_condition は全 disabled → matchCondition は常に true → 次 tick で即遷移
            // s2 から先 (loop で s0 戻り) は s0.match_condition (開始姿勢) 待ち
            const char* end_key = r["end_key"] | "";
            if (end_key[0] && rule.states_count < 4) {
                rule.states_count = 3;
                State& s2 = rule.states[2];
                s2.match_condition.logic_op = LogicOp::OP_AND;
                // posture/button/accel/gyro 全部 disabled = 常時 true
                uint8_t end_mods = r["end_modifiers"] | 0;
                uint8_t kc = 0;
                if (end_mods & 0x01) s2.on_enter.keys[kc++] = 0x80;  // KEY_LEFT_CTRL
                if (end_mods & 0x02) s2.on_enter.keys[kc++] = 0x81;  // KEY_LEFT_SHIFT
                if (end_mods & 0x04) s2.on_enter.keys[kc++] = 0x82;  // KEY_LEFT_ALT
                if (end_mods & 0x08) s2.on_enter.keys[kc++] = 0x83;  // KEY_LEFT_GUI
                s2.on_enter.keys[kc++] = (uint8_t)end_key[0];
                s2.on_enter.keys_len = kc;
                s2.on_enter.modifiers = end_mods;
                s2.on_enter.type = ActionType::AT_FIRE_ONCE;
                s2.on_enter.duration_ms = r["end_duration_ms"] | 30;
            }
        }

        g_engine.addRule(rule);
        autoSaveActiveProfile();
        out["type"] = "ack";
        out["cmd"] = "rule.add";
        out["ok"] = true;
        out["id"] = rule.id;
        out["rule_count"] = (uint32_t)g_engine.ruleCount();
    }
    // ================ HW ボタン GPIO 構成 (Phase 5.1) ================
#if defined(BOARD_M5STICKC)
    else if (strcmp(cmd, "hw.buttons.get") == 0) {
        // {"cmd":"hw.buttons.get"}
        // ← {"type":"hw.buttons","buttons":[{"idx":1,"gpio":0,"active_low":true,"pull_mode":1},...]}
        out["type"] = "hw.buttons";
        JsonArray arr = out["buttons"].to<JsonArray>();
        const auto& cfg = g_buttons.config();
        for (size_t i = 0; i < cfg.size(); i++) {
            JsonObject b = arr.add<JsonObject>();
            b["idx"] = (uint8_t)(i + 1);
            b["gpio"] = cfg[i].gpio;
            b["active_low"] = cfg[i].active_low;
            b["pull_mode"] = cfg[i].pull_mode;
        }
        out["bitmap"] = g_buttons.bitmap();
    }
    else if (strcmp(cmd, "hw.buttons.set") == 0) {
        // {"cmd":"hw.buttons.set","buttons":[
        //   {"gpio":0,"active_low":true,"pull_mode":1},
        //   {"gpio":36,"active_low":false,"pull_mode":0},
        //   {"gpio":26,"active_low":false,"pull_mode":1}
        // ]}
        // pull_mode: 0=INPUT, 1=PULLUP, 2=PULLDOWN
        JsonArray arr = in["buttons"];
        if (arr.isNull() || arr.size() == 0) {
            out["type"] = "err"; out["cmd"] = "hw.buttons.set"; out["err"] = "missing_buttons";
            return;
        }
        if (arr.size() > ButtonsGpio::MAX_BUTTONS) {
            out["type"] = "err"; out["cmd"] = "hw.buttons.set"; out["err"] = "too_many";
            out["max"] = (uint8_t)ButtonsGpio::MAX_BUTTONS;
            return;
        }
        std::vector<ButtonConfig> cfg;
        for (JsonObject b : arr) {
            ButtonConfig bc;
            bc.gpio = b["gpio"] | 0;
            bc.active_low = b["active_low"] | true;
            bc.pull_mode = b["pull_mode"] | 1;
            cfg.push_back(bc);
        }
        bool ok = g_buttons.setConfig(cfg);
        out["type"] = ok ? "ack" : "err";
        out["cmd"] = "hw.buttons.set";
        out["ok"] = ok;
        out["count"] = (uint8_t)cfg.size();
        if (!ok) out["err"] = "nvs_write_failed";
    }
    else if (strcmp(cmd, "hw.buttons.reset") == 0) {
        // NVS クリア + デフォルト (G0/G36/G26) に戻す
        g_buttons.clearNvs();
        g_buttons.begin(ButtonsGpio::defaultM5StickC());
        out["type"] = "ack";
        out["cmd"] = "hw.buttons.reset";
        out["ok"] = true;
        out["count"] = (uint8_t)g_buttons.buttonCount();
    }
#endif
    else {
        out["type"] = "err";
        out["cmd"] = cmd;
        out["err"] = "unknown_cmd";
    }
}

// =========================================================
// センサ状態の更新 (100Hz 推奨)
// =========================================================
static void updateSensor() {
    if (!g_imu_ok) return;
    float accel[3], gyro_rad[3];
    if (!g_imu.read(accel, gyro_rad)) return;

    // Gyro bias 補正 (calibrate.simple で取得済の場合)
    gyro_rad[0] -= g_gyro_bias_rad[0];
    gyro_rad[1] -= g_gyro_bias_rad[1];
    gyro_rad[2] -= g_gyro_bias_rad[2];

    // Accel bias + scale 補正 (calibrate.full で取得済の場合)
    if (g_accel_calibrated) {
        accel[0] = (accel[0] - g_accel_bias[0]) * g_accel_scale[0];
        accel[1] = (accel[1] - g_accel_bias[1]) * g_accel_scale[1];
        accel[2] = (accel[2] - g_accel_bias[2]) * g_accel_scale[2];
    }

    uint32_t now = millis();
    float dt = (g_last_imu_ms == 0) ? 0.01f : ((now - g_last_imu_ms) * 0.001f);
    if (dt > 0.5f) dt = 0.5f;   // 1 回目や停止後のクランプ
    g_last_imu_ms = now;

    // Mahony update (accel in g)
    g_mahony.update(gyro_rad[0], gyro_rad[1], gyro_rad[2],
                    accel[0]/9.80665f, accel[1]/9.80665f, accel[2]/9.80665f, dt);

    g_sensor_state.accel[0] = accel[0];
    g_sensor_state.accel[1] = accel[1];
    g_sensor_state.accel[2] = accel[2];
    // gyro は deg/s で stream する方が UI 読みやすいので変換しておく
    constexpr float RAD2DEG = 57.29577951308232f;
    g_sensor_state.gyro[0] = gyro_rad[0] * RAD2DEG;
    g_sensor_state.gyro[1] = gyro_rad[1] * RAD2DEG;
    g_sensor_state.gyro[2] = gyro_rad[2] * RAD2DEG;

    float accel_g_x = accel[0] / 9.80665f;
    float accel_g_y = accel[1] / 9.80665f;
    float accel_g_z = accel[2] / 9.80665f;
    g_sensor_state.accel_abs = sqrtf(accel_g_x*accel_g_x + accel_g_y*accel_g_y + accel_g_z*accel_g_z);
    g_sensor_state.gyro_abs = sqrtf(g_sensor_state.gyro[0]*g_sensor_state.gyro[0] +
                                    g_sensor_state.gyro[1]*g_sensor_state.gyro[1] +
                                    g_sensor_state.gyro[2]*g_sensor_state.gyro[2]);

    g_mahony.getQuat(g_sensor_state.quat);
    g_mahony.getEuler(g_sensor_state.euler[0], g_sensor_state.euler[1], g_sensor_state.euler[2]);
    g_sensor_state.timestamp_ms = now;

#if defined(BOARD_M5STICKC)
    // 物理ボタン bitmap 更新 (button_idx 1=BtnA bit0, 2=BtnB bit1)
    g_sensor_state.buttons_bitmap = g_buttons.update(now);
#endif

    // Phase 5.39.2.6: Button sim override (btn.sim コマンドで設定された bit を上書き)
    if (g_btn_sim_mask != 0) {
        g_sensor_state.buttons_bitmap =
            (g_sensor_state.buttons_bitmap & ~g_btn_sim_mask) | (g_btn_sim_value & g_btn_sim_mask);
    }

    // Phase 5.32: Mode 別に tick を切替
    if (g_device_mode == DeviceMode::MOUSE) {
        runMouseModeTick(g_sensor_state);
    } else {
        g_engine.tick(g_sensor_state);
    }
}

// =========================================================
// sensor stream 出力
// =========================================================
static void streamSensorIfDue() {
    if (g_stream_rate_hz == 0) return;
    uint32_t interval = 1000 / g_stream_rate_hz;
    uint32_t now = millis();
    if ((now - g_last_stream_ms) < interval) return;
    g_last_stream_ms = now;

    JsonDocument doc;
    doc["type"] = "sensor";
    doc["t"] = g_sensor_state.timestamp_ms;
    doc["ax"] = g_sensor_state.accel[0];
    doc["ay"] = g_sensor_state.accel[1];
    doc["az"] = g_sensor_state.accel[2];
    doc["gx"] = g_sensor_state.gyro[0];
    doc["gy"] = g_sensor_state.gyro[1];
    doc["gz"] = g_sensor_state.gyro[2];
    doc["pitch"] = g_sensor_state.euler[1];
    doc["roll"]  = g_sensor_state.euler[0];
    doc["yaw"]   = g_sensor_state.euler[2];
    doc["qw"] = g_sensor_state.quat[0];
    doc["qx"] = g_sensor_state.quat[1];
    doc["qy"] = g_sensor_state.quat[2];
    doc["qz"] = g_sensor_state.quat[3];
    doc["btn"] = g_sensor_state.buttons_bitmap;
    g_serial.sendJson(doc);
    // BLE NUS 接続中なら同じデータを notify
    if (g_ble_nus_started && g_ble_nus.isConnected()) {
        g_ble_nus.sendJson(doc);
    }
}

// =========================================================
// setup / loop
// =========================================================
void setup() {
    g_boot_ms = millis();

    // Serial 最優先で初期化 (何よりも先に print できるように)
    g_serial.begin(SERIAL_BAUD);
    delay(100);
    g_serial.setHandler(handleCommand);

    // LittleFS init
    bool fs_ok = Profile::init();
    {
        JsonDocument doc;
        doc["type"] = "boot";
        doc["stage"] = "fs_init";
        doc["fs_ok"] = fs_ok;
        g_serial.sendJson(doc);
    }

    // 起動メッセージ (IMU 初期化前に送る、hang 検出用)
    {
        JsonDocument doc;
        doc["type"] = "boot";
        doc["fw"] = FW_VERSION;
        doc["board"] =
            #if defined(BOARD_M5STICKC)
                "m5stickc"
            #else
                "unknown"
            #endif
        ;
        doc["stage"] = "pre_imu_init";
        g_serial.sendJson(doc);
    }

#if defined(BOARD_M5STICKC)
    // AXP192 初期化 (LCD 電源 ON、bias 等)
    g_axp_ok = g_axp.begin();
    delay(50);
    // LCD 初期化
    g_lcd_ok = g_lcd.begin();
    if (g_lcd_ok) {
        g_lcd.fillScreen(M5StickCDisplay::BLACK);
        g_lcd.drawString(2, 4, "Burst Motion", M5StickCDisplay::WHITE, M5StickCDisplay::BLACK, 1);
        g_lcd.drawString(2, 14, FW_VERSION, M5StickCDisplay::CYAN, M5StickCDisplay::BLACK, 1);
        g_lcd.drawString(2, 26, "booting...", M5StickCDisplay::YELLOW, M5StickCDisplay::BLACK, 1);
    }
    {
        JsonDocument doc;
        doc["type"] = "boot";
        doc["stage"] = "axp_lcd";
        doc["axp_ok"] = g_axp_ok;
        doc["lcd_ok"] = g_lcd_ok;
        g_serial.sendJson(doc);
    }
#endif

    // 物理ボタン初期化 (デフォルト: GPIO 0/36/26 = レガシー M5StickC 互換)
    // NVS に保存済みの構成があれば優先、なければ defaultM5StickC を使う
    g_buttons.begin(ButtonsGpio::defaultM5StickC());

    // 6 点キャリブ結果を NVS から復元 (Phase 5.9)
    loadCalibFromNvs();
    // Device mode を NVS から復元 (Phase 5.32)
    loadDeviceModeNvs();

    // IMU 初期化 (失敗しても FW は動作継続)
    g_imu_ok = g_imu.begin();

    {
        JsonDocument doc;
        doc["type"] = "boot";
        doc["stage"] = "post_imu_init";
        doc["imu"] = g_imu.typeName();
        doc["imu_ok"] = g_imu_ok;
        g_serial.sendJson(doc);
    }

    // 起動時自動 gyro bias キャリブレーション (静止前提、500ms × 50 サンプル)
    // ユーザーが置いてから電源 ON すれば drift 抑止される
    if (g_imu_ok) {
        delay(200);  // 起動振動を待つ
        double sx=0, sy=0, sz=0;
        const int N = 50;
        int got = 0;
        for (int i = 0; i < N; i++) {
            float a[3], gv[3];
            if (g_imu.read(a, gv)) {
                sx += gv[0]; sy += gv[1]; sz += gv[2];
                got++;
            }
            delay(10);
        }
        if (got > 0) {
            g_gyro_bias_rad[0] = (float)(sx / got);
            g_gyro_bias_rad[1] = (float)(sy / got);
            g_gyro_bias_rad[2] = (float)(sz / got);
            g_gyro_calibrated = true;
            JsonDocument d;
            d["type"] = "boot";
            d["stage"] = "auto_calibrated";
            constexpr float RAD2DEG = 57.29577951308232f;
            JsonArray bias = d["gyro_bias_dps"].to<JsonArray>();
            bias.add(g_gyro_bias_rad[0] * RAD2DEG);
            bias.add(g_gyro_bias_rad[1] * RAD2DEG);
            bias.add(g_gyro_bias_rad[2] * RAD2DEG);
            g_serial.sendJson(d);
        }
    }

    g_engine.setHidSink(&g_ble_hid);

    // Phase 5.39.2.7: watch event (trigger.hit / lock) を USB Serial + BLE NUS 両方に routing
    g_engine.setEventOutputFn([](JsonDocument& doc) {
        g_serial.sendJson(doc);
        if (g_ble_nus_started && g_ble_nus.isConnected()) {
            g_ble_nus.sendJson(doc);
        }
    });

    // Active profile があれば自動ロード
    String active = Profile::getActive();
    if (active.length() > 0) {
        std::vector<ActionRule> rules;
        JsonDocument errOut;
        if (Profile::load(active.c_str(), rules, errOut)) {
            for (auto& r : rules) g_engine.addRule(r);
            JsonDocument d;
            d["type"] = "boot";
            d["stage"] = "profile_loaded";
            d["name"] = active;
            d["rule_count"] = (uint32_t)rules.size();
            g_serial.sendJson(d);
        }
    }

    // BLE HID + BLE NUS を自動開始 (Phase 5.11)
    // 設定中に誤入力したくない場合は ble.stop コマンドで停止可能
    g_ble_hid.begin();
    g_ble_hid_started = true;
    if (g_ble_nus.begin()) {
        g_ble_nus.setHandler(handleCommand);
        g_ble_nus_started = true;
    }
    // Phase 5.31: 接続安定化 (MTU 247 + ConnParams 15-30ms / 4s timeout)
    tuneBleConnection();
    {
        JsonDocument d;
        d["type"] = "boot";
        d["stage"] = "ble_auto_started";
        d["ble_hid"] = g_ble_hid_started;
        d["ble_nus"] = g_ble_nus_started;
        g_serial.sendJson(d);
    }
}

#if defined(BOARD_M5STICKC)
// LCD 更新を行毎に分散 + 差分描画で stream blocking 回避
//   1 ループあたり 1 行だけ描画 (~5ms)、全 10 行で 500ms cycle
//   前回と同じ文字列なら描画スキップ
static char s_lcd_prev[12][32] = {{0}};

static void updateLcdRow(uint32_t now, int row) {
    char buf[32] = {0};
    uint16_t fg = M5StickCDisplay::WHITE;
    uint16_t y = 2 + row * 12;
    if (y > 158) return;

    switch (row) {
        case 0:  // タイトル
            strcpy(buf, "Burst Motion");
            fg = M5StickCDisplay::WHITE;
            break;
        case 1:  // FW
            strncpy(buf, FW_VERSION, sizeof(buf) - 1);
            fg = M5StickCDisplay::CYAN;
            break;
        case 2:  // BLE
            snprintf(buf, sizeof(buf), "BLE:%s",
                g_ble_hid_started ? (g_ble_hid.isConnected() ? "PAIR" : "ADV") : "OFF");
            fg = g_ble_hid.isConnected() ? M5StickCDisplay::GREEN :
                 g_ble_hid_started ? M5StickCDisplay::YELLOW : M5StickCDisplay::GRAY;
            break;
        case 3:  // Battery %
            if (g_axp_ok) {
                snprintf(buf, sizeof(buf), "Bat:%3d%%%s",
                    g_battery_percent, g_battery_charging ? "+" : "");
                fg = g_battery_percent > 30 ? M5StickCDisplay::GREEN :
                     g_battery_percent > 10 ? M5StickCDisplay::YELLOW : M5StickCDisplay::RED;
                if (g_battery_charging) fg = M5StickCDisplay::CYAN;
            }
            break;
        case 4:  // 電圧
            if (g_axp_ok) {
                snprintf(buf, sizeof(buf), "%.2fV", g_battery_voltage);
                fg = M5StickCDisplay::WHITE;
            }
            break;
        case 5:  // Rules
            snprintf(buf, sizeof(buf), "Rules:%d", (int)g_engine.ruleCount());
            fg = M5StickCDisplay::WHITE;
            break;
        case 6:  // uptime
        {
            uint32_t up_s = (now - g_boot_ms) / 1000;
            snprintf(buf, sizeof(buf), "%dh%02dm%02ds",
                (int)(up_s/3600), (int)((up_s/60)%60), (int)(up_s%60));
            fg = M5StickCDisplay::GRAY;
            break;
        }
        case 7:  // Roll
            snprintf(buf, sizeof(buf), "R:%+4d", (int)g_sensor_state.euler[0]);
            fg = M5StickCDisplay::CYAN;
            break;
        case 8:  // Pitch
            snprintf(buf, sizeof(buf), "P:%+4d", (int)g_sensor_state.euler[1]);
            fg = M5StickCDisplay::CYAN;
            break;
        case 9:  // Yaw
            snprintf(buf, sizeof(buf), "Y:%+4d", (int)g_sensor_state.euler[2]);
            fg = M5StickCDisplay::CYAN;
            break;
        case 10:  // ABS accel
            snprintf(buf, sizeof(buf), "G:%4.2f", g_sensor_state.accel_abs);
            fg = M5StickCDisplay::ORANGE;
            break;
        case 11:  // profile
        {
            String act = Profile::getActive();
            if (act.length() > 0) {
                snprintf(buf, sizeof(buf), "%s", act.c_str());
                fg = M5StickCDisplay::MAGENTA;
            }
            break;
        }
        default:
            return;
    }

    // 差分検出: 文字列が同じなら描画しない (大幅な負荷削減)
    if (strncmp(s_lcd_prev[row], buf, sizeof(buf)) == 0) return;
    strncpy(s_lcd_prev[row], buf, sizeof(s_lcd_prev[row]) - 1);

    // 行全体を bg で消去してから描画 (短くなった場合の残像クリア)
    g_lcd.fillRect(2, y, M5StickCDisplay::WIDTH - 2, 8, M5StickCDisplay::BLACK);
    if (buf[0]) {
        g_lcd.drawString(2, y, buf, fg, M5StickCDisplay::BLACK, 1);
    }
}

static void updateLcd(uint32_t now) {
    if (!g_lcd_ok) return;
    // Stream ON 中はゆっくり (100ms × 12 = 1.2s cycle)、OFF なら早め (50ms × 12 = 600ms cycle)
    // Stream の throughput 優先
    uint32_t interval = (g_stream_rate_hz > 0) ? 100 : 50;
    static uint32_t last_row_update = 0;
    static int next_row = 0;
    if ((now - last_row_update) < interval) return;
    last_row_update = now;

    // バッテリは行 3,4 を更新する直前に読む (ADC アクセス)
    if ((next_row == 3) && g_axp_ok) {
        g_battery_voltage = g_axp.batteryVoltage();
        g_battery_percent = g_axp.batteryPercent();
        g_battery_charging = g_axp.isUsbConnected();
    }

    updateLcdRow(now, next_row);
    next_row = (next_row + 1) % 12;
}
#endif

#if defined(BOARD_M5STICKC)
// 電源ボタン状態 (AXP192 PEK)
//   short press: シリアルストリーム ON/OFF トグル
//   long press : 4秒で AXP がシャットダウン (ハード処理)、その前に通知
static void handlePowerButton(uint32_t now) {
    static uint32_t last_check = 0;
    if (!g_axp_ok) return;
    if ((now - last_check) < 100) return;  // 100ms 毎にチェック
    last_check = now;

    uint8_t st = g_axp.powerButtonState();
    if (st == 0) return;

    JsonDocument doc;
    doc["type"] = "power_button";
    if (st == 2) {
        // short press → sensor stream toggle
        doc["press"] = "short";
        if (g_stream_rate_hz == 0) {
            g_stream_rate_hz = 50;
            doc["action"] = "stream_on";
        } else {
            g_stream_rate_hz = 0;
            doc["action"] = "stream_off";
        }
    } else if (st == 1) {
        // long press → BLE start (もし未起動なら)
        doc["press"] = "long";
        if (!g_ble_hid_started) {
            g_ble_hid.begin();
            g_ble_hid_started = true;
            if (g_ble_nus.begin()) {
                g_ble_nus.setHandler(handleCommand);
                g_ble_nus_started = true;
            }
            tuneBleConnection();  // Phase 5.31
            doc["action"] = "ble_start";
        } else {
            doc["action"] = "ble_already_on";
        }
    }
    g_serial.sendJson(doc);
}
#endif

void loop() {
    uint32_t now = millis();

    // 1. Serial command 処理を最優先 (応答性確保)
    g_serial.process();

    // 2. IMU 100Hz
    static uint32_t last_imu_loop = 0;
    if ((now - last_imu_loop) >= 10) {
        last_imu_loop = now;
        updateSensor();
    }

    // 3. Sensor stream (USB Serial と BLE NUS 両方に送信)
    streamSensorIfDue();

    // 4. Serial 再処理 (LCD 描画前に最新コマンドを処理)
    g_serial.process();

#if defined(BOARD_M5STICKC)
    // 5. LCD 周期更新 (1 行ずつ分散、~5ms blocking)
    updateLcd(now);

    // 6. 電源ボタン処理
    handlePowerButton(now);
#endif

    // Cooperative yield、ただし sleep しすぎない
    // 1ms より短い周期は WDT 等への影響あるので 1ms 維持
    delay(1);
}
