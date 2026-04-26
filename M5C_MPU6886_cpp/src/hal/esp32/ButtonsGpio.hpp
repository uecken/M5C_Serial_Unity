// Burst Motion - hal/esp32/ButtonsGpio.hpp
// 動的ボタン GPIO 構成 (M5 lib 非依存)
//
// 規約:
//   button_idx は 1-origin、bitmap は 0-bit から
//   設定は std::vector<ButtonConfig> で実行時可変、`hw.buttons.set` コマンドで上書き可能
//   NVS Preferences `bm_buttons` キーに永続化
//
// レガシー (M5StickC + 旧 motion_controller) のデフォルト:
//   button_idx=1: GPIO 0,  active_low,  INPUT_PULLUP   (BOOT 兼用、SW で GND に落として押下判定)
//   button_idx=2: GPIO 36, active_high, INPUT          (input only、3.3V を入力として押下判定)
//   button_idx=3: GPIO 26, active_low,  INPUT_PULLUP   (SW で GND に落として押下判定、ユーザー実機配線)
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <vector>

namespace BurstMotion {

struct ButtonConfig {
    uint8_t gpio;          // GPIO ピン番号
    bool active_low;       // true: 押下時 LOW (active_low pull-up 構成)、false: 押下時 HIGH
    uint8_t pull_mode;     // 0=INPUT (浮遊), 1=INPUT_PULLUP, 2=INPUT_PULLDOWN
};

class ButtonsGpio {
public:
    static constexpr const char* NVS_NS = "bm_btn";
    static constexpr const char* NVS_KEY = "cfg";
    static constexpr size_t MAX_BUTTONS = 8;

    // レガシー M5StickC の 3 ボタン構成 (旧 MotionController.hpp:185 互換、ユーザー実機検証済み)
    static std::vector<ButtonConfig> defaultM5StickC() {
        return {
            { 0,  true,  1 },   // GPIO 0,  active_low,  PU    (BOOT 兼用、SW で GND)
            { 36, false, 0 },   // GPIO 36, active_high, INPUT (input only、3.3V 入力で押下)
            { 26, true,  1 },   // GPIO 26, active_low,  PU    (SW で GND)
        };
    }

    // 起動時に NVS から構成ロード、なければ default を採用
    void begin(const std::vector<ButtonConfig>& fallback) {
        Preferences prefs;
        if (prefs.begin(NVS_NS, true /* readonly */)) {
            size_t blob_size = prefs.getBytesLength(NVS_KEY);
            if (blob_size > 0 && blob_size <= MAX_BUTTONS * sizeof(ButtonConfig)) {
                std::vector<ButtonConfig> tmp(blob_size / sizeof(ButtonConfig));
                prefs.getBytes(NVS_KEY, tmp.data(), blob_size);
                prefs.end();
                applyConfig(tmp);
                return;
            }
            prefs.end();
        }
        applyConfig(fallback);
    }

    // 構成を変更 (`hw.buttons.set` から呼ばれる)。NVS にも書込み。
    bool setConfig(const std::vector<ButtonConfig>& cfg) {
        if (cfg.size() > MAX_BUTTONS) return false;
        applyConfig(cfg);
        Preferences prefs;
        if (!prefs.begin(NVS_NS, false)) return false;
        prefs.putBytes(NVS_KEY, cfg.data(), cfg.size() * sizeof(ButtonConfig));
        prefs.end();
        return true;
    }

    // factory reset 用
    bool clearNvs() {
        Preferences prefs;
        if (!prefs.begin(NVS_NS, false)) return false;
        prefs.clear();
        prefs.end();
        return true;
    }

    const std::vector<ButtonConfig>& config() const { return cfg_; }

    // 50Hz 程度で呼ぶ。digitalRead → 簡易チャタリング除去 → bitmap 確定
    // bitmap bit 0 = button_idx 1 (cfg_[0]) の押下状態
    uint16_t update(uint32_t now_ms) {
        uint16_t raw = 0;
        for (size_t i = 0; i < cfg_.size() && i < 16; i++) {
            int v = digitalRead(cfg_[i].gpio);
            bool pressed = cfg_[i].active_low ? (v == LOW) : (v == HIGH);
            if (pressed) raw |= (1 << i);
        }
        if (raw != raw_prev_) {
            raw_prev_ = raw;
            stable_since_ms_ = now_ms;
            return bitmap_;
        }
        if ((now_ms - stable_since_ms_) >= 5) {
            bitmap_ = raw;
        }
        return bitmap_;
    }

    uint16_t bitmap() const { return bitmap_; }
    size_t buttonCount() const { return cfg_.size(); }

private:
    std::vector<ButtonConfig> cfg_;
    uint16_t bitmap_ = 0;
    uint16_t raw_prev_ = 0;
    uint32_t stable_since_ms_ = 0;

    void applyConfig(const std::vector<ButtonConfig>& cfg) {
        cfg_ = cfg;
        for (auto& b : cfg_) {
            uint8_t mode = INPUT;
            if (b.pull_mode == 1) mode = INPUT_PULLUP;
            else if (b.pull_mode == 2) mode = INPUT_PULLDOWN;
            // GPIO 36/39 等 RTC input only は内部 PU 不可、INPUT 強制
            if (b.gpio == 34 || b.gpio == 35 || b.gpio == 36 || b.gpio == 39) {
                mode = INPUT;
            }
            pinMode(b.gpio, mode);
        }
        bitmap_ = 0;
        raw_prev_ = 0;
    }
};

}  // namespace BurstMotion
