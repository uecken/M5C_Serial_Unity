// Burst Motion - hid/IHidSink.hpp
// HID 出力の抽象インタフェース。BLE / USB どちらでも同じ API。
#pragma once
#include <stdint.h>

namespace BurstMotion {

class IHidSink {
public:
    virtual ~IHidSink() {}

    virtual bool begin() = 0;
    virtual bool isConnected() const = 0;

    // Keyboard
    virtual void pressKey(uint8_t key) = 0;
    virtual void releaseKey(uint8_t key) = 0;
    virtual void releaseAll() = 0;

    // Mouse
    virtual void moveMouse(int16_t dx, int16_t dy, int8_t wheel = 0) = 0;
    virtual void clickMouse(uint8_t buttons) = 0;
    // Phase 5.32: 押しっぱなし対応 (built-in mouse mode で必要)
    virtual void pressMouseButton(uint8_t buttons) {}
    virtual void releaseMouseButton(uint8_t buttons) {}

    // Enable/disable (output.set で切替)
    virtual void setEnabled(bool enabled) = 0;
    virtual bool isEnabled() const = 0;
};

}  // namespace BurstMotion
