// Burst Motion - hid/BleHidSink.hpp
// BLE HID 出力 (NimBLE + BleCombo)
#pragma once
#include "IHidSink.hpp"
#include <NimBLEDevice.h>
#include <BleCombo.h>

namespace BurstMotion {

class BleHidSink : public IHidSink {
public:
    BleHidSink(const char* name = "Burst Motion") : name_(name), enabled_(true), started_(false) {}

    bool begin() override {
        ble_combo_.setName(name_);
        ble_combo_.begin();
        // BleCombo は setScanResponse(false) で名前を広告しないので、
        // scan response にデバイス名を載せて Windows などから見えるようにする
        NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
        if (adv) {
            NimBLEAdvertisementData scanResp;
            scanResp.setName(name_);
            adv->setScanResponseData(scanResp);
            adv->setScanResponse(true);
            adv->stop();
            adv->start();
        }
        started_ = true;
        return true;
    }

    // BLE HID advertising 停止 + 接続切断 (再開は begin() を呼ぶ)
    bool stop() {
        if (!started_) return false;
        NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
        if (adv) adv->stop();
        // 接続中のクライアントを全切断
        NimBLEServer* server = NimBLEDevice::getServer();
        if (server) {
            // NimBLE 1.4: getConnectedCount + iterate
            size_t cnt = server->getConnectedCount();
            for (size_t i = 0; i < cnt; i++) {
                NimBLEConnInfo info = server->getPeerInfo(0);  // 切断するたび index ずれるので 0 固定
                server->disconnect(info.getConnHandle());
            }
        }
        started_ = false;
        return true;
    }

    bool isConnected() const override {
        return started_ && const_cast<BleCombo&>(ble_combo_).isConnected();
    }

    void pressKey(uint8_t key) override {
        if (!enabled_ || !isConnected()) return;
        ble_combo_.press(key);
    }

    void releaseKey(uint8_t key) override {
        if (!isConnected()) return;
        ble_combo_.release(key);
    }

    void releaseAll() override {
        if (!isConnected()) return;
        ble_combo_.releaseAll();
    }

    void moveMouse(int16_t dx, int16_t dy, int8_t wheel = 0) override {
        if (!enabled_ || !isConnected()) return;
        // BleCombo.move は signed char (-127..+127) 制限
        while (dx > 127 || dx < -127 || dy > 127 || dy < -127) {
            int8_t step_x = dx > 127 ? 127 : (dx < -127 ? -127 : dx);
            int8_t step_y = dy > 127 ? 127 : (dy < -127 ? -127 : dy);
            ble_combo_.move(step_x, step_y, 0);
            dx -= step_x;
            dy -= step_y;
        }
        ble_combo_.move((int8_t)dx, (int8_t)dy, wheel);
    }

    void clickMouse(uint8_t buttons) override {
        if (!enabled_ || !isConnected()) return;
        ble_combo_.press((MouseButton)buttons);
        delay(10);
        ble_combo_.release((MouseButton)buttons);
    }

    void setEnabled(bool enabled) override { enabled_ = enabled; }
    bool isEnabled() const override { return enabled_; }

private:
    BleCombo ble_combo_;
    const char* name_;
    bool enabled_;
    bool started_;
};

}  // namespace BurstMotion
