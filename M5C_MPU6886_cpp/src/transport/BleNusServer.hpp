// Burst Motion - transport/BleNusServer.hpp
// Nordic UART Service (NUS) を BLE HID と並列で同一 NimBLE サーバーに登録
// JSON Lines プロトコルを USB Serial と同じ形式で BLE 経由で受信/送信
//
// NUS UUIDs (Nordic 標準):
//   Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   RX     : 6E400002-... (Web → FW、WRITE)
//   TX     : 6E400003-... (FW → Web、NOTIFY)
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <functional>

namespace BurstMotion {

class BleNusServer : public NimBLECharacteristicCallbacks {
public:
    using CommandHandler = std::function<void(JsonDocument& in, JsonDocument& out)>;

    BleNusServer() : tx_(nullptr), rx_(nullptr), connected_(false), buf_idx_(0) {}

    // BleHidSink::begin() の後に呼ぶ。NimBLEDevice は既に init 済み前提。
    bool begin() {
        NimBLEServer* server = NimBLEDevice::getServer();
        if (!server) {
            // BleCombo がまだ begin していない場合
            return false;
        }

        NimBLEService* svc = server->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

        // TX: FW → Web (notify)
        tx_ = svc->createCharacteristic(
            "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
            NIMBLE_PROPERTY::NOTIFY);

        // RX: Web → FW (write)
        rx_ = svc->createCharacteristic(
            "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
        rx_->setCallbacks(this);

        svc->start();

        // advertising に NUS UUID を追加
        NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
        if (adv) {
            adv->addServiceUUID(svc->getUUID());
            // 既に advertising 中なら再起動
            adv->stop();
            adv->start();
        }
        return true;
    }

    void setHandler(CommandHandler h) { handler_ = h; }

    // FW → Web に JSON 1 行送信
    void sendJson(const JsonDocument& doc) {
        if (!tx_) return;
        // ATT MTU 制限を考慮 (default 23B、ESP32 拡張で 244B、安全に 200B 区切り)
        char out[512];
        size_t n = serializeJson(doc, out, sizeof(out) - 2);
        if (n + 1 >= sizeof(out)) n = sizeof(out) - 2;
        out[n++] = '\n';
        out[n] = '\0';

        const size_t MTU_CHUNK = 200;
        for (size_t off = 0; off < n; off += MTU_CHUNK) {
            size_t len = (n - off) > MTU_CHUNK ? MTU_CHUNK : (n - off);
            tx_->setValue((uint8_t*)(out + off), len);
            tx_->notify();
            // 過渡期に詰まらないよう小さく待機
            delay(5);
        }
    }

    void sendAck(const char* cmd, bool ok = true) {
        JsonDocument d;
        d["type"] = "ack";
        d["cmd"] = cmd;
        d["ok"] = ok;
        sendJson(d);
    }

    void sendError(const char* cmd, const char* err) {
        JsonDocument d;
        d["type"] = "err";
        d["cmd"] = cmd;
        d["err"] = err;
        sendJson(d);
    }

    bool isConnected() const {
        NimBLEServer* s = NimBLEDevice::getServer();
        return s && s->getConnectedCount() > 0;
    }

    // NimBLECharacteristicCallbacks 実装: RX に書込みがあった
    void onWrite(NimBLECharacteristic* characteristic) override {
        std::string val = characteristic->getValue();
        for (char c : val) {
            if (c == '\r') continue;
            if (c == '\n' || buf_idx_ >= BUF_SIZE - 1) {
                buf_[buf_idx_] = '\0';
                if (buf_idx_ > 0) handleLine(buf_);
                buf_idx_ = 0;
            } else {
                buf_[buf_idx_++] = c;
            }
        }
    }

private:
    static constexpr size_t BUF_SIZE = 1024;
    NimBLECharacteristic* tx_;
    NimBLECharacteristic* rx_;
    char buf_[BUF_SIZE];
    size_t buf_idx_;
    bool connected_;
    CommandHandler handler_;

    void handleLine(const char* line) {
        JsonDocument in;
        DeserializationError err = deserializeJson(in, line);
        if (err) {
            sendError("", "parse_error");
            return;
        }
        if (!handler_) return;

        JsonDocument out;
        handler_(in, out);
        if (!out.isNull()) sendJson(out);
    }
};

}  // namespace BurstMotion
