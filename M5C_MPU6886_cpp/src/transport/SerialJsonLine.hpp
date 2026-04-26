// Burst Motion - transport/SerialJsonLine.hpp
// JSON Lines over USB Serial プロトコル
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <functional>

namespace BurstMotion {

class SerialJsonLine {
public:
    // コマンドハンドラ: incoming JSON → outgoing JSON を書込む関数
    using CommandHandler = std::function<void(JsonDocument& in, JsonDocument& out)>;

    SerialJsonLine() : buf_idx_(0) {}

    void begin(uint32_t baud) {
        // RX buffer を 2048 byte に拡大 (default 256 だと連続 rule.add で overflow)
        // setRxBufferSize は Serial.begin の前に呼ぶ必要がある (Arduino-ESP32)
        Serial.setRxBufferSize(2048);
        Serial.begin(baud);
        delay(50);
    }

    void setHandler(CommandHandler h) { handler_ = h; }

    // 毎 loop 呼出、受信バッファから JSON 行を抽出
    void process() {
        while (Serial.available()) {
            char c = Serial.read();
            if (c == '\r') continue;
            if (c == '\n' || buf_idx_ >= BUF_SIZE - 1) {
                buf_[buf_idx_] = '\0';
                if (buf_idx_ > 0) {
                    handleLine(buf_);
                }
                buf_idx_ = 0;
            } else {
                buf_[buf_idx_++] = c;
            }
        }
    }

    // FW → Web にイベント送信
    void sendJson(const JsonDocument& doc) {
        serializeJson(doc, Serial);
        Serial.write('\n');
    }

    // 簡便: {type: ...} を即送信
    void sendEvent(const char* type) {
        JsonDocument doc;
        doc["type"] = type;
        sendJson(doc);
    }

    void sendError(const char* cmd, const char* err) {
        JsonDocument doc;
        doc["type"] = "err";
        doc["cmd"] = cmd;
        doc["err"] = err;
        sendJson(doc);
    }

    void sendAck(const char* cmd, bool ok = true) {
        JsonDocument doc;
        doc["type"] = "ack";
        doc["cmd"] = cmd;
        doc["ok"] = ok;
        sendJson(doc);
    }

private:
    static constexpr size_t BUF_SIZE = 2048;
    char buf_[BUF_SIZE];
    size_t buf_idx_;
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
        if (!out.isNull()) {
            sendJson(out);
        }
    }
};

}  // namespace BurstMotion
