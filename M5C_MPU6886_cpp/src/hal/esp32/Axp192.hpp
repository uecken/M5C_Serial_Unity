// Burst Motion - hal/esp32/Axp192.hpp
// M5StickC 内蔵 AXP192 PMIC ドライバ (M5 lib 非依存)
// I2C: SDA=21, SCL=22, addr=0x34
//
// 主要機能:
//  - LDO2 (LCD バックライト) ON/OFF
//  - LDO3 (LCD 駆動) ON/OFF
//  - バッテリ電圧 / 電流測定
//  - 電源ボタン状態
#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace BurstMotion {

class Axp192 {
public:
    static constexpr uint8_t I2C_ADDR = 0x34;

    bool begin() {
        // Wire.begin() は IMU 側で実施済み前提だが、idempotent なので呼んでも OK
        Wire.begin(21, 22);
        Wire.setClock(400000);

        // 存在確認: 0x03 (Power Status) を読んで 0xFF ではないこと
        uint8_t s = readReg(0x03);
        if (s == 0xFF) return false;

        // M5StickC の AXP192 標準シーケンス
        writeReg(0x12, 0x4D);   // DC-DC1=ON, EXTEN=ON, LDO2=ON, LDO3=ON
        writeReg(0x28, 0xCC);   // LDO2 = 3.0V, LDO3 = 3.0V (LCD)
        writeReg(0x82, 0xFF);   // ADC enable 1: battery V/I, USB V/I, AC V/I, APS V, TS
        writeReg(0x83, 0x80);   // ADC enable 2: temperature sensor
        writeReg(0x84, 0x32);   // ADC sample rate 25Hz, TS = battery temp
        writeReg(0x33, 0xC1);   // 充電 4.2V, 100mA
        writeReg(0x36, 0x4C);   // PEK: 4s power off, 128ms long press
        writeReg(0x32, 0x46);   // バッテリ過放電 protection 3.0V
        return true;
    }

    void setLcdBacklight(bool on) {
        // LDO2 BL ON/OFF (bit 2 of reg 0x12)
        uint8_t v = readReg(0x12);
        if (on)  v |= (1 << 2);
        else     v &= ~(1 << 2);
        writeReg(0x12, v);
    }

    // バッテリ電圧 [V] (1.1mV/LSB)
    float batteryVoltage() {
        uint16_t raw = readRegPair12(0x78);
        return raw * 0.0011f;
    }

    // バッテリ充電電流 [mA] (0.5mA/LSB)
    float batteryChargeCurrent() {
        uint16_t raw = readRegPair12(0x7A);
        return raw * 0.5f;
    }

    // バッテリ放電電流 [mA] (0.5mA/LSB)
    float batteryDischargeCurrent() {
        uint16_t raw = ((uint16_t)readReg(0x7C) << 5) | (readReg(0x7D) & 0x1F);
        return raw * 0.5f;
    }

    // USB 接続中？ (充電中)
    bool isUsbConnected() {
        return (readReg(0x00) & 0x20) != 0;
    }

    // 残量推定 [%] (Li-ion 1S 簡易計算: 4.2V=100%, 3.3V=0%)
    uint8_t batteryPercent() {
        float v = batteryVoltage();
        if (v >= 4.15f) return 100;
        if (v <= 3.3f) return 0;
        return (uint8_t)((v - 3.3f) / (4.15f - 3.3f) * 100);
    }

    // 電源ボタン状態 (1=長押し, 2=短押し, 0=なし) — M5 ライブラリ互換 read-and-clear
    uint8_t powerButtonState() {
        uint8_t v = readReg(0x46);
        if (v) writeReg(0x46, v);  // ack
        // bit0=short, bit1=long
        if (v & 0x02) return 1;
        if (v & 0x01) return 2;
        return 0;
    }

private:
    void writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(I2C_ADDR);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }
    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(I2C_ADDR);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)1);
        if (Wire.available()) return Wire.read();
        return 0;
    }
    // 12-bit ADC value: hi[7:0] | lo[3:0]
    uint16_t readRegPair12(uint8_t reg) {
        uint8_t hi = readReg(reg);
        uint8_t lo = readReg(reg + 1);
        return ((uint16_t)hi << 4) | (lo & 0x0F);
    }
};

}  // namespace BurstMotion
