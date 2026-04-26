// Burst Motion - hal/esp32/ImuMpu6886.hpp
// MPU6886 の IMU 読取 (Wire.h 直接 I2C、M5 lib 非依存)
// M5StickC では SDA=21, SCL=22 が MPU6886 接続
#pragma once
#include "ImuReader.hpp"
#include <Wire.h>
#include <stdint.h>

namespace BurstMotion {

class ImuMpu6886 : public IImuReader {
public:
    // M5StickC internal MPU6886 default: 0x68
    static constexpr uint8_t I2C_ADDR = 0x68;
    static constexpr int SDA_PIN = 21;
    static constexpr int SCL_PIN = 22;

    // ACCEL FS = ±8g、GYRO FS = ±2000dps で初期化
    static constexpr float ACC_LSB_TO_G = 1.0f / 4096.0f;     // ±8g: 4096 LSB/g
    static constexpr float GYRO_LSB_TO_DPS = 1.0f / 16.4f;     // ±2000dps: 16.4 LSB/dps
    static constexpr float DEG2RAD = 0.01745329251f;
    static constexpr float G_TO_MS2 = 9.80665f;

    // MPU6886 レジスタ
    enum Reg : uint8_t {
        SMPLRT_DIV   = 0x19,
        CONFIG       = 0x1A,
        GYRO_CONFIG  = 0x1B,
        ACCEL_CONFIG = 0x1C,
        ACCEL_XOUT_H = 0x3B,
        PWR_MGMT_1   = 0x6B,
        WHO_AM_I     = 0x75   // 期待値 0x19
    };

    bool begin() override {
        Wire.begin(SDA_PIN, SCL_PIN);
        Wire.setClock(400000);
        delay(10);

        // WHO_AM_I 確認
        uint8_t who = readReg(WHO_AM_I);
        if (who != 0x19) {
            return false;
        }

        // Reset
        writeReg(PWR_MGMT_1, 0x80);
        delay(100);
        // Wake up + auto select clock
        writeReg(PWR_MGMT_1, 0x01);
        delay(10);
        // Accel FS = ±8g (0b00010000 = 0x10)
        writeReg(ACCEL_CONFIG, 0x10);
        // Gyro FS = ±2000dps (0b00011000 = 0x18)
        writeReg(GYRO_CONFIG, 0x18);
        // SMPLRT_DIV = 0 (1kHz / (1+0))
        writeReg(SMPLRT_DIV, 0x00);
        // DLPF = 41Hz
        writeReg(CONFIG, 0x03);
        delay(10);
        return true;
    }

    bool read(float accel[3], float gyro[3]) override {
        uint8_t raw[14];
        if (!readRegs(ACCEL_XOUT_H, raw, 14)) return false;

        int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);
        // raw[6..7] = TEMP_OUT
        int16_t gx_raw = (int16_t)((raw[8]  << 8) | raw[9]);
        int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

        // [g] → [m/s^2]
        float ax_g = ax_raw * ACC_LSB_TO_G;
        float ay_g = ay_raw * ACC_LSB_TO_G;
        float az_g = az_raw * ACC_LSB_TO_G;

        // 軸リマップ (MVP: chip ≡ body frame、Phase 2 で実測)
        // M5StickC で USB-C 左、LCD 上向きが標準姿勢
        accel[0] = ax_g * G_TO_MS2;
        accel[1] = ay_g * G_TO_MS2;
        accel[2] = az_g * G_TO_MS2;

        // [dps] → [rad/s]
        gyro[0] = gx_raw * GYRO_LSB_TO_DPS * DEG2RAD;
        gyro[1] = gy_raw * GYRO_LSB_TO_DPS * DEG2RAD;
        gyro[2] = gz_raw * GYRO_LSB_TO_DPS * DEG2RAD;
        return true;
    }

    const char* typeName() const override { return "MPU6886"; }

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

    bool readRegs(uint8_t reg, uint8_t* buf, size_t len) {
        Wire.beginTransmission(I2C_ADDR);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;
        size_t got = Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)len);
        if (got != len) return false;
        for (size_t i = 0; i < len; i++) {
            if (!Wire.available()) return false;
            buf[i] = Wire.read();
        }
        return true;
    }
};

}  // namespace BurstMotion
