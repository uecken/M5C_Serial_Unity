// Burst Motion - hal/esp32/ImuReader.hpp
// IMU 抽象インタフェース
#pragma once
#include <stdint.h>

namespace BurstMotion {

class IImuReader {
public:
    virtual ~IImuReader() {}
    virtual bool begin() = 0;

    // 1 サンプル読取、body frame に変換済み。
    // accel [m/s^2], gyro [deg/s] or [rad/s] (実装依存、Mahony へ渡す際に rad/s 必須)
    virtual bool read(float accel[3], float gyro[3]) = 0;

    virtual const char* typeName() const = 0;
};

}  // namespace BurstMotion
