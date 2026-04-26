// Burst Motion - core/MahonyFilter.hpp
// Mahony 姿勢推定フィルタ (6軸、body frame)
// 参考: xioTechnologies/Fusion および古典 Mahony 実装
#pragma once
#include <stdint.h>
#include <math.h>

namespace BurstMotion {

class MahonyFilter {
public:
    MahonyFilter(float twoKp = 2.0f, float twoKi = 0.0f)
        : twoKp_(twoKp), twoKi_(twoKi),
          q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f),
          integralFBx_(0.0f), integralFBy_(0.0f), integralFBz_(0.0f) {}

    // gyro: [rad/s]、accel: [g] (正規化済み推奨)、dt: [s]
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt) {
        float recipNorm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // 加速度が有効な場合のみ重力補正
        if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
            // Normalize accelerometer
            recipNorm = invSqrt(ax*ax + ay*ay + az*az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // 重力方向 (body frame での推定)
            halfvx = q1_*q3_ - q0_*q2_;
            halfvy = q0_*q1_ + q2_*q3_;
            halfvz = q0_*q0_ - 0.5f + q3_*q3_;

            // 加速度測定値と推定重力の外積 → 誤差ベクトル
            halfex = (ay*halfvz - az*halfvy);
            halfey = (az*halfvx - ax*halfvz);
            halfez = (ax*halfvy - ay*halfvx);

            // 積分補正 (ドリフト対策)
            if (twoKi_ > 0.0f) {
                integralFBx_ += twoKi_ * halfex * dt;
                integralFBy_ += twoKi_ * halfey * dt;
                integralFBz_ += twoKi_ * halfez * dt;
                gx += integralFBx_;
                gy += integralFBy_;
                gz += integralFBz_;
            }

            // 比例補正
            gx += twoKp_ * halfex;
            gy += twoKp_ * halfey;
            gz += twoKp_ * halfez;
        }

        // 積分 (quaternion dot)
        gx *= 0.5f * dt;
        gy *= 0.5f * dt;
        gz *= 0.5f * dt;

        qa = q0_; qb = q1_; qc = q2_;
        q0_ += (-qb*gx - qc*gy - q3_*gz);
        q1_ += ( qa*gx + qc*gz - q3_*gy);
        q2_ += ( qa*gy - qb*gz + q3_*gx);
        q3_ += ( qa*gz + qb*gy - qc*gx);

        // 正規化
        recipNorm = invSqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
        q0_ *= recipNorm;
        q1_ *= recipNorm;
        q2_ *= recipNorm;
        q3_ *= recipNorm;
    }

    // Quaternion 取得
    void getQuat(float q[4]) const {
        q[0] = q0_; q[1] = q1_; q[2] = q2_; q[3] = q3_;
    }

    // Euler (roll, pitch, yaw) in degrees
    void getEuler(float& roll, float& pitch, float& yaw) const {
        const float RAD2DEG = 57.29577951308232f;
        // ZYX intrinsic
        float sinr_cosp = 2.0f * (q0_*q1_ + q2_*q3_);
        float cosr_cosp = 1.0f - 2.0f * (q1_*q1_ + q2_*q2_);
        roll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;

        float sinp = 2.0f * (q0_*q2_ - q3_*q1_);
        if (fabsf(sinp) >= 1.0f)
            pitch = copysignf(90.0f, sinp);
        else
            pitch = asinf(sinp) * RAD2DEG;

        float siny_cosp = 2.0f * (q0_*q3_ + q1_*q2_);
        float cosy_cosp = 1.0f - 2.0f * (q2_*q2_ + q3_*q3_);
        yaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;
    }

    void setGain(float twoKp, float twoKi) {
        twoKp_ = twoKp;
        twoKi_ = twoKi;
    }

    void reset() {
        q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
        integralFBx_ = integralFBy_ = integralFBz_ = 0.0f;
    }

private:
    float twoKp_, twoKi_;
    float q0_, q1_, q2_, q3_;
    float integralFBx_, integralFBy_, integralFBz_;

    static float invSqrt(float x) {
        // 高速逆平方根 (Quake 由来、精度優先なら 1.0f/sqrtf(x) でも可)
        return 1.0f / sqrtf(x);
    }
};

}  // namespace BurstMotion
