#include "IMU.h"
#include <math.h>
#include <Arduino.h>
#include "M5StickC.h"

IMU::IMU() {
}

int IMU::Init(float Fs) {
    sampleFreq = Fs;
    int imu_flag = M5.Sh200Q.Init();
    imuType = IMU_UNKNOWN;

    if (imu_flag != 0) {
        imu_flag = M5.Mpu6886.Init();
        if (imu_flag == 0) {
            imuType = IMU_MPU6886;
        }
    } else {
        imuType = IMU_SH200Q;
    }

    return (imuType == IMU_UNKNOWN) ? -1 : 0;
}

void IMU::getGres() {
    switch (imuType) {
        case IMU_SH200Q:
            gRes = M5.Sh200Q.gRes;
            break;
        case IMU_MPU6886:
            gRes = M5.Mpu6886.gRes;
            break;
        case IMU_MPU6050:
            gRes = M5.Mpu6050.gRes;
            break;
    }
}

void IMU::getAres() {
    switch (imuType) {
        case IMU_SH200Q:
            aRes = M5.Sh200Q.aRes;
            break;
        case IMU_MPU6886:
            aRes = M5.Mpu6886.aRes;
            break;
        case IMU_MPU6050:
            aRes = M5.Mpu6050.aRes;
            break;
    }
}

void IMU::getAccelAdc(int16_t *ax, int16_t *ay, int16_t *az) {
    switch (imuType) {
        case IMU_SH200Q:
            M5.Sh200Q.getAccelAdc(ax, ay, az);
            break;
        case IMU_MPU6886:
            M5.Mpu6886.getAccelAdc(ax, ay, az);
            break;
        case IMU_MPU6050:
            M5.Mpu6050.getAccelAdc(ax, ay, az);
            break;
    }
}

void IMU::getAccelData(float *ax, float *ay, float *az) {
    switch (imuType) {
        case IMU_SH200Q:
            M5.Sh200Q.getAccelData(ax, ay, az);
            break;
        case IMU_MPU6886:
            M5.Mpu6886.getAccelData(ax, ay, az);
            break;
        case IMU_MPU6050:
            M5.Mpu6050.getAccelData(ax, ay, az);
            break;
    }
}

void IMU::getGyroAdc(int16_t *gx, int16_t *gy, int16_t *gz) {
    switch (imuType) {
        case IMU_SH200Q:
            M5.Sh200Q.getGyroAdc(gx, gy, gz);
            break;
        case IMU_MPU6886:
            M5.Mpu6886.getGyroAdc(gx, gy, gz);
            break;
        case IMU_MPU6050:
            M5.Mpu6050.getGyroAdc(gx, gy, gz);
            break;
    }
}

void IMU::getGyroData(float *gx, float *gy, float *gz) {
    switch (imuType) {
        case IMU_SH200Q:
            M5.Sh200Q.getGyroData(gx, gy, gz);
            break;
        case IMU_MPU6886:
            M5.Mpu6886.getGyroData(gx, gy, gz);
            break;
        case IMU_MPU6050:
            M5.Mpu6050.getGyroData(gx, gy, gz);
            break;
    }
}

void IMU::getTempAdc(int16_t *t) {
    switch (imuType) {
        case IMU_SH200Q:
            M5.Sh200Q.getTempAdc(t);
            break;
        case IMU_MPU6886:
            M5.Mpu6886.getTempAdc(t);
            break;
        case IMU_MPU6050:
            M5.Mpu6050.getTempAdc(t);
            break;
    }
}

void IMU::getTempData(float *t) {
    switch (imuType) {
        case IMU_SH200Q:
            M5.Sh200Q.getTempData(t);
            break;
        case IMU_MPU6886:
            M5.Mpu6886.getTempData(t);
            break;
        case IMU_MPU6050:
            M5.Mpu6050.getTempData(t);
            break;
    }
}

float* IMU::getAhrsData(float *pitch,float *roll,float *yaw, float aOX,float aOY,float aOZ,float gOX,float gOY,float gOZ){

  float accX = 0; 
  float accY = 0;
  float accZ = 0;

  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;

  getGyroData(&gyroX,&gyroY,&gyroZ);
  getAccelData(&accX,&accY,&accZ);

  float* quatanion = MahonyAHRSupdateIMU((gyroX-gOX) * DEG_TO_RAD, (gyroY-gOY) * DEG_TO_RAD, (gyroZ-gOZ) * DEG_TO_RAD, (accX-aOX), (accY-aOY), (accZ-aOZ),pitch,roll,yaw,sampleFreq);
  return quatanion;
}

void IMU::setQuaternion(float* Q){
    //Serial.printf("%f,%f,%f,%f",Q[0],Q[1],Q[2],Q[3]);
    setQ(Q);
}