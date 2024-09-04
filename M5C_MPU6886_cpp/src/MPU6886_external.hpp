#ifndef MPU6886_EXTERNAL_HPP
#define MPU6886_EXTERNAL_HPP

#include <Wire.h>
#include <M5StickC.h>
#include <math.h>

class MPU6886_External {
public:
    MPU6886_External(uint8_t sda_pin, uint8_t scl_pin, uint8_t addr = MPU6886_ADDRESS);
    bool begin();
    void init();
    void getAccelData();
    void calculateaccABS();

    float accX, accY, accZ;
    float accABS;
    bool isInitialized;
    bool accEvent;
    float accEventTh;
private:
    TwoWire I2C_External;
    uint8_t _addr;
    uint8_t _sda_pin;
    uint8_t _scl_pin;

    void I2C_Write_NBytes(uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer);
    void I2C_Read_NBytes(uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer);
};

MPU6886_External::MPU6886_External(uint8_t sda_pin, uint8_t scl_pin, uint8_t addr) 
    : I2C_External(0), _addr(addr), _sda_pin(sda_pin), _scl_pin(scl_pin),
      accX(0), accY(0), accZ(0), accABS(0), isInitialized(false) {
}

bool MPU6886_External::begin() {
    I2C_External.begin(_sda_pin, _scl_pin);
    I2C_External.beginTransmission(_addr);
    if (I2C_External.endTransmission() == 0) {
        init();
        isInitialized = true;
        return true;
    }
    return false;
}

void MPU6886_External::init() {
    uint8_t regdata;
    
    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = (0x01<<7);
    I2C_Write_NBytes(MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = (0x01<<0);
    I2C_Write_NBytes(MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

    regdata = 0x10;
    I2C_Write_NBytes(MPU6886_ACCEL_CONFIG, 1, &regdata);
    delay(1);

    regdata = 0x18;
    I2C_Write_NBytes(MPU6886_GYRO_CONFIG, 1, &regdata);
    delay(1);

    regdata = 0x01;
    I2C_Write_NBytes(MPU6886_CONFIG, 1, &regdata);
    delay(1);

    regdata = 0x05;
    I2C_Write_NBytes(MPU6886_SMPLRT_DIV, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_INT_ENABLE, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_ACCEL_CONFIG2, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_USER_CTRL, 1, &regdata);
    delay(1);

    regdata = 0x00;
    I2C_Write_NBytes(MPU6886_FIFO_EN, 1, &regdata);
    delay(1);

    regdata = 0x22;
    I2C_Write_NBytes(MPU6886_INT_PIN_CFG, 1, &regdata);
    delay(1);

    regdata = 0x01;
    I2C_Write_NBytes(MPU6886_INT_ENABLE, 1, &regdata);

    delay(100);
}

void MPU6886_External::getAccelData() {
    uint8_t buf[6];
    I2C_Read_NBytes(MPU6886_ACCEL_XOUT_H, 6, buf);
    
    int16_t accX_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t accY_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t accZ_raw = (int16_t)((buf[4] << 8) | buf[5]);

    accX = (float)accX_raw / 4096.0;
    accY = (float)accY_raw / 4096.0;
    accZ = (float)accZ_raw / 4096.0;

    calculateaccABS();
}

void MPU6886_External::calculateaccABS() {
    accABS = sqrt(accX*accX + accY*accY + accZ*accZ);
}

void MPU6886_External::I2C_Write_NBytes(uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer) {
    I2C_External.beginTransmission(_addr);
    I2C_External.write(start_Addr);
    I2C_External.write(write_Buffer, number_Bytes);
    I2C_External.endTransmission();
}

void MPU6886_External::I2C_Read_NBytes(uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer) {
    I2C_External.beginTransmission(_addr);
    I2C_External.write(start_Addr);
    I2C_External.endTransmission(false);
    I2C_External.requestFrom(_addr, number_Bytes);
    
    uint8_t i = 0;
    while (I2C_External.available() && i < number_Bytes) {
        read_Buffer[i++] = I2C_External.read();
    }
}

#endif // MPU6886_EXTERNAL_HPP