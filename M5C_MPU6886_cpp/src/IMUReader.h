#ifndef imu_H
#define imu_H

class IMUReader{
    public:
        IMUReader();
        ~IMUReader();
        void readIMU();
        float unwrappingYaw(float yaw);
        float unwrappingRoll(float roll,float pitch);
};

#endif