#ifndef imu_H
#define imu_H

namespace imur{
    class IMUReader{
        public:
            IMUReader();
            ~IMUReader();
            void readIMU();
            float wrappingYaw(float yaw);
            float wrappingRoll(float roll,float pitch);
    };
}

#endif