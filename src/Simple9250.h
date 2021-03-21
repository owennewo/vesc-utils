#pragma once
#include <SparkFunMPU9250-DMP.h>


struct euler_angles_t {
    float roll; 
    float pitch;
    float yaw;
    long dt_micros; // time since last sample
};

class Simple9250 {
    public:
        Simple9250();
        int resetI2C(int sda_pin, int scl_pin);
        void begin(unsigned short sampleRate);
        bool readData(euler_angles_t* angles);
    private:
        MPU9250_DMP* imu;
};
