#include <Arduino.h>
#include <Adafruit_BNO08x.h>

struct euler_t {
  float yaw;
  float pitch;
  float roll;
  float status;
  float interval_us;
};

class ImuBNO08X {

public:
    ImuBNO08X();
    bool begin(sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV, long _report_interval_us = 5000);
    bool readEuler();


    euler_t euler;
    sh2_SensorValue_t sensorValue;


private:
    Adafruit_BNO08x*  bno08x;
    void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
    void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false);
    void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false);


    
    long report_interval_us;

};
