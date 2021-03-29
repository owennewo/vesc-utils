#include <Arduino.h>
#include <Adafruit_BNO08x.h>

struct euler_t {
  float yaw;
  float pitch;
  float roll;
  float status;
  float interval_us;
};

class VescIMU {

public:
    VescIMU();
    bool begin(sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV, long _report_interval_us = 5000, int sda_pin = I2C2_SDA, int scl_pin = I2C2_SCL);
    bool readEuler();
    int resetBus(int sda_pin, int scl_pin);
    void scan();

    euler_t euler;
    sh2_SensorValue_t sensorValue;

private:
    Adafruit_BNO08x*  bno08x;
    void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
    void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false);
    void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false);
    bool setReports();


    sh2_SensorId_t report_type;
    long report_interval_us;

};
