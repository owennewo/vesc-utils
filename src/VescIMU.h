#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
  float status;
  float interval_us;
};

class VescIMU {

public:
    VescIMU(Adafruit_BNO08x* bno08x);
    bool begin(sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV, long _report_interval_us = 10000, int sda_pin = I2C2_SDA, int scl_pin = I2C2_SCL, int interrupt_pin = PC0);
    bool readEuler(bool forceRead = false);
    int resetBus(int sda_pin = I2C2_SDA, int scl_pin = I2C2_SCL);
    void scan();
    void print(Stream& printer);

    euler_t euler;
    sh2_SensorValue_t sensorValue;
    int reset_count = 0;

private:
    Adafruit_BNO08x*  bno08x;
    void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
    void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false);
    void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false);
    bool setReports();
    void dataInterrupt();

    sh2_SensorId_t report_type;
    long report_interval_us;
    bool setup_complete = false;
    bool dataReady = false;
    long interruptCount = 0;
};
