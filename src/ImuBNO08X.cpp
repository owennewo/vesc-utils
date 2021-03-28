#include <ImuBNO08X.h>

ImuBNO08X::ImuBNO08X()
{
    bno08x = new Adafruit_BNO08x();
}

bool ImuBNO08X::begin(sh2_SensorId_t reportType, long _report_interval_us)
{
    report_interval_us = _report_interval_us;

    if (!bno08x->begin_I2C())
    {
        return false;
    }

    if (!bno08x->enableReport(reportType, report_interval_us))
    {
        return false;
    }

    return true;
}

void ImuBNO08X::quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees)
{

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees)
    {
        ypr->yaw *= RAD_TO_DEG;
        ypr->pitch *= RAD_TO_DEG;
        ypr->roll *= RAD_TO_DEG;
    }
}

void ImuBNO08X::quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void ImuBNO08X::quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

bool ImuBNO08X::readEuler()
{

    // if (bno08x->wasReset()) {
    //     setReports(reportType, reportIntervalUs);
    // }
  
  if (bno08x->getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &euler, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &euler, true);
        break;
    }

    static long last = 0;
    long now = micros();
    euler.status = sensorValue.status;
    euler.interval_us = now - last;
    last = now;
    
  }
}