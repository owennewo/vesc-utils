#include <VescIMU.h>

VescIMU::VescIMU(Adafruit_BNO08x *_bno08x)
{
  bno08x = _bno08x; //new Adafruit_BNO08x(BNO08X_RESET);
}

bool VescIMU::setReports()
{
  if (!bno08x->enableReport(report_type, report_interval_us))
  {
      return false;
  }
  return true;
}

int VescIMU::resetBus(int sda_pin, int scl_pin)
{
  pinMode(scl_pin, INPUT_PULLUP);
  pinMode(sda_pin, INPUT_PULLUP);
  delay(2);

  if (digitalRead(scl_pin) == LOW)
  {
    // Someone else has claimed master!;
    return 1;
  }

  if (digitalRead(sda_pin) == LOW)
  {
    // slave is communicating and awaiting clocks, we are blocked
    pinMode(scl_pin, OUTPUT);
    for (byte i = 0; i < 16; i++)
    {
      // toggle clock for 2 bytes of data
      digitalWrite(scl_pin, LOW);
      delayMicroseconds(20);
      digitalWrite(scl_pin, HIGH);
      delayMicroseconds(20);
    }
    pinMode(sda_pin, INPUT);
    delayMicroseconds(20);
    if (digitalRead(sda_pin) == LOW)
    {
      // SDA still blocked
      return 2;
    }
    delay(10);
  }
  // SDA is clear (HIGH)
  pinMode(sda_pin, INPUT);
  pinMode(scl_pin, INPUT);

  return 0;
}

bool VescIMU::begin(sh2_SensorId_t _report_type, long _report_interval_us, int sda_pin, int scl_pin, int interrupt_pin)
{
  Wire.setSDA(sda_pin);
  Wire.setSCL(scl_pin);

  pinMode(LED_RED, OUTPUT);
  pinMode(PC0, INPUT_PULLUP);
  attachInterrupt(interrupt_pin, std::bind(&VescIMU::dataInterrupt, this), FALLING);

  // scan();

  resetBus(sda_pin, scl_pin);

  report_interval_us = _report_interval_us;
  report_type = _report_type;

  Wire.setClock(400000);

  if (!bno08x->begin_I2C())
  {
    Serial.println("fail begin i2c");
    return false;
  }
  Wire.setClock(1000000);

  if (!setReports())
  {
    Serial.println("failed reports");
    return false;
  }

  Serial.println("imu begin!!");

  setup_complete = true;
  return true;
}

void VescIMU::scan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(2000);
}

void VescIMU::quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees)
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

void VescIMU::quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void VescIMU::quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector, euler_t *ypr, bool degrees)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

bool VescIMU::readEuler(bool forceRead)
{
  if (bno08x->wasReset())
  {
    Serial.println("IMU RESET!");
    reset_count++;
    setReports();
    delay(500);
    if (reset_count > 1) {
      digitalWrite(LED_RED, HIGH);
    }
    
  }

  static long last_micros = 0;
  long now = micros();

  if (now - last_micros > 20000) {
    Serial.print("late: "); Serial.println((now - last_micros));
    interruptCount++;
    dataReady = true;
  }

  if (interruptCount==0 && !forceRead) {
    // no interrupt saying data is ready
    return false;
  }
  dataReady = false;
  interruptCount--;
  last_micros = now;

  if (!setup_complete)
  {
    return false;
  }
  

  
  if (bno08x->getSensorEvent(&sensorValue))
  {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId)
    {
    case SH2_ARVR_STABILIZED_RV:
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &euler, true);
      break;
    case SH2_GYRO_INTEGRATED_RV:
      // faster (more noise?)
      return false;
      // Serial.println("read5");
      // quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &euler, true);
      break;
    }
    static long last = 0;
    long now = micros();
    euler.status = sensorValue.status;
    euler.interval_us = now - last;
    last = now;
    return true;
  }
  return false;
}

void VescIMU::print(Stream &printer)
{
  Serial.print(euler.interval_us);
  Serial.print("\t");
  Serial.print(euler.status);
  Serial.print("\t"); // This is accuracy in the range of 0 to 3
  Serial.print(euler.yaw);
  Serial.print("\t");
  Serial.print(euler.pitch);
  Serial.print("\t");
  Serial.println(euler.roll);
}

void VescIMU::dataInterrupt() {
  dataReady = true;
  interruptCount++;
}
