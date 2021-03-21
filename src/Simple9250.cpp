#include "Simple9250.h"

Simple9250::Simple9250() {

}

int Simple9250::resetI2C(int sda_pin, int scl_pin) {

  pinMode(scl_pin, INPUT_PULLUP);
  pinMode(sda_pin, INPUT_PULLUP);
  delay(2);

  if (digitalRead(scl_pin) == LOW) {
    // Someone else has claimed master!");
    return 1;
  }

  if(digitalRead(sda_pin) == LOW) {
    // slave is communicating and awaiting clocks, we are blocked
    pinMode(scl_pin, OUTPUT);
    for (byte i = 0; i < 16; i++) {
      // toggle clock for 2 bytes of data
      digitalWrite(scl_pin, LOW);
      delayMicroseconds(20);
      digitalWrite(scl_pin, HIGH);
      delayMicroseconds(20);
    }
    pinMode(sda_pin, INPUT);
    delayMicroseconds(20);
    if (digitalRead(sda_pin) == LOW) { 
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

/**
 * sampleRate - max sample rate is 200Hz
 */
void Simple9250::begin(unsigned short sampleRate) {
    imu = new MPU9250_DMP();
    if (imu->begin() != INV_SUCCESS)
    {
        while (1)
        {
            Serial.println("Unable to communicate with MPU-9250");
            Serial.println("Check connections, and try again.");
            Serial.println();
            delay(5000);
        }
    }

    // interrupts don't work with dmp

    imu->enableInterrupt();
    imu->setIntLevel(INT_ACTIVE_LOW);
    imu->setIntLatched(INT_LATCHED);
    // imu->dmpSetInterruptMode(1);
    imu->dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                     DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                 sampleRate);              // Set DMP FIFO rate to sampleRate
}

/**
 * readData from imu
 * euler_angles_t - struct pointer in which to store pitch/roll/yaw data
 * returns true if new data is available
 */ 
bool Simple9250::readData(euler_angles_t* angles)
{
    // Check for new data in the FIFO
    if (imu->fifoAvailable())
    {
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if (imu->dmpUpdateFifo() == INV_SUCCESS)
        {
            static long last = micros();
            long now = micros();
    
            imu->computeEulerAngles();

            // float q0 = imu.calcQuat(imu.qw);
            // float q1 = imu.calcQuat(imu.qx);
            // float q2 = imu.calcQuat(imu.qy);
            // float q3 = imu.calcQuat(imu.qz);


            angles->roll = imu->roll;
            angles->pitch = imu->pitch;
            angles->yaw = imu->yaw;
            if (angles->roll > 180.0) angles->roll -= 360.0;
            if (angles->pitch > 180.0) angles->pitch -= 360.0;
            angles->dt_micros = (now - last);
            last = now;
            return true;
        }
    }
    return false;
}