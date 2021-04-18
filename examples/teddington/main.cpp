#include <Arduino.h>
#include <SimpleFOC.h>

#include "VescIMU.h"
#include "VescMonitor.h"
#include "VescRemote.h"
#include "VescFOC.h"
#include "VescControl.h"

VescRemote remote;
VescFOC foc(15);
VescMonitor monitor(foc.motor);
VescControl control(&foc);
Adafruit_BNO08x  bno08x(BNO08X_RESET);
VescIMU imu(&bno08x);

void setup() {

  Serial.begin(115200);
  delay(1000);

  foc.begin();

  Serial.println("setup");
  control.begin(NormalCAN);

  if (control.isMaster()) {
    if (!imu.begin()) Serial.println("IMU failure");
  } else if (control.isSlave()) {
    remote.begin();
  } else {
    Serial.println("Role not specified in EEPROM. Hard fault");
    while(true) ;
  }
  Serial.println("Control ready");

}

void loop() {
  
  if (control.isMaster()) {
     if (imu.readEuler()) {
        float angle = imu.euler.roll + 93 + (control.data.angle_trim * 5.0);
        if (foc.reverse) {
          angle *= -1.0;
        }
        control.updateMaster(angle);
      }
  } else if (control.isSlave()) {
    if (remote.readData()) {
        control.updateSlave(remote.data);
    }
    control.updateSlaveVelocity();
    
  }

  control.loop();
  monitor.checkVoltage();

}
