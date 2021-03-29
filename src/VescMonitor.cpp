#include <VescMonitor.h>


VescMonitor::VescMonitor() {

}

float VescMonitor::getVoltage() {
  float raw_voltage = VOLTAGE_REF * (float(analogRead(AN_IN)) / VOLTAGE_RESOLUTION);
  float voltage = raw_voltage * (SUPPLY_VOLTAGE_DIVIDER_1 + SUPPLY_VOLTAGE_DIVIDER_2) / SUPPLY_VOLTAGE_DIVIDER_1;
  return voltage;
}

void VescMonitor::checkVoltage(BLDCMotor *motor) {

  const float min_voltage_alarm = 6.0; // if the voltage is below 6, we can assume we are being powered bu usb/serial
  const float max_voltage_alarm = 20.0;  

  float voltage = getVoltage();

  if (voltage > min_voltage_alarm && voltage < max_voltage_alarm) {
    if (motor) {
      // never recover from undervoltage
      motor->disable();
    }
    
    while (voltage > min_voltage_alarm && voltage < max_voltage_alarm) {
      // motor->disable();

      Serial.print("volatage error: "); Serial.println(voltage);
      digitalWrite(TEMP_MOTOR, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      delay(5);
      digitalWrite(TEMP_MOTOR, LOW);
      digitalWrite(LED_GREEN, LOW);
      delay(995);
      if (!motor) {
        // we can recover from undervoltage if there is no motor to check
        voltage = getVoltage();
      } 
    }    
    Serial.println("voltage recovered");
  }
}

void VescMonitor::print(Stream& printer) {
  float voltage = getVoltage();
  printer.print("Voltage: "); printer.println(voltage);
}
