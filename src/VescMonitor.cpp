#include <VescMonitor.h>


VescMonitor::VescMonitor(BLDCMotor* _motor) {
  motor = _motor;
  pinMode(LED_GREEN, OUTPUT);
}

float VescMonitor::getVoltage() {
  float raw_voltage = VOLTAGE_REF * (float(analogRead(AN_IN)) / VOLTAGE_RESOLUTION);
  float voltage = raw_voltage * (SUPPLY_VOLTAGE_DIVIDER_1 + SUPPLY_VOLTAGE_DIVIDER_2) / SUPPLY_VOLTAGE_DIVIDER_1;
  return voltage;
}

void VescMonitor::checkVoltage() {

  
  float voltage = getVoltage();

  if (voltage < motor->driver->voltage_power_supply * higher_voltage_percentage) {
    if (!fault) {
      // never recover from undervoltage
      motor->disable();
      fault = true;
      digitalWrite(LED_GREEN, HIGH);
      Serial.println("VOLTAGE too low");
    }
  } else if (voltage >= motor->driver->voltage_power_supply * higher_voltage_percentage) {
    if (fault) {
      // never recover from undervoltage
      motor->enable();
      fault = false;
      digitalWrite(LED_GREEN, LOW);
      Serial.print("VOLTAGE recovered: "); Serial.println(fault);
    }
  }
    
    // while (voltage > min_voltage_alarm && voltage < max_voltage_alarm) {
    //   // motor->disable();

    //   Serial.print("volatage error: "); Serial.println(voltage);
    //   digitalWrite(TEMP_MOTOR, HIGH);
    //   digitalWrite(LED_GREEN, HIGH);
    //   delay(5);
    //   digitalWrite(TEMP_MOTOR, LOW);
    //   digitalWrite(LED_GREEN, LOW);
    //   delay(995);
    //   if (!motor) {
    //     // we can recover from undervoltage if there is no motor to check
    //     voltage = getVoltage();
    //   } 
    // }    
  // }
}

void VescMonitor::print(Stream& printer) {
  float voltage = getVoltage();
  printer.print("Voltage: "); printer.println(voltage);
}
