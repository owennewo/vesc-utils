#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

#define VOLTAGE_REF 3.3
#define VOLTAGE_RESOLUTION 1024
#define SUPPLY_VOLTAGE_DIVIDER_1 2200
#define SUPPLY_VOLTAGE_DIVIDER_2 39000

inline float get_voltage() {
  float raw_voltage = VOLTAGE_REF * (float(analogRead(AN_IN)) / VOLTAGE_RESOLUTION);
  float voltage = raw_voltage * (SUPPLY_VOLTAGE_DIVIDER_1 + SUPPLY_VOLTAGE_DIVIDER_2) / SUPPLY_VOLTAGE_DIVIDER_1;
  return voltage;
}

inline void checkVoltage(BLDCMotor *motor = nullptr) {

  const float min_voltage_alarm = 6.0; // if the voltage is below 6, we can assume we are being powered bu usb/serial
  const float max_voltage_alarm = 20.0;  

  float voltage = get_voltage();

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
        voltage = get_voltage();
      } 
      

    }
    
    Serial.println("voltage recovered");
    
  }
}