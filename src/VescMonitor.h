#pragma once
#include <SimpleFOC.h>


#define VOLTAGE_REF 3.3
#define VOLTAGE_RESOLUTION 1024
#define SUPPLY_VOLTAGE_DIVIDER_1 2200
#define SUPPLY_VOLTAGE_DIVIDER_2 39000

class VescMonitor {

public:
    VescMonitor(BLDCMotor* motor);
    void checkVoltage();
    float getVoltage();
    void print(Stream& printer);
    bool fault = false;
private:

    BLDCMotor* motor;
    float higher_voltage_percentage = 0.85;  // if voltage < psu * higher_voltage_percentage then fault
    
};