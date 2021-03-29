#pragma once
#include <SimpleFOC.h>


#define VOLTAGE_REF 3.3
#define VOLTAGE_RESOLUTION 1024
#define SUPPLY_VOLTAGE_DIVIDER_1 2200
#define SUPPLY_VOLTAGE_DIVIDER_2 39000

class VescMonitor {

public:
    VescMonitor();
    void checkVoltage(BLDCMotor *motor = nullptr);
    float getVoltage();
    void print(Stream& printer);

private:
    
};