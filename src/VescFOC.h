#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>

class VescFOC {
public:
    VescFOC(int pole_pairs = 15);
    void begin();
    void move(float target);
    void loop();
    void print(Stream& printer);
    HallSensor* sensor;
    BLDCDriver6PWM* driver;
    BLDCMotor* motor;

private:
    int pole_pairs;
};
