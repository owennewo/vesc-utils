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
    uint8_t reverse; // 0 = no, 1 = reversed
private:
    int pole_pairs;
};
