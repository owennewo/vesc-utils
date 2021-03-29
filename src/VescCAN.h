#pragma once
#include <Arduino.h>
#include <SimpleCAN.h>

#include "stm32f4xx_hal_can.h"



class VescCAN {

public:
    VescCAN();
    void begin();
    void sendAngle(float angle);
    void sendControl(float heading, float speed);
    void print(Stream& printer);

private:

    SimpleCAN _can;
    can_message_t txMessage;
};
