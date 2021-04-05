#pragma once
#include <SimpleCAN.h>
#include <EEPROM.h>

struct ControlData {
    float heading;
    float speed;
    float angle;
};

enum ControlRole {
    MASTER_ROLE = 0xAA,
    SLAVE_ROLE = 0xBB,
    UNKNOWN_ROLE = 0x00,
};


class VescControl {
public:
    ControlData data;
    VescControl();
    void begin(CanMode canMode = CanMode::NormalCAN);
    void setHeadingSpeed(float heading, float speed);
    void setAngle(float angle);
    void print();

    ControlRole role = UNKNOWN_ROLE;

private:

    void loadRole();
    void onCanMessage(can_message_t* message);

    SimpleCAN _can;
    can_message_t txHeadingSpeed = {
      .dlc = 8,
      .id = 0x01,
      .isRTR = false,
      .isStandard = false,
      .data = {0,0,0,0}
    };
    can_message_t txAngle = {
      .dlc = 4,
      .id = 0x02,
      .isRTR = false,
      .isStandard = false,
      .data = {0,0,0,0,0,0,0,0}
    };
};