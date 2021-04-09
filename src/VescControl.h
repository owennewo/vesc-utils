#pragma once
#include <SimpleFOC.h>
#include <SimpleCAN.h>
#include <EEPROM.h>

enum CANIndex {
    CI_MOTOR = 0,
    CI_VELOCITY = 1,
    CI_ANGLE = 1,
    // CC_ENABLE = 0,
    // CC_TARGET = 1,
    // CC_VELOCITY_P = 2,
    // CC_VELOCITY_I = 3,
    // CC_MOTION_TYPE = 4,
    // CC_MOTION_TYPE = 5,
};

enum SubIndexMotor {
    SIM_ENABLE,
    SIM_TARGET,
};

enum SubIndexPID {
    SIP_P,
    SIP_I,
    SIP_D,
};

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
    void sendByte(CANIndex index, uint8_t subindex, byte data);
    void sendFloat(CANIndex index, uint8_t subindex, float data);

    uint16_t can_identifier = 0x01;

    BLDCMotor* motor;

    SimpleCAN _can;
    can_message_t txHeadingSpeed = {
      .dlc = 8,
      .id = 0x01,
      .isRTR = false,
      .isStandard = true,
      .data = {0,0,0,0}
    };
    can_message_t txAngle = {
      .dlc = 4,
      .id = 0x02,
      .isRTR = false,
      .isStandard = true,
      .data = {0,0,0,0,0,0,0,0}
    };

    can_message_t txMessage = {
      .dlc = 0,
      .id = 0x00,
      .isRTR = false,
      .isStandard = true,
      .data = {0,0,0,0,0,0,0,0}
    };  
};