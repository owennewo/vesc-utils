#pragma once
#include <SimpleFOC.h>
#include <SimpleCAN.h>
#include <EEPROM.h>
#include "VescRemote.h"

enum Index {
    I_CONTROL = 0,
    I_MOTOR = 1,
    I_VELOCITY = 2,
    I_ANGLE = 3,
    // CC_ENABLE = 0,
    // CC_TARGET = 1,
    // CC_VELOCITY_P = 2,
    // CC_VELOCITY_I = 3,
    // CC_MOTION_TYPE = 4,
    // CC_MOTION_TYPE = 5,
};

enum SubIndex {
    SI_UNKNOWN = 0,
    // I_CONTROL
    SI_STEERING = 1,
    SI_THROTTLE = 2,
    SI_ANGLE = 3,
    SI_ANGLE_TRIM = 4,
    
    // I_MOTOR
    SI_ENABLE = 1,
    SI_TARGET = 2,
    
};

enum SubIndexPID {
    SIP_P,
    SIP_I,
    SIP_D,
};

struct ControlData {
    float steering;
    float throttle;
    float angle;
    float angle_trim;
    bool kill_switch;
};

enum ControlRole {
    MASTER_ROLE = 0xAA,
    SLAVE_ROLE = 0xBB,
    UNKNOWN_ROLE = 0x00,
};


class VescControl {
public:
    ControlData data;
    VescControl(BLDCMotor* motor);
    void begin(CanMode canMode = CanMode::NormalCAN);
    void updateRemote(remote_data_t data);
    void updateIMU(float angle);
    void print();

    ControlRole role = UNKNOWN_ROLE;

private:

    void loadRole();
    void onCanMessage(can_message_t* message);
    void sendByte(Index index, uint8_t subindex, byte data);
    void sendFloat(Index index, uint8_t subindex, float data);

    uint16_t can_identifier = 0x01;

    BLDCMotor* motor;

    SimpleCAN _can;
    // can_message_t txHeadingSpeed = {
    //   .dlc = 8,
    //   .id = 0x01,
    //   .isRTR = false,
    //   .isStandard = true,
    //   .data = {0,0,0,0}
    // };
    // can_message_t txAngle = {
    //   .dlc = 4,
    //   .id = 0x02,
    //   .isRTR = false,
    //   .isStandard = true,
    //   .data = {0,0,0,0,0,0,0,0}
    // };

    can_message_t txMessage = {
      .dlc = 7,
      .id = 0x00,
      .isRTR = false,
      .isStandard = true,
      .data = {0,0,1,0,0,0,0,0}
    };  
};