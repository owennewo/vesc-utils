#pragma once
#include <SimpleFOC.h>
#include <SimpleCAN.h>
#include <EEPROM.h>
#include "VescRemote.h"
#include "VescFOC.h"

#define SEND_INTERVAL 5
#define MAX_SPEED 4.0
#define MAX_STEER 1.0

enum Index {
    I_CONTROL = 0,
    I_MOTOR = 1,
    I_VELOCITY = 2,
    I_ANGLE = 3,

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
    SI_VOLTAGE = 3,
    SI_VELOCITY = 4,
    
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
    float voltage_master;
    float voltage_slave;
    float voltage_balance;
    float voltage_steering;
    float velocity_slave;
    float velocity_master;
    float velocity_diff;
    
    float reverse;
    bool kill_switch;
};

enum ControlRole {
    MASTER_ROLE = 0xBB,
    SLAVE_ROLE = 0xAA,
    UNKNOWN_ROLE = 0x00,
};


class VescControl {
public:
    ControlData data;
    VescControl(VescFOC* foc);
    void begin(CanMode canMode = CanMode::NormalCAN);
    void updateSlave(remote_data_t data);
    void updateSlaveVelocity();
    void updateMaster(float angle);
    void print();
    void loop();
    bool isMaster();
    bool isSlave();

    ControlRole role = UNKNOWN_ROLE;

private:

    void loadRole();
    void onCanMessage(can_message_t* message);
    void sendByte(Index index, uint8_t subindex, byte data);
    void sendFloat(Index index, uint8_t subindex, float data);

    uint16_t can_identifier = 0x01;

    VescFOC* foc;

    SimpleCAN _can;

    PIDController pid_stb{.P = 1.5, .I = 0.1, .D = 0.08, .ramp = 100000, .limit = 10};
    PIDController pid_vel{.P = 2.0, .I = 0.0, .D = 0.0, .ramp = 10000, .limit = 15};
    LowPassFilter lpf_velocity{.Tf = 0.1};
    LowPassFilter lpf_pitch{.Tf = 0.1};
    LowPassFilter lpf_throttle{.Tf = 0.1};
    LowPassFilter lpf_steering{.Tf = 0.1};

    can_message_t txMessage = {
      .dlc = 7,
      .id = 0x00,
      .isRTR = false,
      .isStandard = true,
      .data = {0,0,1,0,0,0,0,0}
    };  
};
