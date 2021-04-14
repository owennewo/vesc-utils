#include <VescControl.h>

// #define SET_MASTER_ROLE
// #define SET_SLAVE_ROLE   
#define MAX_SPEED 30.0

void VescControl::onCanMessage(can_message_t* message)
{
    Index index = static_cast<Index>((message->data[0] <<8) + message->data[1]);
    uint8_t subindex = message->data[2];

    if (index == I_CONTROL && subindex == SI_STEERING) {
        memcpy(&data.steering, &message->data[3], 4);
    } else if (index == I_CONTROL && subindex == SI_THROTTLE) {
        memcpy(&data.throttle, &message->data[3], 4);
    } else if (index == I_CONTROL && subindex == SI_ANGLE) {
        memcpy(&data.angle, &message->data[3], 4);
    } else if (index == I_CONTROL && subindex == SI_ANGLE_TRIM) {
        memcpy(&data.angle_trim, &message->data[3], 4);
    } else if (index == I_MOTOR && subindex == SI_ENABLE) {
        uint8_t enabled;
        memcpy(&enabled, &message->data[3], 1);
        if (enabled) {
            motor->enable();
        } else {
            motor->disable();
        }
    } else {
        Serial.print("unknown: "); Serial.print(index); Serial.print(":"); Serial.println(subindex);
    }

}

VescControl::VescControl(BLDCMotor* _motor) {
    motor = _motor;
}

void VescControl::sendByte(Index index, uint8_t subindex, byte data) {
    txMessage.dlc = 4;
    txMessage.data[0] = index >> 8;
    txMessage.data[1] = index & 0xFF;
    txMessage.data[2] = subindex;
    txMessage.data[3] = data;
    txMessage.id = can_identifier;
    _can.transmit(&txMessage);
}

void VescControl::sendFloat(Index index, uint8_t subindex, float data) {
    txMessage.dlc = 7;  
    txMessage.data[0] = index >> 8;
    txMessage.data[1] = index & 0xFF;
    txMessage.data[2] = subindex;
    memcpy(&txMessage.data[3], &data, 4);
    txMessage.id = can_identifier;
    _can.transmit(&txMessage);
}

void VescControl::begin(CanMode canMode)
{

    loadRole();

    _can.init(CAN_RX, CAN_TX, CanSpeed::BAUD_500K, canMode);
    _can.filterAcceptAll();
    _can.begin();
    can_callback_function_t function = std::bind(&VescControl::onCanMessage, this, std::placeholders::_1); 
    _can.subscribe(function);
    
}


void VescControl::updateRemote(remote_data_t remote_data) {

    // update steering
    if (remote_data.x1 != data.steering) {
        data.steering = remote_data.x1;
        sendFloat(I_CONTROL, SI_STEERING, data.steering);
        // Serial.print("steer: "); Serial.println(data.steering);
    }

    // update throttle
    if (remote_data.y1 != data.throttle) {
        data.throttle = remote_data.y1;
        sendFloat(I_CONTROL, SI_THROTTLE, data.throttle);
        // Serial.print("throttle: "); Serial.println(data.throttle);
    }

    if (remote_data.pan1 != data.angle_trim) {
        data.angle_trim = remote_data.pan1;
        sendFloat(I_CONTROL, SI_ANGLE_TRIM, data.angle_trim);
        // Serial.print("trim: "); Serial.println(data.angle_trim);
    }
        // Serial.print("enabled::: "); Serial.print(remote_data.switch1); Serial.print(" : "); Serial.println(motor->enabled);
    
    if (remote_data.switch1 != uint8_t(motor->enabled)) {
        // data.angle_trim = remote_data.pan1;
        if (remote_data.switch1) {
            Serial.print("enabled ");
            delay(10);
            motor->enable();
            delay(10);
            
        } else { 
            Serial.print("disable ");
            delay(10);
            motor->disable();
        }
        // sendFloat(I_CONTROL, SI_ANGLE_TRIM, data.angle_trim);
        // Serial.print("Enabled: "); Serial.println(remote_data.switch1);
        delay(10);
        sendByte(I_MOTOR, SI_ENABLE, static_cast<byte>(remote_data.switch1));
        Serial.println("after: ");
        delay(1);

    }

}

void VescControl::updateIMU(float angle) {

    if (angle != data.angle) {
        data.angle = angle;
    }
    sendFloat(I_CONTROL, SI_ANGLE, angle);
    
}

void VescControl::print() {
    // static long count = 0;
    // count ++;
    // if (count%1000 == 0) {
        Serial.print(" "); Serial.print(data.steering); Serial.print(" "); Serial.print(data.throttle); Serial.print(" "); Serial.println(data.angle); 
    // }
}

void VescControl::loadRole() {

#ifdef SET_MASTER_ROLE
    EEPROM.write(0, MASTER_ROLE);
    role = MASTER_ROLE;
#elif defined(SET_SLAVE_ROLE)
    EEPROM.write(0, SLAVE_ROLE);
    role = SLAVE_ROLE;
#else
   uint8_t roleId = EEPROM.read(0);
   if (roleId == MASTER_ROLE) {
       Serial.println("I am MASTER!");
       role = MASTER_ROLE;
   } else if (roleId == SLAVE_ROLE) {
       Serial.println("I am SLAVE!");
       role = SLAVE_ROLE;
   } else {
       role = UNKNOWN_ROLE;
   }
#endif  
    
}
