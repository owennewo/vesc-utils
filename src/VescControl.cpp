#include <VescControl.h>

// #define SET_MASTER_ROLE
// #define SET_SLAVE_ROLE

void VescControl::onCanMessage(can_message_t* message)
{
    static CANIndex index = static_cast<CANIndex>(message->data[0] << 8 + message->data[1]);
    uint8_t subindex = message->data[2];

    if (message->id == 0x01) {
        memcpy(&data.heading, &message->data, 4);
        memcpy(&data.speed, &message->data[4], 4);
        // Serial.print("speed:  "); Serial.println(data.speed);
        // Serial.print("heading:  "); Serial.println(data.heading);
                
        return;
    } else if (message->id == 0x02) {
        memcpy(&data.angle, &message->data, 4);
        Serial.print("angle:  "); Serial.println(data.angle);
        
        return;
    }
    
    switch (index) {
        case CI_MOTOR:
            if (message->isRTR) sendByte(CI_MOTOR, SIM_ENABLE, motor->enabled);
            else (message->data[3]==0)? motor->enable(): motor->disable();
            break;
        default:
            Serial.print("unknown: "); Serial.print(index); Serial.print(":"); Serial.println(subindex);
        case 0x01:
            memcpy(&data.heading, &message->data, 4);
            memcpy(&data.speed, &message->data[4], 4);
            // Serial.print("speed:  "); Serial.println(data.speed);
            // Serial.print("heading:  "); Serial.println(data.heading);
            break;
        case 0x02:
            memcpy(&data.angle, &message->data, 4);
            Serial.print("angle:  "); Serial.println(data.angle);
        break;   
    }
}

VescControl::VescControl() {

}

void VescControl::sendByte(CANIndex index, uint8_t subindex, byte data) {
    txMessage.dlc = 1;
    txMessage.data[0] = index >> 8;
    txMessage.data[1] = index & 0xFF;
    txMessage.data[2] = subindex;
    txMessage.data[3] = data;
    txMessage.id = can_identifier;
}

void VescControl::sendFloat(CANIndex index, uint8_t subindex, float data) {
    txMessage.dlc = 1;
    txMessage.data[0] = index >> 8;
    txMessage.data[1] = index & 0xFF;
    txMessage.data[2] = subindex;
    memcpy(&txMessage.data[3], &data, 4);
    
    txMessage.id = can_identifier;
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

void VescControl::setHeadingSpeed(float heading, float speed) {
    data.heading = heading;
    data.speed = speed * 5;
    memcpy(&txHeadingSpeed.data, &data.heading, 4);
    memcpy(&txHeadingSpeed.data[4], &data.speed, 4);
    // Serial.print("sending: ");
    // Serial.println(heading);
    _can.transmit(&txHeadingSpeed);
}

void VescControl::setAngle(float angle) {
    data.angle = angle;
    memcpy(&txAngle.data, &angle, 4);
    Serial.print("sending: ");
    Serial.println(angle);
    _can.transmit(&txAngle);
}
void VescControl::print() {
    // static long count = 0;
    // count ++;
    // if (count%1000 == 0) {
        Serial.print(" "); Serial.print(data.heading); Serial.print(" "); Serial.print(data.speed); Serial.print(" "); Serial.println(data.angle); 
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
