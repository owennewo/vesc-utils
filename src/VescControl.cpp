#include <VescControl.h>

// #define SET_MASTER_ROLE
// #define SET_SLAVE_ROLE

void VescControl::onCanMessage(can_message_t* message)
{
    switch (message->id) {
        case 0x01:
            memcpy(&data.heading, &message->data, 4);
            memcpy(&data.speed, &message->data[4], 4);
            // Serial.print("speed:  "); Serial.println(data.speed);
            // Serial.print("heading:  "); Serial.println(data.heading);
            break;
        case 0x02:
            memcpy(&data.angle, &message->data, 4);
            // Serial.print("angle:  "); Serial.println(data.angle);
        break;   
    }
}

VescControl::VescControl() {

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
    // Serial.print("sending: ");
    // Serial.println(angle);
    _can.transmit(&txAngle);
}
void VescControl::print() {
    static long count = 0;
    count ++;
    if (count%1000 == 0) {
        Serial.print(data.heading); Serial.print(" "); Serial.print(data.speed); Serial.print(" "); Serial.println(data.angle); 
    }
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
