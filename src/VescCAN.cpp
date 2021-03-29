#include <VescCAN.h>
#include "SimpleCAN_c.h"

void can_cmd_user_adc(can_message_t* txMessage, uint16_t nodeId ) {
      
//   txMessage->id = (nodeId << COMMAND_BITS) + CMD_USER_ADC;
//   txMessage->dlc = 2;
//   txMessage->isRTR = false;
//   txMessage->isStandard = true;

//   float voltage = monitor.getVoltage();
//   uint16_t voltage_data = pack_half_float(voltage);
//   memcpy(&txMessage->data[0], &voltage_data, 2);
  
}

void onMessage(can_message_t *message)
{
    Serial.println(message->id);
}

VescCAN::VescCAN() {

}

void VescCAN::begin()
{
    _can.init(CAN_RX, CAN_TX, CanSpeed::BAUD_500K, CanMode::LoopBackCan);
    _can.filterAcceptAll();
    _can.begin();
    _can.subscribe(onMessage);
}

void VescCAN::sendAngle(float angle)
{
    txMessage.dlc = 4;
    txMessage.id = 0x01;
    txMessage.isRTR = false;
    txMessage.isStandard = false;

    memcpy(&txMessage.data, &angle, 4);
    Serial.print("sending: ");
    Serial.println(angle);
    _can.transmit(&txMessage);
}

void VescCAN::sendControl(float heading, float speed)
{
    txMessage.dlc = 8;
    txMessage.id = 0x02;
    txMessage.isRTR = false;
    txMessage.isStandard = false;

    memcpy(&txMessage.data, &heading, 4);
    memcpy(&txMessage.data[4], &speed, 4);
    Serial.print("sending: ");
    Serial.println(heading);
    _can.transmit(&txMessage);
}

void VescCAN::print(Stream& printer) {
  CAN_InitTypeDef initType = SimpleCAN::_hcan->Init;
  uint32_t ts1 = (initType.TimeSeg1 >> CAN_BTR_TS1_Pos) +1;
  uint32_t ts2 = (initType.TimeSeg2 >> CAN_BTR_TS2_Pos) +1;
  
  uint32_t bitrate = (HAL_RCC_GetPCLK1Freq() / initType.Prescaler) / (1 + ts1 + ts2);
  
  Serial.print("SysClock: "); Serial.println(HAL_RCC_GetSysClockFreq());
  Serial.print("PCKL1: "); Serial.println(HAL_RCC_GetPCLK1Freq());
  Serial.print("Prescaler: "); Serial.println(initType.Prescaler);
  Serial.print("TS1: "); Serial.println(ts1);
  Serial.print("TS2: "); Serial.println(ts2);
  Serial.print("CAN bitrate: "); Serial.println(bitrate);
}
