#pragma once
#include <Arduino.h>

static const uint8_t PROTOCOL_LENGTH = 0x20;
  static const uint8_t PROTOCOL_OVERHEAD = 3; // <len><cmd><data....><chkl><chkh>
  static const uint8_t PROTOCOL_TIMEGAP = 3; // Packets are received very ~7ms so use ~half that for the gap
  static const uint8_t PROTOCOL_CHANNELS = 10;
  static const uint8_t PROTOCOL_COMMAND40 = 0x40; // Command is always 0x40


#define normalize_analog_channel(val) ((val - 1500.0) / 500.0)

// digital switches 0=down 1=up 2=halfway
#define normalize_digital_channel(val) ((val < 1250)? 1: (val > 1750)? 0: 2)

enum ControllerStatus {
    NO_DATA,
    RECEIVING,
};

struct controller_data_t {
    float x1;

    float y1;
    float x2;
    float y2;
    float pan1;
    float pan2;
    uint8_t switch1;
    uint8_t switch2;
    uint8_t switch3;
    uint8_t switch4;
    uint16_t raw[PROTOCOL_CHANNELS];
};

class VescController
{
public:
  VescController();
  void begin(HardwareSerial& serial = Serial6);
  void begin(Stream& stream);
  bool readData();
  void print(Stream& printer);
  long last_read_millis;
  
  controller_data_t data;

private:
  void prepareData();
  enum State
  {
    GET_LENGTH,
    GET_DATA,
    GET_CHKSUML,
    GET_CHKSUMH,
    DISCARD,
  };

  uint8_t state;
  Stream* stream;
  uint32_t last;
  uint8_t buffer[PROTOCOL_LENGTH];
  uint8_t ptr;
  uint8_t len;
  uint16_t chksum;
  uint8_t lchksum;
  ControllerStatus status  = NO_DATA;

};

