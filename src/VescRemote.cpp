#include "VescRemote.h"

#define CONTROLLER_TIMEOUT 1000

VescRemote::VescRemote() {

}

void VescRemote::begin(HardwareSerial& serial)
{
  serial.begin(115200);
  begin((Stream&)serial);
}

void VescRemote::begin(Stream& stream)
{
  this->stream = &stream;
  this->state = DISCARD;
  this->last = millis();
  this->ptr = 0;
  this->len = 0;
  this->chksum = 0;
  this->lchksum = 0;
}

void VescRemote::prepareData() {
    data.x1 = normalize_analog_channel(data.raw[0]);
    data.y1 = normalize_analog_channel(data.raw[1]);
    data.y2 = normalize_analog_channel(data.raw[2]);
    data.x2 = normalize_analog_channel(data.raw[3]);
    data.pan1 = normalize_analog_channel(data.raw[4]);
    data.pan2 = normalize_analog_channel(data.raw[5]);
    data.switch1 = normalize_digital_channel(data.raw[6]);
    data.switch2 = normalize_digital_channel(data.raw[7]);
    data.switch3 = normalize_digital_channel(data.raw[8]);
    data.switch4 = normalize_digital_channel(data.raw[9]);
}

bool VescRemote::readData()
{
  while (stream->available() > 0)
  {
    uint32_t now = millis();
    if (now - last >= PROTOCOL_TIMEGAP)
    {
      state = GET_LENGTH;
    }
    last = now;
    
    uint8_t v = stream->read();
    switch (state)
    {
      case GET_LENGTH:
        if (v <= PROTOCOL_LENGTH)
        {
          ptr = 0;
          len = v - PROTOCOL_OVERHEAD;
          chksum = 0xFFFF - v;
          state = GET_DATA;
        }
        else
        {
          state = DISCARD;
        }
        break;

      case GET_DATA:
        buffer[ptr++] = v;
        chksum -= v;
        if (ptr == len)
        {
          state = GET_CHKSUML;
        }
        break;
        
      case GET_CHKSUML:
        lchksum = v;
        state = GET_CHKSUMH;
        break;

      case GET_CHKSUMH:
        // Validate checksum
        if (chksum == (v << 8) + lchksum)
        {
          // Execute command - we only know command 0x40
          switch (buffer[0])
          {
            case PROTOCOL_COMMAND40:
              // Valid - extract channel data
              for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2)
              {
                uint16_t value = buffer[i] | (buffer[i + 1] << 8);
                data.raw[i / 2] = value;
              }
              prepareData();
              last_read_millis = millis();
              status = RECEIVING;
              state = DISCARD;
              return true;

            default:
              break;
          }
        }
        state = DISCARD;
        break;

      case DISCARD:
      default:
        break;
    }
  }
  if (millis() - last_read_millis > CONTROLLER_TIMEOUT && status == RECEIVING) {
      status = NO_DATA;
      for (int index = 0; index < PROTOCOL_CHANNELS; index++) {
          data.raw[index] = 1500;
      }
      prepareData();
      return true;
  }
  return false;
}

void VescRemote::print(Stream& printer) {
    printer.print(data.x1);             printer.print(" ");
    printer.print(data.y1);             printer.print(" ");
    printer.print(data.x2);             printer.print(" ");
    printer.print(data.y2);             printer.print(" ");
    printer.print(data.pan1);           printer.print(" ");
    printer.print(data.pan2);           printer.print(" ");
    printer.print(data.switch1);        printer.print(" ");
    printer.print(data.switch2);        printer.print(" ");
    printer.print(data.switch3);        printer.print(" ");
    printer.println(data.switch4);
}
