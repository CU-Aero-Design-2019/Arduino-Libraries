#ifndef SPECIBUS_H
#define SPECIBUS_H

// This code is mostly taken from https://github.com/aanon4/FlySkyIBus
#include <inttypes.h>

class HardwareSerial;
class Stream;

class SpecIBUS
{
  public:
    void setup(HardwareSerial &serial);
    void setup(Stream &stream);
    void setup(void);
    void update(void);
    uint16_t readChannel(uint8_t channelNr);
    long receiverUpdateTimer = 0;
    const long receiverUpdatePeriod = 10;

  private:
    enum State
    {
        GET_LENGTH,
        GET_DATA,
        GET_CHKSUML,
        GET_CHKSUMH,
        DISCARD,
    };

    static const uint8_t PROTOCOL_LENGTH = 0x20;
    static const uint8_t PROTOCOL_OVERHEAD = 3; // <len><cmd><data....><chkl><chkh>
    static const uint8_t PROTOCOL_TIMEGAP = 3;  // Packets are received very ~7ms so use ~half that for the gap
    static const uint8_t PROTOCOL_CHANNELS = 14;
    static const uint8_t PROTOCOL_COMMAND40 = 0x40; // Command is always 0x40

    uint8_t state;
    Stream *stream;
    uint32_t last;
    uint8_t buffer[PROTOCOL_LENGTH];
    uint8_t ptr;
    uint8_t len;
    uint16_t channel[PROTOCOL_CHANNELS];
    uint16_t chksum;
    uint8_t lchksum;
};

extern SpecIBUS receiver;

#endif