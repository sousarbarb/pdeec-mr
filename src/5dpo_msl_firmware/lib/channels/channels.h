#ifndef CHANNELS_H
#define CHANNELS_H

#include "Arduino.h"

class channels_t
{
    int8_t frameState;
    char curChannel;
    char frameHexData[8];
  public:
    uint8_t binary_writes;

    void (*process_frame)(char channel, uint32_t value, channels_t& obj);
    void (*serial_write)(uint8_t b);

    channels_t();

    void init(void (*process_frame_function)(char channel, uint32_t value, channels_t& obj),
              void (*serial_write_function)(uint8_t b)
              );
    void StateMachine(byte b);
    void sendFloat(char c, float v);
    void send(char c, int32_t v);
    void sendHexNibble(byte b);
    void sendHexByte(byte b);
    void sendByte(byte b);
};

#endif // CHANNELS_H
