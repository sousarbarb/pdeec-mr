#ifndef ENCODERS_h
#define ENCODERS_h

#include <Arduino.h>

// states encoders 
#define WAIT_0D 0
#define WAIT_0A 1
#define WAIT_ENC1_B1 2
#define WAIT_ENC1_B2 3
#define WAIT_ENC2_B1 4
#define WAIT_ENC2_B2 5
#define WAIT_ENC3_B1 6
#define WAIT_ENC3_B2 7
#define WAIT 8

class Encoders 
{
public:
  uint16_t enc[3],enc_tmp[3],enc_last[3];
  int16_t imp[3];
  bool valid_read;
  bool reset_enc;
  
  void init(void (*serialWriteFunction)(char c, int32_t v));
  void updateStateMachine(byte b);
  void updateEncoders();
  void send();

private:
  void (*serialWrite)(char c, int32_t v);
  byte count_read;
  int state_enc;
};

#endif
