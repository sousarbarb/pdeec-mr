#include "encoders.h"

void Encoders::updateStateMachine(byte b)
{
  switch (state_enc) {
    case WAIT_0A: 
      if (b == 0x0A) {
        state_enc = WAIT_0D;
      }
      break;
    case WAIT_0D:
      if (b == 0x0D) {
        state_enc = WAIT_ENC1_B1;
      } else {
        state_enc = WAIT_0A;  
      }
      break;
    case WAIT_ENC1_B1:
      enc_tmp[0] = b;
      state_enc = WAIT_ENC1_B2;    
      break; 
    case WAIT_ENC1_B2:
      enc[0] = enc_tmp[0] + 256*b;
      state_enc = WAIT_ENC2_B1;    
      break; 
    case WAIT_ENC2_B1:
      enc_tmp[1] = b;
      state_enc = WAIT_ENC2_B2;    
      break; 
    case WAIT_ENC2_B2:
      enc[1] = enc_tmp[1] + 256*b;
      state_enc = WAIT_ENC3_B1;    
      break;    
    case WAIT_ENC3_B1:
      enc_tmp[2] = b;
      state_enc = WAIT_ENC3_B2;    
      break;
    case WAIT_ENC3_B2:
      enc[2] = enc_tmp[2] + 256*b;
      count_read++;
      if (count_read >= 2){
        valid_read = true;
        count_read = 0;
        if (reset_enc) {
          reset_enc   = false;
          enc_last[0] = enc[0];
          enc_last[1] = enc[1];
          enc_last[2] = enc[2];
        }
      }
      state_enc = WAIT_0A;    
      break;
  }
}

void Encoders::init(void (*serialWriteFunction)(char c, int32_t v))
{
  serialWrite = serialWriteFunction;
  count_read  = 0;
  state_enc   = WAIT_0A;
  reset_enc   = true;

  for(int i=0; i<3; i++)
  {
    enc[i] = 0;
    enc_tmp[i] = 0;
    enc_last[i] = 0;
    imp[i] = 0;
  }
}

void Encoders::updateEncoders()
{
  if (!reset_enc) {
    imp[0] = (int16_t)(enc[0] - enc_last[0]);  
    enc_last[0] = enc[0];
    imp[1] = (int16_t)(enc[1] - enc_last[1]);  
    enc_last[1] = enc[1];
    imp[2] = (int16_t)(enc[2] - enc_last[2]);
    enc_last[2] = enc[2];
  }
}

void Encoders::send()
{
  (*serialWrite)('g', (int16_t)imp[0]);
  (*serialWrite)('h', (int16_t)imp[1]);
  (*serialWrite)('i', (int16_t)imp[2]);
  (*serialWrite)('j', (int32_t) micros()); // sample time
}