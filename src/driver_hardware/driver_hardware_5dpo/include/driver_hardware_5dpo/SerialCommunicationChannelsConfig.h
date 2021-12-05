#ifndef SRC_SERIALCOMMUNICATIONCHANNELSCONFIG_H
#define SRC_SERIALCOMMUNICATIONCHANNELSCONFIG_H

#include "serial_communication_channels/serial_communication_channels.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct structDataCommunicationChannels {
  // Arduino >>> PC
  // - encoders:
  int32_t channel_g;
  int32_t channel_h;
  int32_t channel_i;
  // - sample time of the motors:
  int32_t channel_j;
  // - emergency button:
  int32_t channel_k;
  // - ball sensor:
  int32_t channel_l;
  // - battery level:
  int32_t channel_m;
  // - imu:
  int32_t channel_n;
  int32_t channel_o;
  int32_t channel_p;
  // - capacitator level:
  int32_t channel_q;
  // - reset:
  int32_t channel_r;
  // - rollers:
  int32_t channel_s;
  int32_t channel_t;
  // - debug:
  int32_t channel_x;
  int32_t channel_y;
  int32_t channel_z;

  // PC >>> Arduino
  // - motors ref:
  float channel_G;
  float channel_H;
  float channel_I;
  // - kicker:
  int32_t channel_K;
  int32_t channel_L;
  // - solenoid:
  int32_t channel_M;
  // - rollers:
  int32_t channel_N;
  int32_t channel_O;
  // - motors PWM:
  int32_t channel_R;
  int32_t channel_S;
  int32_t channel_T;
  // - robot velocity;
  float channel_U;
  float channel_V;
  float channel_W;
} DataCommunicationChannels;

DataCommunicationChannels* InitCommunications();

#ifdef __cplusplus
}
#endif

#endif //SRC_SERIALCOMMUNICATIONCHANNELSCONFIG_H
