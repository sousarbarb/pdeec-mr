#include "driver_hardware_5dpo/SerialCommunicationChannelsConfig.h"

DataCommunicationChannels data_communication_channels_;

DataCommunicationChannels* InitCommunications() {
  // Arduino >>> PC
  // - encoders:
  AddValueToChannel_int32_t('g',&(data_communication_channels_.channel_g));
  AddValueToChannel_int32_t('h',&(data_communication_channels_.channel_h));
  AddValueToChannel_int32_t('i',&(data_communication_channels_.channel_i));
  // - sample time of the motors:
  AddValueToChannel_int32_t('j',&(data_communication_channels_.channel_j));
  // - emergency button:
  AddValueToChannel_int32_t('k',&(data_communication_channels_.channel_k));
  // - ball sensor:
  AddValueToChannel_int32_t('l',&(data_communication_channels_.channel_l));
  // - battery level:
  AddValueToChannel_int32_t('m',&(data_communication_channels_.channel_m));
  // - imu:
  AddValueToChannel_int32_t('n',&(data_communication_channels_.channel_n));
  AddValueToChannel_int32_t('o',&(data_communication_channels_.channel_o));
  AddValueToChannel_int32_t('p',&(data_communication_channels_.channel_p));
  // - capacitator level:
  AddValueToChannel_int32_t('q',&(data_communication_channels_.channel_q));
  // - reset:
  AddValueToChannel_int32_t('r',&(data_communication_channels_.channel_r));
  // - rollers:
  AddValueToChannel_int32_t('s',&(data_communication_channels_.channel_s));
  AddValueToChannel_int32_t('t',&(data_communication_channels_.channel_t));
  // - debug:
  AddValueToChannel_int32_t('x',&(data_communication_channels_.channel_x));
  AddValueToChannel_int32_t('y',&(data_communication_channels_.channel_y));
  AddValueToChannel_int32_t('z',&(data_communication_channels_.channel_z));

  // PC >>> Arduino
  // - motors ref:
  AddValueToChannel_float('G',&(data_communication_channels_.channel_G));
  AddValueToChannel_float('H',&(data_communication_channels_.channel_H));
  AddValueToChannel_float('I',&(data_communication_channels_.channel_I));
  // - kicker:
  AddValueToChannel_int32_t('K',&(data_communication_channels_.channel_K));
  AddValueToChannel_int32_t('L',&(data_communication_channels_.channel_L));
  // - solenoid:
  AddValueToChannel_int32_t('M',&(data_communication_channels_.channel_M));
  // - rollers:
  AddValueToChannel_int32_t('N',&(data_communication_channels_.channel_N));
  AddValueToChannel_int32_t('O',&(data_communication_channels_.channel_O));
  // - motors PWM:
  AddValueToChannel_int32_t('R',&(data_communication_channels_.channel_R));
  AddValueToChannel_int32_t('S',&(data_communication_channels_.channel_S));
  AddValueToChannel_int32_t('T',&(data_communication_channels_.channel_T));
  // - robot velocity;
  AddValueToChannel_float('U',&(data_communication_channels_.channel_U));
  AddValueToChannel_float('V',&(data_communication_channels_.channel_V));
  AddValueToChannel_float('W',&(data_communication_channels_.channel_W));

  return &data_communication_channels_;
}
