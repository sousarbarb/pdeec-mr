#include "driver_hardware_5dpo/Robot5dpo.h"

namespace driver_hardware_5dpo {

void Motor::UpdateEncThicks(int32_t deltaencthicks) {
  // Encoders data
  enc_thicks_delta = deltaencthicks;
  enc_thicks_prev  = enc_thicks;
  enc_thicks       = enc_thicks_prev + enc_thicks_delta;
  // Angular speed
  w = 2 * M_PI * enc_thicks_delta * MOTORS_CTRL_FREQUENCY /
      (gear_reduction * encoder_res);
}

void Motor::UpdateSampleTime(int32_t sampletime) {
  sample_time_prev = sample_time;
  sample_time      = (float)(sampletime) / 1000000;   // us >>> s
  sample_period    = sample_time - sample_time_prev;
}

void Robot5dpo::UpdateBatteryLevel(int32_t batterylevel) {
  battery_level = batterylevel * 1.0 * K_BATTERY_LEVEL * 5 / 1023;
}

void Robot5dpo::StopMotors() {
  for (auto & motor : motors) {
    motor.w_r = 0;
  }
}

void Kicker::UpdateCapacitatorLevel(
        int32_t capacitatorlevel) {
  capacitator_level = (float)(capacitatorlevel)/100;
}

void RollerMotor::UpdateRollerMotorCurrent(
        int16_t current_) {
  current = (float)(current_)/100;
}

void RollerMotor::UpdateRollerMotorW(int16_t w_) {
  w = (float)(w_)*5/1023;
}

}  // namespace driver_hardware_5dpo
