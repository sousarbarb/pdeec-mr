#ifndef SRC_ROBOT5DPO_H
#define SRC_ROBOT5DPO_H

#include <cstdint>
#include <cmath>

#define CTRL_FREQUENCY 25
#define MOTORS_CTRL_FREQUENCY 100
#define K_BATTERY_LEVEL 6.74468
#define ROLLER_LEFT 0
#define ROLLER_RIGHT 1

namespace driver_hardware_5dpo {

enum KickerMode { LOW_KICK=0 , HIGH_KICK };

struct Motor {
 public:
  float encoder_res=0,gear_reduction=0;
  int32_t enc_thicks=0,enc_thicks_prev=0;
  int32_t enc_thicks_delta=0;
  float w_r=0,w=0;
  float sample_time=0,sample_time_prev=0,sample_period=0;

 public:
  void UpdateEncThicks(int32_t deltaencthicks);
  void UpdateSampleTime(int32_t sampletime);
};

struct Velocity {
 public:
  float v_r=0,vn_r=0,w_r=0;
  float v=0,vn=0,w=0;
};

struct RollerMotor {
 public:
  float w_r=0,w=0;
  float current=0;

 public:
  void UpdateRollerMotorCurrent(int16_t current_);
  void UpdateRollerMotorW(int16_t w_);
};

struct Compass {
 public:
  float angle;
  int16_t x,y,z;
};

struct Accelerometer {
 public:
  int16_t x,y,z;
};

struct Gyroscope {
 public:
  int16_t x,y,z;
};

struct Kicker {
 public:
  int16_t pulse=0;
  float capacitator_level=0;
  enum KickerMode mode=LOW_KICK;

 public:
  void UpdateCapacitatorLevel(int32_t capacitatorlevel);
};

struct Robot5dpo {
 public:
  // Motors
  Motor motors[3];
  // Robot velocity
  Velocity velocity;
  // Rollers
  RollerMotor roller_motors[2];
  // Kicker
  Kicker kicker;
  // IMU
  Compass compass;
  Accelerometer accelerometer;
  Gyroscope gyroscope;
  // Sensors
  float battery_level;
  bool ball_sensor;
  bool emergency;
  bool reset;

 public:
  void UpdateBatteryLevel(int32_t batterylevel);
  void StopMotors();
};

}  // namespace driver_hardware_5dpo

#endif //SRC_ROBOT5DPO_H
