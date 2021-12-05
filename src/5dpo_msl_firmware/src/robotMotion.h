#ifndef ROBOTMOTION_H
#define ROBOTMOTION_H

#include "robotMotionConfig.h"
#include "TimerThree.h"
#include "encoders.h"

class Motor {
  // Variables
  public:
    bool  pi_enabled = true;
  private:
    uint8_t  index;
    Encoders *enc;
    float kp,ki;
    float w, w_r;
    float e,e_sum;
    float v;
    int   pwm;

  // Methods
  public:
    void init(uint8_t new_index, Encoders *encoder);
    void stop();
    void updatePI();
    void resetPI();
    void set_motor_w_r(float new_w_r);
    void set_motor_pwm();
    void set_motor_pwm(int new_pwm);
};

class RobotMotion {
  // Variables
  public:
    void (*serialWrite)(char c, int32_t v);
    bool  robvel_ctrl_enabled = true;
    Motor motors[3];
  private:
    float v_r,vn_r,w_r;

  // Methods
  public:
    void init(void (*serialWriteFunction)(char c, int32_t v), Encoders *encoder);
    void inverseKinematics();
    void update();
    void reset();
    void set_robvel_r(uint8_t index,float new_vi_r);
    void set_robvel_r(float new_v_r, float new_vn_r, float new_w_r);
};

#endif