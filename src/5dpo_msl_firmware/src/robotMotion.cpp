#include "robotMotion.h"

void NonLinearHammersteinBlock(float &vmot){
  if (vmot > MOT_HAMM_VD){
    vmot = (vmot - MOT_HAMM_VD) + MOT_HAMM_V0;
  }
  else if (vmot > -MOT_HAMM_VD){
    if (MOT_HAMM_VD != 0) vmot = vmot * MOT_HAMM_V0 / MOT_HAMM_VD;
    else                  vmot = 0;
  }
  else {
    vmot = (vmot + MOT_HAMM_VD) - MOT_HAMM_V0;
  }
}


void Motor::init(uint8_t new_index, Encoders *encoder){
  // Index of the motor (0..N-1)
  index = new_index;
  enc   = encoder;
  // Outputs
  pinMode(MOT_DIR_PIN[index]  ,OUTPUT);
  pinMode(MOT_TIMER_PIN[index],OUTPUT);
  // Inputs
  pinMode(MOT_CURRA_PIN[index],INPUT);
  pinMode(MOT_CURRB_PIN[index],INPUT);
  // Reset variables
  // - encoders
  enc->imp[index]      = 0;
  enc->enc_last[index] = 0;
  // - pi controllers
  pi_enabled = true;
  kp = PI_KC;
  if (PI_TI == 0) ki = 0;
  else            ki = PI_KC / PI_TI;
  resetPI();
  w_r = 0;
  v   = 0;
  pwm = 0;
  // Timer3
  Timer3.pwm(MOT_TIMER_PIN[index],pwm); 
}

void Motor::stop(){
  v = 0;
  resetPI();
  set_motor_pwm();  // Convert v >>> pwm and setting the duty cycle
}

void Motor::updatePI(){
  if (pi_enabled){
    float temp_e_sum;

    // Current angular velocity
    w = enc->imp[index] * IMP2W;
    // Error
    e = w_r - w;
    temp_e_sum = e_sum + e * CTRL_TIME * (1e-6);
    // Remove integration for zero reference
    if (w_r == 0){
      temp_e_sum = 0;
    }
    // PI controller output
    v = kp * e + ki * temp_e_sum;

    // Dead-zone compensation
    NonLinearHammersteinBlock(v);
    // Anti-windup
    if (((v > MOT_VMAX) && (temp_e_sum > 0)) || ((v < -MOT_VMAX) && (temp_e_sum < 0))){
      v = v + ki * (e_sum - temp_e_sum);
    }
    else{
      e_sum = temp_e_sum;
    }
    // Saturate the output
    if (v > MOT_VMAX){
      v = MOT_VMAX;
    }
    else if (v < -MOT_VMAX){
      v = -MOT_VMAX;
    }

    // Set motors pwm
    set_motor_pwm();
  }
  else{
    resetPI();
  }
}

void Motor::resetPI(){
  w   = 0;
  w_r = 0;
  e = 0;
  e_sum = 0;
}

void Motor::set_motor_w_r(float new_w_r){
  // Closed-loop control >>> enable PI controller
  pi_enabled = true;
  w_r = new_w_r;
}

void Motor::set_motor_pwm(){
  // Converse voltage to pwm
  pwm = round(V2PWM * v);
  // Check the pwm saturation
  if      (pwm >  MOT_PWMMAX) pwm =  MOT_PWMMAX;
  else if (pwm < -MOT_PWMMAX) pwm = -MOT_PWMMAX;
  // Set voltage
  if (pwm >= 0){
    digitalWrite(MOT_DIR_PIN[index],0);
    Timer3.setPwmDuty(MOT_TIMER_PIN[index],pwm);
  }
  else{
    digitalWrite(MOT_DIR_PIN[index],1);
    Timer3.setPwmDuty(MOT_TIMER_PIN[index],-pwm);
  }
}

void Motor::set_motor_pwm(int new_pwm){
  // Open-loop control >>> disable PI controller
  pi_enabled = false;
  pwm = new_pwm;
  resetPI();            // To prevent unexpected behaviours when switching to pi_enabled=TRUE
  // Check the pwm saturation
  if      (pwm >  MOT_PWMMAX) pwm =  MOT_PWMMAX;
  else if (pwm < -MOT_PWMMAX) pwm = -MOT_PWMMAX;
  // Set voltage
  if (pwm >= 0){
    digitalWrite(MOT_DIR_PIN[index],0);
    Timer3.setPwmDuty(MOT_TIMER_PIN[index],pwm);
  }
  else{
    digitalWrite(MOT_DIR_PIN[index],1);
    Timer3.setPwmDuty(MOT_TIMER_PIN[index],-pwm);
  }
}

void RobotMotion::init(void (*serialWriteFunction)(char c, int32_t v), Encoders *encoder){
  uint8_t i;

  // Debug purposes (with serial port)
  serialWrite = serialWriteFunction;
  // Robot
  robvel_ctrl_enabled = true;
  v_r  = 0;
  vn_r = 0;
  w_r  = 0;
  Timer3.initialize(50);
  // Motors
  for(i=0;i<3;i++){
    motors[i].init(i,encoder);
  }
}

void RobotMotion::inverseKinematics(){
  uint8_t i;
  float w_tmp[3];

  // Robot velocity >>> wheels linear velocities
  w_tmp[0] =  0.866025403784*v_r + 0.5*vn_r + ROBOT_L*w_r;
  w_tmp[1] = -0.866025403784*v_r + 0.5*vn_r + ROBOT_L*w_r;
  w_tmp[2] =                          -vn_r + ROBOT_L*w_r;
  // Wheels linear velocities >>> angular velocities
  for(i=0;i<3;i++){
    w_tmp[i] = w_tmp[i] * 2 / ROBOT_WH_D[i];
    motors[i].set_motor_w_r(w_tmp[i]);
  }
}

void RobotMotion::update(){
  uint8_t i;

  // Inverse kinematics
  if (robvel_ctrl_enabled){
    inverseKinematics();
  }
  else {
    v_r  = 0;
    vn_r = 0;
    w_r  = 0;
  }
  // Motor
  for(i=0;i<3;i++){
    motors[i].updatePI();
  }
}

void RobotMotion::reset(){
  uint8_t i;

  // Robot
  robvel_ctrl_enabled = true;
  v_r  = 0;
  vn_r = 0;
  w_r  = 0;
  // Motor
  for(i=0;i<3;i++){
    motors[i].stop();
  }
}

void RobotMotion::set_robvel_r(uint8_t index,float new_vi_r){
  // Enable control of the robot velocity
  robvel_ctrl_enabled = true;
  switch (index){
  case 0:
    v_r  = new_vi_r;
    break;
  
  case 1:
    vn_r  = new_vi_r;
    break;
  
  case 2:
    w_r  = new_vi_r;
    break;
  }
}


void RobotMotion::set_robvel_r(float new_v_r, float new_vn_r, float new_w_r){
  // Enable control of the robot velocity
  robvel_ctrl_enabled = true;
  v_r  = new_v_r;
  vn_r = new_vn_r;
  w_r  = new_w_r;
}
