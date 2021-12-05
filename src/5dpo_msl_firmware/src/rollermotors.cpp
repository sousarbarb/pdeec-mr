// Timer 1 - https://www.pjrc.com/teensy/td_libs_TimerOne.html

#ifndef ROLLERMOTORS_cpp
#define ROLLERMOTORS_cpp

#include "rollermotors.h"

void RollerMotors::limitVoltage(float &volt)
{
  if(volt > ROLLER_MOTORS_MAX_VOLTAGE)
  {
    volt = ROLLER_MOTORS_MAX_VOLTAGE;
  }
  else if(volt < -ROLLER_MOTORS_MAX_VOLTAGE)
  {
    volt = -ROLLER_MOTORS_MAX_VOLTAGE;
  }
}

void RollerMotors::initTimers()
{
  Timer1.initialize(50); //us
  Timer1.pwm(TIMER1_A_PIN, pwmValue[LEFT]);
  Timer1.pwm(TIMER1_B_PIN, pwmValue[RIGHT]);
}

void RollerMotors::init(void (*serialWriteFunction)(char c, int32_t v))
{
  for(uint8_t i=0; i<NUM_ROLLER_MOTORS; i++)
  {
    pwmValue[i] = 0;
    voltage[i] = 0;
    current[i] = 0;
  }
  pinMode(ML_DIR_PIN, OUTPUT);
  pinMode(TIMER1_A_PIN, OUTPUT);

  pinMode(MR_DIR_PIN, OUTPUT);
  pinMode(TIMER1_B_PIN, OUTPUT);

  serialWrite = serialWriteFunction;

  initTimers();
}

uint16_t RollerMotors::voltageToPWMDuty(float volt)
{
  return abs(volt*MAX_PWM/ROLLER_MOTORS_POWER_VOLTAGE); 
}

void RollerMotors::setVoltage(uint8_t num, float volt)
{
  if(num >= 0 && num < NUM_ROLLER_MOTORS)
  {
    limitVoltage(volt);
    voltage[num] = volt;
  }
}

void RollerMotors::sendVoltage()
{
  sendVoltageML();
  sendVoltageMR();
}

void RollerMotors::sendVoltageML()
{
  uint16_t dutyCycle = voltageToPWMDuty(voltage[LEFT]);
  if(voltage[LEFT] >= 0)
  { 
    pwmValue[LEFT] = dutyCycle;
    digitalWrite(ML_DIR_PIN, LOW); 
  }
  else
  {
    pwmValue[LEFT] = MAX_PWM - dutyCycle;
    digitalWrite(ML_DIR_PIN, HIGH);
  }
  (*serialWrite)('z', pwmValue[LEFT]);
  Timer1.setPwmDuty(TIMER1_A_PIN, pwmValue[LEFT]);
}

void RollerMotors::sendVoltageMR()
{
  uint16_t dutyCycle = voltageToPWMDuty(voltage[RIGHT]);
  if(voltage[RIGHT] >= 0)
  {
    pwmValue[RIGHT] = dutyCycle;
    digitalWrite(MR_DIR_PIN, LOW);
  }
  else
  {
    pwmValue[RIGHT] = MAX_PWM - dutyCycle;
    digitalWrite(MR_DIR_PIN, HIGH);
  }
  Timer1.setPwmDuty(TIMER1_B_PIN, pwmValue[RIGHT]);   
}

void RollerMotors::readSpeed()
{
  svsVoltage[LEFT] = analogRead(ML_SPEED_PIN);  // 0-1023 -> 0-5V
  svsVoltage[RIGHT] = analogRead(MR_SPEED_PIN); // 0-1023 -> 0-5V
}

void RollerMotors::readCurrent()
{
  csVoltage[LEFT] = analogRead(ML_CURR_SENSE_PIN);  // 0-1023 -> 0-5V
  csVoltage[RIGHT] = analogRead(MR_CURR_SENSE_PIN); // 0-1023 -> 0-5V

  for(uint8_t i=0; i<NUM_ROLLER_MOTORS; i++)
  {
    current[i] = (float) csVoltage[i]*ADC_TO_VOLTS/DRIVE_CURR_RESISTOR;
    if(current[i] > ROLLER_MOTORS_MAX_CURRENT && ROLLERS_CURR_CONTROL_ENABLE)
    {
      stopMotor(i);
    }
  }
}

void RollerMotors::send()
{
  sendCurrent();
  sendSpeed();
}

void RollerMotors::sendCurrent()
{
  int32_t aux = (int32_t)(current[LEFT]*100) << 16 | (int16_t)(current[RIGHT]*100);
  (*serialWrite)('s', aux);  // *100 (2 decimal cases)
}

void RollerMotors::sendSpeed()
{
  int32_t aux = (int32_t) (svsVoltage[LEFT]) << 16 | (int16_t) svsVoltage[RIGHT]; 
  (*serialWrite)('t', aux); // 0-1023 -> 0-5V
}

void RollerMotors::stopMotor(uint8_t num)
{
  if(num>=0 && num<NUM_ROLLER_MOTORS)
  {
    setVoltage(num, 0);
    switch(num)
    {
      case LEFT:
        sendVoltageML();
        break;
      case RIGHT:
        sendVoltageMR();
        break;
    }
  }
}

#endif
