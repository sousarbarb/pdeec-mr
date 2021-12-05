// Timer 1 - https://www.pjrc.com/teensy/td_libs_TimerOne.html

#ifndef ROLLERMOTORS_h
#define ROLLERMOTORS_h

#include <Arduino.h>
#include "TimerOne.h"

#define LEFT                        0
#define RIGHT                       1
#define NUM_ROLLER_MOTORS           2
// TIMER1_A_PIN - pin 11                // Motor left PWM input
#define ML_DIR_PIN                  10  // Motor left dir
#define ML_CURR_SENSE_PIN           A6  // Motor left current sense
#define ML_SPEED_PIN                A4  // Motor left speed voltage sense
// TIMER1_B_PIN - pin 12                // Motor right PWM input
#define MR_DIR_PIN                  13  // Motor right dir
#define MR_CURR_SENSE_PIN           A7  // Motor right current sense
#define MR_SPEED_PIN                A5  // Motor right speed voltage sense
#define ROLLER_MOTORS_MAX_VOLTAGE   24  // Motor max voltage
#define ROLLER_MOTORS_POWER_VOLTAGE 34  // drive power voltage
#define ROLLER_MOTORS_MAX_CURRENT   2   // motor max current (drive only accepts 2A)
#define MAX_PWM                     1023  
#define ROLLERS_CURR_CONTROL_ENABLE true

static const float DRIVE_CURR_RESISTOR = 0.5;
static const float ADC_TO_VOLTS = 0.004887586;

class RollerMotors
{
private:
  uint16_t pwmValue[NUM_ROLLER_MOTORS];
  float current[NUM_ROLLER_MOTORS];
  float voltage[NUM_ROLLER_MOTORS];
  uint16_t csVoltage[NUM_ROLLER_MOTORS];
  uint16_t svsVoltage[NUM_ROLLER_MOTORS];
  void (*serialWrite)(char c, int32_t v);

  void limitVoltage(float &volt);
  void initTimers();
  uint16_t voltageToPWMDuty(float volt);
  
public:
  void init(void (*serialWriteFunction)(char c, int32_t v)); 
  void setVoltage(uint8_t num, float volt);
  void sendVoltage();
  void sendVoltageML();
  void sendVoltageMR();
  void readSpeed();
  void readCurrent();
  void send();
  void sendCurrent();
  void sendSpeed();
  void stopMotor(uint8_t num);
};

#endif
