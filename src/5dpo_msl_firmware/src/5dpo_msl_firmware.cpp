#include <channels.h>
#include "ballsensor.h"
#include "imu.h"
#include "kicker.h"
#include "rollermotors.h"
#include "robotMotion.h"
#include "encoders.h"

#define PIN_BATTERY_LEVEL    A0  // Battery Level Arduino Pin A0
#define PIN_EMERGENCY_BTN    14
#define MOTION_MOTOR_TIMEOUT 100

// ---------- global variables ----------
unsigned long currentMillis=0, previousMillis=0; 
uint32_t lastMotorMessMillis=0;
uint8_t timeout = 0; // 0 -> false; 1 -> true
channels_t serial_channels;
BallSensor ballSensor;
IMU imu;
int16_t batteryLevel;
Kicker kicker;
RollerMotors rollerMotors;
RobotMotion robotMotion;
Encoders encoders;

// ---------- function headers ----------
void process_serial_packet(char channel, uint32_t value, channels_t& obj);
void serial_write(uint8_t b);
void serialWriteChannels(char c, int32_t value);
void readPCData();
void readEncodersSerial();
void readBatteryLevel();
void sendBatteryLevel();
void checkMotionMotorsTimeout();
void checkEmergencyBtnState();

// ---------- implemenation -------------
void setup() 
{
  Serial.begin(115200); // PC communications
  Serial1.begin(38400); // Encoders
  serial_channels.init(process_serial_packet, serial_write);
  ballSensor.init(serialWriteChannels);
  //imu.init(serialWriteChannels);
  kicker.init(serialWriteChannels);
  //rollerMotors.init(serialWriteChannels);
  encoders.init(serialWriteChannels);
  robotMotion.init(serialWriteChannels,&encoders);
  pinMode(PIN_EMERGENCY_BTN, INPUT_PULLUP);

  serial_channels.send('r', 0); // reset signal

  // TO DEBUG WITHOUT ROBOT
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() 
{
  readPCData();
  readEncodersSerial();
  
  // main loop
  currentMillis = micros();
  if(currentMillis-previousMillis >= CTRL_TIME)
  {
    checkMotionMotorsTimeout();
    if (!timeout)
    {
      previousMillis = micros();
      encoders.updateEncoders();
      ballSensor.readSensor();
      readBatteryLevel();
      //imu.readCompass();
      //imu.readAccel();
      kicker.readCapacitor();
      kicker.chargeCapacitor();
      robotMotion.update();
      //rollerMotors.readCurrent();
      //rollerMotors.readSpeed();
      //rollerMotors.sendVoltage();
      ballSensor.send();
      //imu.send();
      sendBatteryLevel();
      kicker.sendCLevel();
      //rollerMotors.send();
      encoders.send();
      checkEmergencyBtnState();
    }
  }
}

void process_serial_packet(char channel, uint32_t value, channels_t& obj)
{
  switch (channel)
  {
    case 'G':
      lastMotorMessMillis = millis();
      robotMotion.robvel_ctrl_enabled = false;
      robotMotion.motors[0].set_motor_w_r( *((float*) &value) );
      break;
    case 'H':
      robotMotion.robvel_ctrl_enabled = false;
      robotMotion.motors[1].set_motor_w_r( *((float*) &value) );
      break;
    case 'I':
      robotMotion.robvel_ctrl_enabled = false;
      robotMotion.motors[2].set_motor_w_r( *((float*) &value) );
      break;
    case 'K':  // high kick
      kicker.setKickerPulse((int16_t) value);
      if((int16_t)value > 0)
      {
        kicker.highKick();
      }
      break;
    case 'L':  // low kick
      kicker.setKickerPulse((int16_t) value);
      if((int16_t)value > 0)
      {
        kicker.lowKick(); 
      }
      break;
    case 'M':  // solenoid
      if((int32_t)value == 0)
      {
        kicker.testActivateSolenoid();
      }
      break;
    case 'N':  // roller left
      rollerMotors.setVoltage(LEFT, ((int32_t)value)/1000.0);  // 3 decimal cases
      break;
    case 'O':  // roller right
      rollerMotors.setVoltage(RIGHT, ((int32_t)value)/1000.0); // 3 decimal cases
      break;
    case 'R':
      lastMotorMessMillis = millis();
      robotMotion.robvel_ctrl_enabled = false;
      robotMotion.motors[0].set_motor_pwm((int16_t) value);
      break;
    case 'S':
      robotMotion.robvel_ctrl_enabled = false;
      robotMotion.motors[1].set_motor_pwm((int16_t) value);
      break;
    case 'T':
      robotMotion.robvel_ctrl_enabled = false;
      robotMotion.motors[2].set_motor_pwm((int16_t) value);
      break;
    case 'U':
      lastMotorMessMillis = millis();
      robotMotion.set_robvel_r(0,*((float*) &value)); // v
      break;
    case 'V':
      robotMotion.set_robvel_r(1,*((float*) &value)); // vn
      break;
    case 'W':
      robotMotion.set_robvel_r(2,*((float*) &value)); // w
      break;
  }    
}

void serial_write(uint8_t b)
{
  Serial.write(b); 
}

void serialWriteChannels(char c, int32_t value)
{
  serial_channels.send(c, value);
}

void readPCData()
{
  byte serialByte;
  
  if (Serial.available() > 0)
  {
    serialByte = Serial.read();
    serial_channels.StateMachine(serialByte);   
  }
}

void readEncodersSerial()
{
  byte serialByte;
  
  if (Serial1.available() > 0)
  {
    serialByte = Serial1.read();
    encoders.updateStateMachine(serialByte); 
  }
}

void readBatteryLevel()
{
  batteryLevel = analogRead(PIN_BATTERY_LEVEL); // 0-1023 -> 0-5V (ampop)  
}

void sendBatteryLevel()
{
  serial_channels.send('m', (int32_t) batteryLevel);
}

void checkMotionMotorsTimeout(){
  if(millis()-lastMotorMessMillis>MOTION_MOTOR_TIMEOUT){
    robotMotion.reset();
    timeout = 1;
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
  {
    timeout = 0;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void checkEmergencyBtnState()
{
  uint8_t btnValue;
  
  if(digitalRead(PIN_EMERGENCY_BTN) == HIGH)
  {
    btnValue = 1;
  }
  else
  {
    btnValue = 0;
  }

  serialWriteChannels('k', (int32_t)btnValue);  
}
