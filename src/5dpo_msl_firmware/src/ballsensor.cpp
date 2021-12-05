#include "ballsensor.h"

BallSensor::BallSensor()
{
  value = 0;
}

void BallSensor::init(void (*serialWriteFunction)(char c, int32_t v))
{
  serialWrite = serialWriteFunction;
  pinMode(PIN_BALL_SENSOR, INPUT);  
}

void BallSensor::readSensor()
{
  if (digitalRead(PIN_BALL_SENSOR) == HIGH)     // without ball
  {     
    value = 0;
  }
  else if (digitalRead(PIN_BALL_SENSOR) == LOW) // with ball
  {
    value = 1;
  }
}

void BallSensor::send()
{
  (*serialWrite)('l', (int32_t) value);
}
