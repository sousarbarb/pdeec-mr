#ifndef BALLSENSOR_h
#define BALLSENSOR_h

#include <Arduino.h>

#define PIN_BALL_SENSOR    34  // Ball Sensor Arduino Pin

class BallSensor
{
public:
  uint8_t value;

  BallSensor();
  void init(void (*serialWriteFunction)(char c, int32_t v));
  void readSensor();
  void send();

private:
  void (*serialWrite)(char c, int32_t v);
};

#endif
