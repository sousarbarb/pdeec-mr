#ifndef IMU_H
#define IMU_H

#include "MPU9255.h"

const int NUM_CALIBRATE_MEASURES = 500;
const int MAX_ACCEL_CALIBRATION_ITERATIONS = 5000;
const short MAX_ACCEL_OFFSET_ERROR = 10;            // maximum deviation 5 LSB

struct Compass
{
  int16_t x, y, z;
  float offsetX, offsetY, offsetZ;
  float scaleX, scaleY, scaleZ;
};

struct Accel
{
  int16_t x, y, z;
};

class IMU
{
public:
  Compass compass;
  Accel accel;
  
  void init(void (*serialWriteFunction)(char c, int32_t v));
  void calibrateCompass();
  void calibrateAccel();
  void readAll();
  void readCompass();
  void readAccel();
  void send();

private:
  MPU9255 mpu;
  void (*serialWrite)(char c, int32_t v);
};

#endif
