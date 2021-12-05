#include "imu.h"

void IMU::init(void (*serialWriteFunction)(char c, int32_t v))
{
  serialWrite = serialWriteFunction;
  mpu.init();
  // TODO - calibration???
}

void IMU::calibrateCompass()
{
  // calibrate hard iron
  int16_t maxX = -30000, maxY=-30000, maxZ=-30000;
  int16_t minX = 30000, minY = 30000, minZ = 30000;
  
  for(int16_t i=0; i<NUM_CALIBRATE_MEASURES; i++)
  {
    mpu.read_mag();
    if(mpu.mx > maxX)
    {
      maxX = mpu.mx;
    }
    if(mpu.mx < minX)
    {
      minX = mpu.mx;
    }
    if(mpu.my > maxY)
    {
      maxY = mpu.my;
    }
    if(mpu.my < minY)
    {
      minY = mpu.my;
    }
    if(mpu.mz > maxZ)
    {
      maxZ = mpu.mz;
    }
    if(mpu.mz < minZ)
    {
      minZ = mpu.mz;
    }
    delay(10);
  }

  compass.offsetX = (maxX + minX)/2;
  compass.offsetY = (maxY + minY)/2;
  compass.offsetZ = (maxZ + minZ)/2;

  // calibrate soft iron
  float avgDeltaX = (maxX - minX)/2;
  float avgDeltaY = (maxY - minY)/2;
  float avgDeltaZ = (maxZ - minZ)/2;
  float avgDelta = (avgDeltaX + avgDeltaY + avgDeltaZ)/3;

  compass.scaleX = avgDelta/avgDeltaX;
  compass.scaleY = avgDelta/avgDeltaY;
  compass.scaleZ = avgDelta/avgDeltaZ;
}

void IMU::calibrateAccel()
{
  int16_t it = 0;
  int16_t offsetAx = 0;
  int16_t offsetAy = 0;
  int16_t offsetAz = 0;
  bool updateAx = true;
  bool updateAy = true;
  bool updateAz = true;
  
  while(it++ < MAX_ACCEL_CALIBRATION_ITERATIONS)
  {
    mpu.read_acc();

    // adjust offsets
    if(mpu.ax>0 && updateAx==true)
    {
      offsetAx--;
    }
    else if(mpu.ax<0 && updateAx==true)
    {
      offsetAx++;
    }

    if(mpu.ay>0 && updateAy==true)
    {
      offsetAy--;
    }
    else if(mpu.ay<0 && updateAy==true)
    {
      offsetAy++;
    }

    if(mpu.az>0 && updateAz==true)
    {
      offsetAz--;
    }
    else if(mpu.az<0 && updateAz==true)
    {
      offsetAz++;
    }

    // update offsets
    if(updateAx==true)
    {
      mpu.set_acc_offset(X_axis, offsetAx);
    }
    if(updateAy==true)
    {
      mpu.set_acc_offset(Y_axis, offsetAy);
    }
    if(updateAz==true)
    {
      mpu.set_acc_offset(Z_axis, offsetAz);
    }

    // check if is adjusted
    if((abs(mpu.ax)-MAX_ACCEL_OFFSET_ERROR) <= 0)
    {
      updateAx = false;
    }
    if((abs(mpu.ay)-MAX_ACCEL_OFFSET_ERROR) <= 0)
    {
      updateAy = false;
    }
    if((abs(mpu.az)-MAX_ACCEL_OFFSET_ERROR) <= 0)
    {
      updateAz = false;
    }

    if(updateAx==false && updateAy==false && updateAz==false)
    {
      break;
    }

    delay(10);
  }
}

void IMU::readAll()
{
  mpu.read_acc();
  mpu.read_mag();
  mpu.read_gyro();
}

void IMU::readCompass()
{
  mpu.read_mag();
  
  //compass.x = (mpu.mx - compass.offsetX) * compass.scaleX;
  //compass.y = (mpu.my - compass.offsetY) * compass.scaleY;
  //compass.z = (mpu.mz - compass.offsetZ) * compass.scaleZ;
  
  // compass.x = mpu.mx - compass.offsetX;
  // compass.y = mpu.my - compass.offsetY;
  // compass.z = mpu.mz - compass.offsetZ;

  compass.x = mpu.mx;
  compass.y = mpu.my;
  compass.z = mpu.mz;
}

void IMU::readAccel()
{
  mpu.read_acc();

  accel.x = mpu.ax;
  accel.y = mpu.ay;
  accel.z = mpu.az;
}

void IMU::send()
{
  int32_t aux;
  aux = (int32_t) compass.x << 16 | compass.y;
  (*serialWrite)('n', aux);
  aux = (int32_t) compass.z << 16 | accel.z;
  (*serialWrite)('o', aux);
  aux = (int32_t) accel.x << 16 | accel.y;
  (*serialWrite)('p', aux);
}
