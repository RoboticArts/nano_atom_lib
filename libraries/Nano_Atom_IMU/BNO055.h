#ifndef BNO055_H
#define BNO055_H

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "elapsedMillis.h"


class BNO055{ 
  
  public:
 
    struct quaternionOrientation{
      float x;
      float y;
      float z;
      float w;
      };

    struct linearAcceleration{
      float x;
      float y;
      float z;
    };
    struct angularVelocity{
      float x;
      float y;
      float z;
    };

    struct Imu{
      quaternionOrientation orientation;
      linearAcceleration linear_acceleration;
      angularVelocity angular_velocity;
    };

    BNO055(int i2c_address);
    void begin(void);
    BNO055::Imu read(void);
    void print(BNO055::Imu imu_data);
    String status(void);
    
  private:
    Adafruit_BNO055* imu;
    int imu_status;
    enum status_list{IMU_OK, IMU_BAD_INIT, IMU_ERROR, IMU_UNKNOWN};
};

#endif
