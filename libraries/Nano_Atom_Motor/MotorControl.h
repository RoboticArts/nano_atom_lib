#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

#include "PID_v1.h"
#include "MotorDriver.h"
#include "MotorEncoder.h"


class MotorControl{

  private:

     MotorDriver *motor;
     MotorEncoder *encoder;

     double Setpoint, Input, Output;
     int direction;
     float setpoint_threshold, minimum_setpoint;
     PID *mPID;
  
  public:

     MotorControl(MotorDriver &motor, MotorEncoder &encoder, int direction, float setpoint_threshold, float minimun_setpoint, double Kp, double Ki, double Kd);
     void begin(void);
     void run(void);
     void setVelocity(double velocity);
     float getVelocity(String units);
     float getPosition(String units);
     void resetPosition(void);
  

};





#endif
