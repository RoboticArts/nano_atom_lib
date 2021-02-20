#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H


#define PWM_16_BITS 16 // 0 - 65535

#define CLOCKWISE 1
#define COUNTERCLOCKWISE -1

#include <Arduino.h>

class MotorDriver{ 
  
  public:

    MotorDriver(int pwm_pin, int in1_pin, int in2_pin, int stby_pin, int direction, int pwm_resolution);
    void begin(void);
    void setDeadzone(int deadzone);
    void setPWM(int pwm_value);

  private:
  
    int IN1, IN2, STBY, PWMx;
    int direction;
    int pwm_resolution = 8;
    int pwm_limit;
    int deadzone = 0;
    
    int setLimits(int value, int limit);
    
};

#endif
