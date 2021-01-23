#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <Arduino.h>

#define ENCODER_1 0
#define ENCODER_2 1
#define ENCODER_3 2
#define ENCODER_4 3

#define ENCODER_THRESHOLD 250 // Timeout in ms to consider zero speed

class MotorEncoder{ 
  
  public:

    MotorEncoder(int encoder, int encoder_pin_a, int encoder_pin_b, int pulses_per_turn, float wheel_radius);
    void begin(void);
    float getSpeed(String units);
    float getPosition(String units);
    void resetPosition(void);

  private: 

    int pulses_per_turn = 0;
    float wheel_radius = 0;
    float rad_per_pulse = 0;
    float deg_per_pulse = 0;
    int encoder = 0; 
    int encoder_pin_a, encoder_pin_b;

};




#endif
