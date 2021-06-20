#ifndef STM32_NANO_ATOM_H
#define STM32_NANO_ATOM_H

// Led priperties
#define LED_SIZE 18
#define LED_PIN PA7

// Motor properties
#define PULSES_PER_TURN float(1045.86) // Measurement obtained experimentally with an auxiliary tachometer
#define WHEEL_RADIUS float(0.0215)

// Motor names
#define MOTOR_LEFT   0
#define MOTOR_RIGHT  1

// Front left motor
#define ENC_A_FRONT_LEFT PB14
#define ENC_B_FRONT_LEFT PB15
#define PWM_FRONT_LEFT PB1   
#define IN1_FRONT_LEFT PB11
#define IN2_FRONT_LEFT PB10

// Front right motor
#define ENC_A_FRONT_RIGHT PB12
#define ENC_B_FRONT_RIGHT PB13
#define PWM_FRONT_RIGHT PA0  
#define IN1_FRONT_RIGHT PA2
#define IN2_FRONT_RIGHT PA1

#define STBY_FRONT PA3

// Control parameters
#define SETPOINT_THRESHOLD float(1.0) // Setpoints between 0 and SETPOINT_THRESHOLD are considered zero 
#define MINIMUM_SETPOINT float(1.0)    // Setpoints between SETPOINT_THRESHOLD and MINIMUM_SETPOINT are considered MINIMUM_SETPOINT


#endif
