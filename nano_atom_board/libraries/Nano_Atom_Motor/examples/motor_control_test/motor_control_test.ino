
#include <MotorDriver.h>
#include <MotorEncoder.h>
#include <MotorControl.h>

#define ENC_A_MOTOR PB14
#define ENC_B_MOTOR PB15
#define PWM_MOTOR   PB1   
#define IN1_MOTOR   PB11
#define IN2_MOTOR   PB10
#define STBY_MOTOR  PA3

#define PULSES_PER_TURN float(1045.86) // Measurement obtained experimentally with an auxiliary tachometer
#define WHEEL_RADIUS float(0.0215)

#define SETPOINT_THRESHOLD 0.3 // Setpoints between 0 and SETPOINT_THRESHOLD are considered zero 
#define MINIMUM_SETPOINT 1.5    // Setpoints between SETPOINT_THRESHOLD and MINIMUM_SETPOINT are considered MINIMUM_SETPOINT

const double Kp = 10000;   
const double Ki = 200000; 
const double Kd = 10;

MotorDriver motor_driver(PWM_MOTOR, IN1_MOTOR, IN2_MOTOR, STBY_MOTOR, COUNTERCLOCKWISE, PWM_16_BITS);
MotorEncoder motor_encoder(ENCODER_1, ENC_A_MOTOR, ENC_B_MOTOR, PULSES_PER_TURN, WHEEL_RADIUS);
MotorControl motor_control(motor_driver, motor_encoder, DIRECT, SETPOINT_THRESHOLD, MINIMUM_SETPOINT, Kp, Ki, Kd);

float setpoint;
float last_time;

void setup() {
  
  motor_control.begin();

  setpoint = 4.0;
  last_time = millis();

  Serial.begin(115200);
  while(!Serial);
  
}

void loop() {

   motor_control.setVelocity(setpoint);
   motor_control.run();

   if (millis() - last_time > 50){
      
      Serial.println(motor_control.getVelocity("rad/s"));
      last_time = millis();
   }
}
