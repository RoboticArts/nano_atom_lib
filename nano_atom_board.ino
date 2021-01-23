#define USE_USBCON

#include <PID_v1.h>
#include <elapsedMillis.h>

#include <MotorDriver.h>
#include <MotorEncoder.h>
#include <MotorControl.h>
#include <LedsBehavior.h>
#include <BNO055.h>

#include "nano_atom_board.h"


elapsedMillis timeout_sys, timeout_motor, timeout_imu;

BNO055 imu_sensor(0x29);

LedsBehavior leds_behavior(LED_SIZE, LED_PIN);
LedsBehavior::LedProperties led_properties; 

const double Kp = 10000;   
const double Ki = 200000; 
const double Kd = 10; 

MotorDriver left_motor(PWM_FRONT_LEFT,  IN1_FRONT_LEFT,  IN2_FRONT_LEFT,  STBY_FRONT, COUNTERCLOCKWISE, PWM_16_BITS);
MotorEncoder left_encoder(ENCODER_1, ENC_A_FRONT_LEFT, ENC_B_FRONT_LEFT, PULSES_PER_TURN, WHEEL_RADIUS);
MotorControl left_control(left_motor, left_encoder, DIRECT, SETPOINT_THRESHOLD, MINIMUM_SETPOINT,  Kp, Ki, Kd);

MotorDriver right_motor(PWM_FRONT_RIGHT, IN1_FRONT_RIGHT, IN2_FRONT_RIGHT, STBY_FRONT, COUNTERCLOCKWISE, PWM_16_BITS);
MotorEncoder right_encoder(ENCODER_2, ENC_A_FRONT_RIGHT, ENC_B_FRONT_RIGHT, PULSES_PER_TURN, WHEEL_RADIUS);
MotorControl right_control(right_motor, right_encoder, REVERSE, SETPOINT_THRESHOLD, MINIMUM_SETPOINT, Kp, Ki, Kd);
  

void setup(){
  
  imu_sensor.begin();
  left_control.begin();
  right_control.begin();
  leds_behavior.begin();

  pinMode(PB0,INPUT);
  
  Serial.begin(115200);
  while(!Serial);

    
  led_properties.command = LedsBehavior::Blink;
  led_properties.init_led = 0;
  led_properties.end_led = 1;
  led_properties.color = LedsBehavior::Blue;
  led_properties.time = 500;

}

uint64_t posL = 0, posR = 0;
float velL = 0, velR = 0;
float setpoint = 0;


void loop(){

   setpoint = map(analogRead(PB0), 0, 4095, -6300, 6300) / 1000.0; 
 

   left_control.setVelocity(setpoint);
   left_control.run();
   posL = left_control.getPosition("rad");
   velL = left_control.getVelocity("rad/s");

  
   right_control.setVelocity(setpoint);
   right_control.run();
   posR = right_control.getPosition("rad");
   velR = right_control.getVelocity("rad/s");
   
   leds_behavior.setBehavior(led_properties); 
   leds_behavior.run();


  
   if (timeout_motor > 10){
      timeout_motor = 0;

      Serial.print(0);
      Serial.print(",");
      Serial.print(7*100);      
      Serial.print(",");
      Serial.print(setpoint*100);
      Serial.print(",");
      Serial.print(-setpoint*100);
      Serial.print(",");
      Serial.print(velL*100);
      Serial.print(",");
      Serial.println(velR*100);

   }

   if (timeout_imu > 50){
      timeout_imu = 0;
      BNO055::Imu imu_data = imu_sensor.read();
      //imu_sensor.print(imu_data);
   }
   
  }
