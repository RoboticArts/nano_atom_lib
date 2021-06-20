
#include <PID_v1.h>
#include <elapsedMillis.h>

#include <MotorDriver.h>
#include <MotorEncoder.h>
#include <MotorControl.h>
#include <LedsBehavior.h>
#include <SerialInterface.h>
#include <BNO055.h>

#include "nano_atom_board.h"

//#define DEBUG

// Comprobar que la IMU se enciende correctamente (imu status)
// Velocidad cero cuando no hay conexion con rosserial
// Revisar porque cuando se manda setpoint 0 la rueda vibra
//Implementar setBehavior con led properties sobrecargado
// Implementar DEBUG option

elapsedMillis timeout_sys, timeout_motor, timeout_imu;

BNO055 imu_sensor(0x29);

SerialInterface serial_interface;

LedsBehavior leds_behavior(LED_SIZE, LED_PIN);
LedsBehavior::LedProperties led_properties; 

const double Kp = 10000;   
const double Ki = 200000; 
const double Kd = 10; 

MotorDriver  left_driver(PWM_FRONT_LEFT,  IN1_FRONT_LEFT,  IN2_FRONT_LEFT,  STBY_FRONT, COUNTERCLOCKWISE, PWM_16_BITS);
MotorEncoder left_encoder(ENCODER_1, ENC_A_FRONT_LEFT, ENC_B_FRONT_LEFT, COUNTERCLOCKWISE, PULSES_PER_TURN, WHEEL_RADIUS);
MotorControl left_motor(left_driver, left_encoder, SETPOINT_THRESHOLD, MINIMUM_SETPOINT,  Kp, Ki, Kd);

MotorDriver  right_driver(PWM_FRONT_RIGHT, IN1_FRONT_RIGHT, IN2_FRONT_RIGHT, STBY_FRONT, CLOCKWISE, PWM_16_BITS);
MotorEncoder right_encoder(ENCODER_2, ENC_A_FRONT_RIGHT, ENC_B_FRONT_RIGHT, CLOCKWISE, PULSES_PER_TURN, WHEEL_RADIUS);
MotorControl right_motor(right_driver, right_encoder, SETPOINT_THRESHOLD, MINIMUM_SETPOINT, Kp, Ki, Kd);
  

float setpoint_1, setpoint_2;
float position_1, position_2;
float velocity_1, velocity_2;
String led_properties_msg;
BNO055::Imu imu_data;
float orientation[4], angular_velocity[3], linear_acceleration[3];


void setup(){
  
  imu_sensor.begin();
  left_motor.begin();
  right_motor.begin();
  leds_behavior.begin(); 
  
  #ifndef DEBUG
    serial_interface.begin(115200);
    serial_interface.writeSerialFrequency(200);
  #endif

  #ifdef DEBUG
    Serial.begin(9600);
    pinMode(PB0,INPUT);
  #endif


  while(!Serial);
  
  led_properties.command = LedsBehavior::Blink;
  led_properties.init_led = 0;
  led_properties.end_led = 8;
  led_properties.color = LedsBehavior::Blue;
  led_properties.time = 500;

}



void loop(){

  #ifndef DEBUG
  
    // Update system
    left_motor.run();
    right_motor.run();
    leds_behavior.run();
    serial_interface.run();  
  
  
    // Get motor setpoints from ROS
    setpoint_1 = serial_interface.getMotorSetpoint(MOTOR_LEFT);
    left_motor.setVelocity(setpoint_1);
  
    setpoint_2 = serial_interface.getMotorSetpoint(MOTOR_RIGHT);
    right_motor.setVelocity(setpoint_2);
  
    
    // Send motor position and velocity to ROS
    position_1 = left_motor.getPosition("rad");
    velocity_1 = left_motor.getVelocity("rad/s");
    serial_interface.setMotorState(MOTOR_LEFT, position_1, velocity_1);
  
    position_2 = right_motor.getPosition("rad");
    velocity_2 = right_motor.getVelocity("rad/s");
    serial_interface.setMotorState(MOTOR_RIGHT, position_2, velocity_2);
  
  
  
    // Send IMU data to ROS
  
     if (timeout_imu > 20){
        timeout_imu = 0;
        
        imu_data = imu_sensor.read(); 
        
        orientation[0] = imu_data.orientation.x;
        orientation[1] = imu_data.orientation.y;
        orientation[2] = imu_data.orientation.z;
        orientation[3] = imu_data.orientation.w;
        angular_velocity[0] = imu_data.angular_velocity.x;
        angular_velocity[1] = imu_data.angular_velocity.y;
        angular_velocity[2] = imu_data.angular_velocity.z;
        linear_acceleration[0] = imu_data.linear_acceleration.x;
        linear_acceleration[1] = imu_data.linear_acceleration.y;
        linear_acceleration[2] = imu_data.linear_acceleration.z;
        serial_interface.setImuData(orientation, angular_velocity, linear_acceleration);

        led_properties_msg = serial_interface.getLedsProperties(); 
        leds_behavior.setBehavior(led_properties_msg); 
  
     }

      

  // Get led properties as a string
  //led_properties_msg = "FORWARD,blink,2,17,0,10,0,500,";




  #endif

  
  #ifdef DEBUG

    left_motor.run();
    right_motor.run();

    setpoint_1 =  map(analogRead(PB0), 0, 4095, -6300, 6300) / 1000.0; 
    left_motor.setVelocity(setpoint_1);

    setpoint_2 =  map(analogRead(PB0), 0, 4095, -6300, 6300) / 1000.0; 
    right_motor.setVelocity(setpoint_2);


    if (timeout_motor > 10){
      timeout_motor = 0;

      Serial.print(0);
      Serial.print(",");
      Serial.print(7*100);      
      Serial.print(",");
      Serial.print(setpoint_1*100);
      Serial.print(",");
      Serial.print(-setpoint_2*100);
      Serial.print(",");
      Serial.print(1*100);
      Serial.print(",");
      Serial.print(left_motor.getVelocity("rad/s")*100);
      Serial.print(",");
      Serial.println(right_motor.getVelocity("rad/s")*100);

   }

    


    //leds_behavior.run();
    
     /*
     if (timeout_imu > 500){
        timeout_imu = 0;
        imu_data = imu_sensor.read(); 
        imu_sensor.print(imu_data);   
     }
     */

     
    // Get led properties as a string
    //  led_properties_msg = "BACKWARD,blink,0,2,0,100,0,200,";
    //  leds_behavior.setBehavior(led_properties_msg); 



  #endif
  





















/*
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
   */


   
  }
