#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H


#include <Arduino.h>

#define USE_STM32_HW_SERIAL

#include "ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"


class RosInterface{ 

  public:

    RosInterface();
    void begin(int baudrate = 57600);
    void run();

    void setMotorState(int motor, float position, float velocity);
    float getMotorSetpoint(int motor);
    void setImuData(float orientation[4], float angular_velocity[3], float linear_acceleration[3]);
    String getLedsProperties();


  private:

    ros::NodeHandle  nh;

    float motor_setpoint[4] = {0,0,0,0};
 
    sensor_msgs::JointState motor_state;
    
    float pos_state[4]={0,0,0,0};
    float vel_state[4]={0,0,0,0};
    float eff_state[4]={0,0,0,0};
    
    sensor_msgs::Imu imu_data;
    String led_properties;


    void motor_setpoint_callback(const std_msgs::Float32MultiArray& msg);
    void led_properties_callback(const std_msgs::String& msg);
  
    ros::Subscriber<std_msgs::Float32MultiArray, RosInterface> motor_setpoint_sub;
    ros::Publisher motor_state_pub;
    ros::Publisher imu_data_pub; 
    ros::Subscriber<std_msgs::String, RosInterface> led_properties_sub;


    unsigned long time_now = 0, time_last = 0;
    unsigned long refresh_time;

  
};

#endif 
