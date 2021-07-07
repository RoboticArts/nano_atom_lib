#ifndef NANO_DRIVER_H
#define NANO_DRIVER_H

#define JITBUS_BUFFER_SIZE 1000
#define JITBUS_DISABLE_LOG

#include "ros/ros.h"
#include <roboticarts_msgs/SetLeds.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include "jitbus.h"



class NanoDriver{

    public:
        NanoDriver(ros::NodeHandle nodehandle);
        void run();

    private:

        // Jitbus variables

        SerialJitbus jit;
        enum jit_ids {IMU_DATA_ID, MOTOR_STATE_ID, MOTOR_SETPOINT_ID, LED_ID};

        struct Imu {
            float orientation_x;
            float orientation_y;
            float orientation_z;
            float orientation_w;
            float angular_velocity_x;
            float angular_velocity_y;
            float angular_velocity_z;
            float linear_acceleration_x;
            float linear_acceleration_y;
            float linear_acceleration_z;
        };

        struct MotorState{
            
            float position[4];
            float velocity[4];

        };

        Imu jit_imu_data;
        MotorState jit_motor_state;
        float jit_motor_setpoint[4];
        char jit_led_properties[31];
        uint32_t motor_timer;
        uint32_t led_timer;

        // ROS variables 

        ros::NodeHandle _nh;
        std::string node_name;

        XmlRpc::XmlRpcValue leds_signals; 
        bool leds_signals_file_exists;

        ros::Subscriber  ros_motor_setpoint_sub;
        ros::Publisher  ros_motor_state_pub;    
        ros::Publisher ros_imu_data_pub;
        ros::ServiceServer ros_set_leds_service;

        std_msgs::Float32MultiArray ros_motor_setpoint;
        sensor_msgs::JointState ros_motor_state;
        sensor_msgs::Imu ros_imu_data;
        std_msgs::String ros_led_properties_msg;


        void motorSetpointCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        bool setSignalCallback(roboticarts_msgs::SetLeds::Request& req, roboticarts_msgs::SetLeds::Response& res);

        void readRosParams();
        std::string readLedsSignals(std::string signal, bool enable);

        void updateRosMotorState(struct MotorState motor_state);
        void updateRosImuData(struct Imu imu_data);

        double currentTime();

        std::string port;
        int baudrate;

        std::string imu_frame_id;
        std::string imu_topic;

        std::string motor_frame_id;
        std::string motor_setpoint_topic;
        std::string motor_state_topic;

        std::string leds_service;

        bool use_imu;
        bool use_leds;

        int num_joints;

        int refresh_time;
        double time_last;
        int write_serial_frequency;       

};


#endif
