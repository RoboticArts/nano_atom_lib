#ifndef NANO_DRIVER_H
#define NANO_DRIVER_H


#include "ros/ros.h"
#include <roboticarts_msgs/SetLeds.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>


class NanoDriver{

    public:
        NanoDriver(ros::NodeHandle nodehandle);
        void run();

    private:

        ros::NodeHandle _nh;
        std::string node_name;

        XmlRpc::XmlRpcValue leds_signals; 
        bool leds_signals_file_exists;

        ros::Publisher  motor_setpoint_pub;
        ros::Subscriber motor_setpoint_sub;

        ros::Publisher  motor_state_pub;
        ros::Subscriber motor_state_sub;
        
        ros::Subscriber imu_data_sub;
        ros::Publisher imu_data_pub;

        ros::ServiceServer set_leds_service;
        ros::Publisher set_leds_pub;

        std_msgs::Float32MultiArray motor_setpoint;
        sensor_msgs::JointState motor_state;
        sensor_msgs::Imu imu_data;
        std_msgs::String led_properties_msg;

        void motorSetpointCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void motorStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
        void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
        bool setSignalCallback(roboticarts_msgs::SetLeds::Request& req, roboticarts_msgs::SetLeds::Response& res);
       
        void imuDataPublisher();
        void motorSetpointPublisher();
        void motorStatePublisher();

        void readRosParams();
        std::string readLedsSignals(std::string signal, bool enable);

        std::string imu_frame_id;
        std::string imu_topic;

        std::string motor_frame_id;
        std::string motor_setpoint_topic;
        std::string motor_state_topic;

        std::string leds_service;

        bool use_imu;
        bool use_leds;

        int num_joints;

};


#endif
