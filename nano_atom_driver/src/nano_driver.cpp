#include <nano_atom_driver/nano_driver.h>


NanoDriver::NanoDriver(ros::NodeHandle nodehandle):_nh(nodehandle){

    // Get name of this node
    node_name = ros::this_node::getName();

    // Read params from rosparams
    readRosParams();

    // Enable ROS_DEBUG output
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged();

    ros_motor_setpoint_sub = _nh.subscribe(motor_setpoint_topic, 1, &NanoDriver::motorSetpointCallback, this); 

    ros_motor_state_pub = _nh.advertise<sensor_msgs::JointState>(motor_state_topic, 1000);

    if (use_imu == true){

        ros_imu_data_pub = _nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);                           
        // ros::topic::waitForMessage<sensor_msgs::Imu>("hardware_imu/data");
    }

    if (use_leds){

        ros_set_leds_service = _nh.advertiseService(leds_service, &NanoDriver::setSignalCallback, this);    
    }

    refresh_time = 1000 / float(write_serial_frequency);

    num_joints = 2;

    ros_motor_setpoint.data.resize(num_joints);

    ros_motor_state.name.resize(num_joints);
    ros_motor_state.position.resize(num_joints);
    ros_motor_state.velocity.resize(num_joints);
    ros_motor_state.effort.resize(num_joints);

    jit_motor_setpoint[0] = 0;
    jit_motor_setpoint[1] = 0;

    time_last = currentTime();

    motor_timer = 0;
    led_timer = 0;

    // Node ready
    ROS_INFO("%s node ready!", node_name.c_str());

}

void NanoDriver::readRosParams(){

    ros::param::param<std::string>(node_name + "/port", port, "/dev/ttyUSB0");
    ros::param::param<int>(node_name + "/baudrate", baudrate, 115200);

    ros::param::param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu_link");
    ros::param::param<std::string>(node_name + "/imu_topic", imu_topic, "imu/data");

    ros::param::param<std::string>(node_name + "/leds_service", leds_service, "set_leds_signal");

    ros::param::param<std::string>(node_name + "/motor_frame_id", motor_frame_id, "base_link");
    ros::param::param<std::string>(node_name + "/motor_setpoint_topic", motor_setpoint_topic, "motor_controller/set_setpoint");
    ros::param::param<std::string>(node_name + "/motor_state_topic", motor_state_topic, "motor_controller/get_state");

    ros::param::param<bool>(node_name + "/use_imu", use_imu, "true");
    ros::param::param<bool>(node_name + "/use_leds", use_leds, "true");

    ros::param::param<int>(node_name + "/write_serial_frequency", write_serial_frequency, 200);

    if (_nh.hasParam(node_name + "/leds_signals")){

        ROS_INFO("leds_signals.yaml found");
        ros::param::get(node_name + "/leds_signals", leds_signals);    
        leds_signals_file_exists = true;
   
    }
    else{

        ROS_WARN("Led signal configuration not found, check %s/leds_signals param exits", node_name.c_str());
        leds_signals_file_exists = false;

    }

}


std::string NanoDriver::readLedsSignals(std::string signal, bool enable){

    std::string led_properties_msg = "";
    std::vector<std::string> led_properties(8);

    // Get signal
    for (int index = 0; index<leds_signals.size(); index++){

        if (leds_signals[index]["name"] == signal){

            led_properties[0] = std::string(leds_signals[index]["name"]);
            led_properties[1] = std::string(leds_signals[index]["behavior"]);    
            led_properties[2] = std::to_string(int(leds_signals[index]["zone"][0]) - 1); 
            led_properties[3] = std::to_string(int(leds_signals[index]["zone"][1]) - 1);
            led_properties[4] = std::to_string(int(leds_signals[index]["color"][0]));
            led_properties[5] = std::to_string(int(leds_signals[index]["color"][1]));
            led_properties[6] = std::to_string(int(leds_signals[index]["color"][2]));
            led_properties[7] = std::to_string(int(leds_signals[index]["time"]));

        }  
    }

    // Check signal and create msg
    for (int property = 0; property<led_properties.size(); property++){

        led_properties_msg = led_properties_msg + led_properties[property] + ",";

        if (led_properties[property] == ""){
            ROS_WARN("Parameters of signal %s are wrong", signal.c_str());
            led_properties_msg = "";
            property = led_properties.size(); 
        }

    }

    return led_properties_msg;
}



void NanoDriver::motorSetpointCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){


    for (int motor=0; motor < num_joints; motor++){

        jit_motor_setpoint[motor] = msg->data[motor];
    }

}

bool NanoDriver::setSignalCallback(roboticarts_msgs::SetLeds::Request& req,
                                   roboticarts_msgs::SetLeds::Response& res){

    if (leds_signals_file_exists == true){

        std::string led_properties = readLedsSignals(req.signal, req.enable);
        ROS_INFO("%s", led_properties.c_str());
        if(led_properties != ""){
            
            memset(jit_led_properties, 0, sizeof(jit_led_properties));

            strcpy(jit_led_properties, led_properties.c_str()); 
        
            ROS_DEBUG("Signal %s sent", req.signal.c_str());
            
            res.message = "OK";
            res.success = true;
            
        }
        
        else{

            res.message = "Error: signal does not exist or is malformed";
            res.success = false;
        }

    }
    else{
      res.message = "Error: LedSignals file does not exist";
      res.success = false;
    }

  return true;
}

void NanoDriver::updateRosImuData(struct Imu imu_data){

    ros_imu_data.header.seq = ros_imu_data.header.seq + 1;
    ros_imu_data.header.stamp = ros::Time::now();
    ros_imu_data.header.frame_id = imu_frame_id;

    ros_imu_data.orientation.x = imu_data.orientation_x;
    ros_imu_data.orientation.y = imu_data.orientation_y;
    ros_imu_data.orientation.z = imu_data.orientation_z;
    ros_imu_data.orientation.w = imu_data.orientation_w;

    ros_imu_data.angular_velocity.x = imu_data.angular_velocity_x;
    ros_imu_data.angular_velocity.y = imu_data.angular_velocity_y;
    ros_imu_data.angular_velocity.z = imu_data.angular_velocity_z;
    
    ros_imu_data.linear_acceleration.x = imu_data.linear_acceleration_x;
    ros_imu_data.linear_acceleration.y = imu_data.linear_acceleration_y;
    ros_imu_data.linear_acceleration.z = imu_data.linear_acceleration_z;


}

void NanoDriver::updateRosMotorState(struct MotorState motor_state){

    ros_motor_state.header.seq = ros_motor_state.header.seq + 1;
    ros_motor_state.header.stamp = ros::Time::now();
    ros_motor_state.header.frame_id = motor_frame_id;
    
    std::string motor_name[4] = {"front_left_motor", "front_right_motor", "rear_right_motor", "rear_left_motor"};

    for (int motor=0; motor < num_joints; motor++){
      ros_motor_state.name[motor] = motor_name[motor];
      ros_motor_state.velocity[motor] = motor_state.velocity[motor];
      ros_motor_state.position[motor] = motor_state.position[motor];
    }
  
}

double NanoDriver::currentTime(){

    return ros::Time::now().toNSec()/1000000.0;

}


void NanoDriver::run(){

    jit.init(port.c_str(), baudrate, 20);

    ros::Rate loop_rate(5000);

    while (ros::ok()){


        jit.sendPacketHz(jit_led_properties, LED_ID, led_timer, 50); 
        jit.sendPacketHz(jit_motor_setpoint, MOTOR_SETPOINT_ID, motor_timer, 50);

        if(jit.available() > 0){

            if (jit.receivePacket(jit_motor_state, MOTOR_STATE_ID)){

                updateRosMotorState(jit_motor_state);
                ros_motor_state_pub.publish(ros_motor_state);
            }

            if (jit.receivePacket(jit_imu_data, IMU_DATA_ID)){
                
                updateRosImuData(jit_imu_data);
                ros_imu_data_pub.publish(ros_imu_data);  
            }
               
        }

        ros::spinOnce();
        loop_rate.sleep();

    }
    
}







