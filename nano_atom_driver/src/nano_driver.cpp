#include <nano_atom_driver/nano_driver.h>


NanoDriver::NanoDriver(ros::NodeHandle nodehandle):_nh(nodehandle)
{
  // Get name of this node
  node_name = ros::this_node::getName();

  // Read params from rosparams
  readRosParams();

  // Enable ROS_DEBUG output
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
      ros::console::notifyLoggerLevelsChanged();

 
  motor_setpoint_sub = _nh.subscribe(motor_setpoint_topic, 1, &NanoDriver::motorSetpointCallback, this);  // ROS
  motor_setpoint_pub = _nh.advertise<std_msgs::Float32MultiArray>("hardware_motor/setpoint", 1000);       // Real hardware

  motor_state_pub = _nh.advertise<sensor_msgs::JointState>(motor_state_topic, 1);                          // ROS
  motor_state_sub = _nh.subscribe("hardware_motor/state", 1, &NanoDriver::motorStateCallback, this);     // Real hardware

  if (use_imu){
    imu_data_pub = _nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);                           // ROS
    imu_data_sub = _nh.subscribe("hardware_imu/data", 1, &NanoDriver::imuDataCallback, this);  // Real hardware
    // ros::topic::waitForMessage<sensor_msgs::Imu>("hardware_imu/data");
  }

  if (use_leds){
    set_leds_service = _nh.advertiseService(leds_service, &NanoDriver::setSignalCallback, this);   // ROS
    set_leds_pub     =  _nh.advertise<std_msgs::String>("hardware_leds/set_properties", 1000);     // Real hardware
  }

  num_joints = 2;
  
  motor_setpoint.data.resize(num_joints);

  motor_state.name.resize(num_joints);
  motor_state.position.resize(num_joints);
  motor_state.velocity.resize(num_joints);
  motor_state.effort.resize(num_joints);

  imu_data.header.seq = 0;
  imu_data.header.frame_id = imu_frame_id;
  imu_data.orientation.w = 1;

  // Node ready
  ROS_INFO("%s node ready!", node_name.c_str());

}

void NanoDriver::readRosParams(){

  ros::param::param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu_link");
  ros::param::param<std::string>(node_name + "/imu_topic", imu_topic, "imu/data");

  ros::param::param<std::string>(node_name + "/leds_service", leds_service, "set_leds_signal");

  ros::param::param<std::string>(node_name + "/motor_frame_id", motor_frame_id, "base_link");
  ros::param::param<std::string>(node_name + "/motor_setpoint_topic", motor_setpoint_topic, "motor_controller/set_setpoint");
  ros::param::param<std::string>(node_name + "/motor_state_topic", motor_state_topic, "motor_controller/get_state");
  
  ros::param::param<bool>(node_name + "/use_imu", use_imu, "true");
  ros::param::param<bool>(node_name + "/use_leds", use_leds, "true");


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


// ------------------ NANO ATOM CALLBACKS -------------------- //

// From ROS to hardware
void NanoDriver::motorSetpointCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){


    for (int motor=0; motor < num_joints; motor++){
      motor_setpoint.data[motor] = msg->data[motor];
    }

}

// From hardware to ROS
void NanoDriver::motorStateCallback(const sensor_msgs::JointState::ConstPtr& msg){

    motor_state.header.seq = motor_state.header.seq + 1;
    motor_state.header.stamp = ros::Time::now();
    motor_state.header.frame_id = motor_frame_id;
    std::string motor_name[4] = {"front_left_motor", "front_right_motor", "rear_right_motor", "rear_left_motor"};

    for (int motor=0; motor < num_joints; motor++){
      motor_state.name[motor] = motor_name[motor];
      motor_state.velocity[motor] = msg->velocity[motor];
      motor_state.position[motor] = msg->position[motor];
    }
  
}

// From ROS to hardware
void NanoDriver::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg){

    imu_data.header.seq = imu_data.header.seq + 1;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = imu_frame_id;

    imu_data.orientation.x = msg->orientation.x;
    imu_data.orientation.y = msg->orientation.y;
    imu_data.orientation.z = msg->orientation.z;
    imu_data.orientation.w = msg->orientation.w;

    imu_data.angular_velocity.x = msg->angular_velocity.x;
    imu_data.angular_velocity.y = msg->angular_velocity.y;
    imu_data.angular_velocity.z = msg->angular_velocity.z;
    
    imu_data.linear_acceleration.x = msg->linear_acceleration.x;
    imu_data.linear_acceleration.y = msg->linear_acceleration.y;
    imu_data.linear_acceleration.z = msg->linear_acceleration.z;

}


// From ROS to hardware
bool NanoDriver::setSignalCallback(roboticarts_msgs::SetLeds::Request& req,
                                   roboticarts_msgs::SetLeds::Response& res){

    if (leds_signals_file_exists == true){

        std::string led_properties = readLedsSignals(req.signal, req.enable);

        if(led_properties != ""){
          
          led_properties_msg.data = led_properties;
          set_leds_pub.publish(led_properties_msg);
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


// ------------------ NANO ATOM PUBLISHERS -------------------- //


// From ROS to hardware
void NanoDriver::motorSetpointPublisher(){

  motor_setpoint_pub.publish(motor_setpoint);
}


// From hardware to ROS
void NanoDriver::motorStatePublisher(){

  motor_state_pub.publish(motor_state);
}


// From hardware to ROS
void NanoDriver::imuDataPublisher(){

    imu_data_pub.publish(imu_data);
}



// ------------------ NANO ATOM FUNCTIONS -------------------- //


std::string NanoDriver::readLedsSignals(std::string signal, bool enable){

  std::string led_properties_msg = "";
  std::vector<std::string> led_properties(9);

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
      led_properties[8] = std::to_string(enable);
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




void NanoDriver::run(){


  ros::Rate loop_rate(50);

  while (ros::ok()){

      motorSetpointPublisher();
      motorStatePublisher();
      
      if (use_imu)
        imuDataPublisher();

      ros::spinOnce();
      loop_rate.sleep();

  }
       
}

