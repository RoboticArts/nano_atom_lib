

  #include "RosInterface.h"



  RosInterface::RosInterface():
     motor_setpoint_sub("hardware_motor/setpoint", &RosInterface::motor_setpoint_callback, this),
     motor_state_pub("hardware_motor/state", &motor_state),
     imu_data_pub("hardware_imu/data", &imu_data),
     led_properties_sub("hardware_leds/set_properties", &RosInterface::led_properties_callback, this)
     {

       this->refresh_time = 100;


        motor_state.position= pos_state;
        motor_state.velocity= vel_state;
        motor_state.effort= eff_state;
               
        motor_state.position_length=4;
        motor_state.velocity_length=4;
        motor_state.effort_length=4;

     }


  
  void RosInterface::begin(int baudrate){

      nh.getHardware()->setBaud(baudrate);
      nh.initNode();
      nh.subscribe(motor_setpoint_sub);
      nh.advertise(motor_state_pub);
      nh.advertise(imu_data_pub);
      nh.subscribe(led_properties_sub);
    
  } 


  void RosInterface::run(){

    time_now = millis();
    
    if (time_now - time_last >= refresh_time ){

        motor_state_pub.publish(&motor_state);
        imu_data_pub.publish(&imu_data);
        nh.spinOnce();
  
        
    }
  
  
  }


  
  void RosInterface::motor_setpoint_callback(const std_msgs::Float32MultiArray& msg){

    for(int motor = 0; motor<=2; motor++)
        motor_setpoint[motor] = msg.data[motor];


  }
  
  
  
  void RosInterface::led_properties_callback(const std_msgs::String& msg){

    led_properties = msg.data;
     nh.loginfo(msg.data);

 
  }


  void RosInterface::setMotorState(int motor, float position, float velocity){

    motor_state.position[motor] = position;
    motor_state.velocity[motor] = velocity;
    motor_state.effort[motor] = 0;
  
  }
  

  float RosInterface::getMotorSetpoint(int motor){
      
    return motor_setpoint[motor];
    
  }



  void RosInterface::setImuData(float orientation[4], float angular_velocity[3], float linear_acceleration[3]){


    imu_data.orientation.x = orientation[0];
    imu_data.orientation.y = orientation[1];
    imu_data.orientation.z = orientation[2];
    imu_data.orientation.w = orientation[3];
    
    imu_data.angular_velocity.x = angular_velocity[0];
    imu_data.angular_velocity.y = angular_velocity[1];
    imu_data.angular_velocity.z = angular_velocity[2];
    
    imu_data.linear_acceleration.x = linear_acceleration[0];
    imu_data.linear_acceleration.y = linear_acceleration[1];
    imu_data.linear_acceleration.z = linear_acceleration[2];

    
  }



  String RosInterface::getLedsProperties(){

    return led_properties;
    
  }


  
