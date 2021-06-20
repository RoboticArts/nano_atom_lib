
  #include "SerialInterface.h"


  SerialInterface::SerialInterface(){

      this->refresh_time = 10; // 100 Hz;

  }

  void SerialInterface::begin(int baudrate){

      Serial.begin(baudrate);
      jit.begin(Serial);
      jit.waitSerialUSB();
      jit.enable_color = true;
  }

  void SerialInterface::writeSerialFrequency(int frequency){

      this->refresh_time = 1000/float(frequency);

  }


  void SerialInterface::run(){

    if (millis() - time_last > refresh_time){
  
        jit.sendPacket(motor_state, MOTOR_STATE_ID);
        jit.sendPacket(imu_data, IMU_DATA_ID);
        time_last = millis();

    }

    if(jit.available()){

        jit.receivePacket(motor_setpoint, MOTOR_SETPOINT_ID);
        jit.receivePacket(led_properties, LED_ID);
            
    }

  }


  void SerialInterface::setMotorState(int motor, float position, float velocity){

        motor_state.position[motor] = position;
        motor_state.velocity[motor] = velocity;

  }

  float SerialInterface::getMotorSetpoint(int motor){

      return motor_setpoint[motor];
  }


  void SerialInterface::setImuData(float orientation[4], float angular_velocity[3], float linear_acceleration[3]){

    imu_data.orientation_x = orientation[0];
    imu_data.orientation_y = orientation[1];
    imu_data.orientation_z = orientation[2];
    imu_data.orientation_w = orientation[3];
    
    imu_data.angular_velocity_x = angular_velocity[0];
    imu_data.angular_velocity_y = angular_velocity[1];
    imu_data.angular_velocity_z = angular_velocity[2];
    
    imu_data.linear_acceleration_x = linear_acceleration[0];
    imu_data.linear_acceleration_y = linear_acceleration[1];
    imu_data.linear_acceleration_z = linear_acceleration[2];

  }

  String SerialInterface::getLedsProperties(){
    
    return String(led_properties);
  }