

  #include "BNO055.h"
  

  BNO055::BNO055(int i2c_address){

     imu = new Adafruit_BNO055(-1, i2c_address);
  
  }

  
  void BNO055::begin(){

      // Start BNOO055 in IMU mode, usefull for robotics
      if(imu->begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
        imu_status = IMU_OK;
      else
        imu_status = IMU_BAD_INIT;
        
  }

  BNO055::Imu BNO055::read(){

    BNO055::Imu imu_data;
    imu::Quaternion quaternion = imu->getQuat(); 
    imu::Vector<3> linear_acceleration = imu->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> angular_velocity = imu->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    imu_data.orientation.x = quaternion.x();
    imu_data.orientation.y = quaternion.y();
    imu_data.orientation.z = quaternion.z();
    imu_data.orientation.w = quaternion.w();

    imu_data.linear_acceleration.x = linear_acceleration.x();
    imu_data.linear_acceleration.y = linear_acceleration.y();
    imu_data.linear_acceleration.z = linear_acceleration.z();

    imu_data.angular_velocity.x = angular_velocity.x()*(PI/180);
    imu_data.angular_velocity.y = angular_velocity.y()*(PI/180);
    imu_data.angular_velocity.z = angular_velocity.z()*(PI/180);

   return imu_data;
  }

  void BNO055::print(BNO055::Imu imu_data){
    
    String quat_msg = "QUAT x: " + String(imu_data.orientation.x) + " y: " + String(imu_data.orientation.y) +
                      " z: " + String(imu_data.orientation.z) + " z: " + String(imu_data.orientation.w);
 
    imu::Quaternion quaternion(imu_data.orientation.x, imu_data.orientation.y, 
                               imu_data.orientation.z, imu_data.orientation.w);
    
    imu::Vector<3> euler = quaternion.toEuler()*180/PI;

    String euler_msg = "EULER x: " + String(euler.x()) + " y: " + String(euler.y()) + " z: " + String(euler.z());

    String acclin_msg = "ACC LIN x: " + String(imu_data.linear_acceleration.x) + " y: " +
                        String(imu_data.linear_acceleration.y) +" z: " + String(imu_data.linear_acceleration.z);

    String gyr_msg = "GYR x: " + String(imu_data.angular_velocity.x) + " y: " +
                        String(imu_data.angular_velocity.y) +" z: " + String(imu_data.angular_velocity.z);
 
    Serial.println("--------------------");
    Serial.println(quat_msg);
    Serial.println(euler_msg);
    Serial.println(acclin_msg);
    Serial.println(gyr_msg);
    
  }

  String BNO055::status(){

    String status_message;

    switch(imu_status){
      case IMU_OK:       status_message="IMU OK";             break;
      case IMU_BAD_INIT: status_message="IMU bad init";       break;
      case IMU_ERROR:    status_message="IMU error";          break;
      default:           status_message="IMU unknown status"; break;
    }
    
    return status_message;  
  }
