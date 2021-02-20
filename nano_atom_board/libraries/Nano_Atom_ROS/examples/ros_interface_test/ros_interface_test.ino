#include <RosInterface.h>


enum motors {MOTOR_1, MOTOR_2};


RosInterface ros_interface;

void setup() {

  ros_interface.begin();

}

void loop() {

  // Updates interface
  ros_interface.run();

  // Get motor setpoints from ROS
  float setpoint_1 = ros_interface.getMotorSetpoint(0);
  float setpoint_2 = ros_interface.getMotorSetpoint(1);


  // Send motor position and velocity to ROS
  float position_1 = random(0,1000);
  float position_2 = random(0,1000);
  float velocity_1 = random(0,6);
  float velocity_2 = random(0,6);
  ros_interface.setMotorState(MOTOR_1, position_1, velocity_1);
  ros_interface.setMotorState(MOTOR_2, position_2, velocity_2);


  // Send IMU data
  float orientation[4] = {5,4,3,1};
  float angular_velocity[3] = {2.3, 6.7, 3.5};
  float linear_acceleration[3] = {random(0,5), random(0,5), random(0,5)};
  ros_interface.setImuData(orientation, angular_velocity, linear_acceleration);
  

  // Get led properties as a string
  String led_properties_msg = ros_interface.getLedsProperties();
}
