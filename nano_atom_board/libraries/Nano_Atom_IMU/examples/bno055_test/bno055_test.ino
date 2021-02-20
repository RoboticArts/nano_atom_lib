
#include <BNO055.h>

BNO055 imu_sensor(0x29);

void setup() {

  imu_sensor.begin();
  
  Serial.begin(115200);
  while(!Serial);
   
}

void loop() {

  BNO055::Imu imu_data = imu_sensor.read();
  imu_sensor.print(imu_data);
  delay(1000);
  
}
