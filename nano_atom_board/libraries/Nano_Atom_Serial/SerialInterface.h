#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#define JITBUS_BUFFER_SIZE 1000
#define JITBUS_DISABLE_LOG
#include <Arduino.h>
#include <jitbus.h>

//#define USE_STM32_HW_SERIAL

class SerialInterface{ 

  public:

    SerialInterface();
    void begin(int baudrate = 115200);
    void writeSerialFrequency(int frequency = 100);
    void run();
    void setMotorState(int motor, float position, float velocity);
    float getMotorSetpoint(int motor);
    void setImuData(float orientation[4], float angular_velocity[3], float linear_acceleration[3]);
    String getLedsProperties();

    SerialJitbus jit;

  private:

    
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


    Imu imu_data;
    MotorState motor_state;
    float motor_setpoint[4];
    char led_properties[31];

    unsigned long time_last = 0;
    unsigned long refresh_time;

};

#endif