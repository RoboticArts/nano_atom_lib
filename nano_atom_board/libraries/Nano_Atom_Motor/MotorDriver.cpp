

  #include "MotorDriver.h"


  MotorDriver::MotorDriver(int pwm_pin, int in1_pin, int in2_pin, int stby_pin, int direction, int pwm_resolution){

    this->direction = direction;

    this->pwm_limit = pow(2,pwm_resolution);
    this->pwm_resolution = pwm_resolution; 

    this->IN1 = in1_pin;
    this->IN2 = in2_pin;
    this->STBY = stby_pin;    
    this->PWMx  = pwm_pin;
   
  }

  void MotorDriver::begin(){
    
    pinMode(this->IN1, OUTPUT);
    pinMode(this->IN2, OUTPUT);
    pinMode(this->STBY, OUTPUT);
    pinMode(this->PWMx, PWM);
    
    digitalWrite(STBY, HIGH); // High PWMxable driver
  }

  void MotorDriver::setDeadzone(int deadzone){

      this->deadzone = deadzone;
    
  }

  int MotorDriver::setLimits(int value, int limit){
 
    if(value > limit)
        value = limit;

    if(value < -limit)
        value = -limit;

    return value;
    
  }


  void MotorDriver::setPWM(int pwm_value){

    pwm_value = direction * pwm_value;
    pwm_value = setLimits(pwm_value, this->pwm_limit);
  
    pwmWrite(this->PWMx, abs(pwm_value));


    if(pwm_value > this->deadzone){
      
        digitalWrite(this->IN1, HIGH);
        digitalWrite(this->IN2, LOW);
    }

    else if(pwm_value < -this->deadzone){
      
        digitalWrite(this->IN1, LOW);
        digitalWrite(this->IN2, HIGH);
    }

    else{
      
        digitalWrite(this->IN1, LOW);
        digitalWrite(this->IN2, LOW);
    }

    
  }
  
  
