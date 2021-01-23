

  #include "MotorControl.h"

  MotorControl::MotorControl(MotorDriver &motor, MotorEncoder &encoder, int direction, 
                             float setpoint_threshold, float minimun_setpoint, double Kp, double Ki, double Kd){
    
    this->motor = new MotorDriver(motor);
    this->encoder = new MotorEncoder(encoder);
    this->direction = direction;
    this->setpoint_threshold = setpoint_threshold;
    this->minimum_setpoint = minimum_setpoint;
    
    mPID = new PID(&this->Input, &this->Output, &this->Setpoint, Kp, Ki, Kd, DIRECT);


  }

  void MotorControl::begin(void){

    // Motor begin
    motor->begin();
    motor->setDeadzone(0);

    // Encoder begin
    encoder->begin();

   // PID begin
    mPID->SetOutputLimits(-65535, 65535);
    mPID->SetSampleTime(5);
    mPID->SetMode(AUTOMATIC);
    
    // Initial setpoint
    Setpoint = 0.0;
  }

  void MotorControl::run(void){

    if (direction == DIRECT)
      Setpoint = Setpoint;
    if (direction == REVERSE)
      Setpoint = -Setpoint;
   
    
 
    if (abs(Setpoint) >= this->minimum_setpoint){
      
      Input = encoder->getSpeed("rad/s");
      mPID->Compute();
  
      if(Setpoint >= 0 && Output < 0)
        Output = 0;
      if(Setpoint < 0 && Output > 0)
        Output = 0;
      
      motor->setPWM(Output);
    
    }
      
    else if (abs(Setpoint) >= this->setpoint_threshold){
    
        Input = encoder->getSpeed("rad/s");
        if(Setpoint >= 0)
          Setpoint = this->minimum_setpoint;
        else
          Setpoint = - this->minimum_setpoint;
          
        mPID->Compute();
  
        if(Setpoint >= 0 && Output < 0)
          Output = 0;
        if(Setpoint < 0 && Output >= 0)
           Output = 0;
           
        motor->setPWM(Output);
    }
    
    else if (abs(Setpoint) < this->setpoint_threshold){
    
       Input = 0;
       Setpoint = 0;
       mPID->Compute();
       motor->setPWM(0);
    }
  
     //Serial.println(Output);  
  }


  void MotorControl::setVelocity(double velocity){

    this->Setpoint = velocity;
    
  }

  
  float MotorControl::getVelocity(String units){

    //return this->Input;
    return encoder -> getSpeed(units);
  }

  float MotorControl::getPosition(String units){
    
    return encoder->getPosition(units);
  }

  void MotorControl::resetPosition(){

    encoder->resetPosition();
  }
