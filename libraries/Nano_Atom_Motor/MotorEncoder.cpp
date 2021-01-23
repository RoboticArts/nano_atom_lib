
  #include "MotorEncoder.h"

  volatile float filtered_time_now[4] = {0,0,0,0};
  volatile float filtered_last_time[4] = {0,0,0,0};

  volatile int64_t counter_array[4] = {0,0,0,0};
  volatile bool direction_array[4] = {true, true, true, true};

  int encoder_pin_a_array[4] = {0,0,0,0};
  int encoder_pin_b_array[4] = {0,0,0,0};

  
  void measurePulse(int encoder){
    
     static float   time_now_array[4]  = {0,0,0,0};
     static float   last_time_array[4] = {0,0,0,0};

     static float elapsed_time[4] = {0,0,0,0};
     static float last_elapsed_time[4] = {0,0,0,0};

     direction_array[encoder] = digitalRead(encoder_pin_b_array[encoder]);

     if (direction_array[encoder] == true)
          counter_array[encoder]++;  
     if (direction_array[encoder] == false)
          counter_array[encoder]--;

     last_time_array[encoder] = time_now_array[encoder];
     time_now_array[encoder] = micros();

     if (time_now_array[encoder] - last_time_array[encoder] > 0){

        last_elapsed_time[encoder] = elapsed_time[encoder];
        elapsed_time[encoder] =  time_now_array[encoder] -  last_time_array[encoder];

        if ( abs(elapsed_time[encoder] - last_elapsed_time[encoder]) < 200  && last_elapsed_time[encoder] > 0){

            filtered_time_now[encoder] = time_now_array[encoder];
            filtered_last_time[encoder] = last_time_array[encoder];
        }
     }

  }


  // ---------------------  Encoder Interrupts ------------------- //
  
  // Encoder 1  
  void encoder_ISR1(){  
     measurePulse(ENCODER_1);
  }
  
  // Encoder 2  
  void encoder_ISR2(){  
     measurePulse(ENCODER_2);
  }
  
  // Encoder 3  
  void encoder_ISR3(){
     measurePulse(ENCODER_3);
  }
  
  // Encoder 4  
  void encoder_ISR4(){
     measurePulse(ENCODER_4);
  }
  

  // --------------------- Quadrature Encoder Class ------------------- //


  MotorEncoder::MotorEncoder(int encoder, int encoder_pin_a, int encoder_pin_b, int pulses_per_turn, float wheel_radius){

      this->encoder = encoder;
      this->encoder_pin_a = encoder_pin_a;
      this->encoder_pin_b = encoder_pin_b;
      this->pulses_per_turn = pulses_per_turn;
      this->wheel_radius = wheel_radius;

      encoder_pin_a_array[encoder] = encoder_pin_a;
      encoder_pin_b_array[encoder] = encoder_pin_b;

      rad_per_pulse = (2*PI)/float(pulses_per_turn);
      deg_per_pulse = 360/float(pulses_per_turn);
      
  }

  void MotorEncoder::begin(){

      pinMode(encoder_pin_a, INPUT);
      pinMode(encoder_pin_b, INPUT);

      pinMode(PB10, OUTPUT);
      pinMode(PB11, OUTPUT);
 
      digitalWrite(PB10, LOW);
      digitalWrite(PB11, LOW);

      if (encoder == ENCODER_1){
        attachInterrupt(digitalPinToInterrupt(encoder_pin_a), encoder_ISR1, RISING);
      }
      
      if (encoder == ENCODER_2){
        attachInterrupt(digitalPinToInterrupt(encoder_pin_a), encoder_ISR2, RISING);
      }
  
      if (encoder == ENCODER_3){
        attachInterrupt(digitalPinToInterrupt(encoder_pin_a), encoder_ISR3, RISING);
      }
      
      if (encoder == ENCODER_4){
        attachInterrupt(digitalPinToInterrupt(encoder_pin_a), encoder_ISR3, RISING);
      }
      
  }

  float MotorEncoder::getPosition(String units){

    int64_t current_pulses = 0;
    float wheel_position = 0;

    current_pulses = counter_array[encoder];

    // Set units
    if(units.equals("rad"))
      wheel_position = current_pulses * this->rad_per_pulse;

    if(units.equals("deg"))
      wheel_position = current_pulses * this->deg_per_pulse;
  
    if(units.equals("ticks"))
      wheel_position = current_pulses;
      
    return wheel_position;
  }

    
  void MotorEncoder::resetPosition(){
    
    counter_array[encoder] = 0;
  
  }


  float MotorEncoder::getSpeed(String units){

      float velocity_value = 0;
      float time_now = 0, last_time = 0;
      float period_per_pulse, period_per_turn;
      float f, rpm, w, v;
      int encoder = this->encoder;
      bool dir;

      time_now = filtered_time_now[encoder];
      last_time = filtered_last_time[encoder];
      dir = direction_array[encoder];
            
      period_per_pulse = (time_now - last_time)/1000000.0; 
      f = 1.0 / period_per_pulse;
      period_per_turn = period_per_pulse * this->pulses_per_turn;
  
      //Wheel velocities
      rpm = f*60; //rpm
      w = 2*PI*(1/period_per_turn); // rad/s
      v = w * this->wheel_radius; // m/s
  
  
      if(units.equals("m/s"))
          velocity_value = v;
      if(units.equals("rad/s"))
          velocity_value = w;
      if(units.equals("Hz"))
          velocity_value = f;
      if(units.equals("T"))
          velocity_value = period_per_turn;
      if(units.equals("rpm"))
          velocity_value = rpm; 
  
  
    if(dir == true){
      velocity_value = 1*abs(velocity_value);
    }
    else{
      velocity_value = -1*abs(velocity_value);
    }
  
    float current_period_per_pulse = (micros() - last_time)/1000.0;

    if (current_period_per_pulse > ENCODER_THRESHOLD){
      velocity_value = 0;
    }
  
   
    return velocity_value;

  }



 
