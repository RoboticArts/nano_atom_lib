

  #include "LedsBehavior.h"


  LedsBehavior::LedsBehavior(const int &led_strip_size, const int &pin):
      LedsHardware(led_strip_size, pin){

      this->led_strip_size = led_strip_size;
  }


  void LedsBehavior::begin(){
      
     LedsHardware::begin();
  }

  void LedsBehavior::update(int refresh_time){
    
    show_time_now = millis();
    
    if (show_time_now - show_time_last >= refresh_time  ){
  
       show_time_last = show_time_now;     
       LedsHardware::update();
    }
    
  }
  

  // ==========================   Basic behaviors   ========================== //


  void LedsBehavior::paintLeds(int start_led, int end_led, uint32_t color){
  
    LedsHardware::fillLeds(start_led, end_led, color);  
  
  }
   
  void LedsBehavior::blinkLeds(int start_led, int end_led, uint32_t color, int blink_time){
  
    blink_time_now = millis();
  
    if(blink_time_now - blink_time_last > blink_time){
      
      blink_time_last = blink_time_now;
  
      if (blink_state == LOW){
  
        LedsHardware::fillLeds(start_led, end_led, color);
        blink_state = HIGH;
  
      }
      else{
        
        LedsHardware::fillLeds(start_led, end_led, Clear);
        blink_state = LOW;
      
      }
    }
    
  }
  
  
  void LedsBehavior::shiftParts(int start_parts[], int number_parts, int part_size,  uint32_t color, int shift_time, String direction){
    
    shift_time_now = millis();
    
    if(shift_time_now - shift_time_last > shift_time){
    
      shift_time_last = shift_time_now;
  
      int start_led;
      int end_led;
  
      if (shift_counter <= part_size){
        
          for (int part = 0; part < number_parts; part++){
              
              if (direction.equals("left")){
                  start_led = start_parts[part];
                  end_led = start_parts[part] + shift_counter;
              }
              else {
                  end_led = start_parts[part] + part_size;
                  start_led = end_led - shift_counter;
              }
              
              LedsHardware::fillLeds(start_led, end_led, color);     
              
          }
          
          shift_counter++; 
      }
  
      else{
          LedsHardware::fillLeds(0, led_strip_size , Clear);
          shift_counter = 0;
      }
  
           
          
    }
  }



  // ==========================   Composed behaviors   ========================== //

  
  void LedsBehavior::bootingLeds(){
    blinkLeds(0, led_strip_size-1, White, 500);
  }
  
  void LedsBehavior::readyLeds(){
    paintLeds(0, led_strip_size-1, Green );
  }
  
  void LedsBehavior::exitLeds(){
    paintLeds(0, led_strip_size-1, Blue );
  }


  // ==========================   Custom behaviors   ========================== //

  
  void LedsBehavior::customPaint(struct LedProperties* led_properties){
  
    // Disable brightness ans set color
    paintLeds(led_properties->init_led, led_properties->end_led, led_properties->color);
    
    
  }
  
  
  void LedsBehavior::customBlink(struct LedProperties* led_properties){
  
    // Disable brightness ans set color and blik time
    blinkLeds(led_properties->init_led, led_properties->end_led, led_properties->color, led_properties->time);
  }
  
  void LedsBehavior::customShift(struct LedProperties* led_properties){
    
    int start_part[1] = {led_properties->init_led};
    int part_size =  led_properties->end_led - led_properties->init_led;
    int number_parts = 1;
    
    if (led_properties->direction)
      shiftParts(start_part, number_parts, part_size, led_properties->color, led_properties->time, "left");
    else
      shiftParts(start_part, number_parts, part_size, led_properties->color, led_properties->time, "right");
  
  }
  
  
  void LedsBehavior::customTurn(struct LedProperties* led_properties){
  
    int zone = led_properties->end_led - led_properties->init_led;
    int side = floor(led_strip_size/4);
    int start_part;

    if (zone%2 == 0 && side%2 == 0)
       start_part = side/2 - zone/2;
    
    if (zone%2 == 0 && side%2 != 0)
       start_part = ceil(side/2) - zone/2;  

    if (zone%2 != 0 && side%2 == 0)
       start_part = side/2 - ceil(zone/2); 

    if (zone%2 != 0 && side%2 != 0)
       start_part = ceil(side/2) - zone/2;
       
    
    int start_parts[4] = {start_part, start_part+side, start_part+side*2, start_part+side*3};
    int part_size =  zone;
    int number_parts = 4;
    
    if (led_properties->direction)
      shiftParts(start_parts, number_parts, part_size, led_properties->color, led_properties->time, "left");
    else
      shiftParts(start_parts, number_parts, part_size, led_properties->color, led_properties->time, "right");
    
  }


  void LedsBehavior::reset(){
  
    blink_time_now = 0;
    blink_time_last = 0;
    blink_state = LOW;
    
    shift_time_now = 0;
    shift_time_last = 0;
    shift_counter = 0;
  
  }


// ======================= Select Behavior ========================= // 


  void LedsBehavior::setBehavior(struct LedProperties led_properties){

    this->led_properties = led_properties;
      
  }

  void LedsBehavior::setBehavior(String led_properties_msg){

    Serial.println(led_properties_msg);

    String properties[9];
    
    int fromIndex = 0;
    int toIndex = 0;

    for (int i = 0; i<8; i++){
    
      toIndex = led_properties_msg.indexOf(',', fromIndex);
      properties[i] = led_properties_msg.substring(fromIndex, toIndex); 
      Serial.println(properties[i]); 
      fromIndex = toIndex + 1; 
    }
    /*
    this->led_properties.command = Blink;
    this->led_properties.init_led = 0;
    this->led_properties.end_led = 0;
    this->led_properties.color = 0xFF0000;
    this->led_properties.time = 500;
    this->led_properties.direction = 0;
*/

    // Select mode
    if (properties[1] == "paint")
      this->led_properties.command = Paint;
    if (properties[1] == "blink")
      this->led_properties.command = Blink;
    if (properties[1] == "shift")
      this->led_properties.command = Shift;


    // Zone
    this->led_properties.init_led = properties[2].toInt();
    this->led_properties.end_led = properties[3].toInt();

    // Color
    uint32_t color_R = properties[4].toInt();
    uint32_t color_G = properties[5].toInt();
    uint32_t color_B = properties[6].toInt();
    this->led_properties.color = ( (uint32_t(color_R) << 16) | (uint32_t(color_G) << 8) ) | uint32_t(color_B);

    // Time
    this->led_properties.time = properties[7].toInt();

    // Direction
    this->led_properties.direction = properties[8].toInt();
  
  }

  void LedsBehavior::run(){

    if (isNewBehavior(led_properties))
       clearBehavior();
    
    switch(led_properties.command){  
      case Paint: customPaint(&led_properties); break;
      case Blink: customBlink(&led_properties); break;
      case Shift: customShift(&led_properties); break;
      case Turn:  customTurn (&led_properties); break;
      case Booting:  bootingLeds(); break;
      case Ready:  readyLeds(); break;
      case Exit:  exitLeds(); break;
    }

    update(20); // Update led with behavior each 20 ms
  }


  void LedsBehavior::clearBehavior(){

    LedsHardware::clear();
    reset();
  
  }

  bool LedsBehavior::isNewBehavior (struct LedProperties led_properties){
      
    bool isNew = false;

      if (led_properties.command != last_led_properties.command) isNew = true;
      if (led_properties.init_led != last_led_properties.init_led) isNew = true;
      if (led_properties.end_led != last_led_properties.end_led) isNew = true;
      if (led_properties.color != last_led_properties.color) isNew = true;
      if (led_properties.time != last_led_properties.time) isNew = true;
      if (led_properties.direction != last_led_properties.direction) isNew = true;
      
      if (isNew)
         last_led_properties = led_properties;
            
      return isNew;
       
  }
