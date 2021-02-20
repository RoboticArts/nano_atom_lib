


  #include "LedsHardware.h"

  
  LedsHardware::LedsHardware(int led_strip_size, int pin){

      this->leds = new WS2812B (led_strip_size);
      this->led_strip_size = led_strip_size;

  }


  void LedsHardware::begin(){
    
    leds->begin();
    leds->clear();
    leds->show();
  }

  void LedsHardware::update(){
    
     for(int led = 0; led < led_strip_size; led++){

          uint32_t color = this->led_buffer[led];
          leds->setPixelColor(led, color);
     }
     
     leds->show();
    
  }

  void LedsHardware::clear(){

    for (int led = 0; led < led_strip_size; led++)
        this->led_buffer[led] = 0;

  }


  void LedsHardware::fillLeds(int start_led, int end_led, uint32_t color){
  
    if (start_led > end_led){
  
      for(int led=start_led; led < led_strip_size; led++){
        
          this->led_buffer[led] = color;
      
      }
      
      start_led = 0; 
    }
  
    for(int led=start_led; led <= end_led; led++){
        
        if (led < led_strip_size)
        
           this->led_buffer[led] = color;  
        
    }
 
  }
