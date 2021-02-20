#ifndef LEDS_INTERFACE_H
#define LEDS_INTERFACE_H


#include <Arduino.h>
#include "WS2812B.h"


class LedsHardware{ 

  public:
    LedsHardware(int led_strip_size, int pin);
    void begin();
    void update();
    void clear();
    void fillLeds(int start_led, int end_led, uint32_t color);
    
  private:

    WS2812B* leds;
    
    uint32_t led_buffer[100] = {};

    int led_strip_size;
};



#endif
