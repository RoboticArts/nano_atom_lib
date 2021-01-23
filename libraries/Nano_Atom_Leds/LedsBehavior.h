#ifndef LEDS_BEHAVIOR_H
#define LEDS_BEHAVIOR_H

#include <Arduino.h>
#include "utility/LedsHardware.h"


class LedsBehavior : public LedsHardware{
  
  public:

    // Led properties
    struct LedProperties{
    
      uint8_t   command;
      uint8_t   init_led;
      uint8_t   end_led;
      uint32_t  color;
      uint16_t  time;
      uint8_t   direction;
    
    };

    struct LedProperties led_properties;
  
    LedsBehavior(const int &led_strip_size, const int &pin);  
    void begin();
    void run();
    void reset();
    void setBehavior(struct LedProperties led_properties);
    

    enum colors { Red=0x140000,
                  Green=0x001400,
                  Blue=0x000014,
                  White=0x141414, 
                  Clear=0x000000 };

    enum behaviors{
          Paint = 0x08,
          Blink = 0x09,
          Shift = 0x0A,
          Turn = 0x0B,
          Booting = 0xF1,
          Ready = 0xF2,
          Exit = 0xF4
      };

  private:
        
    // Used in blinkLeds
    float blink_time_now = 0;
    float blink_time_last = 0;
    bool  blink_state = LOW;
    
    // Used in shiftParts
    float shift_time_now = 0;
    float shift_time_last = 0;
    int   shift_counter = 0;

    // Used in update
    float show_time_now = 0;
    float show_time_last = 0;

    // Used in setBehavior
    struct LedProperties last_led_properties;

    int led_strip_size = 0;
 
    // Basic behaviors
    void paintLeds(int start_led, int end_led, uint32_t color);
    void blinkLeds(int start_led, int end_led, uint32_t color, int blink_time);
    void shiftParts(int start_parts[], int number_parts, int part_size,  uint32_t color, int shift_time, String direction);

    // Composed behaviors
    void bootingLeds();
    void readyLeds();
    void exitLeds();
               
    // Custom behaviors
    void customPaint(struct LedProperties* led_properties);
    void customBlink(struct LedProperties* led_properties);
    void customShift(struct LedProperties* led_properties);
    void customTurn(struct LedProperties* led_properties);

    // Clear behavior
    void clearBehavior();
    bool isNewBehavior (struct LedProperties led_properties);

    void update(int refresh_time);
  }; 



#endif
