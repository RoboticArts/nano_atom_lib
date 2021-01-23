#include <LedsBehavior.h>

#define LED_SIZE 3
#define LED_PIN PA7

LedsBehavior leds_behavior(LED_SIZE, LED_PIN);
LedsBehavior::LedProperties led_properties;   

void setup() {

  leds_behavior.begin();
  
  led_properties.command = LedsBehavior::Blink;
  led_properties.init_led = 0;
  led_properties.end_led = 1;
  led_properties.color = LedsBehavior::Blue;
  led_properties.time = 500;

  leds_behavior.setBehavior(led_properties); 
}

void loop() {
   
   leds_behavior.run();

}
