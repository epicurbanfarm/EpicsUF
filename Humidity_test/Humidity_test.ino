#include <Adafruit_AHTX0.h>
#include <Adafruit_NeoPixel.h>

#define NUMPIXELS        1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

Adafruit_AHTX0 aht;
int flag = 0;

void setup() {
  #if defined(NEOPIXEL_POWER)
  
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
  #endif
  if (! aht.begin()) {
    flag = 1;

  }
}

void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  if (flag == 1) {
    pixels.fill(0x00FF00);//FFF000:yellow 000FFF:blue 00FF00:green
    pixels.show();
    delay(500);
  }
  if (flag == 0) {
    pixels.fill(0x000FFF);//FFF000:yellow 000FFF:blue 00FF00:green
    pixels.show();
    delay(500);
  }

  delay(500);
}
