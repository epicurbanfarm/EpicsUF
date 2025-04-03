#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_NeoPixel.h>


#define NUMPIXELS        1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
OneWire oneWire(13);
DallasTemperature sensor(&oneWire);

#define UPLOAD_FREQUENCY 1000
unsigned long next_upload = 0;
unsigned long last_upload = 0;
void setup() {
  // put your setup code here, to run once:
  sensor.begin();

}

void loop() {
  sensor.requestTemperatures();
  float temperatureC = sensor.getTempCByIndex(0);

  if (temperatureC == DEVICE_DISCONNECTED_C) {
    pixels.fill(0x0000FF);
    pixels.show();
    pixels.fill(0x000000);
    pixels.show();
    delay(500);
  }
  else if (temperatureC < 20) {
    pixels.fill(0xFFF000);
    pixels.show();
    delay(500);
  }
  else {
    pixels.fill(0x0000FF);
    pixels.show();
    delay(500);
  }
  
  // put your main code here, to run repeatedly:
  Serial.begin(9600);
  Serial.print("Temperature: ");
  Serial.println(temperatureC);
}
