#include "ThingSpeak.h"                     // always include thingspeak header file after other header files and custom macros
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

#define NUMPIXELS        1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

char* ssid = "WAVLINK-N"; // the name of the Wi-Fi, using my hotsot at school
char* password = "epics@urbanfarm2018"; // the Wi-Fi password
//char* username = "athajeb";
int flag = 0;
int count = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  WiFi.mode(WIFI_STA); //set the connection mode
  WiFi.begin(ssid, password);


  while(WiFi.status() != WL_CONNECTED){ //if it is still not connected to wifi, try again
    delay(500);
    count++;
    if(count >= 10) {
      if (WiFi.status() == WL_NO_SSID_AVAIL){
        flag = 1;
        break;
      }
      else if (count >= 100) {
        flag = 0;
        break;
      }
  } 
  if(WiFi.status() == WL_CONNECTED) flag = 2;
  }
}
//FFF000:yellow
//000FFF:blue
//00FF00:green
//FF0000:red
void loop() {
  if(flag == 2){//connected:yellow
    pixels.fill(0xFFF000);
    pixels.show();
    delay(50); // wait half a second
  }
  if(flag == 1){
    pixels.fill(0x00FF00);//green: no ssid
    pixels.show();
    delay(100); // wait half a second
  }
  // turn off
  if(flag == 0) {//blue: failed 10 times
    pixels.fill(0xFFF000);
    pixels.show();
    delay(100); // wait half a second
  }
    
  
  
}