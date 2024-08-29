#include <stdio.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp32_i2c_rw.h"
#include "esp32-hal-i2c.h"
#include "i2c.h"
#include "WiFi.h"
#include "Client.h"
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "WiFiGeneric.h"
#include "AD7746.h"
#include "I2Cdev.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

int AD7156_addr = 0x48;  // address of device during write cycle
       
#define I2C_TX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */

#define I2C_NUM (I2C_NUM_0)

//////////////////*I2C setup*//////////////////////

void I2C_address_detect() //Detect the slave address when slave chip is connected.//Copied from github. No modifications needed. If you want to use it, just put this in the main loop
{
  int32_t error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("done\n");
  }
  delay(5000);
}

AD7746 ADC;

void setup()
{
  
}

bool esp32_i2c_write_add
(
	uint8_t device_address,
	uint8_t size,
	uint8_t* data
)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, device_address, 1);
	i2c_master_write(cmd, data, size - 1, 0);
	i2c_master_write_byte(cmd, data [size - 1], 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return (true);
}//Multi-device i2c communication with same slave address


void ADC_ini()
{
  uint8_t Rest[1]={0xBF};
  esp32_i2c_write_add(AD7746_DEFAULT_ADDRESS,1,Rest); //reset
  delay(1);
  
  uint8_t CAP_SET[1]={0X80};
  uint8_t KKK[1]={0X02};
  uint8_t EXC_SET[1]={0X0B}; 
  uint8_t CON_SET[1]={0x39};
  uint8_t Gain_L_SET[1]={0xE5};
  uint8_t Gain_H_SET[1]={0x00}; //According to the manual of the slave chip, initialize the registers.

  esp32_i2c_write_byte(AD7746_DEFAULT_ADDRESS, AD7746_RA_CAP_SETUP,8, CAP_SET);
  esp32_i2c_write_byte(AD7746_DEFAULT_ADDRESS, AD7746_RA_EXC_SETUP,8, EXC_SET);
  esp32_i2c_write_byte(AD7746_DEFAULT_ADDRESS, AD7746_RA_CAP_GAIN_H,8, Gain_H_SET);
  esp32_i2c_write_byte(AD7746_DEFAULT_ADDRESS, AD7746_RA_CAP_GAIN_L,8, Gain_L_SET);
  esp32_i2c_write_byte(AD7746_DEFAULT_ADDRESS, AD7746_RA_CONFIGURATION,8, CON_SET);
}

static void i2c_master_init()
{
    int i2c_master_port = 0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_14;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_15;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000; //400k
    i2c_param_config(i2c_master_port, &conf);

    i2c_driver_install(i2c_master_port, conf.mode,
    		I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
}

static void i2c_master_init2()
{
    int i2c_master_port = 0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_17;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_18;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000; //400k
    i2c_param_config(i2c_master_port, &conf);

    i2c_driver_install(i2c_master_port, conf.mode,
    		I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
}//Enable I2C communication pins/speed/mode on the ESP32


//////////////////*TCP setup*//////////////////////

char server[]="192.168.137.1";//IP of the TCP server
String serverName ="192.168.137.1";// IP of the TCP server

WiFiClientSecure client;
HTTPClient http;

const char* ssid = "PAL3.0";
const char* password = "*******";//Your purdeu account password.
const char* wpa2_identity="";
const char* wpa2_username="*******";//Your purdeu account username.
const char* ca_pem="";
const char* client_crt="";
const char* client_key="";

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin("test", "test1234");  //Hotspot on your PC or phone. test is the ssid, test1234 is the password
  //WiFi.begin(ssid,WPA2_AUTH_PEAP,wpa2_identity,wpa2_username,password);  //PAL3.0
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void TCP_send(unsigned int cap,int num,int cls)
{
    String S;
    if(num==0 && cls == 0)
    {
      S="\r\ncls\r\n";
    }
    if(num==1)
    {
      S=String(cap)+"cap1end"; //S= Data and string that you want to transfer to the TCP server. numbers needs to be converted to string format.
    }
    if(num==2)
    {
      S=String(cap)+"\r\n";
    }
      if(num==3)
    {
      S="bat="+String(cap)+"\r\n"; //This program is designed to control two ADCs that have the same slave address. The format of the data transfer should be modified according to the requirements
    }
    if ((millis() - 0) > 10) {
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      WiFiClient client;
      HTTPClient http;
      uint8_t * write=(uint8_t *) S.c_str();
      size_t size=S.length();
      // Your Domain name with URL path or IP address with path
      client.connect(server, 8043); //Port of the connected IP address
      http.begin(client, serverName);
      int httpResponseCode =0;// http.POST(S);
      
      client.write(&write[0], size);
      
      http.end();
      // If you need an HTTP request with a content type: application/json, use the following:
      //http.addHeader("Content-Type", "application/json");
      //int httpResponseCode = http.POST("{\"api_key\":\"tPmAT5Ab3j7F9\",\"sensor\":\"BME280\",\"value1\":\"24.25\",\"value2\":\"49.54\",\"value3\":\"1005.14\"}");

      // If you need an HTTP request with a content type: text/plain
      //http.addHeader("Content-Type", "text/plain");
      //int httpResponseCode = http.POST("Hello, World!");
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      Serial.println(S);
      // Free resources
    }
      else {
      Serial.println("WiFi Disconnected");
    }
    }
}

void loop() ///main loop
{
  
  initWiFi();
  uint32_t cap=0;
  uint32_t cap1=0;
  uint32_t cap2=0;
  uint32_t cap_temp=0;
  uint32_t cap_cal=0;
  uint8_t cap_L[1]={0};
  uint8_t cap_m[1]={0};
  uint8_t cap_h[1]={0};
  uint8_t cap_L_cal[1]={0};
  uint8_t cap_H_cal[1]={0};
  uint8_t reset[1]={0X1D};
  int ini1=0;
  int ini2=0;
  TCP_send(cap,0,bootCount);


  while(1)
  {

    Serial.println("start:");

    i2c_master_init2();
    ADC_ini();

    for(int i=0;i<20;i++)
    {
      esp32_i2c_read_byte(AD7746_DEFAULT_ADDRESS, 0x01, cap_h);
      esp32_i2c_read_byte(AD7746_DEFAULT_ADDRESS, 0x02, cap_m);
      esp32_i2c_read_byte(AD7746_DEFAULT_ADDRESS, 0x03, cap_L); //Read data from slave chips' registers

      cap_temp=((uint32_t)cap_h[0] << 16) | ((uint32_t)cap_m[0] << 8) | (uint32_t)cap_L[0];
      cap=cap+cap_temp/20;
    }
    TCP_send(cap,1,bootCount); //bootcount is for clearing the previous plotting data on the receiving software and is not required
    cap=0;
    cap_temp=0;
    cap_L[0]={0};
    cap_m[0]={0};
    cap_h[0]={0};
    delay(1000); //Wrong data may occur if the delay between slave chips with the same address is too short.


    i2c_master_init();
    ADC_ini();
    for(int i=0;i<20;i++)
    {
      esp32_i2c_read_byte(AD7746_DEFAULT_ADDRESS, 0x01, cap_h);
      esp32_i2c_read_byte(AD7746_DEFAULT_ADDRESS, 0x02, cap_m);
      esp32_i2c_read_byte(AD7746_DEFAULT_ADDRESS, 0x03, cap_L);
      cap_temp=((uint32_t)cap_h[0] << 16) | ((uint32_t)cap_m[0] << 8) | (uint32_t)cap_L[0];
      cap=cap+cap_temp/20;
    }
    TCP_send(cap,2,bootCount);
    cap=0;
    cap_temp=0;
    cap_L[0]={0};
    cap_m[0]={0};
    cap_h[0]={0};

    /////////sleep////////In order to reduce the power consumption of the system, ESP32 sleeps and wakes up at the set time.
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); 
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
    Serial.println("Going to sleep now");
    delay(100);
    Serial.flush(); 
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }   
}
