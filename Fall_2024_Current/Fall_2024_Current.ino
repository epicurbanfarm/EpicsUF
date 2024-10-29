/*
 * Spring_2024_Main_code
 *
 * GitHub: https://github.com/epicurbanfarm/EpicsUF
 *
 * Contributer:
 * Jittipat Shobbakbun (Win) [Coder / Spring 2022] - winsatid62@gmail.com (If email me please say it is about EPICS-UF in the subject)
 * Jack Wagner [Network code / Spring 2022]
 * Alex Mate [Physical system and ThingSpeak / Spring 2022]
 * Nathan Joseph [Code and Hardware / Fall 2023-Fall 2024]
 * Franciska Vogel [Code and Hardware / Spring 2024]
 * 
 *
 * Description:
 * This Arduino program is based on the Whitebox T2 Mini MkII (https://www.whiteboxes.ch/docs/tentacle/t2-mkII/#/).
 * The code is originally designed to read data from EZO sensors that is connected to the Whitebox T2 shield.
 * We have added more code for other sensors that we will use (see the list of sensors below). The code is structured
 * so the setup contain the setup for all sensors and the network system. The main loop contain (mostly) non blocking
 * functions that read the data of each sensor and the function that upload those data to ThingSpeak. The code put
 * all the sensor reading in global variables (see the list of reading global variables below). We use ThingSpeak as
 * the cloud that the Arduino upload the data to. The Arduino connect to the internet via ethernet cable connected
 * to the sheild.
 *
 * List of sensors:
 *  1. pH sensor                  - digital - use Ezo_i2c.h library (Fall 2023 drive folder)
 *  2. DO Sensor                  - digital - use Ezo_i2c.h library
 *  3. EC sensor                  - digital - use Ezo_i2c.h library
 *  3. Water temperature sensor   - analog  - It is a thermistor. Measure voltage to calculate resistance and temperature
 *  4. Humidity / Air temp sensor - digital - use DHT.h library
 *
 * List of functions:
 *  1.  void setup()       - default Arduino setup function
 *  2.  void loop()        - default Arduino loop function
 *  3.  void blink_led()   - blink LED every BLINK_FREQUENCY milliseconds
 *  4.  void read_ezo()    - read from EZO sensors
 *  5.  float receive_reading(Ezo_board &Sensor) - Take EZO sensor object, read the data and output it as float
 *  6.  void read_analog_temp()      - Take analog pin number and read temperature probe connected to that pin
 *  7.  float old_temperature_code(float Vo)     - Take in voltage reading from temperature sensor and calculate temperature
 *  8.  void hum_read()         - Read humidity and air temperature from humidity sensor
 *  9.  void update_display()   - Output readings to serial port
 *  10. void upload_cloud()     - Upload to ThingSpeak every UPLOAD_FREQUENCY
 *  11. pump_write()            -

 *
 * List of sensor readings (global variables):
 *  1. float pH_reading  - reading from pH sensor (desired range: 6.5 and 7.0)
 *  2. float _DO_reading - reading from dissolved oxygen sensor (desired range: 5.0 mg/L)
 * 
 *  3. float water_Tc    - Water temperature in Celsius of the temperature probe
 *  4. float water_Tf    - Water temperature in Fahrenheit of the temperature probe (desired range: 65 and 75 degrees)
 *  5. float humidity    - Humidity in the air in percent (Can read from 0-100)
 *  6. float air_Tc      - Air temperature in Celsius from humidity sensor
 *  7. float air_Tf      - Air temperature in Fahrenheit from humidity sensor
 *
 * --------------------------------------------------------------------------- *
 *
 *  GOALS:
 *  - Implement function to check for faulty connection
 *  - Detect faulty connection or sensors
 *      - Maybe EZO have something about checking if there is a conencted sensor
 *      - Maybe check when values are impossibly out of ranges (Absolute zero, for example)
 *  - Upload sensor range alerts to thingspeak
 *  - Power saving mode. Read sensors further aparts and only read every 15 seconds (fastest you can upload) when it detect some error. Maybe implement a sleep to arduino and only wake up every half an hour or some amount of time
 *
 *  STRETCH GOALS:
 *  - Automatically change pH by pumping safe acidic or basic solutions
 *  - Activate air pump when having insufficient dissolved oxygen
 *  - Control HVAC system automatically using temperature and
 *  - Maybe water heater too?
 */

#include <Ezo_i2c.h>                        // include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>                           // enable I2C
#include "ThingSpeak.h"                     // always include thingspeak header file after other header files and custom macros
#include <WiFi.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_NeoPixel.h>

//--Status LED--
#define BLINK_FREQUENCY 250                 // the frequency of the led blinking, in milliseconds
unsigned long next_blink_time;              // holds the next time the led should change state
boolean led_state = LOW;                    // keeps track of the current led state
#define NUMPIXELS        1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

//--Network--
const char* ssid = "WAVLINK-N";
const char* password = "epics@urbanfarm2018";
int flag = 0;
int count = 0;

WiFiClient client;

//--ThingSpeak--
unsigned long myChannelNumber = 2701552;
const char * myWriteAPIKey = "SHGQLQJJJX4H1ALB";
// timer
#define UPLOAD_FREQUENCY 20000              // frequency of uploading to ThingSpeak in milliseconds (ThingSpeak only allow upload every 15 seconds (15000 millis))
unsigned long next_upload = 0;              // time to upload the next set of data
// Code for calculating time interval between each updates
unsigned long last_upload = 0;              // Time of the last update


//--Serial display--
// setups
#define DISPLAY_INDIVIDUAL false            // true to display individual readings as soon as they come in, false to display only aggregated one
#define DISPLAY_DEBUG true                  // true to display debug massages, false to not display them
// timer
#define DISPLAY_FREQUENCY 1000              // frequency to print data of all sensors to Serial
unsigned long next_display = 0;             // variable keeping track of when to display the data next


//--EZO Sensors (pH and Disolved oxygen)--
// setups
Ezo_board ph = Ezo_board(99, "PH");         //create a PH circuit object who's address is 99 and name is "PH"
//Ezo_board _do = Ezo_board(97, "DO");        //create a DO circuit object who's address is 97 and name is "DO"
Ezo_board ec = Ezo_board(100, "EC");        //create an EC circuit object who's address is 100 and name is "EC"
// globals
float pH_reading;                           // global to hold pH
float _DO_reading;                          // global to hold DO
float ec_reading;                           // global to hold EC
// timer
#define EZO_FREQUENCY 1000                  // frequency of checking EZO sensor (EZO sensors need around 1000 milliseconds to process the data, so this should be more than that)
unsigned long next_ezo_time = 0;            // holds the time when the next EZO reading is due
// misc.
boolean request_pending = false;            // wether we've requested a reading from the EZO devices


//--Water temperature--
// setups
#define TEMP_PIN 0                                  // the ANALOG pin of the analog port of the temperature probe
const long RESISTOR_RESISTANCE = 9.78 * 1000;       // the resistance of the resistor connected serially with the temperature sensor
const float ARDUINO_VOLTAGE = 4.74;                 // the measured voltage of the arduino five volt
// globals
float water_Tc;                             // global to hold water temperature in Celsius
float water_Tf;                             // global to hold water temperature in Fahrenheit
float probe_voltage;                        // voltage reading from the analog pin
float probe_resistance;                     // resistance reading from the analog pin
// timer
#define TEMP_CHECK_FREQUENCY 500            // the amount of time between each temperature read, in milliseconds (No minimum)
unsigned long next_temp_check_time;         // holds the next time the program read the temperature
// misc.
const float C1 = 1.009249522e-03, C2 = 2.378405444e-04, C3 = 2.019202697e-07;       // constants for temperature conversion


//--Humidity and air temperature--
// setups
Adafruit_AHTX0 aht;
// globals
float humidity;                             // global to hold humidity
float air_Tc;                               // global to hold air temperature in Celsius
float air_Tf;                               // global to hold air temperature in Fahrenheit
// timer
#define HUMIDITY_FREQUENCY 1000             // frequency to check the humidity (It need around 250 milliseconds to process between each reading)
unsigned long next_hum_time = 0;            // next time to check humidity

//--Ideal sensor ranges-- 
//(values subject to change)
float pH_min = 6.5;
float pH_max = 7.0;
float DO_min = 5.0;
float EC_min = 0;
float EC_max = 10;
float water_Tf_min = 65;
float water_Tf_max = 75;
float humidity_min = 50;
float humidity_max = 70;
float air_Tf_min = 60;
float air_Tf_max = 80;


//--Pump--
// setups
#define PUMP_PIN 7                         // what pin we're connected to (Digital)
// globals
unsigned long pumpFreq = 717UL*60UL*1000UL;      // global to hold frequency of pump activation
unsigned long pumpDur = 3UL*60UL*1000UL;         // global to hold duration of pump activation
bool pumpOn = true;                       // global to hold pump state
unsigned long pump_start_time = 0;         // time when pump is turned on
unsigned long pump_end_time = 0;           // time when pump 

void check_WiFi() {
  pixels.fill(0x5DFC0A);//LED green
  pixels.show();
  
  count = 0;
  while(WiFi.status() != WL_CONNECTED){ //if it is still not connected to wifi, try again
    delay(500);
    count++;
    if (WiFi.status() == WL_NO_SSID_AVAIL){
      pixels.fill(0xFF0000);
      pixels.show();
      delay(500);
      break;
    }
    else if (count >= 10) {
      pixels.fill(0xFFF000);
      pixels.show();
      delay(500);
      break;
    }
    if(WiFi.status() == WL_CONNECTED) {
      pixels.fill(0x00FF00);
      pixels.show();
      delay(500);
      break;
    }
  }
  pixels.fill(0x000000);
  pixels.show();
  delay(500);
}

void setup() {
  
  #if defined(NEOPIXEL_POWER)
  // If this board has a power control pin, we must set it to output and high
  // in order to enable the NeoPixels. We put this in an #if defined so it can
  // be reused for other boards without compilation errors
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
  #endif
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(20); // not so bright

  
  pinMode(13, OUTPUT);                      // set the led output pin
  Wire.begin();                             // enable I2C port.
  pinMode(PUMP_PIN,OUTPUT);                        // set the pump signal output pin
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  aht.begin();
  // ThingSpeak
  ThingSpeak.begin(client);  // Initialize ThingSpeak

  // Set the start read times. The code will check after waiting for sensor to start up and measure the data
  next_ezo_time = millis() + EZO_FREQUENCY;
  next_temp_check_time = millis() + TEMP_CHECK_FREQUENCY;
  next_hum_time = millis() + HUMIDITY_FREQUENCY;
  last_upload = millis();
  next_upload = millis() + UPLOAD_FREQUENCY;
  next_display = millis();                                      // display immediately
  #if defined(NEOPIXEL_POWER)
  
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
  #endif

  digitalWrite(PUMP_PIN, HIGH);
}


void loop() {
  // none of these functions block or delay the execution
  
  check_WiFi();
  
  // read sensors
    read_ezo();
    read_analog_temp();
    
    hum_read();
   

    // blink status update
    blink_led();
    
    
    // Upload to cloud

    upload_cloud();
    // Write to pump
    pump_write();
  
}

// blinks a led on pin 13. this function returns immediately, if it is not yet time to blink
void blink_led() {
  if (millis() >= next_blink_time) {              // is it time for the blink already?
    led_state = !led_state;                       // toggle led state on/off
    digitalWrite(13, led_state);                  // write the led state to the led pin
    next_blink_time = millis() + BLINK_FREQUENCY; // calculate the next time a blink is due
  }
}


// take sensor readings every EZO_FREQUENCY. this function returns immediately, if it is not time to interact with the EZO devices.
void read_ezo() {
  if (request_pending) {                    // is a request pending?
    if (millis() >= next_ezo_time) {        // is it time for the reading to be taken?
      // Get the outputs and print the output in serial
      pH_reading = receive_reading(ph) - 0.55;             // get the reading from the PH circuit
      if (DISPLAY_INDIVIDUAL) {
        Serial.print("  ");
      }
      /*_DO_reading = receive_reading(_do) - 0.05;                 // get the reading from the DO circuit
      if (DISPLAY_INDIVIDUAL) {
        Serial.println();
      }*/
      ec_reading = receive_reading(ec);
      if (DISPLAY_INDIVIDUAL) {
        Serial.println();
      }

      request_pending = false;
      next_ezo_time = millis() + EZO_FREQUENCY;
    }
  } else {                                  // it's time to request a new reading
    ph.send_read_cmd();
    //_do.send_read_cmd();
    ec.send_read_cmd();
    request_pending = true;
  }
}


// function to decode the reading after the read command was issued
float receive_reading(Ezo_board &Sensor) {

  if (DISPLAY_INDIVIDUAL) {                               // Check if we want to display individual data
    Serial.print(Sensor.get_name()); Serial.print(": ");  // print the name of the circuit getting the reading
  }
  Sensor.receive_read_cmd();                              // get the response data

  if (DISPLAY_INDIVIDUAL) {
    switch (Sensor.get_error()) {                         // switch case based on what the response code is.
      case Ezo_board::SUCCESS:
        Serial.print(Sensor.get_last_received_reading()); //the command was successful, print the reading
        break;

      case Ezo_board::FAIL:
        Serial.print("Failed ");                          //means the command has failed.
        break;

      case Ezo_board::NOT_READY:
        Serial.print("Pending ");                         //the command has not yet been finished calculating.
        break;

      case Ezo_board::NO_DATA:
        Serial.print("No Data ");                         //the sensor has no data to send.
        break;
    }
  }

  return Sensor.get_last_received_reading();              //return the sensor reading
}


// function to read the temperature from temperature probe. Return if it is not the time to do it yet.
void read_analog_temp() {
  if (millis() >= next_temp_check_time) {                         // is it the time to check temperature
    float temp_voltage;
    temp_voltage = analogRead(TEMP_PIN);                          // read voltage from analog pin
    probe_voltage = (temp_voltage/1023.0)*ARDUINO_VOLTAGE;        // set global variable for the probe sensor
    
    if (DISPLAY_INDIVIDUAL) {
      Serial.print("Temperature sensor: ");                       // print label for temperature reading to serial port
    }
    float Tc, Tf;                                                 // variables declaration for the temperature calculation

    Tc = old_temperature_code(temp_voltage);                      // run old code
    Tf = (Tc * 9.0)/ 5.0 + 32.0;
    if (DISPLAY_DEBUG && DISPLAY_INDIVIDUAL) {
      Serial.print("Temp sensor voltage: ");
      Serial.println(probe_voltage);                              // print actual voltage of the sensor
    }
    water_Tc = Tc;                                                // set global for water temperature in Celsius
    water_Tf = Tf;                                                // set global for water temperature in Fahrenheit
    next_temp_check_time = millis() + TEMP_CHECK_FREQUENCY;       // set the next time to read the temperature probe
  }
}


// function to calculate temperature, Taken from an old UF team Arduino code (Fall 2020)
float old_temperature_code(float Vo) {
  float logR2, R2, T, Tc, Tf;                               // variables declaration for the temperature calculation
  // Calculate the temperature from the voltage
  R2 = RESISTOR_RESISTANCE * ((1023.0 / (float)Vo) - 1.0);
  logR2 = log(R2);
  T = (1.0 / (C1 + C2*logR2 + C3*logR2*logR2*logR2));       // Used linear regression to figure this out from testing
  // Convert them to Celsius and Fahrenheit
  Tc = T - 273.15;
  Tf = (Tc * 9.0)/ 5.0 + 32.0;

  // Display the data
  if (DISPLAY_INDIVIDUAL) {
    Serial.print("Temperature: ");
    Serial.print(Tf);
    Serial.print(" F; ");
    Serial.print(Tc);
    Serial.println(" C");
  }

  return Tc;
}


// Function for reading humidity sensor.
void hum_read() {
  // adapted from the code from
  if (millis() >= next_hum_time) {
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    //float h = hum_sen.readHumidity();
    sensors_event_t h, temp;

    aht.getEvent(&h, &temp);

    // Read temperature as Celsius
    //float Tc = hum_sen.readTemperature();
    float Tf = (temp.temperature * 9.0)/ 5.0 + 32.0;
    /*
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(Tc)) {
      if (DISPLAY_INDIVIDUAL) {
        Serial.println("Failed to read from DHT sensor!");
      }
    }
    else {*/
      air_Tc = temp.temperature;                        // set global air temperature in Celsius
      air_Tf = Tf;                        // set global air temperature in Fahrenheit
      humidity = h.relative_humidity;                       // set global humidity
      
    next_hum_time = millis() + HUMIDITY_FREQUENCY;        // set the next read time
  }
}


// update the Serial output
void update_display() {
  if (millis() >= next_display) {                       // check if it is time to display data
    next_display = millis() + DISPLAY_FREQUENCY;        // if it is, reset the clock to the next time
    Serial.println();
    Serial.println("Readings:");                        // Print title of the display readings
    // pH
    Serial.print("pH: ");
    Serial.print(pH_reading);
    if (pH_reading <= pH_min){                          // check if pH is within desired values
      Serial.print("   (BELOW OPTIMAL PH)");
    }
    else if (pH_reading >= pH_max){
      Serial.print("   (ABOVE OPTIMAL PH)");
    } 
    Serial.println();
    // DO
    Serial.print("DO: ");
    Serial.print(_DO_reading);
    /*if (_DO_reading <= DO_min){                         // check if DO is within desired values
      Serial.print("   (BELOW OPTIMAL DO)");
    }*/
    Serial.println();
    // EC
    Serial.print("Ec: ");
    Serial.print(ec_reading);
    if (water_Tf <= water_Tf_min){                      // check if water temperature is within desired values
      Serial.print("  (BELOW OPTIMAL EC)");
    }
    Serial.println();
    // Water temperature
    Serial.print("Water temperature: ");
    Serial.print(water_Tf);
    Serial.print(" F");
    if (water_Tf <= water_Tf_min){                      // check if water temperature is within desired values
      Serial.print("   (BELOW OPTIMAL WATER TEMP)");
    }
    else if (water_Tf >= water_Tf_max){
      Serial.print("   (ABOVE OPTIMAL WATER TEMP)");
    }
    Serial.println();
    // display debug information
    if (DISPLAY_DEBUG) {
      // Temperature probe voltage
      Serial.print("Probe voltage: ");
      Serial.print(probe_voltage);
      Serial.println(" V");
    }
    // Air temperature
    Serial.print("Air temperature: ");
    Serial.print(air_Tf);
    Serial.print(" F");
    if (air_Tf <= air_Tf_min){                        // check if air temperature is within desired values
      Serial.print("   (BELOW OPTIMAL AIR TEMP)");
    }
    else if (air_Tf >= air_Tf_max){
      Serial.print("   (ABOVE OPTIMAL AIR TEMP)");
    }
    Serial.println();
    // Humidity
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %");
    if (humidity <= humidity_min){                   // check if humidity is within desired values
      Serial.print("   (BELOW OPTIMAL HUMIDITY)");
    }
    else if (humidity >= humidity_max){
      Serial.print("   (ABOVE OPTIMAL HUMIDITY)");
    }
    Serial.println();
  }
}


// upload data to cloud
void upload_cloud() {
  
  
  if (millis() >= next_upload) {
    for (int i = 0; i < 4; i++) {
      pixels.fill(0x000FFF);//blue
      pixels.show();
      delay(100); // wait half a second
      pixels.fill(0x000000);
      pixels.show();
      delay(100); // wait half a second
    }
    next_upload = millis() + UPLOAD_FREQUENCY;
    if (DISPLAY_DEBUG) {
      Serial.println("Uploading data...");
    }
    // Set data into ThingSpeak fields
    ThingSpeak.setField(1, pH_reading);
    //ThingSpeak.setField(2, _DO_reading);
    ThingSpeak.setField(3, water_Tf);
    ThingSpeak.setField(5, humidity);
    ThingSpeak.setField(6, air_Tf);
    ThingSpeak.setField(4, ec_reading);

    unsigned long time_interval = 0;                                    // variable for keeping track of how long the last successful update was
    // write fields to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (DISPLAY_DEBUG) {
      if(x == 200) {
        Serial.println("Channel update successful.");
        // calculate the time since last upload
        time_interval = millis() - last_upload;
        last_upload = millis();
        // display the time since last upload
        if (DISPLAY_DEBUG) {
          Serial.print("Last successful upload was ");
          Serial.print(time_interval / 1000.0);
          Serial.println("seconds ago.");
        }
      }
      else {
        Serial.println("Problem updating channel. HTTP error code " + String(x));
      }
    }
  }
}

// write to pump
void pump_write() 
{
  // if pump is off and it has been off for at least pumpFreq amount of time
  if (!pumpOn && millis() >= pump_end_time + pumpFreq)
  {
    pump_start_time = millis();
    pumpOn = true;
    // turn on digital signal to relay
    digitalWrite(PUMP_PIN,HIGH);
    Serial.println("PUMP ON");
  }
  // if pump is on and it has been running longer than pumpDur
  else if (pumpOn && millis() >= pump_start_time + pumpDur)
  {
    pump_end_time = millis();
    pumpOn = false;
    // turn off digital signal to relay
    digitalWrite(PUMP_PIN,LOW);
    Serial.println("PUMP OFF");
  }
}
