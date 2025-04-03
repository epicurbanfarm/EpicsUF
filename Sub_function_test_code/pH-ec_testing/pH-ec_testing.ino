#include <Ezo_i2c.h>
#include <Wire.h> 

Ezo_board ph = Ezo_board(99, "PH"); 
Ezo_board ec = Ezo_board(100, "EC");
float pH_reading;
float ec_reading; 
#define EZO_FREQUENCY 1000
unsigned long next_upload = 0;  
unsigned long last_upload = 0;
unsigned long next_ezo_time = 0;
boolean request_pending = 0;
float pH_min = 6.5;
float pH_max = 7.0;
float EC_min = 0;
float EC_max = 10;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(); 
  next_ezo_time = millis() + EZO_FREQUENCY;
  last_upload = millis();
  next_upload = millis();
  ph.send_read_cmd();
  ec.send_read_cmd();
}

void loop() {
  // put your main code here, to run repeatedly:

  if (request_pending) {                    // is a request pending?
    if (millis() >= next_ezo_time) {        // is it time for the reading to be taken?
      // Get the outputs and print the output in serial
      pH_reading = receive_reading(ph) - 0.55;             // get the reading from the PH circuit
      Serial.begin(9600);
      Serial.print("pH: ");
      Serial.println(pH_reading);
      ec_reading = receive_reading(ec);
      Serial.print("ec: ");
      Serial.println(ec_reading);
      request_pending = false;
      next_ezo_time = millis() + EZO_FREQUENCY;
    }
  } else {                                  // it's time to request a new reading
    //ph.send_read_cmd();
    //_do.send_read_cmd();
    //ec.send_read_cmd();
    request_pending = true;
  }
}

float receive_reading(Ezo_board &Sensor) {
  Sensor.receive_read_cmd();                              // get the response data
  Serial.begin(9600);
  Serial.println(Sensor.get_last_received_reading());
  return Sensor.get_last_received_reading();              //return the sensor reading
}
