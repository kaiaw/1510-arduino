#include <GPS.h>
#include <SoftwareSerial.h>
#include "GPS_READER.h"

//xbee uses serial1

#define GPS_IN   8   // digital 8
#define GPS_OUT  9   // digital 9

#define IMU_SCL  1   // analog  1
#define IMU_SDA  0   // analog  0

#define Servo_1  10  // digital 10
#define Servo_2  11  // digital 11
#define Servo_3  12  // digital 12

#define pushbtn  7   // digital 7
#define led      6




void setup() {
  setupGPS();
  
}

uint32_t timer = millis();

// MAIN LOOP.
void loop() {

  getGPSdata();


    // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    
   
     //printGPSdata(GPS_CAM); 
     //printGPSdata(GPS_PERSON); 

   
  }
}
