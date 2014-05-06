#include <GPS.h>
#include <SoftwareSerial.h>
#include "GPS_READER.h"
#include "gpsUtils.h"
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

#include "math.h"

/**
   Calculates camera yaw angle, (or z-axis angle). 

   LatCamera:   Camera Latitude
   LongCamera:  Camera Longitude
   LatPerson:   Person Latitude
   LongPerson:  Person Longitude
 **/
double getDesiredYawAngle(double latCamera, double longCamera, double latPerson,
                                 double longPerson) {
  // Calculate distances fron camera to person in X and Y direction. 
  double dX = longPerson - longCamera;
  double dY = latPerson - latCamera;
  
  // Returns direction in degrees. 
  return atan2(dY, dX) * 180 / M_PI;
}


/**
   Calculates camera pitch angle, (or z-axis angle). 
   
   altCamera:   Camera altitude
   altPerson:   Person altitude
   LatCamera:   Camera latitude
   LongCamera:  Camera longitude
   LatPerson:   Person latitude
   LongPerson:  Person longitude
 **/
double getDiseredPitchAngle(double altCamera, double altPerson, double latCamera, double longCamera, double latPerson, 
                                 double longPerson)
{
  double dX = longPerson - longCamera;
  double dY = latPerson - latCamera;
  
  double distanceXY = sqrt(dX*dX + dY*dY);
  double distanceZ = altPerson - altCamera;
  
  double pitchAngle = atan2(distanceZ, distanceXY) * 180 / M_PI;
  return pitchAngle;
}


void setup() {
  setupGPS();
  
}

uint32_t timer = millis();

// MAIN LOOP.
void loop() {
  
    double bislettBabbLat = 59.920761;
  double bislettBabbLong = 10.733468;
  
  double guttaLat = 59.924514;
  double guttaLong = 10.739519;
  // read the input on analog pin 0:
  double degrees = getDesiredCameraDirection(bislettBabbLat, bislettBabbLong, guttaLat, guttaLong);
  // print out the value you read:
  Serial.println(degrees);
  delay(100);        // delay in between reads for stability


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
