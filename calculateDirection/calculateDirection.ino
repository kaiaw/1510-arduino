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
/*
    Test program for testing the angle functions from test-gps data. 
 */

void setup() {
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
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
}
