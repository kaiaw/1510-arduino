#include "getDesiredAngle.h"

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
