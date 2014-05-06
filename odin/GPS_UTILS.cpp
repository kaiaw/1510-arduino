#include "GPS_UTILS.h"



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

/**
   Calculates the distance between two coordinates given in decimal degrees.

   Returns the Haversine distance (double).
   This is the distance which takes the curvation of the
   earth in concideration when calculating (not hills and mountains).

   latCameraDeg : Camera latitude (decimal degrees)
   longCameraDeg: Camera longitude (decimal degrees)

   latPersonDeg : Person latitude (decimal degrees)
   longPersonDeg: Person longitude (decimal degrees)
**/
double getDistance(double latCameraDeg, double longCameraDeg,
		   double latPersonDeg,double longPersonDeg)
{

  double latCam = degToRad(latCameraDeg);
  double latPer = degToRad(latPersonDeg);

  double deltaLat = latPerson - latCam;
  double deltaLong = degToRad(longPersonDeg) - degToRad(longCameraDeg);

  double a = sin(deltaLat/2)*sin(deltaLat/2) +
    cos(latCam) * cos (latPer) *
    sin(deltaLong/2) * sin(deltaLong/2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

}

double degToRad(double deg)
{
  return deg*M_PI/180:
}

int compassDirection(char direction)
{  
  if(direction = 'S' || direction = 'W') return -1;
  
  return 1;
}
