#ifndef __GPS_UTILS__
#define __GPS_UTILS__

#include "math.h"

#define EARTH_RADIUS 6378137

double getDesiredYawAngle(double latCamera, double longCamera, double latPerson, double longPerson);
double getDesiredPitchAngle(double altCamera, double altPerson, double latCamera, double longCamera, double latPerson, double longPerson);
double getDistance(double latCameraDeg, double longCameraDeg, double latPersonDeg, double longPersonDeg);

double degToRad(double deg);

int compassDirection(char direction);

#endif

