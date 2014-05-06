

#ifndef __GPS_READER__
#define __GPS_READER__

#include "../GPS/GPS.h"
#include "GPS_READER.h"
#include <SoftwareSerial.h>
//required for fmod()
#include <math.h>


extern SoftwareSerial* camSerial;
extern GPS* GPS_CAM;
extern GPS* GPS_PERSON;

 
// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
double convertDegMinToDecDeg (float degMin);
void setupGPS(void);
void getGPSdata(void);
void printGPSdata(GPS *gps);

#endif
