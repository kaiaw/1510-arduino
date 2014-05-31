// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

//This code is intended for use with Arduino Leonardo and other ATmega32U4-based Arduinos


#include "GPS_READER.h"
#include <SoftwareSerial.h>
//required for fmod()
#include <math.h>

SoftwareSerial *camSerial;
GPS *GPS_CAM;
GPS *GPS_PERSON;

int GPS_IN_PIN = 8;   // digital 8
int GPS_OUT_PIN = 9;   // digital 9

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 8
//   Connect the GPS RX (receive) pin to Digital 7
// If using hardware serial:
//   Connect the GPS TX (transmit) pin to Arduino RX1 (Digital 0)
//   Connect the GPS RX (receive) pin to matching TX1 (Digital 1)

 
/**
 * Converts longitude and latitude from
 * Minute-Second format to minute-decimal format.
 *
 * Inspired by Adafruit GPS-library
 */
double convertDegMinToDecDeg (float degMin) {

  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}


/**
 * Set up GPS objects and configure GPS units.
 */
void setupGPS(void)  
{
  
  //C++ does not allow us to use temporary assigned memory adresses.
  //We avoid this by making a temporary object, and then getting that ones adress.
  SoftwareSerial tempSoftSerial = SoftwareSerial(GPS_OUT_PIN, GPS_IN_PIN);
  camSerial = &tempSoftSerial;
  GPS tempGPS_CAM = GPS(camSerial);

  GPS_CAM = &tempGPS_CAM;

  GPS tempGPS_PERSON = GPS(&Serial1);
  GPS_PERSON = &tempGPS_PERSON;

  // Set baud-rate for GPS
  GPS_CAM->begin(115200);
  GPS_PERSON->begin(9600);


  // Configure GPS
  GPS_CAM->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS_PERSON->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  GPS_CAM->sendCommand(PMTK_SET_BAUD_9600);
  GPS_PERSON->sendCommand(PMTK_SET_BAUD_9600);
  
  
  // Set the update rate
  GPS_CAM->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS_PERSON->sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 10 Hz update rate

  //Give the GPS some time to complete configuration before we continue.
  delay(1000); 
}



boolean parseStatusCam;
boolean parseStatusPerson;
/*
  Tried reading from GPS first, which it then does over SWSerial.
  Tried readinf from GPS_PERSON after that, still reads from SWSerial

  From this we know that the GPS data, no matter which is read from, 
  data is stored in the same GPS object. 
*/


/**
 * Read all available GPS data from external GPS.
 */
void getGPSdataFromPerson(void)
{
  char c = -1;
  while (c = GPS_PERSON->read()){
    if (c == -1){
      return;
    }

    if(GPS_PERSON->newNMEAreceived()) {
      //If we received a full dataset, parse the data.
      parseStatusPerson = GPS_PERSON->parse(GPS_PERSON->lastNMEA());
      return;
    }
  }
  
}

/**
 * Read all available GPS data from internal GPS
 */
void getGPSdataFromCamera(void)
{
  char p = 0;
  while ((p = GPS_CAM->read())){
    if (p == -1){
      return;
    }
    
    if(GPS_CAM->newNMEAreceived()) {
      //If we received a full dataset, parse the data.
      parseStatusCam = GPS_CAM->parse(GPS_CAM->lastNMEA());
      return;
    }
  }
}



void getGPSdata(void)
{

  /* Collect data from internal GPS */
  /* !! Discard if it makes the program malfunction !! */
  getGPSdataFromCamera();

  /* Collect data from external GPS */
  getGPSdataFromPerson();
}

/**
 *  Print out test-data to Serial for testing purposes.
 */
void printGPSdata(GPS* gps) {
  
  //Serial.print("\n ******** GPS CAM SoftwareSerial ********* \n");
  Serial.print(F("fix: ")); Serial.print((int)gps->fix);
  Serial.print(F(" quality: ")); Serial.println((int)gps->fixquality);
      
  if (gps->fix) {
    Serial.print(F("Latitude: "));
    Serial.println(convertDegMinToDecDeg(gps->latitude), 6); 
    Serial.print(F("Longitude: ")); 
    Serial.println(convertDegMinToDecDeg(gps->longitude), 6);
    //Serial.print("Angle: "); Serial.println(gps->angle);
    Serial.print(F("Altitude: ")); Serial.println(gps->altitude);
    Serial.print(F("Speed (knots): ")); Serial.println(gps->speed);
    Serial.print(F("Satellites: ")); Serial.println((int)gps->satellites);
  }
  Serial.print(F(" *************************** \n\n"));
      
}

