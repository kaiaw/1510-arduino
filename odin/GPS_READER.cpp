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

GPS* GPS_CAM;
GPS* GPS_PERSON;
SoftwareSerial* camSerial;

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 8
//   Connect the GPS RX (receive) pin to Digital 7
// If using hardware serial:
//   Connect the GPS TX (transmit) pin to Arduino RX1 (Digital 0)
//   Connect the GPS RX (receive) pin to matching TX1 (Digital 1)

 
// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
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


void setupGPS(void)  
{

camSerial = &SoftwareSerial(9, 8); // TX, RX
GPS_CAM = &GPS(camSerial);

// If using hardware serial, comment
// out the above two lines and enable these two lines instead:

GPS_PERSON = &GPS(&Serial1);

    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  delay(5000);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS_CAM->begin(9600);
  GPS_PERSON->begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS_PERSON->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS_CAM->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS_PERSON->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set gps baud rate.
  GPS_CAM->sendCommand(PMTK_SET_BAUD_9600);
  GPS_PERSON->sendCommand(PMTK_SET_BAUD_9600);
  
  
  // Set the update rate
  GPS_CAM->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS_PERSON->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  /* GPS.sendCommand(PGCMD_ANTENNA); */
  /* GPS_PERSON->sendCommand(PGCMD_ANTENNA); */

  delay(1000); 
  // Ask for firmware version
  /* camSerial.println(PMTK_Q_RELEASE); */
  /* personSerial.println(PMTK_Q_RELEASE); */
}



boolean parseStatusCam;
boolean parseStatusPerson;
/*
  Tried reading from GPS first, which it then does over SWSerial.
  Tried readinf from GPS_PERSON after that, still reads from SWSerial

  From this we know that the GPS data, no matter which is read from, 
  data is stored in the same GPS object. 
*/


void getGPSdata(void)                     // run over and over again
{
  char c, p;
  
  c = GPS_CAM->read();
  p = GPS_PERSON->read();

  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS_CAM->newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    // this also sets the newNMEAreceived() flag to false

    /* Serial.print("CAM GPS SOFTWARESERIAL: \n"); */
    /* Serial.println(GPS.lastNMEA()); */
    /* Serial.println("\n"); */

    
    // Save parse status 
    parseStatusCam = GPS_CAM->parse(GPS_CAM->lastNMEA());
  }
  
  // Do the same for the other GPS. 
  if (GPS_PERSON->newNMEAreceived()) {

    /* Serial.print("PERSON GPS HARDWARESERIAL: \n"); */
    /* Serial.println(GPS_PERSON->lastNMEA()); */
    /* Serial.println("\n"); */
    
    parseStatusPerson = GPS_PERSON->parse(GPS_PERSON->lastNMEA());   
  }  

}

void printGPSdata(GPS* gps) {

  
  Serial.print("\n ******** GPS CAM SoftwareSerial ********* \n");
  Serial.print("fix: "); Serial.print((int)gps->fix);
  Serial.print(" quality: "); Serial.println((int)gps->fixquality);
      
  if (gps->fix) {
    Serial.print("Latitude: ");
    Serial.println(convertDegMinToDecDeg(gps->latitude), 6); 
    Serial.print("Longitude: "); 
    Serial.println(convertDegMinToDecDeg(gps->longitude), 6);
    //Serial.print("Angle: "); Serial.println(gps->angle);
    Serial.print("Altitude: "); Serial.println(gps->altitude);
    Serial.print("Speed (knots): "); Serial.println(gps->speed);
    Serial.print("Satellites: "); Serial.println((int)gps->satellites);
  }
  Serial.print(" *************************** \n\n");
      
}

