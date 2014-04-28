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

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 8
//   Connect the GPS RX (receive) pin to Digital 7
// If using hardware serial:
//   Connect the GPS TX (transmit) pin to Arduino RX1 (Digital 0)
//   Connect the GPS RX (receive) pin to matching TX1 (Digital 1)

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial camSerial(8, 7);
Adafruit_GPS GPS_CAM(&camSerial);

// If using hardware serial, comment
// out the above two lines and enable these two lines instead:
Adafruit_GPS GPS_PERSON(&Serial1);
HardwareSerial personSerial = Serial1;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

//required for fmod()
#include <math.h>;
 
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


void setup()  
{
    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  delay(5000);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS_CAM.begin(9600);
  GPS_PERSON.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS_CAM.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS_PERSON.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS_CAM.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS_PERSON.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS_CAM.sendCommand(PGCMD_ANTENNA);
  GPS_PERSON.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  camSerial.println(PMTK_Q_RELEASE);
  personSerial.println(PMTK_Q_RELEASE);
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  char c = GPS_CAM.read();
  char p = GPS_PERSON.read();

  boolean parseStatusCam;
  boolean parseStatusPerson;

  // if you want to debug, this is a good time to do it!
 // if ((c) && (GPSECHO))
   // Serial.write(c); 
    
 // if ((p) && (GPSECHO))
  //  Serial.write(p);
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS_CAM.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    // Save parse status 
    parseStatusCam = GPS_CAM.parse(GPS_CAM.lastNMEA());
  }

  // Do the same for the other GPS. 
  if (GPS_PERSON.newNMEAreceived()) {
    parseStatusPerson = GPS_PERSON.parse(GPS_PERSON.lastNMEA());   
      }  

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    //Serial.print("\nTime: ");
    //Serial.print(GPS.hour, DEC); Serial.print(':');
    //Serial.print(GPS.minute, DEC); Serial.print(':');
    //Serial.print(GPS.seconds, DEC); Serial.print('.');
    //Serial.println(GPS.milliseconds);
    //Serial.print("Date: ");
    //Serial.print(GPS.day, DEC); Serial.print('/');
    //Serial.print(GPS.month, DEC); Serial.print("/20");
    //Serial.println(GPS.year, DEC);

    if (parseStatusCam) {
      
      Serial.print("\n ******** GPS CAM SoftwareSerial ********* \n");
      Serial.print("Fix: "); Serial.print((int)GPS_CAM.fix);
      Serial.print(" quality: "); Serial.println((int)GPS_CAM.fixquality);
      
      if (GPS_CAM.fix) {
        Serial.print("Latitude: ");
        Serial.println(convertDegMinToDecDeg(GPS_CAM.latitude), 6); 
        Serial.print("Longitude: "); 
        Serial.println(convertDegMinToDecDeg(GPS_CAM.longitude), 6);
        //Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS_CAM.altitude);
        Serial.print("Speed (knots): "); Serial.println(GPS_CAM.speed);
        Serial.print("Satellites: "); Serial.println((int)GPS_CAM.satellites);
      }
      Serial.print(" *************************** \n\n");
    }

    if (parseStatusPerson) {
      Serial.print("\n ******* GPS PERSON HardwareSerial ******* \n");
      Serial.print("Fix: "); Serial.print((int)GPS_PERSON.fix);
      Serial.print(" quality: "); Serial.println((int)GPS_PERSON.fixquality);
 
      if (GPS_PERSON.fix) {
        Serial.print("Latitude: ");
        Serial.println(convertDegMinToDecDeg(GPS_PERSON.latitude), 6); 
        Serial.print("Longitude: "); 
        Serial.println(convertDegMinToDecDeg(GPS_PERSON.longitude), 6);
        //Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS_PERSON.altitude);
        Serial.print("Speed (knots): "); Serial.println(GPS_PERSON.speed);
        Serial.print("Satellites: "); Serial.println((int)GPS_PERSON.satellites);
      }
      Serial.print(" *************************** \n\n");
    }
  }
}
