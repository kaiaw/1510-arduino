
#include "GPS.h"
#include "GPS_UTILS.h"

GPS::GPS(SoftwareSerial *ser)
{
  common_init();     // Set everything to common state, then...
  gpsSwSerial = ser; // ...override gpsSwSerial with value passed.
}

GPS::GPS(HardwareSerial *ser) {
  common_init();  // Set everything to common state, then...
  gpsHwSerial = ser; // ...override gpsHwSerial with value passed.
}

// Initialization code used by all constructor types
void GPS::common_init(void) {
  recvdflag   = false;
  lineidx     = 0;
  currentline = line1;
  lastline    = line2;
  gpsHwSerial = NULL;
  gpsSwSerial = NULL;

  hour = minute = seconds = year = month = day =
    fixquality = satellites = 0; // uint8_t
  lat = lon = mag = 0; // char
  fix = false; // boolean
  milliseconds = 0; // uint16_t
  latitude = longitude = geoidheight = altitude =
    speed = angle = magvariation = HDOP = 0.0; // float
}

void GPS::begin(uint16_t baud)
{
  if(gpsSwSerial != NULL){
    gpsSwSerial->begin(baud);
    while(!gpsSwSerial);
    gpsSwSerial->listen();
  }
  else{
    gpsHwSerial->begin(baud);
    while(!gpsHwSerial);
  }
  delay(1);
}

void GPS::sendCommand(char *str) {
  if(gpsSwSerial) 
    gpsSwSerial->println(str);
  else
    gpsHwSerial->println(str);
}


/**
 * Parse data for the given GPS and store values.
 * This will find longitude, latitude, altitude, fix and more.
 * Values are used to calculate height and range differences and direction.
 */
boolean GPS::parse(char *nmea) {

  // first look if we even have one
  if (nmea[strlen(nmea)-4] == '*') {

    uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
    sum += parseHex(nmea[strlen(nmea)-2]);
    
    // Calculate checksum diff
    for (uint8_t i=1; i < (strlen(nmea)-4); i++)
      sum ^= nmea[i];
    
    // Check for bad checksum
    if (sum != 0)
      return false;

  }

  // Look for known keyword -> GGA
  if (strstr(nmea, "$GPGGA")) {
    char *p = nmea;

    // Get time
    p = strchr(p, ',')+1;
    
    // Parse out latitude
    p = strchr(p, ',')+1;
    latitude = atof(p);

    p = strchr(p, ',')+1;

    latitude = latitude*compassDirection(p[0]);
    
    // Parse out longitude
    p = strchr(p, ',')+1;
    longitude = atof(p);
    
    p = strchr(p, ',')+1;
    longitude = longitude*compassDirection(p[0]);
    
    // Parse rest of the stored information
    p = strchr(p, ',')+1;
    fixquality = atoi(p);

    p = strchr(p, ',')+1;
    satellites = atoi(p);
    
    p = strchr(p, ',')+1;
    HDOP = atof(p);
    
    p = strchr(p, ',')+1;
    altitude = atof(p);

    p = strchr(p, ',')+1;
    p = strchr(p, ',')+1;
    geoidheight = atof(p);

    return true;
  }

  // Look for known keyword -> RMC
  if (strstr(nmea, "$GPRMC")) {
    char *p = nmea;

    // Get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);
    milliseconds = fmod(timef, 1.0) * 1000;
    p = strchr(p, ',')+1;

    if (p[0] == 'A') 
      fix = true;
    else if (p[0] == 'V')
      fix = false;
    else
      return false;

    // Parse latitude
    p = strchr(p, ',')+1;
    latitude = atof(p);

    p = strchr(p, ',')+1;
    if (p[0] == 'N') lat = 'N';
    else if (p[0] == 'S') lat = 'S';
    else if (p[0] == ',') lat = 0;
    else return false;

    // parse out longitude
    p = strchr(p, ',')+1;

    longitude = atof(p);

    p = strchr(p, ',')+1;

    if (p[0] == 'W') lon = 'W';
    else if (p[0] == 'E') lon = 'E';
    else if (p[0] == ',') lon = 0;
    else return false;

    // Velocity
    p = strchr(p, ',')+1;
    speed = atof(p);

    // Angle
    p = strchr(p, ',')+1;
    angle = atof(p);

    p = strchr(p, ',')+1;
    uint32_t fulldate = atof(p);
    day = fulldate / 10000;
    month = (fulldate % 10000) / 100;
    year = (fulldate % 100);

    // There is still some information left,
    // but this is currently not relevant.
    return true;
  }

  return false;
}

// Hexadecimal to Decimal converter
uint8_t GPS::parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
}


/**
 * Read a character from the given GPS and store it.
 */
char GPS::read(void) {
  char c = 0;

  //Check if this GPS-object communicates via SoftwareSerial
  if(gpsSwSerial) {

    //If the given serial is not available, return 0.
    if(!gpsSwSerial->available())
      return c;
    else
      c = gpsSwSerial->read();

  } else { // Else it reads from HardwareSerial

    //If the given serial is not available, return 0.
    if(!gpsHwSerial->available())
      return c;
    else
      c = gpsHwSerial->read();
  }
    
  if (c == '$') {
    currentline[lineidx] = 0;
    lineidx = 0;
  }

  if (c == '\n') {
    currentline[lineidx] = 0;

    if (currentline == line1) {
      currentline = line2;
      lastline = line1;
    } else {
      currentline = line1;
      lastline = line2;
    }

    lineidx = 0;
    recvdflag = true;
  }

  currentline[lineidx++] = c;
  if (lineidx >= MAXLINELENGTH)
    lineidx = MAXLINELENGTH-1;

  return c;
}

// Does the object contain a full string of data?
boolean GPS::newNMEAreceived(void) {
  return recvdflag;
}

// Get the last set of complete data
char *GPS::lastNMEA(void) {
  recvdflag = false;
  return (char *)lastline;
}
