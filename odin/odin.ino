#include <SoftwareSerial.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#include <Wire.h>
#include "IMU.h"
#include <Servo.h>

#include "GPS.h"
#include "GPS_READER.h"
#include "GPS_UTILS.h"

//#include <MemoryFree.h>

//xbee uses serial1

#define GPS_IN   8   // digital 8
#define GPS_OUT  9   // digital 9

#define IMU_SCL  1   // analog  1
#define IMU_SDA  0   // analog  0

#define ServoRollPin   10  // digital 10
#define ServoPitchPin  11  // digital 11
#define ServoYawPin    12  // digital 12

#define pushbtn  7   // digital 7
#define led      6   // digital 6

#define MIN_ROLL 40  
#define MAX_ROLL 140

Servo servoYaw, servoPitch, servoRoll;

double presentYawAngle, desiredYawAngle;
double presentPitchAngle, desiredPitchAngle;

double GPS_ROLL, GPS_PITCH, GPS_YAW;
double IMU_ROLL, IMU_PITCH, IMU_YAW;

double rollServoDegrees;
double pitchServoDegrees;
double yawServoTorque;

uint32_t timer = millis();


void setup() {
  Serial.begin(115200);
  //Serial.begin(9600);
  while(!Serial);
  
  setupGPS();
  delay(500);
  Serial.println(F("Serial begin success"));
  delay(500);
  //setupIMU();
  //Serial.println(F("Setup IMU success, kind of"));
  //delay(500);
  //servoRoll.attach(ServoRollPin); 
  //servoPitch.attach(ServoPitchPin); 
  //servoYaw.attach(ServoYawPin);
  //pinMode(pushbtn, INPUT);
}


// MAIN LOOP.
void loop() {
  //delay(1000);
  delay(100);

  /* if the push button is pressed we reset the position
     and returns
   */
   

  /* double bislettBabbLat = 59.920761;
  double bislettBabbLong = 10.733468;
  
  double guttaLat = 59.924514;
  double guttaLong = 10.739519;
  */

  // read the input on analog pin 0:
  //double degrees = getDesiredPitchAngle(bislettBabbLat, bislettBabbLong, guttaLat, guttaLong);
  // print out the value you read:
  //Serial.println(degrees);
  // delay(100);        // delay in between reads for stability




  /* updating GPS_CAM and CPS_PERSON*/

  /* Get angles between Bara Dur GPS and The Ring GPS */
  GPS_PITCH = getDesiredPitchAngle(GPS_CAM->altitude, GPS_PERSON->altitude,
				   GPS_CAM->latitude, GPS_CAM->longitude,
				   GPS_PERSON->latitude, GPS_PERSON->longitude);

  GPS_YAW = getDesiredYawAngle(GPS_CAM->latitude, GPS_CAM->longitude,
			       GPS_PERSON->latitude, GPS_PERSON->longitude);
   
   //Serial.println(F("\n\nPERSON\n\n"));
   //delay(20);
   //Serial.print(F("FIX "));
   //Serial.println(GPS_PERSON->fix);
   getGPSdata();
   Serial.print(F("fix: "));
   Serial.println((int)GPS_PERSON->fix);
   Serial.print(F("Lat: "));
   Serial.println(convertDegMinToDecDeg(GPS_PERSON->latitude), 6); 
   Serial.print(F("Long: "));
   Serial.println(convertDegMinToDecDeg(GPS_PERSON->longitude), 6); 
   Serial.print(F("Sat: "));
   Serial.println(GPS_PERSON->satellites); 
   
   //printGPSdata(GPS_PERSON);
   //delay(100);
  /* Get angle between camera position and the init position for Bara Dur
     subtracted 90 degrees for yaw since IMU gives 0 degrees pointing north.
     GPS gives 0 degress pointing east 
  */
  delay(20);
  return;

  IMU_YAW = getYawIMU();
  //Serial.println(IMU_YAW);


  //Serial.println("getPitch");

  IMU_PITCH = getPitchIMU();
  //Serial.println(IMU_PITCH);
  
  //Serial.println("getRolling stones");
  IMU_ROLL = getRollIMU();
  //Serial.println(IMU_ROLL);
  
   
  // Limit roll to between +/- 50 degrees. 
  //if (IMU_ROLL < MIN_ROLL) IMU_ROLL = MIN_ROLL;
  //else if (IMU_ROLL > MAX_ROLL) IMU_ROLL = MAX_ROLL;

  // 'orientation' should have valid .roll and .pitch fields
     
  rollServoDegrees = map(IMU_ROLL, -90, 90, 0, 179);

  /* Mapping the difference between IMU and GPS to step values
  */ 
  pitchServoDegrees = map(IMU_PITCH-GPS_PITCH, -90, 90, 0, 179);
  
  /* Mapping the difference between IMU and GPS to torque values between 120 - 135.
     126-128 degrees stops the servo.
  */
  yawServoTorque = map(GPS_YAW-IMU_YAW, -180, 180, 97, 157); // BYTTET UT GPS_YAW med 0!!! OG ENDRET GRENSEVERDIER
  
  servoRoll.write(rollServoDegrees);
  servoPitch.write(pitchServoDegrees);
  servoYaw.write(yawServoTorque);
  //servoRoll.write(rollServoDegrees);
  //servoPitch.write(pitchServoDegrees);
  //servoYaw.write(yawServoTorque);
  delay(40);
  
  return;

    // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
   //printGPSdata(GPS_CAM); 
   //printGPSdata(GPS_PERSON); 
   
  }
}
