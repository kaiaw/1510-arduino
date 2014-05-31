// Librarys used by IMU
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include "IMU.h"
#include <Wire.h>

// Librarys to use servos
#include <Servo.h>

// Librarys used by GPS
#include "GPS.h"
#include "GPS_READER.h"
#include "GPS_UTILS.h"
#include <SoftwareSerial.h>

//xbee uses serial1

#define GPS_IN   8   // digital 8
#define GPS_OUT  9   // digital 9

#define IMU_SCL  1   // analog  1
#define IMU_SDA  0   // analog  0

#define ServoRollPin   11  // digital 11
#define ServoPitchPin  10  // digital 10
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


double GPS_SYNC_LATITUDE = 0;
double GPS_SYNC_LONGITUDE = 0;
double GPS_SYNC_ALTITUDE = 0;
boolean GPS_SYNC_SUCCESS = false;

uint32_t timer = millis();


void setup() {
  pinMode(led, OUTPUT);
  
  servoRoll.attach(ServoRollPin);
  servoPitch.attach(ServoPitchPin);
  servoYaw.attach(ServoYawPin);

  //Set servos to init position
  servoPitch.write(90);
  servoRoll.write(90);
  servoYaw.write(127);
  
  pinMode(pushbtn, INPUT_PULLUP);
  
  Serial.begin(9600);  
  Serial1.begin(9600);
  
  Serial.println(F("GPS Setup done"));

  setupGPS();
  setupIMU();
  Serial.println(F("Setup IMU success"));

  //idicates that ODIN is finished with setup
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
}


// MAIN LOOP.
void loop() {

  IMU_YAW = getYawIMU();
  IMU_PITCH = getPitchIMU();
  IMU_ROLL = getRollIMU();
  
  Serial1.print("YAW  ");
  Serial1.print(IMU_YAW, 3);
  Serial1.print("  PITCH  ");
  Serial1.print(IMU_PITCH, 3);
  Serial1.print("  ROLL  ");
  Serial1.println(IMU_ROLL, 3);
  
  
  return;
  
  if (digitalRead(pushbtn) == LOW && GPS_PERSON->fix){
    digitalWrite(led, HIGH);
    GPS_SYNC_LATITUDE = GPS_PERSON->latitude;
    GPS_SYNC_LONGITUDE = GPS_PERSON->longitude;
    GPS_SYNC_ALTITUDE = GPS_PERSON->altitude;
    GPS_SYNC_SUCCESS = true;
    //If we synced a good position, tell the user.
    digitalWrite(led, LOW);
    delay(500);
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    delay(500);
    digitalWrite(led, HIGH);
    delay(500);
  } else if (GPS_SYNC_SUCCESS) {
    digitalWrite(led, HIGH);
  } else {
    digitalWrite(led, LOW);
  }

  getGPSdata();

  //Workaround for complications with internal GPS
  if (GPS_PERSON->fix && GPS_SYNC_SUCCESS){
    setServos();
  } else {
    stopServos();
  }
 

}

// Calculating servo torque and degrees and setting them to servos.
void setServos(){
  
  GPS_PITCH = getDesiredPitchAngle(GPS_SYNC_ALTITUDE, GPS_PERSON->altitude,
				   GPS_SYNC_LATITUDE, GPS_SYNC_LONGITUDE,
				   GPS_PERSON->latitude, GPS_PERSON->longitude);

  GPS_YAW = getDesiredYawAngle(GPS_SYNC_LATITUDE, GPS_SYNC_LONGITUDE,
			       GPS_PERSON->latitude, GPS_PERSON->longitude);

  IMU_YAW = getYawIMU();
  IMU_PITCH = getPitchIMU();
  IMU_ROLL = getRollIMU();

  // 'orientation' should have valid .roll and .pitch fields
     
  rollServoDegrees = map(IMU_ROLL, -90, 90, 0, 179);

  /* Mapping the difference between IMU and GPS to step values
  */ 
  //pitchServoDegrees = map(IMU_PITCH-GPS_PITCH, -90, 90, 0, 179);
  pitchServoDegrees = map(IMU_PITCH-GPS_PITCH, -90, 90, 0, 179);

  if (pitchServoDegrees > 120)
    pitchServoDegrees = 120;
  else if (pitchServoDegrees < 60){
    pitchServoDegrees = 60;
  }
  
  /* Mapping the difference between IMU and GPS to torque values between 120 - 135.
     126-128 degrees stops the servo.
  */
  yawServoTorque = map(GPS_YAW-IMU_YAW, -180, 180, 135, 120); // BYTTET UT GPS_YAW med 0!!! OG ENDRET GRENSEVERDIER
  

  servoRoll.write(rollServoDegrees);
  servoPitch.write(pitchServoDegrees);
  servoYaw.write(yawServoTorque);
  

}

void stopServos(){
  servoYaw.write(127);
}
