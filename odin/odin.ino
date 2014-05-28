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

#define ServoRollPin   11  // digital 10
#define ServoPitchPin  10  // digital 11
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

  servoPitch.write(90);
  servoRoll.write(90);
  servoYaw.write(127);
  
  pinMode(pushbtn, INPUT_PULLUP);
  

  //Serial.begin(115200);
  Serial.begin(9600);
  //while(!Serial);  Might fail if we don't have a USB connected
  
  Serial1.begin(9600);
  

  delay(500);
  Serial.println(F("GPS Setup done"));

  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
  setupGPS();

  setupIMU();
  //Serial.println(F("Setup IMU success, kind of"));
  //delay(500) ;
}


// MAIN LOOP.
void loop() {
  //delay(1000);
  
  
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


  Serial1.println("Read GPS data");
  delay(2);
  getGPSdata();
  
  /*
  Serial1.println("\nGPS Person");
  delay(2);
  Serial1.print(F("fix: "));
  delay(2);
  Serial1.println((int)GPS_PERSON->fix);
  delay(2);
  Serial1.print(F("long: "));
  delay(2);
  Serial1.println((int)GPS_PERSON->longitude);
  delay(2);
  Serial1.print(F("lat: "));
  delay(2);
  Serial1.println((int)GPS_PERSON->latitude);
  delay(2);

  Serial1.println("\nGPS camera");
  delay(2);
  Serial1.print(F("long: "));
  delay(2);
  Serial1.println((int)GPS_SYNC_LONGITUDE);
  delay(2);
  Serial1.print(F("lat: "));
  delay(2);
  Serial1.println((int)GPS_SYNC_LATITUDE);
  */

  if (GPS_PERSON->fix && GPS_SYNC_SUCCESS){
    setServos();
  } else {
    Serial1.println(F("Stop servos"));
    Serial1.println(GPS_SYNC_SUCCESS);
    Serial1.println(GPS_PERSON->fix);
    stopServos();
  }
  
  /*
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
   //printGPSdata(GPS_CAM); 
   //printGPSdata(GPS_PERSON); 
  }
  */

}


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
  pitchServoDegrees = map(IMU_PITCH, -90, 90, 0, 179);

  if (pitchServoDegrees > 120)
    pitchServoDegrees = 120;
  else if (pitchServoDegrees < 60){
    pitchServoDegrees = 60;
  }
  
  /* Mapping the difference between IMU and GPS to torque values between 120 - 135.
     126-128 degrees stops the servo.
  */
  yawServoTorque = map(GPS_YAW-IMU_YAW, -180, 180, 135, 120); // BYTTET UT GPS_YAW med 0!!! OG ENDRET GRENSEVERDIER
  
  Serial1.print(F("PITCH: "));
  Serial1.println(IMU_PITCH);
  Serial1.println(pitchServoDegrees);
  Serial1.print(F("YAW:   "));
  Serial1.println(IMU_YAW);
  Serial1.println(yawServoTorque);
  
  
  servoPitch.write(pitchServoDegrees);

  servoYaw.write(yawServoTorque);

}

void stopServos(){
  servoYaw.write(127);
}
