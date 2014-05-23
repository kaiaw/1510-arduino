#include "GPS.h"
#include <SoftwareSerial.h>
#include "GPS_READER.h"
#include "GPS_UTILS.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Wire.h>
#include "IMU.h"
#include <Servo.h>
//xbee uses serial1

#define GPS_IN   8   // digital 8
#define GPS_OUT  9   // digital 9

#define IMU_SCL  3   // analog  3
#define IMU_SDA  2   // analog  2

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
//  setupGPS();
Serial.begin(9600);
  delay(2000);
   Serial.println("setup beguin");
  setupIMU();
   Serial.println("imu setupd done");
  servoRoll.attach(ServoRollPin); 
  servoPitch.attach(ServoPitchPin); 
  servoYaw.attach(ServoYawPin);
   Serial.println("attatching servos");
  delay(1000);
  servoRoll.write(90);     // Set to tart position.
  servoPitch.write(90);    // 
  servoYaw.write(127.5);   // 127.5 is zero speed!
  delay(2000);             // Time to check start pos. 
  pinMode(pushbtn, INPUT);
  Serial.write("setup done");
   Serial.println("setup done");
}


// MAIN LOOP.
void loop() {

  /* if the push button is pressed we reset the position
     and returns
   */
  if(digitalRead(pushbtn) == HIGH){
      servoRoll.write(90);
      servoPitch.write(90);
      servoYaw.write(127.5);
      return;
  }
  
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
 // getGPSdata();
  Serial.write("getting gps data");

  /* Get angles between Bara Dur GPS and The Ring GPS */
  GPS_PITCH = getDesiredPitchAngle(GPS_CAM->altitude, GPS_PERSON->altitude,
				   GPS_CAM->latitude, GPS_CAM->longitude,
				   GPS_PERSON->latitude, GPS_PERSON->longitude);

  GPS_YAW = getDesiredYawAngle(GPS_CAM->latitude, GPS_CAM->longitude,
			       GPS_PERSON->latitude, GPS_PERSON->longitude);
   
  /* Get angle between camera position and the init position for Bara Dur
     subtracted 90 degrees for yaw since IMU gives 0 degrees pointing north.
     GPS gives 0 degress pointing east 
  */
  IMU_YAW = getYawIMU()-90;
  
  IMU_PITCH = getPitchIMU();
  
  IMU_ROLL = getRollIMU();
  
   
  // Limit roll to between +/- 50 degrees. 
  if (IMU_ROLL < MIN_ROLL) IMU_ROLL = MIN_ROLL;
  else if (IMU_ROLL > MAX_ROLL) IMU_ROLL = MAX_ROLL;

  // 'orientation' should have valid .roll and .pitch fields
     
  rollServoDegrees = map(IMU_ROLL, -90, 90, 0, 179); // 

  /* Mapping the difference between IMU and GPS to step values
  */ 
  pitchServoDegrees = map(IMU_PITCH-GPS_PITCH, -90, 90, 0, 179);

  /* Mapping the difference between IMU and GPS to torque values between 120 - 135.
     126-128 degrees stops the servo.
  */
  yawServoTorque = map(GPS_YAW-IMU_YAW, -180, 180, 120, 135);

  /* setting values to servos
   */
  servoRoll.write(rollServoDegrees);
  servoPitch.write(pitchServoDegrees);
  servoYaw.write(yawServoTorque);

    // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
   //printGPSdata(GPS_CAM); 
   //printGPSdata(GPS_PERSON); 
   
  }
}
