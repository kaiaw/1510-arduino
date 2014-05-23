// IMU includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include "IMU.h"

/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

double roll;
double pitch;
double yaw;

void setupIMU()
{
  Serial.println("Setting up IMU");
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  
  Serial.println("Accel works");
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  Serial.println("SetupIMU done!");
}

double getRollIMU(void)
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  
  double prev_roll = roll;

  /* Use the new fusionGetOrientation function to merge accel/mag data */  
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {    
    roll = orientation.roll;
    if(abs(prev_roll - roll) > 2) return roll;
  }
}

double getPitchIMU(void)
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

 /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  double prev_pitch = pitch;

  /* Use the new fusionGetOrientation function to merge accel/mag data */  
  if(dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    pitch = orientation.pitch;
    // returnerer nå kun hvis denne intreffer ??????
    if(abs(prev_pitch - pitch) > 2) return pitch;
  }
}
double getYawIMU(void)
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);


  double prev_yaw = yaw;

  if(dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    yaw = orientation.heading;
//  if(abs(prev_yaw - yaw) > 2) 
    return yaw;
  }
}
