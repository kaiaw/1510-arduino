// Servo includes
#include <Servo.h>

// IMU includes
#include <Wire.h>
//#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>


/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Declare Servo motors */
Servo servo_roll;
Servo servo_pitch;
#define servo_roll_pin 9
#define servo_pitch_pin 10
int pitch, roll;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

/* Initiliaze Servo */

void initServos() 
{
   servo_pitch.attach(servo_pitch_pin);
   servo_roll.attach(servo_roll_pin);
}


/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();
  initServos();
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  
  
  int prev_roll = roll;
  int prev_pitch = pitch;
  /* Use the new fusionGetOrientation function to merge accel/mag data */  
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    //Serial.print(F("Orientation: "));
    //Serial.print(orientation.roll);
    roll = map(orientation.roll, -90, 90, 0, 179);
    if(abs(prev_roll - roll) > 2) servo_roll.write(roll);
    //Serial.print(F(" "));
    //Serial.print(orientation.pitch);
    pitch = map(orientation.pitch, 90, -90, 0, 179);
    if(abs(prev_pitch - pitch) > 2) servo_pitch.write(pitch);
    //Serial.println(F(""));
  }
  
  delay(10);
}
