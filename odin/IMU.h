
#ifndef __IMU__
#define __IMU__

// IMU includes
//# include <Wire.h>
//# include <Adafruit_Sensor.h>
//# include <Adafruit_LSM303_U.h>
//# include <Adafruit_L3GD20_U.h>
//# include <Adafruit_9DOF.h>

/* Assign a unique ID to the sensors */
/* extern Adafruit_9DOF                 dof;//   = Adafruit_9DOF(); */
/* extern Adafruit_LSM303_Accel_Unified accel;// = Adafruit_LSM303_Accel_Unified(30301); */
/* extern Adafruit_LSM303_Mag_Unified   mag; //   = Adafruit_LSM303_Mag_Unified(30302); */
/* extern int roll; */

void setupIMU();
int getRoll(void);

#endif
