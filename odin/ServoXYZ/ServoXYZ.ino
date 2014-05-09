// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 
 
Servo myservoX, myservoY, myservoZ;  // create servo object to control a servo 
 
int potpin = 0;  // analog pin used to connect the potentiometer
int valX;    // variable to read the value from the analog pin 
int valY; 
int valZ;
int rot = 0;

void setup() 
{ 
  myservoX.attach(5);  // attaches the servo on pin 9 to the servo object 
  myservoY.attach(6);
  myservoZ.attach(9);
  pinMode(8, INPUT);
} 
 
void loop() 
{ 
  valX = analogRead(A0);            // reads the value of the potentiometer (value between 0 and 1023) 
  valX = map(valX, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  myservoX.write(valX);
  
  valY = analogRead(A1);            // reads the value of the potentiometer (value between 0 and 1023) 
  valY = map(valY, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  myservoY.write(valY);
  
  Serial.println(8);
  
  if(digitalRead(8) == HIGH){
      rot += 1;
      myservoY.write(rot);
      delay(15);
  }
    // scale it to use it with the servo (value between 0 and 180) 
   // sets the servo position according to the scaled value 
  
  delay(15);                           // waits for the servo to get there 
} 
