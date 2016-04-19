#include "DualVNH5019MotorShield.h"
#include <Servo.h>
//YOU HAVE TO TELL THE WHEELS TO STOP
DualVNH5019MotorShield md;
unsigned long time;
Servo firstServo;
Servo secondServo;
int rotate360Speed = 140; // 1.29s per rotation; 12.91s for 10 rotations
int rotate360Time = 1376; // off by a fraction of a degree.
int rotateStopSpeed = 52;
int pin32 = 32;
int pin34 = 34;
void setup()
{
  Serial.begin(115200);
  md.init();
  firstServo.attach(pin32);
  secondServo.attach(pin34);
}

void loop()
{
  time = millis();
  //md.setM1Speed(300);//Forward right side
  //md.setM2Speed(300); //Forward left side
  //delay(1000);  
  forward();
  //rotate360();
  secondServo.write(92);
  firstServo.write(140);
  delay(2000);
  secondServo.write(140);
  firstServo.write(95);
  delay(2000);
  //backUp();
  //delay(1000);
  //md.setM1Speed(300);//Reverse
  //md.setM2Speed(300);//Reverse
  //delay(1000);
  //stopMotors();
  //md.setM1Speed(0);
  //md.setM2Speed(0);
  //delay(1000);
  //turnLeft();
  //delay(500);
  //turnRight();
  //delay(500);
  /*
  turnSlightRight();
  delay(500);
  turnSlightLeft();
  delay(500);
  Serial.print("Time: "); Serial.println(time);
  */
  //md.setM2Speed(i);
  //md.setM2Speed(i);
}
void forward() {//Careful, invert the speed on one side to go forward
  md.setM1Speed(300);//Forward right side
  md.setM2Speed(300); //Forward left side
}
// - - - - End of forward()  - - - - - - - - - - - - - - - - - - - - - - - - - -

// ==== backUp() ===============================================================
// =============================================================================
void backUp() {
  md.setM1Speed(-300);//Reverse
  md.setM2Speed(-300);//Reverse
}
// - - - - End of backup() - - - - - - - - - - - - - - - - - - - - - - - - - - -

// ==== turnLeft() =============================================================
// =============================================================================
void turnLeft() {
  md.setM1Speed(-300); //Forward right side
  md.setM2Speed(-300);//Reverse left side
}
// - - - - End of turnLeft() - - - - - - - - - - - - - - - - - - - - - - - - - -

// ==== turnRight() ============================================================
// =============================================================================
void turnRight() {
 md.setM1Speed(300); //Reverse right side
 md.setM2Speed(300);//Forward left side
}
// - - - - End of turnRight()  - - - - - - - - - - - - - - - - - - - - - - - - - 

// ==== turnSlightRight() ======================================================
// =============================================================================
void turnSlightRight() {
  md.setM1Speed(-100); //right side //300
  md.setM2Speed(300);//left side   //380
}
// - - - - End of turnSlightRight()  - - - - - - - - - - - - - - - - - - - - - - 

// ==== turnSlightleft =========================================================
// =============================================================================
void turnSlightLeft() {
  md.setM1Speed(-300); //right side //300
  md.setM2Speed(-100);//left side   //380
}
void stopMotors() {
 md.setM1Speed(0);
 delayMicroseconds(10);
 md.setM2Speed(0);
 delayMicroseconds(10);
}
/*
void rotate360() {
  myServo.write(rotate360Speed);
  delay(rotate360Time);
}
*/
