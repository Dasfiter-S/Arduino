/*
  Coded by
  Ricardo Carretero
  & servo functions by Tim Enbom
*/
//from front of sensor
//220 ohm resistor is signal, ground, 5v
//3/24/2016 EDIT
//Servo notes: closer toward input power is the line sensor on the right
//looking at the sensors from the front of the robot.
//Make sure the ground all goes to the same place
//Red dot on motor goes to white wire on to Motorshield A port
#include "DualVNH5019MotorShield.h"
#include <Servo.h>  
DualVNH5019MotorShield md;
Servo myServo;
Servo myServo2;
//Pins for the sensors========================================================================================
#define Pin2 22 //left side
#define Pin3 24
#define Pin4 26
#define Pin5 28
#define Pin6 30 // right side
//============================================================================================================
uint64_t currentTime = 0;
uint64_t startTime = 0;
unsigned long runTime = 0;
int rotate360Speed = 40; // 1.29s per rotation; 12.91s for 10 rotations
int rotate360Time = 1376; // off by a fraction of a degree.
int rotateStopSpeed = 92;
int Pin32 = 32;
int Pin34 = 34;
int lastTurn = 0;
float ratio = 0.166;

int pathState=0;
int pathArray[7] = {1,0,1,1,2,0,0}; //1 = left, 0 straight, 2 right

void setup() {
  Serial.begin(9600);
  md.init();
  myServo.attach(Pin32); 
  myServo2.attach(Pin34);
  setUp();
}
//MAIN LOOP===================================================================================================
void loop() {
  //currentTime = micros();    // Refresh Timer
  //Serial.print("Time: "); 
  //long time = currentTime; //If using currentTime print won't work by reference
  //Serial.print(time );
  //Serial.print("    ");
  drive(sensorState());
  //inputValues();           //Debug values for sensors
  //virtualTrack();          //Virtual Track for debugging
  //rotate360();
}
//Servo Functions=============================================================================================================
void setUp() {
  myServo.write(96);
  delay(10);
  myServo2.write(92);
  delay(10);
}
void rotate360() {
  myServo.write(rotate360Speed);
  myServo2.write(rotate360Speed);
  delay(rotate360Time);
  setUp();
}

//Sensor Functions============================================================================================================
int isOnLine(int sensorIn){       
   //startTime = millis();
   pinMode(sensorIn, OUTPUT);     // Make pin OUTPUT
   digitalWrite(sensorIn, HIGH);  // Pin HIGH (discharge capacitor)
   pinMode(sensorIn, INPUT);      // Make pin INPUT
   digitalWrite(sensorIn, LOW);   // Turn off internal pullups
   if (!digitalRead(sensorIn)) {  // Pin goes HIGH which means its on the black line
      return 0;
   
 }
   if(digitalRead(sensorIn)){     // Pin goes LOW which means its not on the black line
      return 1;
   }
   /*
   runTime = millis();
   runTime -= startTime;
   Serial.print("Time: "); Serial.println(runTime);
   */
}
/*
  Taking the position of each of the 5 sensors into account, I arranged the 
  function to produce a binary digit that is being created as base 2 but is 
  stored as its base 10 value. This allows us to produce a single variable 
  state for our entire array of line sensors as opposed to having to create
  a for loop that would ruin the continuity of multi tasking for the robot. 
*/
int sensorState() { 
  //startTime = millis();
  int currentState = 0;
  currentState = ((isOnLine(Pin6)*16) + (isOnLine(Pin5)*8) + (isOnLine(Pin4)*4) + (isOnLine(Pin3)*2) + isOnLine(Pin2));
  //Serial.println(currentState);
  /*
  runTime = millis();
  runTime -= startTime;
  unsigned int temp = runTime;
  Serial.print("Time: "); Serial.println(temp);
  */
  return currentState;
}

//PATTERN RECOGNITION=========================================================================================
void drive(int state) {
 if (state == 4) {    //00100
  forward();
  //stopMotors();
  //Serial.println("Straight");
 }
 if (state == 0) {    //00000
  //forward();
   stopMotors();
  //Serial.println("Start Area");
 } 
 if (state == 7) {   //00111
 /*
  if (lastMovement != 24 || 28) {
    stopMotors();
    //rotate360();
    Serial.println("Dropping Ring!");
  }
  */
  if (lastTurn != (24 || 28)) {
    stopMotors();
    gotTurn();
    //leftTurn();
  }
  lastTurn == state;
  //Serial.println("Left turn");
 } 
 /*
 if (state == 3) {   //00011
  if (lastMovement != 7 || 3) {
    rightTurn();
  }  
  Serial.println("Left turn");
  lastMovement == state;
 }
 */
 if (state == 28) {   //11100
   if (lastTurn != (7 || 3)) {
     //gotTurn();
    //rightTurn();
  }
  //Serial.println("Right turn");
  lastTurn == state;
 } 
 /*
 if (state == 24) {   //11000
  if (lastMovement != (7 || 3)) {
    rightTurn();
  }
  Serial.println("Right turn");
  lastMovement == state;
 }
 */
 if (state == 31) {   //11111 second time we encounter this it has to be a right turn
    //forward();
    gotTurn();
  //stopMotors();  
    //lastTurn == state;
 } 
 
 //Correction routines-------------
 
 if (state == 12) {    //01100
 slightRight();
 //forward();
 //Serial.println("Correct right");
 } 
 if (state == 8) {    //01000
  slightRight();
  //Serial.println("Correct right");
 } 
     
 if (state == 6) {    //00110
  slightLeft();
  //forward();
  //Serial.println("Correct left");
 }
  
 if (state == 2) {    //00010
  slightLeft();
  //Serial.println("Correct left");
 } 
}
//MOTOR FUNCTIONS=============================================================================================
void gotTurn(){
  //still ahve to add failsafes, if there is a bad readibng it doesn't throw the whole track off?
  if(pathArray[pathState]==0){
    //go forward
    forward();
    delay(150);
  }
  else if(pathArray[pathState]==1){
    //go left
    leftTurn();
  }
  else{
    //go right
    rightTurn();
  }
  pathState++;
}
void forward() {  //Moves the robot forward
  md.setM1Speed(300); //right side                 300 or  200
  md.setM2Speed(-247);//left side //more powerful -247 or -165 
  //delay(10);
}
void backward() { //Moves the robot backwards
  md.setM1Speed(-300); //right side 300
  md.setM2Speed(247);//left side    247
}
void leftTurn() { //Turns the robot left from the center|| both have to be positive
  md.setM1Speed(400); //Forward right side 400
  md.setM2Speed(20);//Reverse left side     10
  delay(630);
}
void rightTurn() { //Turns the robot right from the center|| both have to be negative
  md.setM1Speed(-20); //Forward right side                  400
  md.setM2Speed(-(motorRatio(400)));//Reverse left side     10
 delay(630);
}
void slightLeft() {
  md.setM1Speed(65); //right side //             75 
  md.setM2Speed(45);//left side  //              65
}
void slightRight() {
  md.setM1Speed(-40); //right side //           -50
  md.setM2Speed(-55);//left side  //            -75
}
void stopMotors() {//Stop the robot
 md.setM1Speed(-10);
 delay(10);
 md.setM2Speed(10);
 delay(10);
}
float motorRatio(int power) {
  return (power - power*ratio);
}
//END OF MOTOR FUNCTIONS===============================================================
void inputValues() {
  Serial.print("Sensor 1: ");
  Serial.print(isOnLine(Pin2));   // Connect to pin 2, display results
  Serial.print(", Sensor 2: ");
  Serial.print(isOnLine(Pin3));   // Same for these
  Serial.print(", Sensor 3: ");
  Serial.print(isOnLine(Pin4));
  Serial.print(", Sensor 4: ");
  Serial.print(isOnLine(Pin5));
  Serial.print(", Sensor 5: ");
  Serial.print(isOnLine(Pin6));
  Serial.println();
}
//VIRTUAL TRACK=============================================================================
/*
  This function is a copy of isOnLine but is only used to test the track path
  pattern.
*/
int testSensor(int tPin0, int tPin1, int tPin2, int tPin3, int tPin4) {
  int currentState = 0;
  currentState = ((tPin0*16) + (tPin1*8) + (tPin2*4) + (tPin3*2) + tPin4);
  //Serial.println(currentState);
  return currentState;
}
/*
  Produces a virtual track that mimics the real thing.
  It contains the same patterns the robot would 
  encounter on the track.
*/
void virtualTrack() {
  if ((currentTime > 0) && (currentTime < 1000)) { //Start 00000
    drive(testSensor(0,0,0,0,0));
  }
  if ((currentTime > 1000) && (currentTime < 1500)) { //Out of start Area 11111
    drive(testSensor(1,1,1,1,1));
  }
  if ((currentTime > 1500) && (currentTime < 4000)) { //First Straight 00100
    drive(testSensor(0,0,1,0,0));
  }
  if ((currentTime > 4000) && (currentTime < 4100)) { //First Left Turn 11100
    drive(testSensor(1,1,1,0,0));
  }
  //First Quadrant END============================================================================
  if ((currentTime > 4100) && (currentTime < 6000)) { //Second Straight 00100
    drive(testSensor(0,0,1,0,0));
  }
  if ((currentTime > 6000) && (currentTime < 6100)) { //Ignore turn and go Straight 11100
    drive(testSensor(1,1,1,0,0));
  }
  if ((currentTime > 6100) && (currentTime < 8400)) { //Third Straight 00100
    drive(testSensor(0,0,1,0,0));
  }
  if ((currentTime > 8400) && (currentTime < 8500)) { //Second Turn Left 11100
    drive(testSensor(1,1,1,0,0));
  }
  //Second Quadrant END==========================================================================
  if ((currentTime > 8500) && (currentTime < 9500)) { //Keep Straight 00100
    drive(testSensor(0,0,1,0,0));
  }
  if ((currentTime > 9500) && (currentTime < 9600)) { //Third Left turn 11100
    drive(testSensor(1,1,1,0,0));
  }
  //Third Quadrant END===========================================================================
  if ((currentTime > 9600) && (currentTime < 10600)) { //Straight short 00100
    drive(testSensor(0,0,1,0,0));
  }
  if ((currentTime > 10600) && (currentTime < 10700)) { //First right turn 11111
    drive(testSensor(1,1,1,1,1));
  }
  if ((currentTime > 10700) && (currentTime < 11700)) { //Second straight short 00100
    drive(testSensor(0,0,1,0,0));
  }
  if ((currentTime > 11700) && (currentTime < 11800)) { //Second right turn 00111
    drive(testSensor(0,0,1,1,1));
  }
  if ((currentTime > 11800) && (currentTime < 12800)) { //Third straight short 00100
    drive(testSensor(0,0,1,0,0));
  }
  if ((currentTime > 12800) && (currentTime < 12900)) { //Fourth Left turn 11100
    drive(testSensor(1,1,1,0,0));
  }
  //Fourth Quadrant END==================================================================================
  if ((currentTime > 12900) && (currentTime < 13900)) { //Keep Straight 00100
    drive(testSensor(0,0,1,0,0));
  }
  if ((currentTime > 13900) && (currentTime < 14000)) { //Fifth Left turn 11100
    drive(testSensor(1,1,1,0,0));
  }
  //Fifth Quadrant END=========================================================================
  if ((currentTime > 14000) && (currentTime < 15900)) { //Keep Straight 00100
    drive(testSensor(0,0,1,0,0));
  }
  if ((currentTime > 15900) && (currentTime < 16000)) { //Sixth Left turn 11100
    drive(testSensor(1,1,1,0,0));
  }
//END OF VIRTUAL TRACK==========================================================================
}
/*
  Robot path
  00000 Start
  11111 Out of the start area
  00100 first straight
  11100 first left turn
  |OOOOOOOO|>>>>>>>>>>>>>>>>>>Drop rings 1 and 2
  =====First Quadrant End-----------------------------
  
  00100 second straight
  11100 ignore and go straight
  00100 third straight
  11100 second turn left 
  |OOOOOOOO|>>>>>>>>>>>>>>>>>>Drop rings 3 and 4
  =====Second Quadrant End----------------------------
  
  00100 go straight
  11100 third turn left
  |OOOOOOOO|>>>>>>>>>>>>>>>>>>Drop ring 5
  =====Third Quadrant End-----------------------------
  
  00100 go straight SHORT
  11111 turn right
  00100 go straight SHORT
  |OOOOOOOO|>>>>>>>>>>>>>>>>>>Drop ring 6
  00111 turn right
  00100 go straight SHORT
  =====Fourth Quadrant End----------------------------
  
  11100 turn left
  |OOOOOOOO|>>>>>>>>>>>>>>>>>>Drop ring 7
  00100 go straight
  11100 turn left
  |OOOOOOOO|>>>>>>>>>>>>>>>>>>Drop rings 8 and 9
  =====Fifth Quadrant END-----------------------------
  
  00100 go straight
  11100 turn left
  =====Sixth Quadrant END-----------------------------
  
  00100 go straight
  11111 turn right
  00100 go straight
  11100 turn left
  00100 go straight
  |OOOOOOOO|>>>>>>>>>>>>>>>>>>Drop ring 11
  11100 turn left
  00100 go straight
  11111 turn left
  00100 go straight
  11111 ignore and go straight
  00100 straight very short
  11111 turn left
  =====Inside track END-------------------------------
  
  00100 go straight
  11100 left turn
  |OOOOOOOO|>>>>>>>>>>>>>>>>>>Drop ring 12
  00100 go straight
  11111 END!
  //END=======================================================================================================
  
  
*/

