/*
  Coded by
  Ricardo Carretero
  & Tim Enbom
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
#include <SoftwareSerial.h> //touchscreen communication serial
#include <Nextion.h>        //Touchscreen library
DualVNH5019MotorShield md;
Servo myServo;
Servo myServo2;
SoftwareSerial nextion(50, 51); //initialize 50 to RX(receive) and 51 to TX(send)
Nextion myNextion(nextion, 9600);
//Pins for the QTI sensors========================================================================================
#define Pin2 22 //left side
#define Pin3 24
#define Pin4 26
#define Pin5 28
#define Pin6 30 // right side
//============================================================================================================
#define Pin11 11
#define Pin12 12
#define Pin13 13
#define Pin14 14
#define Pin15 15
uint64_t currentTime = 0;
uint64_t startTime = 0;
unsigned long runTime = 0;
int rotate360Speed = 40; // 1.29s per rotation; 12.91s for 10 rotations
int rotate360Time = 1260; // off by a fraction of a degree.
int rotateStopSpeed = 92;
int Pin32 = 32;
int Pin34 = 34;
float ratio = 0.166;
boolean stopRefresh = true;
int lastTurn = 0;
int batteryPower = 1;
int pathState = 0;  //quadrant 1| quadrant 2    | center 
int pathArray[20] = {1, 0, 1, 1, 2, 2, 2, 1, 1, 1, 0, 0, 1, 1 }; //1 = left, 0 straight, 2 right
int ringState = 0;
int ringArray[] = {0, 0, 1, 2, 1, 0, 3, 1, 3, 0}; //0 both arms, 1 right, 2 left, 3 none
boolean startTrack = true;
//See Path.png for navigation path
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
  //testSensors();
  //testServos(6);
  //currentTime = millis();
  drive(sensorState());
  //inputValues();           //Debug values for sensors
  //rotate360();
}
void testServos(int num){
  for(int i=0;i<num;i++){
    rotate360();
  }
  delay(100000);
}
//Servo Functions=============================================================================================================
void setUp() {
  myServo.write(95);
  delay(10);
  myServo2.write(95);
  delay(10);
}
void rotateLeft() {
  myServo.write(rotate360Speed);
  delay(rotate360Time);
  setUp();
}
void rotateRight() {
  myServo2.write(rotate360Speed);
  delay(rotate360Time);
  setUp();
}
void rotate360() {
  myServo.write(rotate360Speed);
  myServo2.write(rotate360Speed);
  delay(rotate360Time);
  setUp();
}
void loadRing(){
  myServo.write(150);
  myServo2.write(150);
  delay(rotate360Time);
  setUp();
}

void ringDrop() {
  if (ringArray[ringState] == 0) {
    rotate360();
  }
  else if (ringArray[ringState] == 1) {
    rotateRight();
  }
  else if (ringArray[ringState] == 2){
    rotateLeft();
  }
  else {
  } 
  ringState++;
}
//Sensor Functions============================================================================================================
int isOnLine(uint8_t sensorIn) {
  if(analogRead(sensorIn) > 500) {
    return 1;
  } else {
    return 0;
  }
}
/*
  Taking the position of each of the 5 sensors into account, I arranged the
  function to produce a binary digit that is being created as base 2 but is
  stored as its base 10 value. This allows us to produce a single variable
  state for our entire array of line sensors as opposed to having to create
  a for loop that would ruin the continuity of multi tasking for the robot.
*/
int sensorState() {
  int currentState = 0;
  currentState = ((isOnLine(Pin11) * 16) + (isOnLine(Pin12) * 8) + (isOnLine(Pin13) * 4) + (isOnLine(Pin14) * 2) + isOnLine(Pin15));
  //Serial.println(currentState);
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
  
  if (startTrack) {
      forward();
  }
      //Serial.println("Start Area");
  }
  if (state == 7) {   //00111
      //stopMotors();
      gotTurn();
      //Serial.println("Left turn");
  }
  if (state == 28) {   //11100
      //stopMotors();
      gotTurn();
      //rightTurn();
      //Serial.println("Right turn");
      //lastTurn == state;
  }
  if (state == 31) {   //11111 second time we encounter this it has to be a right turn
      //forward();
      gotTurn();
      //stopMotors();
  }
  //Correction routines------------------------------------

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
void corrections(int state) {
  if (state == 4) {    //00100
      forward();
  }
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
void gotTurn() {
  //still have to add failsafes, if there is a bad reading it doesn't throw the whole track off?
  if(startTrack){
    startTrack=false;
    forward();
    delay(65);
    return;
  }
  if (pathArray[pathState] == 0) {
    //go forward
    if (pathState == 10) {
       startTime = millis();
       while ((millis() - startTime) <= 2120/batteryPower) {
         //forward();
         corrections(sensorState());
       }
       stopMotors();
       middleTurn();
       forward();
       delay(90);
       stopMotors();
       ringDrop();
       backward();
       delay(90);
       undoTurn();
       forward();
       startTime=millis();
       while ((millis() - startTime) <= 1600/batteryPower) {
         corrections(sensorState());
       }
       
    }
    if (pathState == 1) {
      forward();
      delay(100);
    }
    else {
      forward();
      //delay(30);
    }
    //Serial.println("Foward");
  }
  else if (pathArray[pathState] == 1) {
    if (pathState ==0) startTime= millis();
    else if (pathState ==2){
      batteryPower = millis()-startTime;
      Serial.print(batteryPower);
      batteryPower = 1;  //(millis()-startTime)/timeLong;
    } 
    stopMotors();
    backward();
    delay(20);
    stopMotors();
    ringDrop();
    leftTurn();
    //Serial.println("Left Path called");
  }
  else if (pathArray[pathState] == 2){
    if (pathState != 5) {
      stopMotors();
      backward();
      delay(40);
      stopMotors();
      rightTurn();
    }
    if (pathState == 5) {
     forward();
     delay(120);
     stopMotors();
     ringDrop();
     backward();
     delay(240);
    }
    //Serial.println("Right Path called");
  }
  else {
    stopMotors();
    delay(1000000);
  }
  pathState++;
  if (pathState >= 15) {
   forward();
   delay(450);
   stopMotors();
   delay(1000000);
  }
}
//END gotTurn()-------------------------------------------------------------------------------------
void forward() {  //Moves the robot forward
  md.setM1Speed(400); //right side                 390
  md.setM2Speed(-400);//left side                 -400
}
void backward() { //Moves the robot backwards
  md.setM1Speed(-400); //right side -400
  md.setM2Speed(400);//left side    400
}
void leftTurn() { //Turns the robot left from the center|| both have to be positive
  md.setM2Brake(300);
  md.setM1Brake(300);
  delay(5);
  md.setM1Speed(200); //400
  md.setM2Speed(60);  //80
  delay(400);// original   610
}
void rightTurn() { //Turns the robot right from the center|| both have to be negative
  md.setM2Brake(300);
  md.setM1Brake(300);
  delay(5);
  md.setM1Speed(-60); //Forward right side    -80
  md.setM2Speed(-200);//Reverse left side     -400
  delay(400);
}
void slightLeft() {
  //md.setM2Brake(300);
  //md.setM1Brake(300);
  //delay(5);
  md.setM1Brake(300);
  md.setM2Speed(-200);//left side  // -200
  //delay(1);
}
void slightRight() {
  //md.setM2Brake(300);
  //md.setM1Brake(300);
  //delay(5);
  md.setM1Speed(200); //right side //  200
  md.setM2Brake(300);
  //delay(1);
}
void middleTurn() {
  md.setM1Brake(200); //right side //  200
  md.setM2Speed(-200);
  delay(700);
}
void undoTurn() {
  md.setM1Speed(-200); //right side //-40           -50
  //md.setM2Speed(-55);//left side  // -55           -75
  md.setM2Brake(200);
  delay(700);
}
void stopMotors() {//Stop the robot
  md.setM1Brake(300);
  md.setM2Brake(300);
}
float motorRatio(int power) {
  return (power - power * ratio);
}
//END OF MOTOR FUNCTIONS===============================================================
void inputValues() {
  Serial.print("Sensor 1: ");
  Serial.print(isOnLine(Pin11));   // Connect to pin 2, display results
  Serial.print(", Sensor 2: ");
  Serial.print(isOnLine(Pin12));   // Same for these
  Serial.print(", Sensor 3: ");
  Serial.print(isOnLine(Pin13));
  Serial.print(", Sensor 4: ");
  Serial.print(isOnLine(Pin14));
  Serial.print(", Sensor 5: ");
  Serial.print(isOnLine(Pin15));
  Serial.println();
}
