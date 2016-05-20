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
int startTrack = 0;
int rotate360Speed = 40; // 1.29s per rotation; 12.91s for 10 rotations
int rotate360Time = 1376; // off by a fraction of a degree.
int rotateStopSpeed = 92;
int Pin32 = 32;
int Pin34 = 34;
float ratio = 0.166;
int lastTurn = 0;

int pathState = 0;
int pathArray[16] = {1, 0, 1, 1, 2, 2, 1, 1, 1, 2, 1, 1, 1, 0, 1, 1}; //1 = left, 0 straight, 2 right
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
  drive(sensorState());
  //startRace();
  //readScreen();
  //inputValues();           //Debug values for sensors
  //rotate360();
}
void startRace() {
  if (startTrack == 1) {
    drive(sensorState());
    //Serial.println("Driving");
  }
  else {
    stopMotors();
    pathState = 0;
  }
}
//Touch Screen functions======================================================================================
void readScreen() {
  String message = myNextion.listen();
  if (message == "65 0 1 1 ffff ffff ffff") {
    //Serial.println("Stoping...");
    startTrack = 0;
    myNextion.setComponentText("tTimeD", String(static_cast<float>(millis())/1000));
  }
  if (message == "65 0 2 1 ffff ffff ffff") {
    //Serial.println("Starting run!");
    startTrack = 1;
  }
  if (message == "65 0 3 1 ffff ffff ffff") {
    //Serial.println("Loading rings");
    loadRing();
  }
  /*
  if (message == "65 0 4 1 ffff ffff ffff") {
    //myNextion.buttonToggle(button4State, "b1", 0, 2);
  }
  */
  if (message == "65 1 2 1 ffff ffff ffff") {
    myNextion.setComponentText("tTimeD", String(static_cast<float>(millis())/1000));
  }
  /*
  if (message == "65 1 1 1 ffff ffff ffff") {
    //myNextion.buttonToggle(button6State, "b1", 0, 2);
  }
  */
  /*
  myNextion.setComponentText("timeD", String(static_cast<float>(millis())/1000));
  myNextion.setComponentText("ringD", String(value++));
  myNextion.setComponentText("turnNextD", "Left");
  myNextion.setComponentText("turnLastD", "Blank");
  */
}
//End Touch screen functions================================================================================== 
//Servo Functions=============================================================================================================
void setUp() {
  myServo.write(95);
  delay(10);
  myServo2.write(95);
  delay(10);
}
void rotate360() {
  myServo.write(rotate360Speed);
  myServo2.write(rotate360Speed);
  delay(rotate360Time);
  setUp();
}
void loadRing(){
  myServo.write(144);
  myServo2.write(144);
  delay(rotate360Time);
  setUp();
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
      //forward();
      stopMotors(); 
      //backward();
      //Serial.println("Start Area");
  }
  if (state == 7) {   //00111
      stopMotors();
      gotTurn();
      //Serial.println("Left turn");
  }
  /*
  if (state == 3) {   //00011
      stopMotors();
      gotTurn();
      //Serial.println("Left turn");
  }
  */
  if (state == 28) {   //11100
      stopMotors();
      gotTurn();
      //rightTurn();
      //Serial.println("Right turn");
      //lastTurn == state;
  }
  /*
  if (state == 24) {   //11000
      stopMotors();
      gotTurn();
      //Serial.println("Right turn");
  }
  */
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
//MOTOR FUNCTIONS=============================================================================================
void gotTurn() {
  //still ahve to add failsafes, if there is a bad readibng it doesn't throw the whole track off?
  if (pathArray[pathState] == 0) {
    //go forward
    forward();
    //delay(50);
    //if (pathArray[pathState-1]!= 0) //not sure if this works. It is almost always true
    //Serial.println("Foward Path  called");
  }
  else if (pathArray[pathState] == 1) {
    stopMotors();
    //rotate360();
    leftTurn();
    //Serial.println("Left Path called");
  }
  else {
    rightTurn();
    //Serial.println("Right Path called");
  }
  pathState++;
}
void forward() {  //Moves the robot forward
  md.setM1Speed(400); //right side                 300 or  200
  md.setM2Speed(-(motorRatio(400)));//left side //more powerful -247 or -165
  //delay(10);
}
void backward() { //Moves the robot backwards
  md.setM1Speed(-300); //right side 300
  md.setM2Speed(247);//left side    247
}
void leftTurn() { //Turns the robot left from the center|| both have to be positive
  md.setM1Speed(400); //Forward right side 400
  md.setM2Speed(20);//Reverse left side     10
  //md.setM2Brake(300);
  delay(630);
}
void rightTurn() { //Turns the robot right from the center|| both have to be negative
  md.setM1Speed(-20); //Forward right side                  400
  //md.setM1Brake(300);
  md.setM2Speed(-(motorRatio(400)));//Reverse left side     10
  delay(630);
}
void slightLeft() {
  //md.setM1Speed(55); //right side //55             75
  md.setM1Brake(300);
  md.setM2Speed(-(motorRatio(400)));//left side  // 45             65
}
void slightRight() {
  md.setM1Speed(400); //right side //-40           -50
  //md.setM2Speed(-55);//left side  // -55           -75
  md.setM2Brake(300);
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
/*
void testSensors(){
  while(1){
    inputValues();
    delay(300);
  }
}
*/
