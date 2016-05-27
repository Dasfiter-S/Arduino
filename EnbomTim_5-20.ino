/*
 * BUGS
 * 
 * Stopped in betweens the 2nd and 3rd sensors and goes rotates counter clockwise.
 *  Could be the stop command isn't entirely stopped.
 *turned too late, it caught the turn but hit the wall
 * 
 * Possible make a lastCorrect, don't resend goForward if already forward
 * 
 * 5/13
 * ---Changed delay to before correction check, moved stop to right as 0 or 4 is spotted
 * ---Changed, doesn't check for 2 in turn code anymore
 * ---maybe if it gets a [0] or [4] reading go back a lil in that direction.  Or I can implement the slower around turn time code.  Measure it going full speed from one to another, make an array that starts the code right before.
 * ----now will not correct if multiple sensor readings [3] and [1] at the same time or [0] and [4] since that is impossible short of a turn.  
 * can get off track during 'go forward past turn' (1)                (code error)
 * better code for only sensor [0] or [4] reading (3)                 (code error)
 * does a turn right after coming off a turn (3)                      (code error)
 * missed a turn!!! went straight over it (4)                         (update time/sensor error)
 * got completely off line after doing a right correction. (1)        (update time/sensor error)
 * went into a turn at angled so 0 or 4 sens missed (2)               (update time/sensor error)
 *   only rightmost sensor is reading and it is trying to go left     (sensor error)
   random readings of left correct or turn                            (sensor error)

   5/20
   turns missed 2/40, 1/30
   gap between sensors        need to add 'last direction' for what it does in gaps
   
 */


#include <Servo.h>
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
Servo myServo1;
Servo myServo2;
//=========================================STRUCTS=============================================================
struct ServoInfo{
  int pinNum;
  int turnSpeed;
  int turnTime;
  int stopSpeed;
};
//========================================GLOBAL DEFINES======================================================
//int sensPins[5] = {22,24,26,28,30};
int sensPins[5] = {11,12,13,14,15};
bool sensor[5] = {false, false, false, false, false};
int numSensors=5;
//bool dropRing[11][2]={{true, true},{true, true}, {false, true}, {false, true}, {true, true}, {false, false}, {false, true}, {false, true}, {false, false}, {false, false}, {false, true}};
ServoInfo servo1 = {32, 52, 1258, 95};
ServoInfo servo2 = {34, 52, 1258, 95};

//int path[16] = {1, 0, 1, 1, 2, 2, 1, 1, 1, 2, 1, 1, 1, 0, 1, 1};  //0 = straight, 1 = left, 2 = right
int path[16] = {1, 0, 1, 1, 2, 2, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1};
int pathLength=16;

int pathState, loopCounter, lastActionCounter = 0;
int lastAction = 0;

int mdFull = 400; //full power for the motor
int mdRFlip = 1;
int mdLFlip = -1;
float mdLOffset = .98;//.915; //.8233 on May 5th in Ricardo's code; 
//.78~ Offset for low power levels

//SETUP=======================================================================================================
void setup() {
  Serial.begin(9600);
  md.init();
  myServo1.attach(servo1.pinNum);
  myServo2.attach(servo2.pinNum);
  myServo1.write(servo1.stopSpeed);
  myServo2.write(servo2.stopSpeed);
  //testForward();
  //testStraight1();
  //testSensors();
  //testSensorCode();
  while(1){  //do not start the loop() code until the robot is placed on the line, helps with debugging pathState
    readSensor(3);
    if(!sensor[3]){
      readSensor(1);
      readSensor(2);
      if(sensor[2] and !sensor[1]){
        Serial.println("on line, starting code");
        break;
      }
    }
    delay(1000);
    printSensors();
  }
  rotateServo(); //unlatches the ring arm.
}

//MAIN LOOP===================================================================================================
//checks the turn sensor (4 if the next turn is right, 0 if the next turn is left) every 33ms.
//checks for on-line corrections every 100ms
void loop() {
  //md in a method's name means it is going to change motor direction.
  loopCounter++;
  if(isTurn()){
    Serial.print("isTurn:true:");
    Serial.print(path[pathState]);
    mdPathState();  //get the motors going in the correct direction (left, right or forward)

    if(path[pathState]==0){ //if forward
      delay(250/(mdFull/200)); //go forward for...
    }
    else{ //left or right
      delay(900/(mdFull/200)); // turn for...
    }
    //mdForward(1);
    loopCounter=3; // must straighten out.  So go to mdCorrectForward right away.
    lastAction=0;
    pathState++;  
    if(pathState>pathLength-5){ //done with path
      Serial.print("PATHSTATE DONE");
      mdForward(1);
      delay(500);
      mdForward(-.05);
      delay(50000);
    }
    Serial.println(":FinishedTurn");
  }
  delay(4);
  //every 3 loops, check if you need forward corrections
  if(loopCounter>=3){
    
    loopCounter=0;
    mdCorrectForward();
  }
}
//checks the one sensor that would signal for the next turn (either sensor 4 or sensor 0)
bool isTurn(){
  if(path[pathState]==2){
    //right turn is next, so check sensor[0v]
    readSensor(4);
    if(sensor[4]){
      mdRotate(.22);
      //mdForward(-.05); // need to change this to go slightly counter clockwise
      //if it is pinging, double check it and the other turn sensors.
      for(int i =0;i<1;i++){
        readSensor(2);
        if(!sensor[2] and lastAction !=1) return false;
        readSensor(3);
        if(!sensor[3]) return false;
        readSensor(4);
        if(!sensor[4]) return false;
      }
    }
    else return false;
  }
  else{
    //next turn is left or straight.
    readSensor(0);
    if(sensor[0]){
      //mdForward(-.05); //need to change this to go slightly clockwise
      mdRotate(-.22);
      for(int i =0;i<1;i++){
        //readSensor(2);
        if(!sensor[2] and lastAction != -1) return false;
        readSensor(1);
        if(!sensor[1]) return false;
        readSensor(0);
        if(!sensor[0]) return false;
      }
    }
    else return false;
  }
  return true;
}

void mdPathState(){
    //starts the motor on the next pathState
    if(path[pathState]==1){ //next turn is a left
      //if(pathState != xxx) to not drop rings at certain left turns
      mdForward(-.05);
      rotateServo();
      mdLeft(-.26);
    }
    else if(path[pathState]==2){ //next turn is a right
        mdRight(-.26);
    }
    else{
      mdForward(1);
    }
}

void stopServos() {
  myServo1.write(servo1.stopSpeed);
  delay(10);
  myServo2.write(servo2.stopSpeed);
  delay(10);
}
void rotateServo() {
  myServo1.write(servo1.turnSpeed);
  myServo1.write(servo2.turnSpeed);
  delay(servo1.turnTime);
  stopServos();
}

void mdCorrectForward(){
  //it has already been determined that there is no turn so just correct to stay on the line.
  
  readSensor(2);

  if(sensor[2]){
    mdForward(1);
    lastAction = 0;
  }
  else{
    readSensor(3);
    readSensor(1);
    //left correction sensor is reading
    if(sensor[1] and !sensor[3]){
      Serial.println("Correct Left");
      lastAction = 1;
      mdLeft(-.6,1);
      lastActionCounter=0;
    }
    //right correction sensor is reading
    else if(sensor[3] and !sensor[1]){
      Serial.println("Correct Right");   
      lastAction = -1; 
      mdRight(-.6,1);
      lastActionCounter=0;
    }
    else{
      readSensor(0);
      readSensor(4);
      //far off the line.  FIX NEEDED.  It must go backwards and re-find the line at this point.  So as to not miss a turn.
      if(sensor[0] and !sensor[4]){
        mdLeft(-.2);
        lastAction = 2;
      }
      else if(sensor[4] and !sensor[0]){
        mdRight(-.2);
        lastAction = -2;
      }
      else if(lastAction==0){
        mdForward(1);
      }
      else if(lastAction==1){
        mdRight(-.6);
        lastAction=0;
      }
      else if(lastAction==-1){
        mdLeft(-.6);
        lastAction=0;
      }
      //no sensors reading
      else{
        mdForward(-.05);
      }
    }
  }
}

void printSensors(){
  for(int i=0;i<5;i++){
    Serial.print(sensor[i]);
  }
  Serial.println("");
}
void readSensor(int num){
  sensor[num] = sensorState(sensPins[num]);
}
//read everything
void readSensors(){
  for(int i=0;i<5;i++){
    sensor[i] = sensorState(sensPins[i]);
  }
}
void resetSensor(int sensorIn){
  pinMode(sensorIn, OUTPUT);     // Make pin OUTPUT
  if (sensorIn == 22)  PORTA |= _BV(PA0);
  else if (sensorIn == 24)  PORTA |= _BV(PA2);
  else if (sensorIn == 26)  PORTA |= _BV(PA4);
  else if (sensorIn == 28)  PORTA |= _BV(PA6);
  else if (sensorIn == 30)  PORTC |= _BV(PC7);
  pinMode(sensorIn, INPUT);      // Make pin INPUT
  if (sensorIn == 22)  PORTA &= ~_BV(PA0);
  else if (sensorIn == 24)  PORTA &= ~_BV(PA2);
  else if (sensorIn == 26)  PORTA &= ~_BV(PA4);
  else if (sensorIn == 28)  PORTA &= ~_BV(PA6);
  else if (sensorIn == 30)  PORTC &= ~_BV(PC7);
}
bool sensorState(int sensorIn){       //
  //resetSensor(sensorIn);
  if(analogRead(sensorIn)>500){     // Pin goes HIGH which means it is on the black line
    return true;
  }
  else{
    return false;
  }
}

//MOTOR FUNCTIONS=============================================================================================
//v signals how much power should go to the Right or Left engine.  Usually set to (-.05)->(.7) starting a turn or mild correction.
void mdRight(float v) {
    //md.setM1Brake(400);
    //md.setM2Brake(0);
    md.setM1Speed(mdRFlip*mdFull*v);
    md.setM2Speed(mdLFlip*(mdFull*mdLOffset));
}
void mdLeft(float v) {
    //md.setM1Brake(0);
    //md.setM2Brake(400);
    md.setM1Speed(mdRFlip*mdFull);
    md.setM2Speed(mdLFlip*(mdFull*mdLOffset)*v);
}
void mdRight(float v, float y) {
    md.setM1Brake(300);
    //md.setM2Brake(0);
    //md.setM1Speed(mdRFlip*mdFull*v*y);
    md.setM2Speed(mdLFlip*(mdFull*mdLOffset)*y);
}
void mdLeft(float v, float y) {
    //md.setM1Brake(0);
    md.setM2Brake(300);
    md.setM1Speed(mdRFlip*mdFull*y);
    //md.setM2Speed(mdLFlip*(mdFull*mdLOffset)*v*y);
}
void mdRotate(float v){ //stop one motor, go slightly backwards with the other
  if(v>0){  //stop right, go backwards with left
    v*=-1;
    md.setM1Speed(mdRFlip*mdFull*v);
    md.setM2Speed(mdLFlip*-.05);
  }
  else{  //stop left, go backwards with right
    md.setM1Speed(mdRFlip*-.05);
    md.setM2Speed(mdLFlip*mdFull*v);
  }
}
void mdForward(float v) {
    //md.setM1Brake(0);
    //md.setM2Brake(0);
    md.setM1Speed(mdRFlip*mdFull*v);
    md.setM2Speed(mdLFlip*(mdFull*mdLOffset)*v);
}

//TEST FUNCTIONS===============================================================
void testTurn(){
  while(1){
    mdLeft(.7);
    delay(100);
    mdRight(.7);
    delay(200);
    mdLeft(.7);
    delay(100);
  }
}
void testSensors(){
  while(1){
    /*
    loopCounter++;
    isTurn();
    if(loopCounter<40){
      mdForward(1);
    }
    else if(loopCounter<45){
      mdLeft(.95);
    }
    else{
      loopCounter=0;
    }
    */
    readSensors();
    printSensors();
    delay(30);
    
  }
}
void testForward(){
  md.setM1Brake(0);
  md.setM2Brake(0);
  while(1){
    mdForward(1);
    //delay(200);
    //mdRotate(-.22);
    delay(20000);
  }
}
void testStraight1(){
  int accelerating = true;
  int lastSpeed = 0;
  float accCorrection = 1.3;
  float accCorrectionIncrement = .06;
  md.setM1Brake(0);
  md.setM2Brake(0);
  while(1){
    if(accelerating){
      mdLeft(accCorrection);
      accCorrection-=accCorrectionIncrement;
      if(accCorrection<=1) accelerating = false;
      delay(63);
    }
    else{
      mdForward(1);
      delay(1000);
      mdForward(0);
      md.setM1Brake(400);
      md.setM2Brake(400);
    }
  }
}

void testStraight2(){
  //when going straight, use a different speed
  int accelerating = true;
  float accCounter = 8;
  md.setM1Brake(0);
  md.setM2Brake(0);
  while(1){
    if(accelerating){
      mdLeft(1.15);
      accCounter--;
      if(accCounter==0) accelerating = false;
      else delay(63);
    }
    else{
      mdForward(1);
      delay(1000);
      mdForward(0);
      md.setM1Brake(400);
      md.setM2Brake(400);
    }
  }
}

void testTrack(){
   mdForward(.5);
  while(1){
    readSensors();
    printSensors();
    if(sensor[3] or sensor[1]){
      mdForward(-.05);
      delay(100);
      mdForward(.5);
    }
    delay(500);
  }
}
void testSensorCode(){
  int LED = 13; // Use the onboard Uno LED
  int isObstaclePin = sensor[0];  // This is our input pin
  int isObstacle = HIGH;  // HIGH MEANS NO OBSTACLE

  pinMode(LED, OUTPUT);
  pinMode(isObstaclePin, INPUT);
  Serial.begin(9600);

  while(1){
    for(int i=11;i<=15;i++){
      Serial.print(analogRead(i));
      Serial.print(":");
    }
    Serial.println("");
    
    delay(200);
  }
}


