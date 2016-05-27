//TIMERS LIBRARY LINK: https://github.com/JChristensen/Timer
                       You need timer.cpp/h which also needs event.cpp/h
#include <Servo.h>
#include <EEPROM.h>
#include "Timer.h"
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
//========================================SETTINGS======================================================
bool timeTrack = true;
int mdFast = 400; //not near a turn
int mdSlow = 200; //near a turn
//========================================GLOBAL DEFINES======================================================
int path[16] = {1, 0, 1, 1, 2, 2, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1};
int pathLength=16;
int ringArray[13] = {3, 3, 1, 2, 1, 3, 0, 0, 2, 0, 0, 0, 3};  //0=nothing, 1=left, 2=right, 3=both
ServoInfo servo1 = {32, 52, 1258, 95};
ServoInfo servo2 = {34, 52, 1258, 95};

//int sensorPins[5] = {22,24,26,28,30};
int sensorPins[5] = {11,12,13,14,15};
bool sensor[5] = {false, false, false, false, false};
int numSensors=5;
Timer t;
int eventID = -1;
unsigned long timeArray[16];
unsigned long msec;
//int path[16] = {1, 0, 1, 1, 2, 2, 1, 1, 1, 2, 1, 1, 1, 0, 1, 1};  //0 = straight, 1 = left, 2 = right
int pathState, ringState, loopCounter, lastAction = 0;

int mdPower = 400; //full power for the motor
int mdRFlip = 1;
int mdLFlip = -1;
float mdLOffset = .98;//.915; //.8233 on May 5th in Ricardo's code; 
//.78~ Offset for low power levels

//SETUP=======================================================================================================
void setup() {
  Serial.begin(9600);
  initialize();
  
  t.every(20, mdCorrectForward); //every 20ms check for forward corrections
  
  if(timeTrack){ //timing the track, so always check for turns
    startTurnChecks();
    msec=millis();
  }
  else{ //not timing the track, so go fast untill it is time to check for turns
    stopTurnChecks();
    for(int i=0;i<pathLength;i++){
      timeArray[i]=(EEPROM.read(i)/2.1)-300; //timed when going 200 power, so must convert to 400 power
    }
    t.after(timeArray[pathState], startTurnChecks); //when near the next turn, start turn checking
  }
}

void initialize(){
  md.init();
  myServo1.attach(servo1.pinNum);
  myServo2.attach(servo2.pinNum);
  myServo1.write(servo1.stopSpeed);
  myServo2.write(servo2.stopSpeed);
  //testForward();
  //testSensors();
  waitTillPlacedOnLine();
}

void waitTillPlacedOnLine(){
  while(1){  
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
}

//MAIN LOOP===================================================================================================
//timer, every XXms it is running mdCorrectForward() and after XXms runs startTurnChecks (when it is close to a turn)
void loop() {
  t.update();
}

void checkTurn(){
  if(path[pathState]==2){
    if(!isRightTurn()) return; //no turn, so leave method
  }
  else{
    if(!isLeftTurn()) return; //no turn, so leave method
  }
  //found turn
  if(timeTrack) storeTime(pathState, millis()-msec);  //store time-to-turn
  
  mdTurn();                                           //turn
  lastAction=0;
  pathState++;
  
  if(!timeTrack){                                     //fallow the line untill the timer thinks you are near the next turn
    stopTurnChecks();
    t.after(timeArray[pathState], startTurnChecks);
  }
  else msec=millis();
  
  if(pathState>=pathLength-5){ //done with path
    finishTrack();
  }
}

bool isRightTurn(){
  //right turn is next, so check sensor[4]
  readSensor(4);
  if(sensor[4]){
    mdForward(-.05);
    //if it is pinging, double check it and the other turn sensors.
    for(int i =0;i<2;i++){
      //readSensor(2);
      //if(!sensor[2] and lastAction !=1) return false;
      readSensor(3);
      if(!sensor[3]) return false;
      readSensor(4);
      if(!sensor[4]) return false;
    }
  }
  else return false;
  return true;
}
bool isLeftTurn(){
  //next turn is left or straight.
  readSensor(0);
  if(sensor[0]){
    mdForward(-.05); //need to change this to go slightly clockwise
    //mdRotate(-.22);
    for(int i =0;i<2;i++){
      //readSensor(2);
      //if(!sensor[2] and lastAction != -1) return false;
      readSensor(1);
      if(!sensor[1]) return false;
      readSensor(0);
      if(!sensor[0]) return false;
    }
  }
  else return false;
  return true;
}

void mdTurn(){
  //starts the motor on the next pathState
  if(path[pathState]==0){  //go forward
    mdForward(1);
    delay(((250*200)/(mdPower))); //go forward for...
  }
  else{  //turn
    if(path[pathState]==1){ // left
      mdForward(-.05);
      ringDrop();
      mdLeft(-.26);
    }
    else if(path[pathState]==2){ // right
      if (pathState == 5)
      {
        mdForward(1);
        delay(200);
        mdForward(-.05);
        ringDrop();
        mdForward(-1);
        delay(200);
        }
      mdRight(-.26);
    }
    delay(((900*200)/(mdPower))); // turn for...
  }
}

void mdCorrectForward(){
  //it has already been determined that there is no turn so just correct to stay on the line.
  
  if(lastAction==0){
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
        lastAction++;
        mdLeft(.7);
      }
      //right correction sensor is reading
      else if(sensor[3] and !sensor[1]){
        Serial.println("Correct Right");   
        lastAction--; 
        mdRight(.7);
      }
    }
  }
  else if(lastAction>0){
    readSensor(1);
    if(!sensor[1]){
      mdRight(.7);
      lastAction--;
      if(lastAction!=0)lastAction--;
    }
    else lastAction++;
  }
  else{
    readSensor(3);
    if(!sensor[3]){
      mdLeft(.7);
      lastAction++;
      if(lastAction!=0)lastAction++;
    }
    else lastAction--;
  }
}

void startTurnChecks(){
  mdBrake(.3);
  mdPower=mdSlow;
  eventID = t.every(5, checkTurn);
}
void stopTurnChecks(){
  mdPower=mdFast;
  t.stop(eventID);
}

void finishTrack(){
  Serial.print("PATHSTATE DONE");
  mdForward(1);
  delay(500);
  mdForward(-.05);
  delay(50000);
}

void storeTime(int address, int t){
  EEPROM.write(address, t);
}

//=========================================SERVOS========================
void ringDrop() {
  if (ringArray[ringState] != 0) {
    if (ringArray[ringState] == 1) {
      myServo2.write(servo2.turnSpeed); //start left servo
      delay(servo2.turnTime);
    }
    else{
      myServo1.write(servo1.turnSpeed); //start right servo
      if(ringArray[ringState] == 3){
        myServo2.write(servo2.turnSpeed); //start both servos
      }
      delay(servo1.turnTime);
    }
    stopServos();
    ringState++;
  }
}

void stopServos() {
  myServo1.write(servo1.stopSpeed);
  delay(10);
  myServo2.write(servo2.stopSpeed);
  delay(10);
}
//===========================================SENSORS===========================
void printSensors(){
  for(int i=0;i<5;i++){
    Serial.print(sensor[i]);
  }
  Serial.println("");
}
void readSensor(int num){
  sensor[num] = sensorState(sensorPins[num]);
}
//read everything
void readSensors(){
  for(int i=0;i<5;i++){
    sensor[i] = sensorState(sensorPins[i]);
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
bool sensorState(int sensorIn){
  //resetSensor(sensorIn);
  if(analogRead(sensorIn)>500){     // Pin is not getting reflection, which means it is on the black line
    return true;
  }
  else{
    return false;
  }
}

//=============================================MOTOR FUNCTIONS==========================================
//v signals how much power should go to the Right or Left engine.  Usually set to (-.05)->(.7) starting a turn or mild correction.
void mdRight() {
    //md.setM1Brake(400);
    //md.setM2Brake(0);
    md.setM1Speed(mdRFlip*mdPower*(-.26));
    md.setM2Speed(mdLFlip*(mdPower*mdLOffset));
}
void mdLeft() {
    //md.setM1Brake(0);
    //md.setM2Brake(400);
    md.setM1Speed(mdRFlip*mdPower);
    md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*(-.26));
}
void mdRight(float v) {
    md.setM1Brake(400*v);
    //md.setM2Brake(0);
    //md.setM1Speed(mdRFlip*mdPower*v*y);
    md.setM2Speed(mdLFlip*(mdPower*mdLOffset));
}
void mdLeft(float v) {
    //md.setM1Brake(0);
    md.setM2Brake(400*v);
    md.setM1Speed(mdRFlip*mdPower);
    //md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*v*y);
}
void mdBrake(float v){
  md.setM1Brake(400*v);
  md.setM2Brake(400*v);
}
void mdRotate(float v){ //stop one motor, go slightly backwards with the other
  if(v>0){  //stop right, go backwards with left
    v*=-1;
    md.setM1Speed(mdRFlip*mdPower*v);
    md.setM2Speed(mdLFlip*-.05);
  }
  else{  //stop left, go backwards with right
    md.setM1Speed(mdRFlip*-.05);
    md.setM2Speed(mdLFlip*mdPower*v);
  }
}
void mdForward(float v) {
    //md.setM1Brake(0);
    //md.setM2Brake(0);
    md.setM1Speed(mdRFlip*mdPower*v);
    md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*v);
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

   5/23 can set up the framework for getting path times.  EEPROM.read(address), EEPROM.write(address, int) 
   millis() --- add timer to his delay after pathState 5 stuff and put mdCorrect in a while loop until the appropriate time is reached
 */

