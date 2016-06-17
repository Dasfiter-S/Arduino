/*
 * BUGS
 * 
 * Stopped in betweens the 2nd and 3rd sensors and goes rotates counter clockwise.
 *  Could be the stop command isn't entirely stopped.
 *turned too late, it caught the turn but hit the wall
 * 
 * Possible make a lastCorrection, don't resend goForward if already forward
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
   millis() --- add timer to his delay after turnState 5 stuff and put mdCorrect in a while loop until the appropriate time is reached
 */


#include <Servo.h>
#include <EEPROM.h>
#include "DualVNH5019MotorShield.h"
#include "Timer.h"
DualVNH5019MotorShield md;
//=========================================STRUCTS=============================================================
struct ServoInfo{
  int pinNum;
  int turnSpeed;
  int turnTime;
  int stopSpeed;
};
#define none 0
#define left 1
#define right 2
#define both 3
//========================================SETTINGS======================================================
bool timing = false;
int mdFast = 400; //not near a turn
int mdSlow = 250; //near a turn
//========================================VARS======================================================
Servo servoL, servoR;
int servoSpeed = 40; // 1.29s per rotation; 12.91s for 10 rotations
int servoTime = 1260; // off by a fraction of a degree.
int servoStop = 92;
int servoState = 0;
int servoArray[] = {both, both, right, left, right, both, none, right, none, both}; //0 both arms, 1 right, 2 left, 3 none
//int sensorPins[5] = {11,12,13,14,15};
//bool sensor[5] = {false, false, false, false, false};
//int numSensors=5;

int sensorPins[5] = {11, 12,13,14,15};
bool sensor[5] = {false, false, false, false};
int numSensors=5;

int turnState = 0;  //quadrant 1| quadrant 2    | center 
int turnArray[14] = {left, none, left, left, right, right, right, left, left, left, none, none, left, left}; //1 = left, 0 straight, 2 right
int numTurns = 14;
bool startingZone, turnChecks;

unsigned long correctMS, turnMS, t =0;
unsigned long timeArray[14];
Timer loopTimer;

int mdPower = mdFast; //full power for the motor
int mdRFlip = 1;
int mdLFlip = -1;
float mdLOffset = 1;//.98;//.915; //.8233 on May 5th in Ricardo's code; 
float mdROffset = 1;
//.78~ Offset for low power level

//SETUP=======================================================================================================
void setup() {
  Serial.begin(9600);
  initialize();
  //testSensors();
  //mdPower = mdSlow;
  //startingZone = turnChecks = true;
  
  //waitTillPlacedOnTrack();
  //gotTurn();
  //while(1) mdCorrectForward();
}

void initialize(){
  md.init();
  servoL.attach(32);
  servoR.attach(34);
  stopServos();
}

void waitTillPlacedOnTrack(){
  while(1){  
    readSensor(3);
    if(!sensor[3]){
      readSensor(0);
      readSensor(1);
      readSensor(2);
      //readSensor(4);
      if(!sensor[0] and sensor[1] and !sensor[2]){
        Serial.println("on track, starting code");
        startingZone = false;
        break;
      }
    }
    delay(1000);
    printSensors();
  }
}

void startTurnChecks(){
  mdPower = mdSlow;
  turnChecks= true;
}
void stopTurnChecks(){
  mdPower = mdFast;
  turnChecks= true;
}

//MAIN LOOP===================================================================================================
//timer, every XXms it is running mdCorrectmdSpeed(1, 1) and after XXms runs startTurnChecks (when it is close to a turn)
void loop() {
  readSensors();
  //printSensors();
  //delay(100);
  turnCheck();
  //mdSpeed(1,1);
  mdCorrectForward();
  //if(turnChecks) turnCheck();
  //else loopTimer.update(); //timer untill turnC flips to true
  
}

void turnCheck() {
  if (!sensor[0] and sensor[3] and sensor[4]) {   //0--11
      gotTurn();
  }
  else if (sensor[0] and sensor[1] and !sensor[4]) {   //11--0
      gotTurn();
  }
  else if (sensor[0] and sensor[2] and sensor[4]) {   //1-1-1 second time we encounter this it has to be a right turn
      gotTurn();
  }
}
bool leftS = false;
bool rightS = false;
bool centerS = false;
unsigned long sTime;
int brakeAmount = 40;
int counterC = 0;
int counter = 0;
bool turning = false;
void mdCorrectForward(){
  if(turning){
    if(sensor[2]) turning = false;
    else return;
  }
  if(sensor[1]){
    if(!leftS){
      Serial.println("leftS");
      leftS = true;
      rightS = false;
      counter = 0;
      if(centerS){
        mdBrake(.8,0);
        delay(brakeAmount/2);
        centerS = false;
      }
      sTime = millis();
      mdSpeed(.8,1);
    }
    else if(millis()-sTime>90){
      mdBrake(.3,0);
      delay(brakeAmount/3);
      sTime = millis();
    }
    else mdSpeed(1,1);
  }
  else if(sensor[3]){
    if(!rightS){
      Serial.println("rightS");
      leftS = false;
      rightS = true;
      counter = 0;
      if(centerS){
        mdBrake(0,.8);
        delay(brakeAmount/2);
        centerS = false;
      }
      sTime = millis();
      mdSpeed(1,.8);
    }
    else if(millis()-sTime>90){
      counterC--;
      mdBrake(0,.3);
      sTime = millis();
    }
    else mdSpeed(1,1);
  }
  else if(sensor[2]){
    mdSpeed(1,1);
    leftS = false;
    rightS = false;
    centerS = true;
    counter = 0;
    if(counterC<0){ 
      Serial.println("Counter Correct left");
      mdBrake(0,1);
      counterC*=-1;
      delay((brakeAmount*counterC)/20);
      counterC = 0;
    }
    else if(counterC>0){ 
      Serial.println("Counter Correct Right");
      mdBrake(1,0);
      delay((brakeAmount*counterC)/20);
      counterC = 0;
    }
    /*
    if(leftS){
      Serial.println("leftS");
      leftS = false;
      mdBrake(.5,0);
      delay(brakeAmount);
    }
    else if(rightS){
      rightS = false;
      mdBrake(0,.5);
      delay(brakeAmount);
    }*/
  }
  else if(rightS){
    Serial.println("OFF TRACK RIGHT");
    if(counter==3){
      sTime = millis();
      counterC = 30;
      mdSpeed(1,0);
      counter++;
    }
    else if(counter>3){
      if(millis()-sTime>300){
        sTime = millis();
        counterC += 10;
      }
    }
    else counter++;
  }
  else if(leftS){
    Serial.println("OFF TRACK LEFT");
    if(counter==3){
      sTime = millis();
      counterC = -30;
      mdSpeed(0,1);
      counter++;
    }
    else if(counter>3){
      if(millis()-sTime>300){
        counterC -= 10;
        sTime = millis();
      }
    }
    else counter++;
  }
}
/*
bool rightS, leftS, justTurned = false;
unsigned long sTime = 0;
int facing = 0;

void correct(int facing){
  if(facing>0){
    mdSpeed(1,0);
    mdBrake(0,1);
    delay(facing*10);
  }
  else{
    mdSpeed(0,1);
    mdBrake(1,0);
    delay(facing*-10);
  }
  delay(80);
  mdSpeed(1,1);
  delay(60);
  if(facing>0){
    mdSpeed(0,1);
    mdBrake(1,0);
  }
  else{
    mdSpeed(1,0);
    mdBrake(0,1);
  }
  delay(100);
}

void mdCorrectForward(){
  if(justTurned){
    if(sTime != 0){
      if(sensor[1]){
        leftS = true;
        rightS = false;
      }
      else if(sensor[2]){
        rightS = true;
        leftS = false;
      }
      else{
        sTime = millis() - sTime;
        if(sTime > 400) facing = 1;
        else if(sTime > 250) facing = 2;
        else if(sTime > 100) facing = 3;
        else facing = 5;
        sTime = 0;
        if(rightS) facing*=-1;
        correct(facing); // curve left/right then right/left
        justTurned = false;
        mdSpeed(1,1);
      }
    }
    else{
      if(sensor[1]) leftS = true;
      if(sensor[2])rightS = true;
      if(leftS and rightS){
        mdBrake(1,1);
        delay(500);
        sTime = millis();
        mdSpeed(1,1);
      }
    }
  }
  else if(sensor[1]){
    if(facing != -2){
      facing = -2;
      mdSpeed(1,1);
    }
  }
  else if(sensor[2]){
    if(facing != 2){
      facing = 2;
      mdSpeed(1,1);
    }
  }
  else correct(facing);
}
*/
void beforeTurn(){
  centerS = false;
  rightS = false;
  leftS = false;
  turning = true;
  counterC = 0;
  
  if(timing){
    turnMS = millis()-turnMS;
    Serial.println(turnMS);
    if(!startingZone) EEPROMWritelong(turnState*4, turnMS); //store time for last leg
    turnMS = millis(); //start timing next leg
  }
  else if(turnState != 5){ //super short leg
    stopTurnChecks();
    //loopTimer.after(timeArray[turnState], startTurnChecks);
  }
}

void gotTurn() {
  beforeTurn();
  //still have to add failsafes, if there is a bad reading it doesn't throw the whole track off?
  if(startingZone){
    startingZone=false;
    mdPower = mdFast;
    mdSpeed(1, 1);
    delay(65);
    return;
  }
  if (turnArray[turnState] == none) {
    //go forward
    if (turnState == 10) {
      mdSpeed(1, 1);
      delay(65);
       t = millis();
       while ((millis() - t) <= (2120-555)) {
         mdCorrectForward();
       }
       mdBrake(1, 1);
       middleTurn();
       mdSpeed(1, 1);
       delay(180);
       mdBrake(1, 1);
       ringDrop();
       mdSpeed(-1, -1);
       delay(190);
       undoTurn();
       mdBrake(1, 1);
       delay(80);
       mdSpeed(1, 1);
       t=millis();
       while ((millis() - t) <= 1600) {
         mdCorrectForward();
       }
       
    }
    if (turnState == 1) {
      mdSpeed(1, 1);
      delay(100);
    }
    else {
      mdSpeed(1, 1);
      //delay(30);
    }
    //Serial.println("Foward");
  }
  else if (turnArray[turnState] == left) {
    mdBrake(1, 1);
    if(turnState !=9 && turnState !=12) straighten();
    ringDrop();
    mdLeftTurn();
    delay(300);
    
    //Serial.println("Left Path called");
  }
  else if (turnArray[turnState] == right){
    if (turnState != 5) {
      //mdForward(-.05);
      mdRightTurn();
      delay(420);
    }
    if (turnState == 5) {
     mdSpeed(1, 1);
     delay(200);
     mdBrake(1, 1);
     ringDrop();
     mdSpeed(-1, -1);
     delay(140);//260 was working w/ new battery
     mdSpeed(-.5,-.5);
     delay(160);
     mdBrake(1,1);
     delay(80);
    }
    //Serial.println("Right Path called");
  }
  else {
    mdBrake(1, 1);
    delay(1000000);
  }
  turnState++;
  if (turnState >= 15) {
   mdSpeed(1, 1);
   delay(450);
   mdBrake(1, 1);
   delay(1000000);
  }
}

void straighten(){
  delay(300);
  mdSpeed(-.3,-.3);
    //backwardSlow();
    int count0 = 0;
    int count1 = 0;
    while(1){
      readSensor(0);
      readSensor(1);
      if(sensor[0]){
        if(sensor[1]) break;
        count0++;
      }
      else if(sensor[1]) count1++;
      delay(1);
    }
    if(count0!=0){
      pivotLeft();
      delay(count0*2);
    }
    else{
      pivotRight();
      delay(count1*2);
    }
    mdBrake(1, 1);
}
//EEPROM======================================================================================================
void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);
  
  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}
//===========================================RING DROP
void stopServos() {
  servoL.write(95);
  delay(10);
  servoR.write(95);
  delay(10);
}
void rotateLeft() {
  servoL.write(servoSpeed);
  delay(servoTime);
  stopServos();
}
void rotateRight() {
  servoR.write(servoSpeed);
  delay(servoTime);
  stopServos();
}
void rotateBoth() {
  servoL.write(servoSpeed);
  servoR.write(servoSpeed);
  delay(servoTime);
  stopServos();
}

void ringDrop() {
  if (servoArray[servoState] == both) {
    rotateBoth();
  }
  else if (servoArray[servoState] == right) {
    rotateRight();
  }
  else if (servoArray[servoState] == left){
    rotateLeft();
  }
  else {
  } 
  servoState++;
}
//===========================================SENSORS===========================
void printSensors(){
  for(int i=0;i<numSensors;i++){
    Serial.print(sensor[i]);
  }
  Serial.println("");
}
void readSensor(int num){
  if(num<numSensors) sensor[num] = sensorState(sensorPins[num]);
  else {
    Serial.print("INVALID SENSORS NUM:");
    Serial.print(num);
  }
}
//read everything
void readSensors(){
  for(int i=0;i<numSensors;i++){
    sensor[i] = sensorState(sensorPins[i]);
  }
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
void pivotRight(){
  mdSpeed(.7, -.5);
}
void pivotLeft(){
  mdSpeed(-.5, .7);
}

void mdRightTurn() {
  mdSpeed(1, -.1);
}
void mdLeftTurn() {
  mdSpeed(-.05, 1);
}
void middleTurn() {
  mdSpeed(1,.05);
  delay(480);
}
void undoTurn() {
  mdSpeed(1,-.5);
  delay(280);
}
void mdSpeed(float l, float r){
  md.setM1Speed(mdRFlip*mdPower*mdROffset*r);
  md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*l);
}
void mdBrake(float l, float r){
  if(l!=0)  md.setM2Brake(400*l);
  if(r!=0) md.setM1Brake(400*r);
}

void testSensors(){
  while(1){
    readSensors();
    printSensors();
    delay(200);
  }
}

void testStraight(){
  while(1){
    mdSpeed(1,1);
    delay(1000);
    mdBrake(1,0);
    delay(400);
    mdSpeed(1,1);
    mdBrake(0,1);
    delay(400);
    mdBrake(1,1);
    delay(50000);
  }
}
