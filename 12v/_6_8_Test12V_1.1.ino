/*
 * Added methods: yesTurnChecks, noTurnChecks, beforeTurn
 * Added timeArray[] that holds all the timed lengths of each leg.
 * Added loopTimer that updates, counting down untill the robot is near a turn
   6/8
   Make sure the EEPROM is storing and reading full integers.
   Test harder breaking in corrections
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
#define straight 0
#define left 1
#define right 2
#define both 0
#define rightArm 1
#define leftArm 2
#define misc 3
//========================================SETTINGS======================================================
bool timeTrack = true;
int mdFast = 400; //not near a turn
int mdSlow = 250; //near a turn or timing
//========================================GLOBAL DEFINES======================================================
int pathState = 0;  //quadrant 1| quadrant 2    | center 
int pathArray[15] = {left, straight, left, left, right, right, right, left, left, left, straight, straight, left, left, straight}; //1 = left, 0 straight, 2 right
int pathLength = 15;
int ringState = 0;
int ringArray[] = {both, both, rightArm, leftArm, rightArm, both, misc, rightArm, misc, both}; //0 both arms, 1 right, 2 left, 3 none
//int ringArray[] = {0, 0, 1, 2, 1, 0, 3, 1, 3, 0}; //0 both arms, 1 right, 2 left, 3 none
unsigned int timeArray[14];
boolean startTrack = true;
bool turnCheck;
Servo myServo;
Servo myServo2;
int rotate360Speed = 40; // 1.29s per rotation; 12.91s for 10 rotations
int rotate360Time = 1260; // off by a fraction of a degree.
int rotateStopSpeed = 92;
//int sensorPins[5] = {22,24,26,28,30};
int sensorPins[5] = {11,12,13,14,15};
bool sensor[5] = {false, false, false, false, false};
int numSensors=5;
unsigned long csec;
//int path[16] = {1, 0, 1, 1, 2, 2, 1, 1, 1, 2, 1, 1, 1, 0, 1, 1};  //0 = straight, 1 = left, 2 = right
int loopCounter, correction = 0;
int startTime = 0;
int intSize = sizeof(int);
bool c = true;
unsigned long msec;
Timer correctionTimer;
Timer loopTimer;
int mdPower = 400; //full power for the motor
int mdRFlip = 1;
int mdLFlip = -1;
float mdLOffset = .98;//.915; //.8233 on May 5th in Ricardo's code; 
//.78~ Offset for low power levels

//SETUP=======================================================================================================
void setup() {
  Serial.begin(9600);
  initialize();
  //testmdForward(1);
  //testSensors();
  yesTurnChecks();
  if(!timeTrack){
    for(int i=0;i<pathLength;i++){
      timeArray[i]=EEPROMReadInt(i*intSize);      //each spot takes up intSize bytes
      Serial.print(timeArray[i]);
      Serial.print(" : ");
      timeArray[i]=(timeArray[i]*2.5)/4-400;    //timed going 250 speed, will be going at 400 speed, slow down 400ms before the turn
      Serial.println(timeArray[i]);
    }
  }
}

void initialize(){
  md.init();
  myServo.attach(32);
  myServo2.attach(34);
  setUp();
}

void EEPROMWriteInt(int p_address, int p_value)
     {
     byte lowByte = ((p_value >> 0) & 0xFF);
     byte highByte = ((p_value >> 8) & 0xFF);

     EEPROM.write(p_address, lowByte);
     EEPROM.write(p_address + 1, highByte);
     }

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
     {
     byte lowByte = EEPROM.read(p_address);
     byte highByte = EEPROM.read(p_address + 1);

     return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
     }


//MAIN LOOP===================================================================================================
//timer, every XXms it is running mdCorrectmdForward(1) and after XXms runs startTurnChecks (when it is close to a turn)
void loop() {
  if(turnCheck) checkTurn();
  else loopTimer.update(); //turns on turnCheck when you get near a turn
  mdCorrectForward();
}

void startC(){
  c=true;
}

void checkTurn() {
  readSensors();
  if(pathArray[pathState]==2){ //check for right turn
    readSensor(4);
    if(sensor[4]){
      readSensor(0);
      readSensor(2);
      readSensor(3);
      if(sensor[2] and sensor[3] and !sensor[0]) turnRight(); //0-111
    }
  }
  else if(pathArray[pathState]==1){ //check for left turn
    readSensor(0);
    if(sensor[0]){
      readSensor(1);
      readSensor(2);
      readSensor(4);
      if(sensor[1] and sensor[2] and !sensor[4]) turnLeft(); //111-0
    }
  }
  else{
    readSensor(0);
    if (sensor[0]){ //check for forward
      readSensor(4);
      readSensor(2);
      if(sensor[2] and sensor[4]) turnForward(); //1-1-1
    }
  }
}
void mdCorrectForward(){
  if(!c){
    correctionTimer.update();
    return;
  }
  //readSensors();
  //it has already been determined that there is no turn so just correct to stay on the line.
  if(correction==0){
    readSensor(2);
    if(sensor[2] or startTrack){
      mdForward(1);
    }
    else{
      readSensor(3);
      readSensor(1);
      csec = millis();
      //left correction sensor is reading
      if(sensor[1] and !sensor[3]){
        correction=1;
        mdBrakeLeft(.4);
      }
      //right correction sensor is reading
      else if(sensor[3] and !sensor[1]){ 
        correction=-1; 
        mdBrakeRight(.4);
      }
      else{
        readSensor(4);
        readSensor(0);
        csec = millis();
        if(sensor[0] and !sensor[4]){
          correction=2;
          mdBrakeLeft(.2);
        }
        //right correction sensor is reading
        else if(sensor[4] and !sensor[0]){ 
          correction=-2; 
          mdBrakeRight(.2);
        }
      }
    }
  }
  else if(correction>0){
    readSensor(2);
    if(sensor[2]){
      correction=0;
      mdRight();
      c=false;
      correctionTimer.after((millis()-csec)/4, startC);
      //mdBrake(1);
      //delay(20);
    }
  }
  else{
    readSensor(2);
    if(sensor[2]){
      correction=0;
      mdLeft();
      c=false;
      correctionTimer.after((millis()-csec)/4, startC);
      //mdBrake(1);
      //delay(20);
    }
  }
}

void noTurnChecks(){
  //go fast and don't check for turns
  //called when there is no turn near
  mdPower = mdFast;
  turnCheck = false;
}
void yesTurnChecks(){
  //go slow and check for turns
  //called when getting close to a turn or timing the track
  mdPower = mdSlow;
  turnCheck = true;
}

void beforeTurn(){
  //called right after finding a turn, before doing anything about it
  if(timeTrack){
    if(!startTrack) EEPROMWriteInt(pathState*intSize, (unsigned int)(millis()-msec)); //store time for last leg
    msec = millis(); //start timing next leg
  }
  else{
    if(pathState !=5){ //super short leg
      noTurnChecks(); //go fast
      loopTimer.after(timeArray[pathState], yesTurnChecks);  //untill you get near the next turn
    }
  }
}

void turnForward(){
  beforeTurn();
  if (pathState == 10) {
    mdForward(1);
    delay(65);
     startTime = millis();
     while ((millis() - startTime) <= (2120-65)) {
       mdCorrectForward();
     }
     mdForward(0);
     middleTurn();
     delay(300);
     mdForward(1);
     delay(90);
     mdForward(0);
     ringDrop();
     mdForward(-1);
     delay(90);
     undoTurn();
     delay(300);
     mdForward(1);
     startTime=millis();
     while ((millis() - startTime) <= 1600) {
       mdCorrectForward();
     }
  }
  else if(pathState==14){
   mdForward(1);
   delay(450);
   mdForward(0);
   delay(1000000);
  }
  else {
    mdForward(1);
    delay(65);
  }
  pathState++;
}
void turnRight(){
  beforeTurn();
  if (pathState == 5) {
   mdForward(1);
   delay(200);
   mdForward(0);
   ringDrop();
   mdForward(-1);
   delay(700);
  }
  else{
    //mdForward(-.05);
    mdRight();
    delay(300);
  }
  pathState++;
}

void turnLeft(){
  beforeTurn();
  if(startTrack){
    startTrack=false;
    mdForward(1);
    delay(65);
    return;
  }
  mdForward(0);
  if(pathState !=9 && pathState !=12) straightLeft();
  ringDrop();
  mdLeft();
  delay(300);
  pathState++;
}

void straightLeft(){
  delay(300); //stopping time
    //backwardSlow();
    int count = 0;
    while(1){
      readSensor(2);
      readSensor(0);
      if(sensor[2]){
        pivotLeft();
        count++;
        if(sensor[0]){
          if(count >3){
            delay(30); //it is off-center so keep pivoting
            break;
          }
        }
      }
      else if(sensor[0]){
        pivotRight(); //chance it will miss the line
        count++;
        if(count>3){
          readSensor(1);
          if(sensor[1]){
            delay(30);
            break;
          }
        }
      }
      else mdForward(-.3);
      delay(1);
    }
    mdForward(0);
}
//===========================================RING DROP
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
void pivotLeft(){
  md.setM1Speed(-100*mdLFlip);
  md.setM2Speed(350*mdRFlip);
}
void pivotRight(){
  md.setM1Speed(10*mdLFlip);
  md.setM2Speed(-200*mdRFlip);
}

void mdRight() {
  md.setM1Speed(mdRFlip*mdPower*(-.26));
  md.setM2Speed(mdLFlip*(mdPower*mdLOffset));
  //md.setM1Speed(mdRFlip*mdPower*(-.26));
  //md.setM2Speed(mdLFlip*(mdPower*mdLOffset));
}
void mdLeft() {
  md.setM1Speed(mdRFlip*mdPower);
  md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*(-.05));
}
void mdBrakeRight(float v) {
  md.setM1Brake(400*v);
  md.setM2Speed(mdLFlip*(mdPower*mdLOffset));
}
void mdBrakeLeft(float v) {
  md.setM2Brake(400*v);
  md.setM1Speed(mdRFlip*mdPower);
}
void mdBrake(float v){
  md.setM1Brake(400*v);
  md.setM2Brake(400*v);
}
void mdForward(float v) {
  if(v==0) mdBrake(1);
  else{
    md.setM1Speed(mdRFlip*mdPower*v);
    md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*v);
  }
}

void middleTurn() {
  md.setM1Brake(200); //right side //  200
  md.setM2Speed(-200);
}
void undoTurn() {
  md.setM1Speed(-200); //right side //-40           -50
  //md.setM2Speed(-55);//left side  // -55           -75
  md.setM2Brake(200);
}
