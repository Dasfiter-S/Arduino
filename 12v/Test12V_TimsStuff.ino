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


#include <Servo.h>
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
bool trialCorrect = false;
bool timeTrack = true;
int mdFast = 400; //not near a turn
int mdSlow = 200; //near a turn
//========================================GLOBAL DEFINES======================================================
int pathState = 0;  //quadrant 1| quadrant 2    | center 
int pathArray[20] = {left, straight, left, left, right, right, right, left, left, left, straight, straight, left, left}; //1 = left, 0 straight, 2 right
//int pathArray[20] = {1, 0, 1, 1, 2, 2, 2, 1, 1, 1, 0, 0, 1, 1 }; //1 = left, 0 straight, 2 right
//int pathArray[20] = {2, 2, 1, 1, 2, 2, 2, 1, 1, 1, 0, 0, 1, 1 }; //1 = left, 0 straight, 2 right
int ringState = 0;
int ringArray[] = {both, both, rightArm, leftArm, rightArm, both, misc, rightArm, misc, both}; //0 both arms, 1 right, 2 left, 3 none
//int ringArray[] = {0, 0, 1, 2, 1, 0, 3, 1, 3, 0}; //0 both arms, 1 right, 2 left, 3 none
boolean startTrack = true;
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
bool c = true;
unsigned long msec;
Timer t;
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
  //waitTillPlacedOnLine();
}

void initialize(){
  md.init();
  myServo.attach(32);
  myServo2.attach(34);
  setUp();
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
//timer, every XXms it is running mdCorrectmdForward(1) and after XXms runs startTurnChecks (when it is close to a turn)
void loop() {
  turn();
  mdCorrectForward();
  //t.update();
}

void startC(){
  c=true;
}

void turn() {
  readSensors();
  if (sensor[2] and sensor[3] and sensor[4]) {   //00111
      gotTurn();
  }
  else if (sensor[0] and sensor[1] and sensor[2]) {   //11100
      gotTurn();
  }
  else if (sensor[0] and sensor[2] and sensor[4]) {   //11111 second time we encounter this it has to be a right turn
      gotTurn();
  }
}

void mdCorrectForward(){
  if(!c){
    t.update();
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
        Serial.println("Correct Left");
        correction=1;
        mdLeft(.4);
      }
      //right correction sensor is reading
      else if(sensor[3] and !sensor[1]){
        Serial.println("Correct Right");   
        correction=-1; 
        mdRight(.4);
      }
      else{
        readSensor(4);
        readSensor(0);
        csec = millis();
        if(sensor[0] and !sensor[4]){
          Serial.println("Correct Left");
          correction=2;
          mdLeft(.2);
        }
        //right correction sensor is reading
        else if(sensor[4] and !sensor[0]){
          Serial.println("Correct Right");   
          correction=-2; 
          mdRight(.2);
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
      t.after((millis()-csec)/4, startC);
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
      t.after((millis()-csec)/4, startC);
      //mdBrake(1);
      //delay(20);
    }
  }
}

void gotTurn() {
  //still have to add failsafes, if there is a bad reading it doesn't throw the whole track off?
  if(startTrack){
    startTrack=false;
    mdForward(1);
    delay(65);
    return;
  }
  if (pathArray[pathState] == 0) {
    //go forward
    if (pathState == 10) {
      mdForward(1);
      delay(65);
       startTime = millis();
       while ((millis() - startTime) <= (2120-65)) {
         mdCorrectForward();
       }
       mdForward(0);
       middleTurn();
       mdForward(1);
       delay(90);
       mdForward(0);
       ringDrop();
       mdForward(-1);
       delay(90);
       undoTurn();
       mdForward(1);
       startTime=millis();
       while ((millis() - startTime) <= 1600) {
         mdCorrectForward();
       }
       
    }
    if (pathState == 1) {
      mdForward(1);
      delay(100);
    }
    else {
      mdForward(1);
      //delay(30);
    }
    //Serial.println("Foward");
  }
  else if (pathArray[pathState] == 1) {
    mdForward(0);
    if(pathState !=9 && pathState !=12) straightLeft();
    ringDrop();
    if(pathState==7) mdPower*=2;
    mdLeft();
    delay(300);
    if(pathState ==3) mdPower/=2;
    
    //Serial.println("Left Path called");
  }
  else if (pathArray[pathState] == 2){
    if (pathState != 5) {
      //mdForward(-.05);
      if(pathState <=7){
        mdSlowRight();
        delay(500);
      }
      else {
        mdRight();
        delay(300);
      }
    }
    if (pathState == 5) {
     mdForward(1);
     delay(200);
     mdForward(0);
     ringDrop();
     mdForward(-1);
     delay(700);
    }
    //Serial.println("Right Path called");
  }
  else {
    mdForward(0);
    delay(1000000);
  }
  pathState++;
  if (pathState >= 15) {
   mdForward(1);
   delay(450);
   mdForward(0);
   delay(1000000);
  }
}

void straightLeft(){
  delay(300);
    //backwardSlow();
    bool r = false;
    while(1){
      readSensor(2);
      readSensor(0);
      if(sensor[2]){
        readSensor(0);
        readSensor(1);
        if(sensor[0]) break;
        else if(r and sensor[1]) break;
        pivotLeft();
      }
      else if(sensor[0]){
        pivotRight();
        r = true;
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
    //md.setM1Brake(400);
    //md.setM2Brake(0);
    md.setM1Speed(mdRFlip*mdPower*(-.6));
    md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*.5);
}
void mdSlowRight() {
    //md.setM1Brake(400);
    //md.setM2Brake(0);
    md.setM1Speed(mdRFlip*mdPower*(-.26));
    md.setM2Speed(mdLFlip*(mdPower*mdLOffset));
}
void mdLeft() {
    //md.setM1Brake(0);
    //md.setM2Brake(400);
    md.setM1Speed(mdRFlip*mdPower);
    md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*(-.05));
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
void mdRightB() {
    md.setM1Brake(-200*mdRFlip);
    //md.setM2Brake(0);
    //md.setM1Speed(mdRFlip*mdPower*v*y);
    md.setM2Speed(mdLFlip*(mdPower*mdLOffset));
}
void mdLeftB() {
    //md.setM1Brake(0);
    md.setM2Speed(-200*mdLFlip);
    md.setM1Speed(mdRFlip*mdPower);
    //md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*v*y);
}
void mdBrake(float v){
  md.setM1Brake(400*v);
  md.setM2Brake(400*v);
}
void mdForward(float v) {
    //md.setM1Brake(0);
    //md.setM2Brake(0);
    if(v==0) mdBrake(1);
    else{
      md.setM1Speed(mdRFlip*mdPower*v);
      md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*v);
    }
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
