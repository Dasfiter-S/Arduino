
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
//=========================================STRUCTS=============================================================

//========================================SETTINGS======================================================
bool timing = false;
int mdFast = 400; //not near a turn
int mdSlow = 250; //near a turn
//========================================VARS======================================================

int sensorPins[4] = {12,13,14,15};
bool sensor[4] = {false, false, false, false};
int numSensors=4;

unsigned long correctMS, turnMS, t =0;

int mdPower = mdFast; //full power for the motor
int mdRFlip = 1;
int mdLFlip = -1;
float mdLOffset = 1;//.98;//.915; //.8233 on May 5th in Ricardo's code; 
float mdROffset = .9;
//.78~ Offset for low power level

//SETUP=======================================================================================================
void setup() {
  Serial.begin(9600);
  initialize();
  //mdPower = mdSlow;
  //startingZone = turnChecks = true;
  
  mdSpeed(1,1);
  delay(400);
  correct(1);
  delay(3000);
  //waitTillPlacedOnTrack();
  //gotTurn();
  //while(1) mdCorrectForward();
}

void initialize(){
  md.init();
}

void loop() {
  readSensors();
  mdCorrectForward();
}

bool rightS, leftS, justTurned = false;
unsigned long sTime = 0;
int facing = 0;

void correct(int facing){
  if(facing>0){
    mdSpeed(1,0);
    mdBrake(0,1);
  }
  else{
    mdSpeed(0,1);
    mdBrake(1,0);
  }
  delay(30+facing*10);
  if(facing>0){
    mdSpeed(0,1);
    mdBrake(1,0);
  }
  else{
    mdSpeed(1,0);
    mdBrake(0,1);
  }
  delay(80);
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

void beforeTurn(){
  justTurned = true;
  rightS = leftS = false;
  sTime = 0;
  facing = 0;
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
void pivotLeft(){
  mdSpeed(1, -.3);
}
void pivotRight(){
  mdSpeed(-.5, .05);
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
