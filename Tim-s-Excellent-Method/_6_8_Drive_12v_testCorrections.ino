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
bool trialCorrect = true;
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
unsigned long csec;
//int path[16] = {1, 0, 1, 1, 2, 2, 1, 1, 1, 2, 1, 1, 1, 0, 1, 1};  //0 = straight, 1 = left, 2 = right
int pathState, ringState, loopCounter, correction = 0;

int mdPower = 400; //full power for the motor
int mdRFlip = 1;
int mdLFlip = -1;
float mdLOffset = .98;//.915; //.8233 on May 5th in Ricardo's code; 
//.78~ Offset for low power levels

//SETUP=======================================================================================================
void setup() {
  Serial.begin(9600);
  initialize();
  //testForward();
  //testSensors();
  waitTillPlacedOnLine();
}

void initialize(){
  md.init();
  myServo1.attach(servo1.pinNum);
  myServo2.attach(servo2.pinNum);
  myServo1.write(servo1.stopSpeed);
  myServo2.write(servo2.stopSpeed);
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
  if(trialCorrect) mdCorrectTrial();
  else mdCorrectForward();
  //t.update();
}

void mdCorrectTrial(){
  //it has already been determined that there is no turn so just correct to stay on the line.
  readSensor(2);
  if(sensor[2]){
    mdForward(1);
  }
  else{
    readSensor(3);
    readSensor(1);
    //left correction sensor is reading
    if(sensor[1] and !sensor[3]){
      Serial.println("Correct Left");
      mdLeft(.2);
    }
    //right correction sensor is reading
    else if(sensor[3] and !sensor[1]){
      Serial.println("Correct Right");   
      mdRight(.2);
    }
    else{
      readSensor(4);
      readSensor(0);
      csec = millis();
      if(sensor[0] and !sensor[4]){
        Serial.println("Correct Left");
        mdLeft(.6);
      }
      //right correction sensor is reading
      else if(sensor[4] and !sensor[0]){
        Serial.println("Correct Right");   
        mdRight(.6);
      }
    }
  }
}

void mdCorrectForward(){
  //it has already been determined that there is no turn so just correct to stay on the line.
  if(correction==0){
    readSensor(2);
    if(sensor[2]){
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
      correction--;
      if(correction!=0){
        correction*=-1;
        mdRight();
        csec=(millis()-85);
      }
    }
    else if(millis()-csec>120){
      mdLeft(.2);
      csec=millis();
      correction=2;
    }
    else if(millis()-csec>70) {
    }
    else if(millis()-csec>50) mdForward(1);
  }
  else{
    readSensor(2);
    if(sensor[2]){
      correction++;
      if(correction!=0){
        correction*=-1;
        mdLeft();
        csec=(millis()-85);
      }
    }
    else if(millis()-csec>120){
      mdLeft(.2);
      csec=millis();
      correction=2;
    }
    else if(millis()-csec>70) {
    }
    else if(millis()-csec>50) mdForward(1);
  }
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
void mdForward(float v) {
    //md.setM1Brake(0);
    //md.setM2Brake(0);
    md.setM1Speed(mdRFlip*mdPower*v);
    md.setM2Speed(mdLFlip*(mdPower*mdLOffset)*v);
}
