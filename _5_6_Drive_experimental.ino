//from front of sensor
//220 ohm resistor is signal, ground, 5v
//3/24/2016 EDIT
//Make sure the ground all goes to the same place

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
int sensPins[5] = {22,24,26,28,30};
bool sensor[5] = {false, false, false, false, false};

ServoInfo servo1 = {32, 52, 1376, 95};
ServoInfo servo2 = {34, 52, 1376, 92};

int path[9] = {1, 0, 1, 1, 2, 2, 1, 1};  //0 = straight, 1 = left, 2 = right
int pathState, loopCounter = 0;

int mdFull = 200; //full power for the motor
int flipLeftMotor = 1;
int flipRightMotor = -1;
float rightMotorOffset = .915; //.8233 on May 5th in Ricardo's code; 
//.78~ Offset for low power levels

//SETUP=======================================================================================================
void setup() {
  Serial.begin(9600);
  md.init();
  myServo1.attach(servo1.pinNum);
  myServo2.attach(servo2.pinNum);
  myServo1.write(servo1.stopSpeed);
  myServo2.write(servo2.stopSpeed);
  
  //do not start the loop() code untill the robot is placed on the line, helps with debugging pathState
  while(1){
    readSensor(3);
    readSensor(1);
    if(!sensor[3] and !sensor[1]){
      readSensor(2);
      if(sensor[2]){
        Serial.println("out of setup");
        break;
      }
    }
    delay(1000);
  }
}

//MAIN LOOP===================================================================================================
//checks the turn sensor (4 if the next turn is right, 0 if the next turn is left) every 33ms.
//checks for on-line corrections every 100ms
void loop() {
  //readSensors(); //for Debugging purposes
  //printSensors();
  
  loopCounter++;
  if(isTurn()){
    Serial.print("isTurn:true:");
    Serial.print(path[pathState]);
    mdPathState();  //get the motors going in the correct direction (left, right or forward)

    if(path[pathState]==0){ //if forward
      delay(450); //go forward for 450ms
    }
    else{ //left or right
      delay(1000); // turn for 1000ms
    }
    
    loopCounter=3; // must straighten out.  So go to mdCorrectForward right away.
    pathState++;  
    if(pathState>9){ //done with path
      Serial.print("PATHSTATEW OVER 9");
      delay(50000);
    }
    Serial.println(":FinishedTurn");
  }
  //every 3 loops, check if you need forward corrections
  if(loopCounter==3){
    loopCounter=0;
    mdCorrectForward();
  }
  delay(33);
}
//checks the one sensor that would signal for the next turn (either sensor 4 or sensor 0)
bool isTurn(){
  if(path[pathState]==2){
    //right turn is next, so check sensor[0]
    readSensor(4);
    if(sensor[4]){
      //if it is pinging, double check it and the other turn sensors.
      for(int i =0;i<2;i++){
        readSensor(2);
        if(!sensor[2]) return false;
        mdForward(-.05);
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
      for(int i =0;i<2;i++){
        readSensor(2);
        if(!sensor[2]) return false;
        mdForward(-.05);
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
        mdLeft(-.05);
    }
    else if(path[pathState]==2){ //next turn is a right
        mdRight(-.05);
    }
    else{ //skip this turn
        mdForward(1);
    }
}

void mdCorrectForward(){
  //it has already been determined that there is no turn so just correct to stay on the line.
  readSensor(1);
  readSensor(2);
  readSensor(3);
  
  //left correction sensor is reading
  if(sensor[1]){
    Serial.println("Correct Left");
    if(sensor[2]){          //left correction sensor and center sensor
      mdLeft(.3);
    }
    else{                   //left correction sensor only
      mdLeft(-.05);
    }
  }
  //right correction sensor is reading
  else if(sensor[3]){
    Serial.println("Correct Right");
    if(sensor[2]){          //right correction sensor and center sensor
      mdRight(.3);
    }
    else{                    //right correction sensor only
      mdRight(-.05);
    }
  }
  
  else if(sensor[2]){       //center sensor only
    mdForward(1);
  }
  else{
    readSensor(0);
    readSensor(4);
    //far off the line.  FIX NEEDED.  It must go backwards and re-find the line at this point.  So as to not miss a turn.
    if(sensor[0]){
      mdLeft(.1);
    }
    else if(sensor[4]){
      mdRight(.1);
    }
    //no sensors reading
    else{
      //stop
      mdForward(-.05);
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

bool sensorState(int sensorIn){       //
   pinMode(sensorIn, OUTPUT);     // Make pin OUTPUT
   digitalWrite(sensorIn, HIGH);  // Pin HIGH (discharge capacitor)
   pinMode(sensorIn, INPUT);      // Make pin INPUT
   digitalWrite(sensorIn, LOW);   // Turn off internal pullups
   if(digitalRead(sensorIn)){     // Pin goes HIGH which means it is on the black line
      return true;
   }
   else{
    return false;
   }
}

//MOTOR FUNCTIONS=============================================================================================
//v signals how much power should go to the Right or Left engine.  Usually set to (-.05)->(.7) starting a turn or mild correction.
void mdRight(float v) {
    md.setM1Speed(flipLeftMotor*mdFull*v);
    md.setM2Speed(flipRightMotor*(mdFull*rightMotorOffset));
}
void mdLeft(float v) {
    md.setM1Speed(flipLeftMotor*mdFull);
    md.setM2Speed(flipRightMotor*(mdFull*rightMotorOffset)*v);
}
void mdForward(float v) {
    md.setM1Speed(flipLeftMotor*mdFull*v);
    md.setM2Speed(flipRightMotor*(mdFull*rightMotorOffset)*v);
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
    delay(100);
  }
}
void testForward(){
  while(1){
    mdForward(1.0);
    delay(800);
    mdForward(-.05);
    delay(100);
    mdForward(.7);
    delay(900);
    mdForward(.45);
    delay(1000);
    mdForward(-.05);
    delay(100);
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


