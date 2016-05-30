//from front of sensor
//5v, ground, signal
//Pins for the sensors=======================================
#define Pin2 2
#define Pin3 3
#define Pin4 4
#define Pin5 5
#define Pin6 6
//===========================================================
uint64_t currentTime = 0;
uint64_t lastTime = 0;
uint8_t state = 0;
void setup() {
  Serial.begin(9600);
}

void loop() {
  currentTime = millis();
  Serial.println(RCTime(Pin2));	    // Connect to pin 2, display results
  //Serial.println(RCTime(Pin3));
  //Serial.println(RCTime(Pin4));
  //Serial.println(RCTime(Pin5));
  //Serial.println(RCTime(Pin6));
  //delay(250);			    // Wait 250 ms
}

uint64_t RCTime(int sensorIn){    //uint64_t means unsigned 64bit integer
   pinMode(sensorIn, OUTPUT);     // Make pin OUTPUT
   digitalWrite(sensorIn, HIGH);  // Pin HIGH (discharge capacitor)
   pinMode(sensorIn, INPUT);      // Make pin INPUT
   digitalWrite(sensorIn, LOW);   // Turn off internal pullups
   if(digitalRead(sensorIn)){  // Wait for pin to go LOW
      lastTime = millis();
   }
   if (!digitalRead(sensorIn)) {
     uint64_t temp = lastTime;
     lastTime = 0;
     return temp;
   }
   return 0;
}

int isOnTheLine(uint64_t currentPin) {//has to receive pin readings from left to right
   if (currentPin > 300) { //sensor is on the line
     return 1;
   }
   if (currentPin < 300) { //sensor is NOT on the line
    return 0;
   } 
}

uint8_t sensorState() {
  state = ((isOnTheLine(RCTime(Pin6))*pow(2,4)) + (isOnTheLine(RCTime(Pin5))*pow(2,3)) + (isOnTheLine(RCTime(Pin4))*pow(2,2)) + (isOnTheLine(RCTime(Pin3))*pow(2,1)) + (isOnTheLine(RCTime(Pin2))));
  return state;
}

void drive {
 if (state == 4) {    //00100
  //straight();
 }
 if (state == 0) {    //00000
  //leftTurn();
 } 
 if (state == 28) {   //11100
  //leftTurn();
 } 
 if (state == 31) {   //11111
  //straight();
 } 
 if (state == 0) {    //00000
  //();
 } 
 if (state == 0) {    //00000
  //();
 }  
}
