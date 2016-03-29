/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */
int LDR = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(4, OUTPUT);
  pinMode(LDR, INPUT);
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  int v = 0;
  digitalWrite(4, HIGH);   // turn the LED on (HIGH is the voltage level)
  v = analogRead(LDR);
  Serial.println(v);
  delay(1000);              // wait for a second
  digitalWrite(4, LOW);    // turn the LED off by making the voltage LOW
  v = analogRead(LDR);
  Serial.println(v);
  delay(1000);              // wait for a second
  //no box
  //973~ high
  //937+ low
  //box----
  //970~ high
  //900+ low
}
