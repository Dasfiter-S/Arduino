//ground center,  
//red 5v, blue signal, green ground
 int LDR = 0;
 
 // initialize the serial port
 // and declare inputs and outputs
 void setup() {
   //pinMode(8, INPUT);
   //pinMode(9, INPUT);
   //pinMode(10, INPUT);
   //pinMode(11, INPUT);
   pinMode(2, OUTPUT);
  PORTE |= _BV(PE4);
  //pinMode(3, OUTPUT);
  //PORTD |= _BV(PD3);
  //pinMode(4, OUTPUT);
  //PORTD |= _BV(PD4);
  //pinMode(5, OUTPUT);
  //PORTD |= _BV(PD5);
   Serial.begin(9600);
 }
 
 // read from the analog input connected to the LDR
 // and print the value to the serial port.
 // the delay is only to avoid sending so much data
 // as to make it unreadable.
 void loop() {
   int v = analogRead(10);
   Serial.print("Sensor 1: ");Serial.println(v);
   v = analogRead(11);
   Serial.print(" Sensor 2: ");Serial.println(v);
   v = analogRead(12);
   Serial.print(" Sensor 3: ");Serial.println(v);
   //v = analogRead();
   //Serial.print(" Sensor 4: ");Serial.println(v);
   delay(500);
 }
