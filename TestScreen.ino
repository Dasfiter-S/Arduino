
#include <SoftwareSerial.h>
#include <Nextion.h>

SoftwareSerial nextion(2, 3);
Nextion myNextion(nextion, 9600);
int value = 0;
void setup() {
  Serial.begin(9600);
  myNextion.init();
}

void loop() {
  readScreen();
  
}

void readScreen() {
  String message = myNextion.listen();
  if (message == "65 0 1 1 ffff ffff ffff") {
    //myNextion.buttonToggle(button1State, "b0", 0, 2);
    Serial.println("Stoping...");
  }
  if (message == "65 0 2 1 ffff ffff ffff") {
    //myNextion.buttonToggle(button2State, "b1", 0, 2);
    Serial.println("Starting run!");
  }
  if (message == "65 0 3 1 ffff ffff ffff") {
    //myNextion.buttonToggle(button3State, "b0", 0, 2);
    Serial.println("Loading rings");
  }
  /*
  if (message == "65 0 4 1 ffff ffff ffff") {
    //myNextion.buttonToggle(button4State, "b1", 0, 2);
  }
  
  if (message == "65 1 2 1 ffff ffff ffff") {
    //myNextion.buttonToggle(button5State, "b0", 0, 2);
  }
  if (message == "65 1 1 1 ffff ffff ffff") {
    //myNextion.buttonToggle(button6State, "b1", 0, 2);
  }
  */
  myNextion.setComponentText("timeD", String(millis()));
  myNextion.setComponentText("ringD", String(value++));
  myNextion.setComponentText("turnNextD", "Left");
  myNextion.setComponentText("turnLastD", "Blank");
}
