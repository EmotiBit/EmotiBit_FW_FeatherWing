#include <Arduino.h>
#include "EmotiBitLedController.h"
#include "EmotiBitVersionController.h"
#define SERIAL_BAUD_RATE 115200

EmotiBitLedController controller;

void printAllLedStates()
{
  bool stateR = false, stateB = false, stateY = false;
  stateR = controller.getState(EmotiBitLedController::Led::RED);
  stateB = controller.getState(EmotiBitLedController::Led::BLUE);
  stateY = controller.getState(EmotiBitLedController::Led::YELLOW);
  Serial.print("State RED: ");Serial.print(stateR);
  Serial.print("\tState BLUE: ");Serial.print(stateB);
  Serial.print("\tState YELLOW: ");Serial.println(stateY);
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("starting unit test: Getter and Setter functions of the LedController for KTD2026");
  controller.setHwVersion(EmotiBitVersionController::EmotiBitVersion::V07A);
  Serial.println("Hardware version set");
  printAllLedStates();
  // setting RED
  Serial.println("Setting RED");
  controller.setState(EmotiBitLedController::Led::RED,true);
  printAllLedStates();

  // setting BLUE
  Serial.println("Setting BLUE");
  controller.setState(EmotiBitLedController::Led::BLUE,true);
  printAllLedStates();

  // setting YELLOW
  Serial.println("Setting YELLOW");
  controller.setState(EmotiBitLedController::Led::YELLOW,true);
  printAllLedStates();
  
  
  // resetting RED
  Serial.println("resetting RED");
  controller.setState(EmotiBitLedController::Led::RED,false);
  printAllLedStates();

  // resetting BLUE
  Serial.println("resetting BLUE");
  controller.setState(EmotiBitLedController::Led::BLUE,false);
  printAllLedStates();

  // resetting YELLOW
  Serial.println("resetting YELLOE");
  controller.setState(EmotiBitLedController::Led::YELLOW,false);
  printAllLedStates();

  Serial.println("End of test");
  while(1);

}

void loop() {
  // put your main code here, to run repeatedly:
}
