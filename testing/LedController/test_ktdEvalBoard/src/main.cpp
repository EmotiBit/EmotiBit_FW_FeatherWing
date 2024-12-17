/*!
  @details Use this test to validate ledController functionality using a modified KTD026 eval board.
  The modifications include:
  1. [Optional] Changing the standard LED on the Eval board with the EmotiBit LEDs, along with replacing the series resistor in each channel
  2. Replacing the standard KTD2026 with the KTD2026B, which is used on the EmotiBit
*/
#include <Arduino.h>
#include "EmotiBitLedController.h"
#include "EmotiBitVersionController.h"
#define SERIAL_BAUD_RATE 115200

EmotiBitLedController controller;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("starting unit test: Control LEDs on eval board");
  Wire.begin();
  Wire.setClock(400000);
  controller.begin(&Wire, EmotiBitVersionController::EmotiBitVersion::V07A);

  // setting RED
  Serial.println("Setting RED");
  controller.setState(EmotiBitLedController::Led::RED,true,true);
  delay(500);
  // setting BLUE
  Serial.println("Setting BLUE");
  controller.setState(EmotiBitLedController::Led::BLUE,true,true);
  delay(500);
  // setting YELLOW
  Serial.println("Setting YELLOW");
  controller.setState(EmotiBitLedController::Led::YELLOW,true,true);
  delay(500);
  
  
  // resetting RED
  Serial.println("resetting RED");
  controller.setState(EmotiBitLedController::Led::RED,false,true);
  delay(500);

  // resetting BLUE
  Serial.println("resetting BLUE");
  controller.setState(EmotiBitLedController::Led::BLUE,false,true);
  delay(500);

  // resetting YELLOW
  Serial.println("resetting YELLOW");
  controller.setState(EmotiBitLedController::Led::YELLOW,false,true);
  delay(500);
  Serial.println("End of test");
  while(1);

}

void loop() {
  // put your main code here, to run repeatedly:
}
