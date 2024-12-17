/*
This example tests the LEDs on EmotiBit using the Led Controller class.
The LEDs on EmotiBit should sequentially turn ON and then OFF.
Tested with EmotiBit v5+Feather ESP32 Huzzah+EmotiBit_V5
ToDo: Test with Feather M0
*/

#include <Arduino.h>
#include "EmotiBitLedController.h"
#include "EmotiBitVersionController.h"
#define SERIAL_BAUD_RATE 115200

EmotiBitVersionController emotiBitVersionController;
EmotiBitLedController emotibitLedController;
TwoWire* _EmotiBit_i2c = nullptr;
uint32_t i2cRate = 100000;
int forceHwVersion = -1;
EmotiBitVersionController::EmotiBitVersion hwVersion = EmotiBitVersionController::EmotiBitVersion::UNKNOWN;
EmotiBitNvmController emotibitNvmController;
String emotiBitSku, emotibitDeviceId;
uint32_t emotibitSerialNumber; 

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("Baud rate set to:" + String(SERIAL_BAUD_RATE));

  // EmotiBit Ready
  if (emotiBitVersionController.isEmotiBitReady())
  {
    Serial.println("EmotiBit ready");
  }
  else
  {
    Serial.println("Version neither detected nor specified. Halting.");
    while(1);
  }

// setup i2c
#ifdef ADAFRUIT_FEATHER_M0
	Serial.println("Setting up I2C For M0...");
	_EmotiBit_i2c = new TwoWire(&sercom1, EmotiBitVersionController::EMOTIBIT_I2C_DAT_PIN, EmotiBitVersionController::EMOTIBIT_I2C_CLK_PIN);
	_EmotiBit_i2c->begin();
	// ToDo: detect if i2c init fails
	pinPeripheral(EmotiBitVersionController::EMOTIBIT_I2C_DAT_PIN, PIO_SERCOM);
	pinPeripheral(EmotiBitVersionController::EMOTIBIT_I2C_CLK_PIN, PIO_SERCOM);
#elif defined ARDUINO_FEATHER_ESP32
	_EmotiBit_i2c = new TwoWire(1);
	Serial.println("Setting up I2C For ESP32...");
	bool status = _EmotiBit_i2c->begin(EmotiBitVersionController::EMOTIBIT_I2C_DAT_PIN, EmotiBitVersionController::EMOTIBIT_I2C_CLK_PIN);
	if (status)
	{
		Serial.println("I2c setup complete");
	}
	else
	{
		Serial.println("I2c setup failed");
	}
#endif

	Serial.print("Setting clock to ");
	Serial.println(i2cRate);
	_EmotiBit_i2c->setClock(i2cRate);

  // init NVM controller
  Serial.print("Initializing NVM controller: ");
	if (emotibitNvmController.init(*_EmotiBit_i2c))
	{
		Serial.println("success");
	}
  else
  {
    Serial.println("failure");
  }

  //get emotibit version details from NVM
	if (!emotiBitVersionController.getEmotiBitVariantInfo(emotibitNvmController, hwVersion, emotiBitSku, emotibitSerialNumber, emotibitDeviceId))
	{
		if (!emotiBitVersionController.detectVariantFromHardware(*(_EmotiBit_i2c), hwVersion, emotiBitSku))
		{
			Serial.println("CANNOT IDENTIFY HARDWARE");
      while(1);
		}
	}
  
  // chance to override version
  uint32_t timeSinceWait = millis();
  Serial.println("To override auto version detection, choose a version below. Press 0 to continue");
  Serial.println("3. V3\n4. V4\n5. V5\n6. V6\n7. V7");
  while(!Serial.available())
  {
    delay(1000);
  }

  if(Serial.available())
  {
    String input = Serial.readString();
    forceHwVersion = input.toInt();
    switch(forceHwVersion)
    {
      case 0:
        break;
      case 7:
        hwVersion = EmotiBitVersionController::EmotiBitVersion::V07A;
        break;
      case 6:
        hwVersion = EmotiBitVersionController::EmotiBitVersion::V06A;
        break;
      case 5:
        hwVersion = EmotiBitVersionController::EmotiBitVersion::V05C;
        break;
      case 4:
        hwVersion = EmotiBitVersionController::EmotiBitVersion::V04A;
        break;
      case 3:
        hwVersion = EmotiBitVersionController::EmotiBitVersion::V03B;
        break;
      default:
        Serial.println("Invalid input. Reset and start again");
        while(1);
    }
    if(forceHwVersion)
    {
      Serial.println("forcing hw version: " + String(EmotiBitVersionController::getHardwareVersion(hwVersion)));
    }
  }


  // setup ledcontroller
  if(emotibitLedController.begin(_EmotiBit_i2c, hwVersion))
  {
    Serial.println("Led driver initialized");
  }
  else
  {
    Serial.println("Led driver init failed.");
    while(1);
  }

}

void loop()
{
  // sequential ON
  emotibitLedController.setState(EmotiBitLedController::Led::RED, true, true);
  delay(500);
  emotibitLedController.setState(EmotiBitLedController::Led::BLUE, true, true);
  delay(500);
  emotibitLedController.setState(EmotiBitLedController::Led::YELLOW, true, true);
  delay(500);
  
  // sequential OFF
  emotibitLedController.setState(EmotiBitLedController::Led::RED, false, true);
  delay(500);
  emotibitLedController.setState(EmotiBitLedController::Led::BLUE, false, true);
  delay(500);
  emotibitLedController.setState(EmotiBitLedController::Led::YELLOW, false, true);
  delay(500);
}