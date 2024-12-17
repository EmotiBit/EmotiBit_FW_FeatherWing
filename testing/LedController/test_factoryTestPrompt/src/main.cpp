/*
This example tests the LEDs on EmotiBit using the Led Controller class.
The LEDs on the EmotiBit can be toggled ON/OFF using serial prompts that leverage Factory test prompts.
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
// ToDo: Add support for Feather M0 i2
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
  Serial.println("Enter one of the following into the Srial input to test LED response to Factory test commands");
  Serial.println("@+R~");
  Serial.println("@-R~");
  Serial.println("@+B~");
  Serial.println("@-B~");
  Serial.println("@+Y~");
  Serial.println("@-Y~");
  while(!Serial.available());
  // ToDo: Make the EmotiBit::processFactoryTestMessages reusable outside the emotibit class. This is a copy + modification to use the local lecController object.
  if (Serial.available() > 0 && Serial.read() == EmotiBitSerial::MSG_START_CHAR)
	{
		Serial.print("FactoryTestMessage: ");
		String msg = Serial.readStringUntil(EmotiBitSerial::MSG_TERM_CHAR);
		String msgTypeTag = msg.substring(0, 2);
		Serial.println(msgTypeTag);
		if (msgTypeTag.equals(EmotiBitFactoryTest::TypeTag::LED_RED_ON))
		{
			emotibitLedController.setState(EmotiBitLedController::Led::RED, true, true);
		}
		else if (msgTypeTag.equals(EmotiBitFactoryTest::TypeTag::LED_RED_OFF))
		{
			emotibitLedController.setState(EmotiBitLedController::Led::RED, false, true);
		}
		else if (msgTypeTag.equals(EmotiBitFactoryTest::TypeTag::LED_BLUE_ON))
		{
			emotibitLedController.setState(EmotiBitLedController::Led::BLUE, true, true);
		}
		else if (msgTypeTag.equals(EmotiBitFactoryTest::TypeTag::LED_BLUE_OFF))
		{
			emotibitLedController.setState(EmotiBitLedController::Led::BLUE, false, true);
		}
		else if (msgTypeTag.equals(EmotiBitFactoryTest::TypeTag::LED_YELLOW_ON))
		{
			emotibitLedController.setState(EmotiBitLedController::Led::YELLOW, true, true);
		}
		else if (msgTypeTag.equals(EmotiBitFactoryTest::TypeTag::LED_YELLOW_OFF))
		{
			emotibitLedController.setState(EmotiBitLedController::Led::YELLOW, false, true);
		}
  }
}