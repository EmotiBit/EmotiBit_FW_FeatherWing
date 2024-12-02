#include <Arduino.h>
#include "EmotiBit.h"

bool pinStatus; // stores digital pin status to detect changes
#if defined ARDUINO_FEATHER_ESP32
const int digitalInputPin = 33;	// Pin on which to read digital input
#endif
#ifdef ADAFRUIT_FEATHER_M0
const int digitalInputPin = 10;	// Pin on which to read digital input
#endif

#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
const uint32_t SERIAL_BAUD = 2000000; //115200

EmotiBit emotibit;
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float data[dataSize];

void onShortButtonPress()
{
	// toggle wifi on/off
	if (emotibit.getPowerMode() == EmotiBit::PowerMode::NORMAL_POWER)
	{
		emotibit.setPowerMode(EmotiBit::PowerMode::WIRELESS_OFF);
		Serial.println("PowerMode::WIRELESS_OFF");
	}
	else
	{
		emotibit.setPowerMode(EmotiBit::PowerMode::NORMAL_POWER);
		Serial.println("PowerMode::NORMAL_POWER");
	}
}

void onLongButtonPress()
{
	emotibit.sleep();
}

void setup() 
{
	Serial.begin(SERIAL_BAUD);
	Serial.println("Serial started");
	delay(2000);	// short delay to allow user to connect to serial, if desired

	// Capture the calling ino into firmware_variant information
	String inoFilename = __FILE__;  // returns absolute path of ino file
	inoFilename.replace("/","\\");  // conform to standard directory separator
	// extract filename only if directory separator found in absolute path
	if(inoFilename.lastIndexOf("\\") != -1)
	{
		inoFilename = inoFilename.substring((inoFilename.lastIndexOf("\\")) + 1,(inoFilename.indexOf(".")));
	}
	emotibit.setup(inoFilename);

	// Attach callback functions
	emotibit.attachShortButtonPress(&onShortButtonPress);
	emotibit.attachLongButtonPress(&onLongButtonPress);
  
  // Attach interrupts to the rising and falling edges of a digital input on a free pin
	// IMPORTANT NOTE: digital input pins must never exceed the range GND-0.6v to VDD+0.6V (i.e. best to keep it within 0V-3.3V)
	// See Feather M0 pinouts: https://learn.adafruit.com/adafruit-feather-m0-wifi-atwinc1500/pinouts
  // See Huzzah32 pinouts: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts
  // See EmotiBit pinouts: https://github.com/EmotiBit/EmotiBit_Docs/tree/master/hardware_files
	// See https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
	// ToDo: consider the best way to toggle attaching sync pulse interrupts
	// Options:
	//	- dedicate specified pin in main-line code
	//	- comment in/out
	//	- boolean / ifdef at top of ino
	//	- separate ino/bin (present method)
	//	- reading field from SD card
	//	- control message from EmotiBit Oscilloscope
	Serial.print("INPUT_PULLDOWN to digital pin ");
	Serial.println(digitalInputPin);
	pinMode(digitalInputPin, INPUT_PULLDOWN);
	//attachInterrupt(digitalPinToInterrupt(digitalInputPin), digitalInputChange, CHANGE); // Interrupt seems to crash ESP32
}

void loop()
{
	//Serial.println("emotibit.update()");
	emotibit.update();
  
  // Handle DIO event data
  bool temp = digitalRead(digitalInputPin);
  if (pinStatus != temp)
  {
    // Pin change detected
    pinStatus = temp;
    float dioData = (float) pinStatus; // cast pin status to a float for compatibility with addPacket
    // ToDo: consider adding EmotiBitPacket::TypeTag::DIGITAL_INPUT_0
    emotibit.addPacket(millis(), "D0", &dioData, 1);	// See EmotiBitPacket for available TypeTags https://github.com/EmotiBit/EmotiBit_XPlat_Utils/blob/master/src/EmotiBitPacket.cpp// 

    Serial.print("D0: ");
    Serial.println((int) dioData);
  }

  // Grab some EmotiBit sensor data and do something with it!
	size_t dataAvailable = emotibit.readData(EmotiBit::DataType::PPG_GREEN, &data[0], dataSize);
	if (dataAvailable > 0)
	{
		// Hey cool, I got some data! Maybe I can light up my shoes whenever I get excited!

		// print the data to view in the serial plotter
		bool printData = false;
		if (printData)
		{
			for (size_t i = 0; i < dataAvailable && i < dataSize; i++)
			{
				// Note that dataAvailable can be larger than dataSize
				Serial.println(data[i]);
			}
		}
	}
}