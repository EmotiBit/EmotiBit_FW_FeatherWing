/**************************************************************************/
/*!
	@file     EmotiBitVersionController.h
	@author   Nitin Nair (EmotiBit)
	@mainpage Controls reading Version from NVM on start or detecting version if NVM not updated, EmotiBit pin and constant asignment on startup
	@section intro_sec Introduction
	This is a library to handle EmotiBit HW version detection/version retrieval from NVM on setup.
	See also "FW update instructions to support new HW version (EmotiBit).gdoc"
	
	EmotiBit invests time and resources providing this open source code,
	please support EmotiBit and open-source hardware by purchasing
	products from EmotiBit!
	
	@section author Author
	Written by Nitin Nair for EmotiBit.
	
	@section  HISTORY
	v1.0  - First release
	
	@section license License
	BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/

#ifndef _EMOTIBIT_VERSION_CONTROLLER_H
#define _EMOTIBIT_VERSION_CONTROLLER_H

#include <SPI.h>
#if defined ARDUINO_FEATHER_ESP32
#include <SD.h>
#else
#include <SdFat.h>
#endif
#include <Arduino.h>
#include <Wire.h>
#include <EmotiBit_Si7013.h>
#include "EmotiBitVariants.h"
#include "EmotiBitFactoryTest.h"
#include "EmotiBitNvmController.h"

// All EmotiBit Pin names should be entered in this class structure.
// ToDo: Pins will move out from version controller when other sub-systems controllers are added.
enum class EmotiBitPinName
{
	EMOTIBIT_BUTTON = 1,
	SPI_CLK = 2,
	SPI_MOSI = 3,
	SPI_MISO = 4,
	PPG_INT = 5,
	BMI_INT1 = 6,
	BMI_INT2 = 7,
	BMM_INT = 8,
	BATTERY_READ_PIN = 9,
	ADS_RDY = 10,
	length = 11 //cannot be more than 28 (16 + 12)
};

enum class MathConstants
{
	VCC = 0,
	ADC_BITS = 1,
	ADC_MAX_VALUE = 2,
	length = 3 // cannot be > than the _MAX_MATH_CONSTANT_COUNT
};

enum class SystemConstants
{
	EMOTIBIT_HIBERNATE_LEVEL = 0,
	LED_DRIVER_CURRENT = 1,
	EMOTIBIT_HIBERNATE_PIN_MODE = 2,
	length = 3 // cannot be greater than the _MAX_SYSTEM_CONSTANT_COUNT
};

enum class VregEnablePinLogic
{
	ACTIVE_LOW,
	ACTIVE_HIGH
};

class EmotiBitVersionController
{
public: 	
	// ToDo: move these pin definitions inside a struct.
#if defined(ADAFRUIT_FEATHER_M0) 
	static const int EMOTIBIT_I2C_CLK_PIN = 13;
	static const int EMOTIBIT_I2C_DAT_PIN = 11;
	static int HIBERNATE_PIN;
	static int SD_CARD_CHIP_SEL_PIN;
#elif defined (ARDUINO_FEATHER_ESP32) 
	static const int EMOTIBIT_I2C_CLK_PIN = 13;
	static const int EMOTIBIT_I2C_DAT_PIN = 27;
	static int HIBERNATE_PIN;
	static int SD_CARD_CHIP_SEL_PIN;
#endif
	
	// !!! The following ORDER of the enum class holding the Version numbers SHOULD NOT BE ALTERED.
	// New versions should be ADDED to the end of this list 
	enum class EmotiBitVersion {
		UNKNOWN = -1,
		V01B = 0,
		V01C = 1,
		V02B = 2,
		V02F = 3,
		V02H = 4,
		V03B = 5,
		V04A = 6,
		V05C = 7,
		V06A = 8,
		V07A = 9,
		length
	};

	struct EmotiBitHardwareParameterTable {
		bool isSi7013Present;
		bool isEepromPresent;
		bool isThermopilePresent;
	};

private:
	static const int _MAX_EMOTIBIT_PIN_COUNT = 28;
	int _assignedPin[_MAX_EMOTIBIT_PIN_COUNT] = { 0 };
	float _assignedMathConstants[(uint8_t)MathConstants::length] = { 0 };
	int _assignedSystemConstants[(uint8_t)SystemConstants::length] = { 0 };
	const int _INVALID_CONSTANT_FOR_VERSION = -1;
	const int _INVALID_REQUEST = -2; // addresses out of bounds(for array indexing) request
	bool _initAssignmentComplete = false;
	// ToDo: This needs to be refactored so that the addresses can be read from individual drivers.
	static const uint8_t SI7013_I2C_ADDR = 0x40;
	static const uint8_t EEPROM_I2C_ADDR = 0x50;
	static const uint8_t MLX90632_I2C_ADDR = 0x3A;

	/*!
		@brief Initializes all EmotiBit Math constant with preset values
		@param version EmotiBit HW version
		@return True is Constants Initialized successfully, else False
	*/
	bool _initMappingMathConstants(EmotiBitVersionController::EmotiBitVersion version);
	
	/*!
		@brief Initializes all EmotiBit System constant with preset values
		@param version EmotiBit HW version
		@return True is Constants Initialized successfully, else False
	*/
	bool _initMappingSystemConstants(VregEnablePinLogic  logic);
	/*!
		@brief Function to populate parameters dependent on HW version.
			These paramets are used for validating the barcode and for detecting HW version, if NVM is not updated.
		@param emotibit_i2c TwoWire instance fro EmotiBit comms
		@param hardwareParameterTable instance of parameter table to be updated
	*/
	void updateVersionParameterTable(TwoWire &emotibit_i2c, EmotiBitHardwareParameterTable &hardwareParameterTable);


public:
	VregEnablePinLogic  vregEnablePinLogic;
	/*!
		@brief Function call to initialize EmotiBit constants mapping.
		@param version HW version of the EmotiBit
		@return True if successfully mapped all constants, else False
	*/
	bool initConstantMapping(EmotiBitVersionController::EmotiBitVersion version);
	
	/*!
		@brief Function call to retrieve Assigned Math constant
		@param constant MathConstant name
		@return value assigned to the constant
	*/
	float getMathConstant(MathConstants constant);

	/*!
		@brief Function call to retrieve Assigned System constant
		@param constant SystemConstant name
		@return value assigned to the constant
	*/
	int getSystemConstant(SystemConstants constant);
	
	/*!
		@brief Prints all EmotiBit constants name and values on Serial
	*/
	void echoConstants();

	/*!
		@brief function call to detect SD-Card and hibernate pin logic to setup emotiBit comms.
		@return true is SD-Card and EmotiBit detected.
	*/
	bool isEmotiBitReady();

	/*!
		@brief Function to validate barcode based on hardware checks on ICs.
		@param emotibit_i2c TwoWire instance for i2c communications
		@param barcode The parsed barcode instance being written to the NVM
		@param hwValidation is set true if bacode has the correct HW version, else false
		@param skuValidation is set to true is barcode has correct SKU version else false
		@return True if barcode has been successfully validated, else fail
	*/
	bool validateBarcodeInfo(TwoWire &emotibit_i2c, Barcode barcode, bool &hwValidation, bool &skuValidation);

	/*!
		@brief Writes Variant Information into NVM.
		@param emotibit_i2c TwoWire instance for i2c comms
		@param emotiBitNvmController Nvm controller instance to access NVM
		@param barcode verified and parsed barcode that has to be written into the NVM
		@return returns True if data is successfully written into the NVM, else False
	*/
	bool writeVariantInfoToNvm(EmotiBitNvmController &emotiBitNvmController, Barcode barcode);

	/*!
		@brief Reads EmotiBit variant information stored in NVM
		@param emotibit_i2c TwoWire instance for i2c comms
		@param emotiBitNvmController Nvm controller instance to access NVM
		@param hwVersion The HW version read from the NVM
		@param sku The SKU version read from the NVM
		@param emotibitSerialNumber the EmotiBit Number read from the version(for V4+ only)
		@return returns True, if Variant information successfully retrieved from the NVM, else False
	*/
	bool getEmotiBitVariantInfo(EmotiBitNvmController &emotiBitNvmController, EmotiBitVersion &hwVersion, String &sku, uint32_t &emotibitSerialNumber, String &barcode);
	
	/*!
		@brief fallback function to detect version from HW, if variant information not stored in NVM
		@param emotibit_i2c TwoWire instance for i2c comms
		@param hwVersion estimate of hardware version detected from hardware
		@param sku estimate of SKU version detected from hardware
		@return returns True if Variant was estimated from hardware, else False
	*/
	bool detectVariantFromHardware(TwoWire &emotibit_i2c, EmotiBitVersion &hwVersion, String &sku);
	
	/*!
		@brief Function used to return Hardware version in char array format
		@param version Version we need the string for
		@return Char array corresponding to the HW version
	*/
	static const char* getHardwareVersion(EmotiBitVersion version);

	/*!
		@brief Maps the pin numbers on EmotiBit based on microcontroller version and emotibit version
		@param version HW version of the EmotiBit
		@return True if mapping completed successfully, else False
	*/
	bool initPinMapping(EmotiBitVersionController::EmotiBitVersion version);

	/*!
		@brief Function call to get the pin number assigned to the pin name requested
		@param pin Pin name for which we require the assigned pin number
		@return returns pin number assigned to the particular pin name
	*/
	int getAssignedPin(EmotiBitPinName pin);

	/*!
		@brief Prints all pin mappings on the screen
	*/
	void echoPinMapping();
};
#endif
