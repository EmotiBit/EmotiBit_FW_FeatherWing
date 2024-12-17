/**************************************************************************/
/*!
	@file     EmotiBitVersionController.cpp
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

#include "EmotiBitVersionController.h"
#include "Arduino.h"
#if defined(ADAFRUIT_FEATHER_M0)
int EmotiBitVersionController::HIBERNATE_PIN = 6;
int EmotiBitVersionController::SD_CARD_CHIP_SEL_PIN = 19;
#elif defined(ARDUINO_FEATHER_ESP32)
int EmotiBitVersionController::HIBERNATE_PIN = 32;
int EmotiBitVersionController::SD_CARD_CHIP_SEL_PIN = 4;
#endif

const char* EmotiBitVersionController::getHardwareVersion(EmotiBitVersion version)
{

	if (version == EmotiBitVersion::V01B)
	{
		return "V01b\0";
	}
	else if (version == EmotiBitVersion::V01C)
	{
		return "V01c\0";
	}
	else if (version == EmotiBitVersion::V02B)
	{
		return "V02b\0";
	}
	else if (version == EmotiBitVersion::V02F)
	{
		return "V02f\0";
	}
	else if (version == EmotiBitVersion::V02H)
	{
		return "V02h\0";
	}
	else if (version == EmotiBitVersion::V03B)
	{
		return "V03b\0";
	}
	else if (version == EmotiBitVersion::V04A)
	{
		return "V04a\0";
	}
	else if (version == EmotiBitVersion::V05C)
	{
		return "V05c\0";
	}
	else if (version == EmotiBitVersion::V06A)
	{
		return "V06a\0";
	}
	else if (version == EmotiBitVersion::V07A)
	{
		return "V07a\0";
	}
	else
	{
		return "NA\0";
	}
}

bool EmotiBitVersionController::initPinMapping(EmotiBitVersionController::EmotiBitVersion version)
{
	const bool checkLogic = false;
	checkLogic ? Serial.println("initPinMapping:") : true;
#if defined(ADAFRUIT_FEATHER_M0)
	checkLogic ? Serial.println("ADAFRUIT_FEATHER_M0") : true;
	// ToDo: Move these pin Assignments(maybe inside a constructor). These are specific to MCU platform and cannot change.
	_assignedPin[(int)EmotiBitPinName::BATTERY_READ_PIN] = A7;
	_assignedPin[(int)EmotiBitPinName::SPI_CLK] = 24;
	_assignedPin[(int)EmotiBitPinName::SPI_MOSI] = 23;
	_assignedPin[(int)EmotiBitPinName::SPI_MISO] = 22;

	_assignedPin[(int)EmotiBitPinName::EMOTIBIT_BUTTON] = 12;
	_assignedPin[(int)EmotiBitPinName::PPG_INT] = 15;
	_assignedPin[(int)EmotiBitPinName::BMI_INT1] = 5;
	_assignedPin[(int)EmotiBitPinName::BMI_INT2] = 10;
	_assignedPin[(int)EmotiBitPinName::BMM_INT] = 14;
	_assignedPin[(int)EmotiBitPinName::ADS_RDY] = -1;

	if (version == EmotiBitVersion::UNKNOWN)
	{
		checkLogic ? Serial.println("EmotiBitVersion::UNKNOWN") : true;
		return true;
	}
	else if ((int)version < (int)EmotiBitVersion::V02B)
	{
		checkLogic ? Serial.println("v01") : true;
		HIBERNATE_PIN = 5;
		SD_CARD_CHIP_SEL_PIN = 6;
		_assignedPin[(int)EmotiBitPinName::EMOTIBIT_BUTTON] = 13;
	}
	else if ((int)version <= (int)EmotiBitVersion::V04A)
	{
		checkLogic ? Serial.println("v02-v04") : true;
	}
	else if ((int)version >= (int)EmotiBitVersion::V05C)
	{
		checkLogic ? Serial.println("v05+") : true;
		_assignedPin[(int)EmotiBitPinName::PPG_INT] = 17;
		_assignedPin[(int)EmotiBitPinName::ADS_RDY] = 15;
	}
	else
	{
		// unknown version
		Serial.println("Unknown Version");
		return false;
	}
#elif defined(ADAFRUIT_BLUEFRUIT_NRF52_FEATHER)
	if (version == EmotiBitVersion::UNKNOWN)
	{
		checkLogic ? Serial.println("EmotiBitVersion::UNKNOWN") : true;
		return true;
	}
	else if ((int)version < (int)EmotiBitVersion::V02B)
	{
		checkLogic ? Serial.println("v01") : true;
		HIBERNATE_PIN = 27;
		_assignedPin[(int)EmotiBitPinName::EMOTIBIT_BUTTON] = 16;
	}
	else
	{
		// unknown version
		Serial.println("Unknown Version");
		return false;
	}
#elif defined(ARDUINO_FEATHER_ESP32)
	checkLogic ? Serial.println("ARDUINO_FEATHER_ESP32") : true;
	// write the code here
	_assignedPin[(int)EmotiBitPinName::BATTERY_READ_PIN] = A13;
	_assignedPin[(int)EmotiBitPinName::SPI_CLK] = 5;
	_assignedPin[(int)EmotiBitPinName::SPI_MOSI] = 18;
	_assignedPin[(int)EmotiBitPinName::SPI_MISO] = 19;

	_assignedPin[(int)EmotiBitPinName::EMOTIBIT_BUTTON] = 12;
	_assignedPin[(int)EmotiBitPinName::PPG_INT] = 25;
	_assignedPin[(int)EmotiBitPinName::BMI_INT1] = 14;
	_assignedPin[(int)EmotiBitPinName::BMI_INT2] = 33;
	_assignedPin[(int)EmotiBitPinName::BMM_INT] = 26;
	_assignedPin[(int)EmotiBitPinName::ADS_RDY] = -1;

	if (version == EmotiBitVersion::UNKNOWN)
	{
		checkLogic ? Serial.println("EmotiBitVersion::UNKNOWN") : true;
		return true;
	}
	else if ((int)version < (int)EmotiBitVersion::V02B)
	{
		checkLogic ? Serial.println("v01") : true;
		_assignedPin[(int)EmotiBitPinName::EMOTIBIT_BUTTON] = 13;
	}
	else if ((int)version <= (int)EmotiBitVersion::V04A)
	{
		checkLogic ? Serial.println("v02-v04") : true;
	}
	else if ((int)version >= (int)EmotiBitVersion::V05C)
	{
		checkLogic ? Serial.println("v05+") : true;
		_assignedPin[(int)EmotiBitPinName::PPG_INT] = 39;
		_assignedPin[(int)EmotiBitPinName::ADS_RDY] = 25;
	}
	else
	{
		// unknown version
		Serial.println("Unknown Version");
		return false;
	}
#else 
	// unknown version
	Serial.println("Unknown Feather");
	return false;
#endif
	return true;
}

int EmotiBitVersionController::getAssignedPin(EmotiBitPinName pin)
{
	if ((int)pin >= _MAX_EMOTIBIT_PIN_COUNT)
	{
		Serial.println("out of bounds pin accessed. Please check pin number again");
		return -1;
	}
	else
	{
		return _assignedPin[(int)pin];
	}
}


void EmotiBitVersionController::echoPinMapping()
{
	Serial.print("EMOTIBIT_I2C_CLOCK: "); Serial.println(EMOTIBIT_I2C_CLK_PIN);
	Serial.print("EMOTIBTI_I2C_DATA: "); Serial.println(EMOTIBIT_I2C_DAT_PIN);
	Serial.print("HIBERNATE: "); Serial.println(HIBERNATE_PIN);
	Serial.print("EMOTIBIT_BUTTON: "); Serial.println(_assignedPin[(int)EmotiBitPinName::EMOTIBIT_BUTTON]);
	Serial.print("SD_CARD_CHIP_SELECT: "); Serial.println(SD_CARD_CHIP_SEL_PIN);
	Serial.print("SPI_CLK: "); Serial.println(_assignedPin[(int)EmotiBitPinName::SPI_CLK]);
	Serial.print("SPI_MOSI: "); Serial.println(_assignedPin[(int)EmotiBitPinName::SPI_MOSI]);
	Serial.print("SPI_MISO: "); Serial.println(_assignedPin[(int)EmotiBitPinName::SPI_MISO]);
	Serial.print("PPG_INT: "); Serial.println(_assignedPin[(int)EmotiBitPinName::PPG_INT]);
	Serial.print("BMI_INT1: "); Serial.println(_assignedPin[(int)EmotiBitPinName::BMI_INT1]);
	Serial.print("BMI_INT2: "); Serial.println(_assignedPin[(int)EmotiBitPinName::BMI_INT2]);
	Serial.print("BMM_INT: "); Serial.println(_assignedPin[(int)EmotiBitPinName::BMM_INT]);
	Serial.print("BATTERY_READ_PIN: "); Serial.println(_assignedPin[(int)EmotiBitPinName::BATTERY_READ_PIN]);

}

bool EmotiBitVersionController::initConstantMapping(EmotiBitVersionController::EmotiBitVersion version)
{
	if (!_initMappingMathConstants(version))
	{
		return false;
	}
	_initAssignmentComplete = true;
	return true;
}

bool EmotiBitVersionController::_initMappingMathConstants(EmotiBitVersionController::EmotiBitVersion version)
{
	_assignedMathConstants[(int)MathConstants::VCC] = 3.3f;
	_assignedMathConstants[(int)MathConstants::ADC_BITS] = 12;
	_assignedMathConstants[(int)MathConstants::ADC_MAX_VALUE] = pow(2, _assignedMathConstants[(int)MathConstants::ADC_BITS]) - 1;;
	return true;
}

bool EmotiBitVersionController::_initMappingSystemConstants(VregEnablePinLogic logic)
{
	if (logic == VregEnablePinLogic::ACTIVE_LOW)
	{
		// For V2
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_LEVEL] = HIGH;
		_assignedSystemConstants[(int)SystemConstants::LED_DRIVER_CURRENT] = 26;
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_PIN_MODE] = INPUT;
	}
	else 
	{
		// For V3 and bove
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_LEVEL] = LOW;
		_assignedSystemConstants[(int)SystemConstants::LED_DRIVER_CURRENT] = 6;
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_PIN_MODE] = INPUT_PULLDOWN;
	}

	return true;
}

float EmotiBitVersionController::getMathConstant(MathConstants constant)
{
	if (_initAssignmentComplete)
	{
		if ((int)constant >= (int)MathConstants::length)
		{
			Serial.println("Invalid request");
			return _INVALID_REQUEST;
		}
		else
		{
			return _assignedMathConstants[(int)constant];
		}
	}
	else
	{
		Serial.println("Constants not initialized yet. call emotiBitVersionController.emotibitConstantMapping.initMapping()");
		return _INVALID_REQUEST;
	}
}

int EmotiBitVersionController::getSystemConstant(SystemConstants constant)
{
	if (_initAssignmentComplete)
	{
		if ((int)constant >= (int)SystemConstants::length)
		{
			Serial.println("Invalid request");
			return _INVALID_REQUEST;
		}
		else
		{
			return _assignedSystemConstants[(int)constant];
		}
	}
	else
	{
		Serial.println("Constants not initialized yet.  call emotiBitVersionController.emotibitConstantMapping.initMapping()");
		return _INVALID_REQUEST;
	}
}

void EmotiBitVersionController::echoConstants()
{
	if (_initAssignmentComplete)
	{
		Serial.print("MathConstants: VCC - "); Serial.println(_assignedMathConstants[(int)MathConstants::VCC]);
		Serial.print("MathConstants: ADC_BITS - "); Serial.println(_assignedMathConstants[(int)MathConstants::ADC_BITS]);
		Serial.print("MathConstants: ADC_MAX_VALUE - "); Serial.println(_assignedMathConstants[(int)MathConstants::ADC_MAX_VALUE]);
		Serial.print("SystemConstant: EMOTIBIT_HIBERNATE_LEVEL - "); Serial.println(_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_LEVEL]);
		Serial.print("SystemConstant: LED_DRIVER_CURRENT - "); Serial.println(_assignedSystemConstants[(int)SystemConstants::LED_DRIVER_CURRENT]);
	}
	else
	{
		Serial.println("Please initilize the constants.  call emotiBitVersionController.emotibitConstantMapping.initMapping()");
	}
}

bool EmotiBitVersionController::isEmotiBitReady()
{
#if defined (ARDUINO_FEATHER_ESP32)
	// Use Arduino SD already defined
#else
	SdFat SD;
#endif

	pinMode(HIBERNATE_PIN, INPUT);

	if (digitalRead(HIBERNATE_PIN) == LOW)
	{
		// V3+: setting hibernate pin low hibernates EmotiBit
		Serial.println("vregEnablePinLogic: Active HIGH(V3+)");
		vregEnablePinLogic = VregEnablePinLogic::ACTIVE_HIGH;
		pinMode(HIBERNATE_PIN, OUTPUT);
		digitalWrite(HIBERNATE_PIN, HIGH);
		delay(100);
		// ToDo: Think about adding battery voltage measurement here as well
		_initMappingSystemConstants(vregEnablePinLogic);
		if (SD.begin(SD_CARD_CHIP_SEL_PIN))
		{
			SD.end();
			return true;
		}
	}
	else
	{
		// V2: setting hibernate pin low hibernates EmotiBit
		Serial.println("vregEnablePinLogic: Active LOW(V2)");
		vregEnablePinLogic = VregEnablePinLogic::ACTIVE_LOW;
		pinMode(HIBERNATE_PIN, OUTPUT);
		digitalWrite(HIBERNATE_PIN, LOW);
		delay(100);
		// ToDo: Think about adding battery voltage measurement here as well
		_initMappingSystemConstants(vregEnablePinLogic);
		if (SD.begin(SD_CARD_CHIP_SEL_PIN))
		{
			return true;
		}
	}
	Serial.println("EmotiBit not ready. Please check if Battery and SD-Card are present on the EmotiBit.");
	SD.end();
	return false;
}

bool EmotiBitVersionController::validateBarcodeInfo(TwoWire &emotibit_i2c, Barcode barcode, bool &hwValidation, bool &skuValidation)
{
	EmotiBitHardwareParameterTable hardwareParameterTable;
	updateVersionParameterTable(emotibit_i2c, hardwareParameterTable);
	String barcodeHwVersion = barcode.hwVersion;
	barcodeHwVersion.remove(barcodeHwVersion.indexOf(EmotiBitVariants::HARDWARE_VERSION_PREFIX), 1);
	if (vregEnablePinLogic == VregEnablePinLogic::ACTIVE_LOW)
	{
		// V2
		if (barcodeHwVersion.toInt() == 2)
		{
			Serial.println("HW validation: Passed");
			hwValidation = true;
		}
		else
		{
			Serial.println("HW validation: Failed");
			hwValidation = false;
		}
	}
	else
	{
		if (hardwareParameterTable.isSi7013Present && !hardwareParameterTable.isEepromPresent)
		{
			// V3
			if (barcodeHwVersion.toInt() == 3)
			{
				Serial.println("HW validation: Passed");
				hwValidation = true;
			}
			else
			{
				Serial.println("HW validation: Failed");
				hwValidation = false;
			}
		}
		else if (hardwareParameterTable.isEepromPresent && !hardwareParameterTable.isSi7013Present)
		{
			if (barcodeHwVersion.toInt() >= 4)
			{
				Serial.println("HW validation: Passed");
				hwValidation = true;
			}
			else
			{
				Serial.println("HW validation: Failed");
				hwValidation = false;
			}
		}
	}
	if (hardwareParameterTable.isThermopilePresent)
	{
		if (barcode.sku.equals(EmotiBitVariants::EMOTIBIT_SKU_MD))
		{
			Serial.println("SKU validation: Passed");
			skuValidation = true;
		}
		else
		{
			skuValidation = false;
			Serial.println("SKU validation Failed. Hardware SKU detected: MD");
		}
	}
	else
	{
		if (barcode.sku.equals(EmotiBitVariants::EMOTIBIT_SKU_EM))
		{
			Serial.println("SKU validation: Passed");
			skuValidation = true;
		}
		else
		{
			skuValidation = false;
			Serial.println("SKU validation Failed. Hardware SKU detected: EM");
		}
	}
	if (hwValidation && skuValidation)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool EmotiBitVersionController::writeVariantInfoToNvm(EmotiBitNvmController &emotiBitNvmController, Barcode barcode)
{
	EmotiBitVariantInfo_V1 emotiBitVariantInfo;
	EmotiBitFactoryTest::convertBarcodeToVariantInfo(barcode, emotiBitVariantInfo);
	Serial.println("Data being written to NVM");
	printEmotiBitVariantInfo(emotiBitVariantInfo);

	uint8_t* data;
	EmotiBitVariantInfo_V1* variantInfo = &emotiBitVariantInfo;
	data = (uint8_t*)variantInfo;
	uint8_t status;
	status = emotiBitNvmController.stageToWrite(EmotiBitNvmController::DataType::VARIANT_INFO, (uint8_t)EmotiBitVariantDataFormat::V1, sizeof(EmotiBitVariantInfo_V1), data, true);
	if (status == 0)
	{
		Serial.println("Variant Information written into the NVM.");
		return true;
	}
	else
	{
		Serial.print("Error writing Variant Info. ErrorCode: "); Serial.println(status);
		return false;
	}
}

bool EmotiBitVersionController::getEmotiBitVariantInfo(EmotiBitNvmController &emotiBitNvmController, EmotiBitVersion &hwVersion, String &sku, uint32_t &emotibitSerialNumber, String &barcode)
{
	uint8_t* nvmData;
	uint8_t datatypeVersion;
	uint32_t dataSize;
	uint8_t status;
	status = emotiBitNvmController.stageToRead(EmotiBitNvmController::DataType::VARIANT_INFO, datatypeVersion, dataSize, nvmData, true);
	if (status == 0)
	{
		Serial.println("Successfully read variant info from NVM");
		if (datatypeVersion == (uint8_t)EmotiBitVariantDataFormat::V0 && dataSize == 1)
		{
			EmotiBitVariantInfo_V0* variantInfo;
			variantInfo = (EmotiBitVariantInfo_V0*)nvmData;
			hwVersion = (EmotiBitVersion)(variantInfo->hwVersion);
			Serial.print("[NVM VARIANT INFO] HW version: "); Serial.println(EmotiBitVersionController::getHardwareVersion((EmotiBitVersion)hwVersion));
			sku = EmotiBitVariants::EMOTIBIT_SKU_MD;  // EmotiBit V2 and V3 were only manufactured with the MD SKU
			Serial.print("[NVM VARIANT INFO] SKU: "); Serial.println(sku);
			emotibitSerialNumber = UINT32_MAX;
			Serial.println("No emotibitSerialNumber recorded for this HW version");
		}
		else if (datatypeVersion == (uint8_t)EmotiBitVariantDataFormat::V1)
		{
			EmotiBitVariantInfo_V1* variantInfo;
			variantInfo = (EmotiBitVariantInfo_V1*)nvmData;
			hwVersion = (EmotiBitVersion)variantInfo->hwVersion;
			Serial.print("[NVM VARIANT INFO] HW version: "); Serial.println(EmotiBitVersionController::getHardwareVersion((EmotiBitVersion)hwVersion));
			sku = String(variantInfo->sku);
			Serial.print("[NVM VARIANT INFO] SKU version: "); Serial.println(sku);
			emotibitSerialNumber = variantInfo->emotibitSerialNumber;
			Serial.print("[NVM VARIANT INFO] EmotiBit Number: "); Serial.println(emotibitSerialNumber);
			// ToDo: there should some day be a versioning based on barcode format here
			barcode.reserve(15); // current barcode uses 13 characters
			barcode = sku + EmotiBitFactoryTest::BARCODE_DELIMITER;
			// convert hardware version to barcode format V04a -> V4
			String tempHwVersion = getHardwareVersion(hwVersion);
			tempHwVersion.remove(tempHwVersion.indexOf(EmotiBitVariants::HARDWARE_VERSION_PREFIX), 1); // remove "V"
			int hwVer = tempHwVersion.toInt(); // automatically discards the trailing alphabets. See arduino documentation for more details
			// add converted HW version to barcode
			barcode = barcode + EmotiBitVariants::HARDWARE_VERSION_PREFIX + String(hwVer) + EmotiBitFactoryTest::BARCODE_DELIMITER;
			// add leading zeros
			String leadingZeros = "0";
			for (int i = String(emotibitSerialNumber).length(); i < EmotiBitVariants::BARCODE_SERIAL_NUM_LENGTH; i++)
			{
				barcode += leadingZeros;
			}
			barcode += String(emotibitSerialNumber);
			Serial.print("[NVM VARIANT INFO] EmotiBit device ID: "); Serial.println(barcode);
		}
		return true;
	}
	else if (status == (uint8_t)EmotiBitNvmController::Status::MEMORY_NOT_UPDATED)
	{
		Serial.println("NVM not updated with variant info");
		return false;
	}
	else
	{
		Serial.print("Failed to read Variant Info. Error Code: "); Serial.println(status);
		return false;
	}
}

void EmotiBitVersionController::updateVersionParameterTable(TwoWire &emotibit_i2c, EmotiBitHardwareParameterTable &hardwareParameterTable)
{
	uint8_t status;
	Serial.println("Updating HardwareParameterTable");
	emotibit_i2c.beginTransmission(SI7013_I2C_ADDR);
	status = emotibit_i2c.endTransmission();
	if (status == 0)
	{
		hardwareParameterTable.isSi7013Present = true;
		Serial.println("Si-7013: Present");
	}
	else
	{
		hardwareParameterTable.isSi7013Present = false;
		Serial.print("Si-7013: Absent."); Serial.print(" I2C Error code:"); Serial.println(status);
	}

	emotibit_i2c.beginTransmission(EEPROM_I2C_ADDR);
	status = emotibit_i2c.endTransmission();
	if (status == 0)
	{
		hardwareParameterTable.isEepromPresent = true;
		Serial.println("EEPROM: Present");
	}
	else
	{
		hardwareParameterTable.isEepromPresent = false;
		Serial.print("EEPROM: Absent."); Serial.print(" I2C Error code:"); Serial.println(status);
	}

	emotibit_i2c.beginTransmission(MLX90632_I2C_ADDR);
	status = emotibit_i2c.endTransmission();
	if (status == 0)
	{
		hardwareParameterTable.isThermopilePresent = true;
		Serial.println("Thermopile: Present");
	}
	else
	{
		hardwareParameterTable.isThermopilePresent = false;
		Serial.print("Thermopile: Absent."); Serial.print(" I2C Error code:"); Serial.println(status);
	}
}

bool EmotiBitVersionController::detectVariantFromHardware(TwoWire &emotibit_i2c, EmotiBitVersion &hwVersion, String &sku)
{
	Serial.println("Detecting version from HW");
	EmotiBitHardwareParameterTable hardwareParameterTable;
	updateVersionParameterTable(emotibit_i2c, hardwareParameterTable);

	if (vregEnablePinLogic == VregEnablePinLogic::ACTIVE_LOW)
	{
		if (hardwareParameterTable.isSi7013Present && hardwareParameterTable.isThermopilePresent)
		{
			Serial.println("HW Version Detected: V2");
			hwVersion = EmotiBitVersion::V02H;
			sku = EmotiBitVariants::EMOTIBIT_SKU_MD;
			return true;
		}
		else
		{
			Serial.println("HW Version not detected.");
			return false;
		}
	}
	else
	{
		if (hardwareParameterTable.isSi7013Present && !hardwareParameterTable.isEepromPresent)
		{
			Serial.println("HW Version Detected: V3");
			hwVersion = EmotiBitVersion::V03B;
			sku = EmotiBitVariants::EMOTIBIT_SKU_MD;
			return true;
		}
		else if (hardwareParameterTable.isEepromPresent && !hardwareParameterTable.isSi7013Present)
		{
			Serial.println("HW Version Detected: V4");
			hwVersion = EmotiBitVersion::V04A;
			if (hardwareParameterTable.isThermopilePresent)
			{
				Serial.println("SKU Version Detected: MD");
				sku = EmotiBitVariants::EMOTIBIT_SKU_MD;
				return true;
			}
			else
			{
				Serial.println("SKU Version Detected: EM");
				sku = EmotiBitVariants::EMOTIBIT_SKU_EM;
				return true;
			}
		}
		else
		{
			Serial.print("Unable to detect HW version");
			return false;
		}
	}
	
}
