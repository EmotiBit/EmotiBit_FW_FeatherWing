/**************************************************************************/
/*!
    @file     EmotiBitEda.cpp
    @author   Sean Montgomery (EmotiBit)

    @mainpage Electrodermal activity (EDA) handler for EmotiBit

    @section intro_sec Introduction

    This is a library to handle electrodermal activity (EDA) on EmotiBit.

		EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    @section author Author

    Written by Sean Montgomery for EmotiBit.

    @section  HISTORY

    v1.0  - First release

    @section license License

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/

#include "EmotiBitEda.h"
#include "EmotiBitEdaCalibration.h"
// ToDo: remove this include when we solve aperiodic signal process+send
#include "EmotiBit.h"

bool EmotiBitEda::setup(EmotiBitVersionController::EmotiBitVersion version, float samplingRate,
	DoubleBufferFloat* edaBuffer, DoubleBufferFloat* edlBuffer, DoubleBufferFloat* edrBuffer,
	TwoWire* emotibitI2c, BufferFloat* edlOversampBuffer, BufferFloat* edrOversampBuffer)
{
	_emotibitVersion = version;
	_constants.samplingRate = samplingRate;

	// ToDo: Calculate digFiltAlpha

	_edaBuffer = edaBuffer;
	_edlBuffer = edlBuffer;
	_edrBuffer = edrBuffer;
	_edlOversampBuffer = edlOversampBuffer;
	_edrOversampBuffer = edrOversampBuffer;

	String output;
	output.reserve(30);

	output = "edaSeriesResistance: " + String(_constants.edaSeriesResistance);
	Serial.println(output);
	output = "samplingRate: " + String(_constants.samplingRate);
	Serial.println(output);

	if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{	
		output = "Configuring ADS ADC... ";
		Serial.println(output);

		_constants.adcBits = 16;

		// NOTE: if these values are changed in code, we should add parameters to _info.json
		_ads.setDataRate(RATE_ADS1115_128SPS);	// set to 128Hz to allow for 75Hz oversampling
		//_ads.setDataRate(RATE_ADS1115_250SPS);	// set to 250Hz to allow for 150Hz oversampling
		//_ads.setDataRate(RATE_ADS1115_475SPS);	// set to 475Hz to allow for 300Hz oversampling
		_ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV

		_constants.clipMin = -26500;
		_constants.clipMax = 26500;

		output = "enableDigitalFilter: " + String(_constants.enableDigitalFilter);
		Serial.println(output);
		output = "clipMin: " + String(_constants.clipMin);
		Serial.println(output);
		output = "clipMax: " + String(_constants.clipMax);
		Serial.println(output);
		output = "adcBits: " + String(_constants.adcBits);
		Serial.println(output);
		output = "_ads.setDataRate: " + String("RATE_ADS1115_475SPS");
		Serial.println(output);
		output = "_ads.setGain: " + String("GAIN_TWO");
		Serial.println(output);
		output = "edaTransformSlope: " + String(_constants_v4_plus.edaTransformSlope);
		Serial.println(output);
		output = "edaTransformIntercept: " + String(_constants_v4_plus.edaTransformIntercept);
		Serial.println(output);

		return _ads.begin(0x48, emotibitI2c, false); // callBegin -> false. the i2c wire has already been init in setup

	}
	else if (_emotibitVersion <= EmotiBitVersionController::EmotiBitVersion::V03B)
	{
		output = "Configuring SAMD ADC... ";
		Serial.println(output);

		_constants.adcBits = 12;
		analogReadResolution(_constants.adcBits);
		_constants_v2_v3.adcRes = pow(2, _constants.adcBits) - 1;

		_constants.clipMin = 10;
		_constants.clipMax = _constants_v2_v3.adcRes - 20;

		_constants.enableDigitalFilter = true;
		
		output = "enableDigitalFilter: " + String(_constants.enableDigitalFilter);
		Serial.println(output);
		output = "clipMin: " + String(_constants.clipMin);
		Serial.println(output);
		output = "clipMax: " + String(_constants.clipMax);
		Serial.println(output);
		output = "adcBits: " + String(_constants.adcBits);
		Serial.println(output);
		output = "vcc: " + String(_constants_v2_v3.vcc);
		Serial.println(output);
		output = "edrAmplification: " + String(_constants_v2_v3.edrAmplification);
		Serial.println(output);
		output = "vRef1: " + String(_constants_v2_v3.vRef1);
		Serial.println(output);
		output = "vRef2: " + String(_constants_v2_v3.vRef2);
		Serial.println(output);
		output = "feedbackAmpR: " + String(_constants_v2_v3.feedbackAmpR);
		Serial.println(output);
		output = "crossoverFilterFreq: " + String(_constants_v2_v3.crossoverFilterFreq);
		Serial.println(output);
		output = "adcRes: " + String(_constants_v2_v3.adcRes);
		Serial.println(output);
		output = "edlPin: " + String(_constants_v2_v3.edlPin);
		Serial.println(output);
		output = "edrPin: " + String(_constants_v2_v3.edrPin);
		Serial.println(output);
		output = "isrOffsetCorr: " + String(_constants_v2_v3.isrOffsetCorr);
		Serial.println(output);

		return true;
	}

	return false;
}


bool EmotiBitEda::stageCalibStorage(EmotiBitNvmController * nvmController, String &edaCalibPacket, bool autoSync)
{
	if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		uint8_t dataVersion;
		EmotiBitEdaCalibration::RawValues_V2 rawVals;
		if (EmotiBitEdaCalibration::unpackCalibPacket(edaCalibPacket, dataVersion, rawVals))
		{
			Serial.println("Writing calibration data:");
			if (dataVersion == EmotiBitEdaCalibration::V2)
			{
				//Serial.println(edaCalibPacket);
				EmotiBitEdaCalibration::print(rawVals);

				Serial.println("Staging to write...");
				if(nvmController == nullptr) return false;
				uint8_t status = nvmController->stageToWrite(EmotiBitNvmController::DataType::EDA, dataVersion, sizeof(EmotiBitEdaCalibration::RawValues_V2), (uint8_t *)(&rawVals), autoSync);

				if (status == (uint8_t)EmotiBitNvmController::Status::SUCCESS)
				{
					Serial.println("sucess");
					return true;
				}
				else
				{
					Serial.println("nvmController->stageToWrite() failed: " + String(status));
					return false;
				}
			}
			else
			{
				// Write cases for other dataVersions
				Serial.println("stageCalibStorage() failed: version not supported");
				return false;
			}
		}
		else
		{
			Serial.println("unpackCalibPacket() failed");
			return false;
		}
	}
	else
	{
		Serial.println("Storing calibration data on V02/V03 HW requires firmware v1.2.86");  
		return false;
	}

}


bool EmotiBitEda::stageCalibLoad(EmotiBitNvmController * nvmController, bool autoSync)
{
	uint8_t dataVersion;
	uint32_t dataSize;
	uint8_t* data = nullptr;
	if(nvmController == nullptr) return false;
	uint8_t status = nvmController->stageToRead(EmotiBitNvmController::DataType::EDA, dataVersion, dataSize, data, autoSync);

	if (status != (uint8_t)EmotiBitNvmController::Status::SUCCESS || dataSize == 0)
	{
		Serial.print("[status=");
		Serial.print(status);
		Serial.print("] ");
		return false;
	}
	if (dataVersion == EmotiBitEdaCalibration::V2 && dataSize == sizeof(EmotiBitEdaCalibration::RawValues_V2))
	{
		EmotiBitEdaCalibration::RawValues_V2 *rawVals = (EmotiBitEdaCalibration::RawValues_V2 *)data;
		EmotiBitEdaCalibration::print(*rawVals);
		EmotiBitEdaCalibration::calculate(*rawVals, _constants_v4_plus.edaTransformSlope, _constants_v4_plus.edaTransformIntercept);
		Serial.print("edaTransformSlope = ");
		Serial.println(_constants_v4_plus.edaTransformSlope);
		Serial.print("edaTransformIntercept = ");
		Serial.println(_constants_v4_plus.edaTransformIntercept);
	}
	else if (dataVersion == EmotiBitEdaCalibration::V0 && dataSize == sizeof(EmotiBitEdaCalibration::RawValues_V0))
	{
		EmotiBitEdaCalibration::RawValues_V0 *rawVals = (EmotiBitEdaCalibration::RawValues_V0 *)data;
		EmotiBitEdaCalibration::print(*rawVals);
		EmotiBitEdaCalibration::calculate(*rawVals, _constants_v2_v3.vRef1, _constants_v2_v3.vRef2, _constants_v2_v3.feedbackAmpR);
		Serial.print("vRef1 = ");
		Serial.println(_constants_v2_v3.vRef1);
		Serial.print("vRef2 = ");
		Serial.println(_constants_v2_v3.vRef2);
		Serial.print("feedbackAmpR = ");
		Serial.println(_constants_v2_v3.feedbackAmpR);
	}
	else
	{
		if (data != nullptr)
		{
			delete[] data;
		}
		Serial.print("[");
		Serial.print("dataSize=");
		Serial.print(dataSize);
		Serial.print(", dataVersion=");
		Serial.print(dataVersion);
		Serial.print("] ");
		return false;
	}

	if (data != nullptr)
	{
		delete[] data;
	}

	return true;
}

uint8_t EmotiBitEda::readData()
{
	int8_t status = 0;
	float edlTemp;	// Electrodermal Activity 
	float edrTemp;	// Electrodermal Activity 


	if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		if(_edlOversampBuffer == nullptr || _edlBuffer == nullptr) return 16; // BufferFloat::ERROR_PTR_NULL = 16. But, any non-zero value sohuld work.
		// Code to debug missed conversions
		//static uint16_t completed = 0;
		//static uint16_t total = 0;

		// Reads EDA data from ADS1113
		if (_ads.conversionComplete())
		{
			edlTemp = _ads.getLastConversionResults();
			_ads.startADC_Differential_0_1();
			// ToDo: consider how to utilize edl & edr buffers for different EmotiBit versions to minimize RAM footprint & code clarity
			status = status | _edlOversampBuffer->push_back(edlTemp);

			// Check for clipping
			if (edlTemp < _constants.clipMin || edlTemp > _constants.clipMax)
			{
				status = status | _edlOversampBuffer->incrClippedCount();
			}

			// Code to debug missed conversions
			//completed++;
			//total++;
			//if (completed == 100)
			//{
			//	Serial.println(": " + String(completed) + "/" + String(total));
			//	completed = 0;
			//	total = 0;
			//}
		}
		else
		{
			_edlBuffer->incrOverflowCount(DoubleBufferFloat::BufferSelector::IN);	// Count an overflow event if conversion was late
			
			// Code to debug missed conversions
			//total++;
			//Serial.print(String(total) + ",");
		}

		// Check if ready for downsampling
		if (_edlOversampBuffer->isFull()) 
		{
			// ToDo: Consider how to have version-specific changes in oversampling -- Using isFull/capacity of _edlOversampBuffer won't work
			// Note: data is saved in _edlBuffer to make factory test calibration easy
			// ToDo: Consider refactoring to use _edaBuffer
			status = status | _edlBuffer->downsample(_edlOversampBuffer);
			_edlOversampBuffer->clear();
		}
	}
	else
	{
		// Reads EDA data from ADC
		if(_edlOversampBuffer == nullptr || _edrOversampBuffer == nullptr || _edlBuffer == nullptr || _edrBuffer == nullptr) return 16; // BufferFloat::ERROR_PTR_NULL = 16. But, any non-zero value sohuld work.
		// Check EDL and EDR voltages for saturation
#if defined(ARDUINO_FEATHER_ESP32)
		// analogReadMillis is much more accurate for ESP
		// and needs to be converted to ADC bits to follow EDA pipeline
		static const float millisToBits = ((float)_constants_v2_v3.adcRes) / _constants_v2_v3.vcc / 1000.f;
		edlTemp = analogReadMilliVolts(_constants_v2_v3.edlPin) * millisToBits;
		edrTemp = analogReadMilliVolts(_constants_v2_v3.edrPin) * millisToBits;
#else
		edlTemp = analogRead(_constants_v2_v3.edlPin);
		edrTemp = analogRead(_constants_v2_v3.edrPin);
#endif

		status = status | _edlOversampBuffer->push_back(edlTemp);
		status = status | _edrOversampBuffer->push_back(edrTemp);

		// Check for clipping
		if (edlTemp < _constants.clipMin || edlTemp > _constants.clipMax)
		{
			status = status | _edlOversampBuffer->incrClippedCount();
		}
		if (edrTemp < _constants.clipMin || edrTemp > _constants.clipMax)
		{
			status = status | _edrOversampBuffer->incrClippedCount();
		}

		// Check if ready for downsampling
		if (_edlOversampBuffer->isFull())
		{
			status = status | _edlBuffer->downsample(_edlOversampBuffer);
			_edlOversampBuffer->clear();
		}
		if (_edrOversampBuffer->isFull())
		{
			status = status | _edrBuffer->downsample(_edrOversampBuffer);
			_edrOversampBuffer->clear();
		}
	}
	_readFinishedTime = micros();
	return status;
}


bool EmotiBitEda::processData()
{
	if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		if(_edlBuffer == nullptr || _edaBuffer == nullptr) return false;
		size_t n;
		float * edlData;
		float edaTemp;
		uint32_t timestamp;
		n = _edlBuffer->getData(&edlData, &timestamp, true);
		for (size_t i = 0; i < n; i++)
		{
			// ToDo: Calculate slope/intercept to avoid expensive division in loop
			edaTemp = edlData[i] * _constants_v4_plus.edaTransformSlope + _constants_v4_plus.edaTransformIntercept;
			edaTemp = max(100.f , edaTemp); // Clamp the EDA measurement at 100 Ohm (0.0001 Siemens)
			edaTemp = 1000000.f / edaTemp;
			// ToDo: Consider filtering
			_edaBuffer->push_back(edaTemp, &timestamp);
		}
		// Transfer clipped counts
		_edaBuffer->incrClippedCount(DoubleBufferFloat::BufferSelector::IN, 
			_edlBuffer->getClippedCount(DoubleBufferFloat::BufferSelector::OUT)
		);

		// Transfer overflow counts
		_edaBuffer->incrOverflowCount(DoubleBufferFloat::BufferSelector::IN,
			_edlBuffer->getOverflowCount(DoubleBufferFloat::BufferSelector::OUT)
		);

		// Swap EDA buffer
		_edaBuffer->swap();
	}
	else
	{
		if(_edlBuffer == nullptr || _edrBuffer == nullptr || _edaBuffer == nullptr) return false;
		size_t n, edlN, edrN;
		float *edlData, *edrData;
		uint32_t edlTs, edrTs;
		float edlTemp, edrTemp, edaTemp;

		// Wait for readData() to complete to avoid EDL/EDR size mismatch
		// NOTE: this can create a small main loop delay
		// This wouldn't be necessary with a ring buffer

		static const int samplingInterval = 1000000 / (_constants.samplingRate * _edrOversampBuffer->capacity());
		static const int minSwapTime = max(500, min(samplingInterval / 10, 3500));

		//Serial.println("window: " + String(samplingInterval - (micros() - _thermReadFinishedTime)));
		unsigned long int waitStart = micros();
		unsigned long int waitEnd = micros();
		unsigned long int readFinishedTime = _readFinishedTime;
		while (samplingInterval - (waitEnd - readFinishedTime) < minSwapTime)
		{
			// Wait until we have at least minSwapTime usec to do swap
			//Serial.println("WAIT");
			if (waitEnd - waitStart > 100000)
			{
				Serial.println("Timeout waiting for _readFinishedTime");
				break;
			}
			waitEnd = micros();
			readFinishedTime = _readFinishedTime;
		}
		
		// Swap EDL and EDR buffers with minimal delay to avoid size mismatch
		unsigned long int swapStart = micros();
		_edlBuffer->swap();
		_edrBuffer->swap();
		unsigned long int swapEnd = micros();
		//Serial.println("swap: " + String(swapEnd - swapStart));

		// Get pointers to the data buffers
		edlN = _edlBuffer->getData(&edlData, &edlTs, false);
		edrN = _edrBuffer->getData(&edrData, &edrTs, false);

		if (edlN != edrN)
		{
			Serial.println("WARNING: therm0AMB and therm0Sto buffers different sizes");
			Serial.println("minSwapTime: " + String(minSwapTime));
			Serial.println("_readFinishedTime: " + String(_readFinishedTime));
			Serial.println("readFinishedTime: " + String(readFinishedTime));
			Serial.println("waitEnd: " + String(waitEnd));
			Serial.println("waitStart: " + String(waitStart));
			Serial.println("micros(): " + String(micros()));
			Serial.println("window: " + String(samplingInterval - (waitEnd - readFinishedTime)));
			Serial.println("swap: " + String(swapEnd - swapStart));
			Serial.println("edlN: " + String(edlN));
			Serial.println("edrN: " + String(edrN));
			// ToDo: Consider how to manage buffer size differences
			// One fix option is to switch to ring buffers instead of double buffers
			
			// Add overflow event(s) to account for the mismatched sizes
			size_t mismatch = abs(((int)edlN) - ((int)edrN));
			_edlBuffer->incrOverflowCount(DoubleBufferFloat::BufferSelector::OUT, mismatch);
			_edrBuffer->incrOverflowCount(DoubleBufferFloat::BufferSelector::OUT, mismatch);
		}

		// Loop through the data buffers and perform calculations
		n = min(edlN, edrN);
		for (uint8_t i = 0; i < n; i++)
		{
			edlTemp = edlData[i];
			edrTemp = edrData[i];

			// Correction for ADC value changes when ISR running
			edlTemp -= _constants_v2_v3.isrOffsetCorr;
			edrTemp -= _constants_v2_v3.isrOffsetCorr;

			// Perform data conversion
			// Convert ADC to Volts
			edlTemp = edlTemp * _constants_v2_v3.vcc / ((float) _constants_v2_v3.adcRes);
			edrTemp = edrTemp * _constants_v2_v3.vcc / ((float) _constants_v2_v3.adcRes);

			// In-place update buffers to Volts
			edlData[i] = edlTemp;
			edrData[i] = edrTemp;

			// EDL Digital Filter to remove noise
			if (_constants_v2_v3.crossoverFilterFreq > 0)// use only is a valid crossover freq is assigned
			{
				static DigitalFilter filterEda(DigitalFilter::FilterType::IIR_LOWPASS, _constants.samplingRate, _constants_v2_v3.crossoverFilterFreq);
				if (_constants.enableDigitalFilter)
				{
					edlTemp = filterEda.filter(edlTemp);
				}
			}

			// Link to diff amp biasing: https://ocw.mit.edu/courses/media-arts-and-sciences/mas-836-sensor-technologies-for-interactive-environments-spring-2011/readings/MITMAS_836S11_read02_bias.pdf
			edaTemp = (edrTemp - _constants_v2_v3.vRef2) / _constants_v2_v3.edrAmplification;	// Remove VGND bias and amplification from EDR measurement
			edaTemp = edaTemp + edlTemp;                     // Add EDR to EDL in Volts

			if (edaTemp - _constants_v2_v3.vRef1 < 0.000086f)
			{
				edaTemp = 0.0001f; // Clamp the EDA measurement at 100 Ohm (0.0001 Siemens)
			}
			else
			{
				edaTemp = _constants_v2_v3.vRef1 / 
					((_constants_v2_v3.feedbackAmpR * (edaTemp - _constants_v2_v3.vRef1))
						- (_constants.edaSeriesResistance * _constants_v2_v3.vRef1)
					);
			}

			edaTemp = edaTemp * 1000000.f; // Convert to uSiemens

			// Push calculated EDA value
			_edaBuffer->push_back(edaTemp, &edrTs);
		}
		// Transfer clipped counts
		_edaBuffer->incrClippedCount(DoubleBufferFloat::BufferSelector::IN,
			max(
				_edlBuffer->getClippedCount(DoubleBufferFloat::BufferSelector::OUT),
				_edrBuffer->getClippedCount(DoubleBufferFloat::BufferSelector::OUT)
			)
		);

		// Transfer overflow counts
		_edaBuffer->incrOverflowCount(DoubleBufferFloat::BufferSelector::IN,
			max(
				_edlBuffer->getOverflowCount(DoubleBufferFloat::BufferSelector::OUT),
				_edrBuffer->getOverflowCount(DoubleBufferFloat::BufferSelector::OUT)
			)
		);

		// Swap EDA buffer
		_edaBuffer->swap();
	}
	return true;
}

bool EmotiBitEda::writeInfoJson(File &jsonFile)
{
	if(_edlOversampBuffer == nullptr) return false;
	const uint16_t bufferSize = 1024;
	{
		// Parse the root object
		StaticJsonDocument<bufferSize> jsonDoc;
		JsonObject root = jsonDoc.to<JsonObject>();
		const uint8_t nInfo = 1;
		JsonObject infos[nInfo];
		JsonArray typeTags[nInfo];
		JsonObject setups[nInfo];
		uint8_t i = 0;
		infos[i] = root.createNestedObject("info");
		infos[i]["name"] = "ElectrodermalActivity";
		infos[i]["type"] = "ElectrodermalActivity";
		typeTags[i] = infos[i].createNestedArray("typeTags");
		typeTags[i].add("EA");
		infos[i]["channel_count"] = 1;
		infos[i]["nominal_srate"] = _constants.samplingRate;
		infos[i]["channel_format"] = "float";
		infos[i]["units"] = "microsiemens";
		setups[i] = infos[i].createNestedObject("setup");
		setups[i]["eda_series_resistance"] = _constants.edaSeriesResistance;
		setups[i]["adc_bits"] = _constants.adcBits;
		setups[i]["enable_digital_filter"] = _constants.enableDigitalFilter;
		setups[i]["samples_averaged"] = _edlOversampBuffer->capacity();
		setups[i]["oversampling_rate"] = _edlOversampBuffer->capacity() * _constants.samplingRate;
		if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
		{
			setups[i]["eda_transform_slope"] = _constants_v4_plus.edaTransformSlope;
			setups[i]["eda_transform_intercept"] = _constants_v4_plus.edaTransformIntercept;
		} 
		else
		{
			setups[i]["voltage_reference_1"] = _constants_v2_v3.vRef1;
			setups[i]["voltage_reference_2"] = _constants_v2_v3.vRef2;
			setups[i]["EDA_feedback_amp_resistance"] = _constants_v2_v3.feedbackAmpR;
			setups[i]["EDR_amplification"] = _constants_v2_v3.edrAmplification;
			setups[i]["EDA_crossover_filter_frequency"] = _constants_v2_v3.crossoverFilterFreq;
			setups[i]["VCC"] = _constants_v2_v3.vcc;
			setups[i]["ISR_ADC_offset_correction"] = _constants_v2_v3.isrOffsetCorr;
		}

		serializeJson(jsonDoc, jsonFile);
	}
	jsonFile.print(",");
	// Skin Coductance Response Amplitude
	{
		// Parse the root object
		StaticJsonDocument<bufferSize> jsonDoc;
		JsonObject root = jsonDoc.to<JsonObject>();
		const uint8_t nInfo = 1;
		JsonObject infos[nInfo];
		JsonArray typeTags[nInfo];
		JsonObject setups[nInfo];
		uint8_t i = 0;
		infos[i] = root.createNestedObject("info");
		infos[i]["name"] = "SkinConductanceResponseAmplitude";
		infos[i]["type"] = "ElectrodermalActivity";
		typeTags[i] = infos[i].createNestedArray("typeTags");
		typeTags[i].add(EmotiBitPacket::TypeTag::SKIN_CONDUCTANCE_RESPONSE_AMPLITUDE);
		infos[i]["channel_count"] = 1;
		infos[i]["channel_format"] = "float";
		infos[i]["units"] = "microsiemens";
		serializeJson(jsonDoc, jsonFile);
	}
	jsonFile.print(",");
	// Skin Coductance Response Frequency
	{
		// Parse the root object
		StaticJsonDocument<bufferSize> jsonDoc;
		JsonObject root = jsonDoc.to<JsonObject>();
		const uint8_t nInfo = 1;
		JsonObject infos[nInfo];
		JsonArray typeTags[nInfo];
		JsonObject setups[nInfo];
		uint8_t i = 0;
		infos[i] = root.createNestedObject("info");
		infos[i]["name"] = "SkinConductanceResponseFrequency";
		infos[i]["type"] = "ElectrodermalActivity";
		typeTags[i] = infos[i].createNestedArray("typeTags");
		typeTags[i].add(EmotiBitPacket::TypeTag::SKIN_CONDUCTANCE_RESPONSE_FREQ);
		infos[i]["channel_count"] = 1;
		infos[i]["nominal_srate"] = _constants.samplingRate / _constants.EDA_SAMPLES_PER_SCR_FREQ_OUTPUT;
		infos[i]["channel_format"] = "float";
		infos[i]["units"] = "count/min";
		serializeJson(jsonDoc, jsonFile);
	}
	jsonFile.print(",");
	// Skin Coductance Response Rise Time
	{
		// Parse the root object
		StaticJsonDocument<bufferSize> jsonDoc;
		JsonObject root = jsonDoc.to<JsonObject>();
		const uint8_t nInfo = 1;
		JsonObject infos[nInfo];
		JsonArray typeTags[nInfo];
		JsonObject setups[nInfo];
		uint8_t i = 0;
		infos[i] = root.createNestedObject("info");
		infos[i]["name"] = "SkinConductanceResponseRiseTime";
		infos[i]["type"] = "ElectrodermalActivity";
		typeTags[i] = infos[i].createNestedArray("typeTags");
		typeTags[i].add(EmotiBitPacket::TypeTag::SKIN_CONDUCTANCE_RESPONSE_RISE_TIME);
		infos[i]["channel_count"] = 1;
		infos[i]["channel_format"] = "float";
		infos[i]["units"] = "secs";
		serializeJson(jsonDoc, jsonFile);
	}
	return true;
}

void EmotiBitEda::setAdcIsrOffsetCorr(float isrOffsetCorr)
{
	_constants_v2_v3.isrOffsetCorr = isrOffsetCorr;
}

void EmotiBitEda::processElectrodermalResponse(EmotiBit* emotibit)
{
	if(_edaBuffer == nullptr || emotibit == nullptr) return;
	static const float samplingFrequency = _constants.samplingRate;
	static const float timePeriod = 1.f / samplingFrequency; // in secs
	static const float scrFreqTimePeriod = _constants.EDA_SAMPLES_PER_SCR_FREQ_OUTPUT  * timePeriod; // timePeriod of the signal scr:FREQ
	static DigitalFilter edaLowpassFilter(DigitalFilter::FilterType::IIR_LOWPASS, samplingFrequency, 1); // for bandpassing eda
	static DigitalFilter edaHighpassFilter(DigitalFilter::FilterType::IIR_HIGHPASS, samplingFrequency, 0.2); // for bandpassing eda
	static DigitalFilter scrFrequencyFilter(DigitalFilter::FilterType::IIR_LOWPASS, 1.f/(scrFreqTimePeriod), 1.f/180.f); // lowPass the calculated scr:FREQ
	float* data;
	uint32_t timestamp;
	size_t dataSize;
	static uint32_t riseTimeSampleCount  = 0; // to count number of samples between scr onset and peak
	static const float threshold = 5000; // in Ohms. detect an onset if (delta eda) > threshold
	static bool onsetDetected = false;
	static float scrAmplitudeOnOnset = 0;  // record the base of the scr peak
	static uint32_t onsetTime;  // in mS
	static float responseFreq;  // in mins
	static const uint8_t APERIODIC_DATA_LEN = 1; // used in pacet header
	static uint16_t scrEventCount = 0; // counter for #scr events
	static uint16_t scrFreqOutputCounter = 0; // counter for eda samples
	static float lastFilteredEdaValue = 0;
	static float filteredEda = 0;
	static float previousEdaValue = 0;

	// Load latest EDA signal
	if (_edaBuffer != nullptr)
	{
		dataSize = _edaBuffer->getData(&data, &timestamp, false);
	}
	// uncomment below comment block to TX new signals to the Oscilloscope
	/*
	// testing bandpass
	static DigitalFilter edaLowpassFilterTest(DigitalFilter::FilterType::IIR_LOWPASS, samplingFrequency, 1.5); // for bandpassing eda
	static DigitalFilter edaHighpassFilterTest(DigitalFilter::FilterType::IIR_HIGHPASS, samplingFrequency, 0.2); // for bandpassing eda
	float tempFiltered[10];
	if (dataSize < 10)
	{
		for (int i = 0; i < dataSize; i++)
		{
			tempFiltered[i] = edaLowpassFilterTest.filter(data[i]);
			tempFiltered[i] = edaHighpassFilterTest.filter(tempFiltered[i]);
		}
		// send a separate stream U1 to view bandpass signal.
		// oscilloscope XML will have to be modified accordingly. oscillosopce min. v1.3.0 required.
		emotibit->addPacket(timestamp, "U1", tempFiltered, dataSize, 4); // 4 = precision
	}
	*/
	for (size_t i = 0; i < dataSize; i++)
	{
		scrFreqOutputCounter++;
		// bandpass filter eda
		lastFilteredEdaValue = filteredEda;
		filteredEda = edaLowpassFilter.filter(data[i]);
		filteredEda = edaHighpassFilter.filter(filteredEda);
		if (!onsetDetected)
		{
			// convert uS to Ohms for thresholding
			float instSkinResistance = 1000000.f / data[i];  // Instantaneous skin Resistance
			float baselineSkinResistance = 1000000.f / (data[i] - filteredEda); // Resistance before Response
			// detect onset if threshold is crossed
			if (baselineSkinResistance - instSkinResistance > threshold)
			{
				// Onset detected!
				onsetDetected = true;
				//back calculate time based on buffer timestamp
				uint32_t timeAdjustment = (dataSize - i - 1) * timePeriod * 1000; // in mS
				onsetTime = timestamp - timeAdjustment; // mS
				scrAmplitudeOnOnset = data[i];  // record the base of the EDA peak
				// reset counter for next response
				riseTimeSampleCount = 0;
			}
		}
		else
		{
			// update sample count after onset detect
			riseTimeSampleCount++;
			// wait for a peak
			if (filteredEda < lastFilteredEdaValue)
			{
				// peak detected. calculate rise time and amplitude
				onsetDetected = false;
				// if the downslope is detected at the first sample of the new buffer, peak = last sample of last buffer
				float amplitude = previousEdaValue - scrAmplitudeOnOnset;
				// only record SCR event if > 0
				if (amplitude > 0)
				{
					float riseTime = (float)(riseTimeSampleCount - 1)  * timePeriod; // Samples since onset*timePeriod (in Secs)
					// increment SCR event count after peak has been detected
					scrEventCount++;
					// Add packet to the output
					emotibit->addPacket(onsetTime, EmotiBitPacket::TypeTag::SKIN_CONDUCTANCE_RESPONSE_AMPLITUDE, &amplitude, APERIODIC_DATA_LEN, 6); // 6 = precision
					emotibit->addPacket(onsetTime, EmotiBitPacket::TypeTag::SKIN_CONDUCTANCE_RESPONSE_RISE_TIME, &riseTime, APERIODIC_DATA_LEN, 6); // 6 = precision
				}
			}
		}
		// check if it time to send scr:FREQ packet
		if (scrFreqOutputCounter == _constants.EDA_SAMPLES_PER_SCR_FREQ_OUTPUT )
		{
			// calculate number of scr events in time period
			float responseFreq = (scrEventCount / scrFreqTimePeriod) * 60.f; // scr:FREQ in count/min
			responseFreq = scrFrequencyFilter.filter(responseFreq);
			uint32_t timstamp = millis();
			// send data
			emotibit->addPacket(timestamp, EmotiBitPacket::TypeTag::SKIN_CONDUCTANCE_RESPONSE_FREQ, &responseFreq, APERIODIC_DATA_LEN, 4); // 4 = precision
			scrEventCount = 0;
			scrFreqOutputCounter = 0;
		}
		// store the current EDA value to be used in the next calculation
		previousEdaValue = data[i];
	}
}