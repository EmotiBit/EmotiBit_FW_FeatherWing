#include "EdaCorrection.h"


bool EdaCorrection::begin(uint8_t emotiBitVersion)
{
	Serial.println("################################");
	Serial.println("#####  EDA Correction Mode  ####");
	Serial.println("################################\n");
	Serial.println("If you are an EmotiBit Tester, enter A to continue.");
	Serial.println("Otherwise enter any key to exit EDA Correction Mode");
	while (!Serial.available());
	char choice = Serial.read();
	if (choice == 'A')
	{
		// clear any extra characters entered in the serial monitor
		while (Serial.available())
		{
			Serial.read();
		}
		EdaCorrection::Status status;
		status = enterUpdateMode(emotiBitVersion, EdaCorrection::OtpDataFormat::DATA_FORMAT_0);
		if (status == Status::SUCCESS)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		Serial.println("Exiting EDA Calibration mode");
		// clear any extra characters entered in the serial monitor
		while (Serial.available())
		{
			Serial.read();
		}
		delay(2000);
		return false;
	}
	
}

EdaCorrection::Status EdaCorrection::enterUpdateMode(uint8_t emotiBitVersion, EdaCorrection::OtpDataFormat dataFormat)
{
	
	Serial.println("Enabling Mode::UPDATE");
	_mode = EdaCorrection::Mode::UPDATE;
	_emotiBitVersion = emotiBitVersion;
	otpDataFormat = dataFormat;
	Serial.print("\nEnter X to perform Actual OTP R/W\n");
	Serial.println("Enter any other key to default into DUMMY MODE");
	while (!Serial.available());
	char choice = Serial.read();
	if(choice == 'X')
	{
		Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		Serial.println("!!!! Actually writing to the OTP  !!!!");
		Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	}
	else
	{
		dummyWrite = true;
		Serial.println("###################");
		Serial.println("## IN DUMMY MODE ##");
		Serial.println("###################");
	}
	// remove any extra chharacters entered
	while (Serial.available())
	{
		Serial.read();
	}
	Serial.println("Initializing state machine. State: SENSOR_CONNECTION_CHECK");
	Serial.println("Follow the instructions to obtain calibration values:\n");
	Serial.println("- The tester should now the use the EmotiBit with the High precision EDA test rig. ");
	Serial.println("- You should use the EmotiBit Oscilloscope in TESTING MODE, connect to the EmotiBit attached to the EDA test rig.");
	Serial.println("- Record EDL and EDR values using the Oscilloscope.");
	Serial.println("- Once you have the appropriate data, please plug in the serial cable and enter the data.");
	Serial.println("- Copy and paste the 5 EDL values and 5 EDR values in the format shown below");
	Serial.println("  - EDL_0R, EDL_10K, EDL_100K, EDL_1M, EDL_10M, EDR_0R, EDR_10K, EDR_100K, EDR_1M, EDR_10M");
	Serial.println("\n\n- Enter any character to proceed with execution\n\n");
	while (!Serial.available()); 
	// remove any extra chharacters entered
	while (Serial.available())
	{
		Serial.read();
	}
	Serial.println("The EmotiBit will resume normal operation. UNPLUG THE USB CABLE. ");
	Serial.println("The tester should open the EmotiBit Oscilloscope in TESTING MODE");
	Serial.println("Follow the steps listed above\n");
	uint8_t msgTimer = 5;
	while (msgTimer > 0)
	{
		Serial.print(msgTimer); Serial.print("\t");
		delay(1000);
		msgTimer--;
	}
	progress = EdaCorrection::Progress::PING_USER;
	return EdaCorrection::Status::SUCCESS;

}


bool EdaCorrection::getEdaCalibrationValues(Si7013* si7013, float &vref1, float &vref2, float &Rfeedback)
{
	if (_mode == Mode::NORMAL)
	{
		Status status;
		status = readFromOtp(si7013);
		if (status == Status::FAILURE)
		{
			if (!isSensorConnected)
			{
				Serial.println("Sensor not connected.");
				return false;
			}
			else if (!isOtpValid)
			{
				Serial.println("OTP has not been updated. Not performing correction calculation.");
				Serial.print("Perform EDA correction first.");
				Serial.println(" Using EDA without correction");
				progress = EdaCorrection::Progress::FINISH;// _mode = NORMAL
				return false;
			}
		}
#ifdef ACCESS_MAIN_ADDRESS
		else
		{
			Serial.println("-------------------------- EDA CALIBRATION DATA ------------------------------------");
			Serial.print("The EmotiBit Version stored for EDA correction is: "); Serial.println((int)_emotiBitVersion);
			Serial.print("The data format version stored for EDA correction is: "); Serial.println((int)otpDataFormat);
			calcEdaCorrection();
			Serial.print("Estimated Rskin values BEFORE correction:|");
			float RskinEst = 0;
			for (int i = 0; i < NUM_EDL_READINGS; i++)
			{
				RskinEst = (((correctionData.edlReadings[i] / vref1) - 1)*Rfeedback);
				Serial.print(RskinEst); Serial.print(" | ");
			}
			vref1 = correctionData.vRef1;// updated vref1
			vref2 = correctionData.vRef2;// updated vref2
			Rfeedback = correctionData.Rfb;// updated edaFeedbackAmpR
			Serial.print("\nEstimated Rskin values AFTER correction: |");
			for (int i = 0; i < NUM_EDL_READINGS; i++)
			{
				RskinEst = (((correctionData.edlReadings[i] / vref1) - 1)*Rfeedback);
				Serial.print(RskinEst); Serial.print(" | ");
			}
			if (dummyWrite)
			{
				Serial.println("\nupdated emotibit class with these values");
				Serial.println("\nYou can now use this EmotiBit without restarting to measure the EDA test rig values");
				Serial.println("-------------------------- EDA CALIBRATION DATA ------------------------------------");
			}
			else
			{
				Serial.println("\n-------------------------- EDA CALIBRATION DATA ------------------------------------");
			}
			progress = EdaCorrection::Progress::FINISH;// _mode = NORMAL
			return true;
		}
#else
		else
		{
			Serial.print("The value read from test Address space is: "); Serial.println(otpData.inFloat,6);
			return true;
		}
#endif
	}
	else
	{
		Serial.println("In UPDATE mode. EDA will be calibrated.");
		return false;
	}

}

EdaCorrection::Mode EdaCorrection::getMode()
{
	return _mode;
}

EdaCorrection::Status EdaCorrection::getFloatFromString()
{
	float input[2 * NUM_EDL_READINGS];
	for (int i = 0; i < 2 * NUM_EDL_READINGS; i++)
	{
		if (Serial.available())
		{
			String splitString = Serial.readStringUntil(',');
			input[i] = splitString.toFloat();
		}
		else // not enough data entered
		{
			Serial.println("Fewer than expected data points");
			return EdaCorrection::Status::FAILURE;
		}
	}
	if (Serial.available()) // more data that should be parsed present.
	{
		Serial.println("More than expected data points");
		while (Serial.available())
			Serial.read();
		return EdaCorrection::Status::FAILURE;
	}

	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		correctionData.edlReadings[i] = input[i];
	}

	for (int i = NUM_EDL_READINGS; i < 2 * NUM_EDL_READINGS; i++)
	{
		correctionData.edrReadings[i - NUM_EDL_READINGS] = input[i];
	}
	
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		correctionData.vRef2 += correctionData.edrReadings[i];
	}
	correctionData.vRef2 = correctionData.vRef2 / NUM_EDL_READINGS;
	
	return EdaCorrection::Status::SUCCESS;
}

void EdaCorrection::update(Si7013* si7013, float &vref1, float &vref2, float &Rfeedback)
{
	if (getMode() == EdaCorrection::Mode::UPDATE)
	{
		if (progress == EdaCorrection::Progress::PING_USER)
		{
			Serial.println("Si7013 detected. Proceeding to WAITING_FOR_SERIAL_DATA");
			progress = Progress::WAITING_FOR_SERIAL_DATA;
		}
		else if (progress == EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA)
		{
#ifdef ACCESS_MAIN_ADDRESS
			if (Serial.available())
			{
				if (getFloatFromString() != EdaCorrection::Status::SUCCESS)
				{
					Serial.println("Please check and enter again");
					Serial.println("Expected Format");
					Serial.println("EDL_0R, EDL_10K, EDL_100K, EDL_1M, EDL_10M, vRef2_0R, vRef2_10K, vRef2_100K, vRef2_1M, vRef2_10M");
				}
				else
				{
					Serial.println("Serial data detected.");
					Serial.println("####################");
					Serial.println("### EDA TESTING  ###");
					Serial.println("####################");
					if (dummyWrite)
					{
						Serial.println("#### DUMMY MODE ####");
					}
					else
					{
						Serial.println("!!! OTP MODE !!!");
						//ToDo: think about removing the serial prints. This removes the dependency on the VersionController class
#ifdef USE_ALT_SI7013
						Serial.println("!!! Writing to Alternate(External) I2C sensor !!!");
#else
						Serial.println("!!! Writing to main EmotiBit I2C sensor !!!");
#endif

						Serial.println("!!! Writing to Main Address space !!!");

					}
					echoEdaReadingsOnScreen();
					progress = EdaCorrection::Progress::WAITING_USER_APPROVAL;
				}
			}
#else
			Serial.println("!!! Writing to Test Address space !!!");
			Serial.print("Writing "); Serial.print(testData,6); Serial.println(" to the test address space");
			echoEdaReadingsOnScreen();
			progress = EdaCorrection::Progress::WAITING_USER_APPROVAL;
#endif
		}
		else if (progress == EdaCorrection::Progress::WAITING_USER_APPROVAL)
		{
			if (_responseRecorded == false)
			{
				getUserApproval();
			}
			else
			{
				if (getApprovalStatus() == true)
				{
					Serial.println("#### GOT APPROVAL ####");
					if (!dummyWrite)
					{
						Serial.println(".....");
						Serial.println("Writing to the OTP");
						Serial.println("...");
					}
					progress = EdaCorrection::Progress::WRITING_TO_OTP;
				}
				else
				{

					correctionData.vRef1 = 0; correctionData.vRef2 = 0; correctionData.Rfb = 0;
					for (int i = 0; i < NUM_EDL_READINGS; i++)
					{
						correctionData.edlReadings[i] = 0;
						correctionData.edrReadings[i] = 0;
					}

					// ask to enter the second time 
					Serial.println("back to Progress::WAITING_FOR_SERIAL_DATA");
					Serial.println("Enter the eda values into the serial monitor");
					progress = EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA;
					_responseRecorded = false;
				}
			}
		}
		else if(progress == Progress::FINISH)
		{
			if (dummyWrite)
			{
#ifdef ACCESS_MAIN_ADDRESS
				// update the correction values to the EmotiBit class
				calcEdaCorrection();
				vref1 = correctionData.vRef1;
				vref2 = correctionData.vRef2; // calculated in getFloatFromString 
				Rfeedback = correctionData.Rfb;
				Serial.println("You can now continue using the emotiBit in a calibrated state in this session");
				_mode = Mode::NORMAL;
#else
				Serial.println("Done with simulation. OTP is not updated in dummy mode with TestAddress space");
#endif
			}
			else
			{
				// if OTP was written and EmotiBit needs to be power cycled
				if (!powerCycled)
				{
					_mode = Mode::NORMAL;
					Serial.println("OTP has been updated. Please power cycle emotibit by removing the battery and the USB(if attached).");
					Serial.println("Stopping code Execution");
					while (1);
				}
				else if (triedRegOverwrite)
				{
					_mode = Mode::NORMAL;
					Serial.print("\ntried to over write Addr: 0x"); Serial.println(overWriteAddr,HEX);
					Serial.print("Value stored in the OTP: "); Serial.println(lastReadOtpValue);
					Serial.println("check OTP state using test code. Stopping execution");
					while (1);
				}
				else if (otpWriteFailed)
				{
					_mode = Mode::NORMAL;
					Serial.print("\nwriting to Addr: 0x"); Serial.println(overWriteAddr, HEX); Serial.println(" Failed");
					Serial.println("check OTP state using test code. Stopping execution");
					while (1);
				}
			}
		}
	}

}


void EdaCorrection::echoEdaReadingsOnScreen()
{
#ifdef ACCESS_MAIN_ADDRESS
	Serial.println("The EDA values entered by the user are:");

	Serial.println("EDL readings:");
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		Serial.print(correctionData.edlReadings[i], FLOAT_PRECISION); Serial.print("\t");
	}
	Serial.println("\nVref2 readings:");
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		Serial.print(correctionData.edrReadings[i], FLOAT_PRECISION); Serial.print("\t");
	}
	Serial.print("\nAvg Vref2 :"); Serial.println(correctionData.vRef2, FLOAT_PRECISION);

	Serial.println("\nIf you are in Dummy mode, NO OTP R/W ACTIONS WILL BE PERFORMED");
	Serial.println("This is how the OTP is will be updated(If NOT IN DUMMY MODE)");
	Serial.println("You can change R/W locations in OTP by toggling ACCESS_MAIN_ADDRESS(in code)\n");

	for (int iterEdl = 0; iterEdl < NUM_EDL_READINGS; iterEdl++)
	{
		otpData.inFloat = correctionData.edlReadings[iterEdl];
		Serial.print("EdlVal:"); Serial.println(correctionData.edlReadings[iterEdl], FLOAT_PRECISION);
		uint8_t baseAddr = otpMemoryMap.edlAddresses[iterEdl];
		for (int byte = 0; byte < BYTES_PER_FLOAT; byte++)
		{
			Serial.print("0x"); Serial.print(baseAddr + byte, HEX); Serial.print(" : ");
			Serial.println(otpData.inByte[byte], DEC);
		}
	}
	// only write the EDR value if writing to the main address space.
	//vref2
	otpData.inFloat = correctionData.vRef2;
	Serial.println();
	Serial.print("vRef2: ");
	Serial.println(correctionData.vRef2, FLOAT_PRECISION);
	for (int byte = 0; byte < BYTES_PER_FLOAT; byte++)
	{
		Serial.print("0x"); Serial.print(otpMemoryMap.edrAddresses + byte, HEX); Serial.print(" : ");
		Serial.println(otpData.inByte[byte], DEC);
	}
	// metadata
	Serial.println("\nmetadata");
	Serial.print("0x"); Serial.print(otpMemoryMap.dataVersionAddr, HEX); Serial.print(" : ");
	Serial.println((uint8_t)otpDataFormat, DEC);
	Serial.print("0x"); Serial.print(otpMemoryMap.emotiBitVersionAddr, HEX); Serial.print(" : ");
	Serial.println(_emotiBitVersion, DEC);
#else
	otpData.inFloat = testData;
	for (int byte = 0; byte < BYTES_PER_FLOAT; byte++)
	{
		Serial.print("0x"); Serial.print(otpMemoryMap.edlTestAddress + byte, HEX); Serial.print(" : ");
		Serial.println(otpData.inByte[byte], DEC);
	}
#endif
	Serial.println("\nProceed with these values?");
	Serial.println("Enter Y for yes and N to enter data again");
}


void EdaCorrection::getUserApproval()
{
	if (Serial.available())
	{
		char response = Serial.read();
		while (Serial.available())
		{
			Serial.read();
		}
		if (response == 'Y')
		{
			setApprovalStatus(true);
			_responseRecorded = true;
		}
		else if (response == 'N')
		{
			setApprovalStatus(false);
			_responseRecorded = true;
		}
		else
		{
			Serial.println("incorrect choice. Please enter either Y for Yes or N for No");
		}

	}
}

void EdaCorrection::setApprovalStatus(bool response)
{
	_approvedToWriteOtp = response;
}


bool EdaCorrection::getApprovalStatus()
{
	return _approvedToWriteOtp;
}

bool EdaCorrection::checkSensorConnection(Si7013* si7013)
{
	while (si7013->getStatus() != Si7013::STATUS_IDLE);
	return (si7013->sendCommand(0x00)); // returns false if failed to send command
}

bool EdaCorrection::isOtpRegWritten(Si7013* si7013, uint8_t addr, bool isOtpOperation)
{
	while (si7013->getStatus() != Si7013::STATUS_IDLE);
	lastReadOtpValue = si7013->readRegister8(addr, isOtpOperation);
	if (lastReadOtpValue == 255)
	{
		return false;
	}
	else
	{
		return true;
	}
}

EdaCorrection::Status EdaCorrection::writeToOtp(Si7013* si7013, uint8_t addr, char val, bool &overWriteDetected, uint8_t mask)
{
	// Moved this to the main writeToOtp function. All addresses are now read together at the begining of the Write cycle
	/*
	if (isOtpRegWritten(si7013, addr)) // if the mem location is already written to
	{
		//Serial.print(addr, HEX); Serial.println(":N");
		overWriteDetected = true;
		return EdaCorrection::Status::FAILURE;
	}
	else
	{
		overWriteDetected = false;
	}
	*/
	// For testing
	/*
	Serial.print(addr, HEX); Serial.print(" .. ");
	return EdaCorrection::Status::SUCCESS;
	*/
	// for real write
	while (si7013->getStatus() != Si7013::STATUS_IDLE);
	bool writeResult = false;
	writeResult = si7013->writeToOtp(addr, val, mask);
	if (writeResult)
	{
		return EdaCorrection::Status::SUCCESS;
	}
	else
	{
		return EdaCorrection::Status::FAILURE;
	}
}


EdaCorrection::Status EdaCorrection::writeToOtp(Si7013* si7013)
{
	if (progress == EdaCorrection::Progress::WRITING_TO_OTP)
	{
		if (dummyWrite)
		{
			// Do nothing more.
			progress = Progress::FINISH;
			return EdaCorrection::Status::SUCCESS;
		}
		else
		{
			uint8_t addrCounter = OTP_ADDRESS_BEGIN;
			while (addrCounter <= OTP_ADDRESS_END)
			{
				if (isOtpRegWritten(si7013, addrCounter))
				{
					triedRegOverwrite = true;
					overWriteAddr = addrCounter;
					progress = Progress::FINISH;
					return EdaCorrection::Status::FAILURE;
				}
				addrCounter++;
			}
			bool overWriteDetected = false;
#ifdef ACCESS_MAIN_ADDRESS
			// writing the metadata- dataFormatVersion
			if (otpMemoryMap.dataVersionWritten == false)
			{
				if ((uint8_t)writeToOtp(si7013, otpMemoryMap.dataVersionAddr, (uint8_t)otpDataFormat, overWriteDetected) == (uint8_t)EdaCorrection::Status::SUCCESS)
				{
					otpMemoryMap.dataVersionWritten = true;
				}
				else
				{
					if (overWriteDetected)
					{
						triedRegOverwrite = true;
					}
					else
					{
						otpWriteFailed = true;
					}
					overWriteAddr = otpMemoryMap.dataVersionAddr;
					progress = Progress::FINISH;
					return EdaCorrection::Status::FAILURE;
				}
			}
			// writing metadata-EmotiBit Version
			if (otpMemoryMap.emotiBitVersionWritten == false)
			{
				if ((uint8_t)writeToOtp(si7013, otpMemoryMap.emotiBitVersionAddr, _emotiBitVersion, overWriteDetected) == (uint8_t)EdaCorrection::Status::SUCCESS)
				{
					otpMemoryMap.emotiBitVersionWritten = true;
				}
				else
				{
					if (overWriteDetected)
					{
						triedRegOverwrite = true;
					}
					else
					{
						otpWriteFailed = true;
					}
					overWriteAddr = otpMemoryMap.emotiBitVersionAddr;
					progress = Progress::FINISH;
					return EdaCorrection::Status::FAILURE;
				}
			}

			// writing the vref 2 to OTP
			otpData.inFloat = correctionData.vRef2;
			for (uint8_t j = 0; j < 4; j++)
			{
				if (otpMemoryMap.edrDataWritten[j] == false)
				{
					if ((uint8_t)writeToOtp(si7013, otpMemoryMap.edrAddresses + j, otpData.inByte[j], overWriteDetected) == (uint8_t)EdaCorrection::Status::SUCCESS)
					{
						otpMemoryMap.edrDataWritten[j] = true;
					}
					else
					{
						if (overWriteDetected)
						{
							triedRegOverwrite = true;
						}
						else
						{
							otpWriteFailed = true;
						}
						overWriteAddr = otpMemoryMap.edrAddresses + j;
						progress = Progress::FINISH;
						return EdaCorrection::Status::FAILURE;
					}
				}
			}
			// writing the main EDL values
			for (uint8_t iterEdl = 0; iterEdl < NUM_EDL_READINGS; iterEdl++)
			{
				otpData.inFloat = correctionData.edlReadings[iterEdl];
				for (uint8_t byte = 0; byte < BYTES_PER_FLOAT; byte++)
				{
					if (otpMemoryMap.edlDataWritten[iterEdl][byte] == false)
					{
						if ((uint8_t)writeToOtp(si7013, otpMemoryMap.edlAddresses[iterEdl] + byte, otpData.inByte[byte], overWriteDetected) == (uint8_t)EdaCorrection::Status::SUCCESS)
						{
							otpMemoryMap.edlDataWritten[iterEdl][byte] = true;
						}
						else
						{
							if (overWriteDetected)
							{
								triedRegOverwrite = true;
							}
							else
							{
								otpWriteFailed = true;
							}
							overWriteAddr = otpMemoryMap.edlAddresses[iterEdl] + byte;
							progress = Progress::FINISH;
							return EdaCorrection::Status::FAILURE;
						}
					}
				}
			}


#else
			//Serial.println("Beginning OTP write");
			otpData.inFloat = testData;
			for (uint8_t byte = 0; byte < BYTES_PER_FLOAT; byte++)
			{
				if (otpMemoryMap.testDataWritten[byte] == false)
				{
					if ((uint8_t)writeToOtp(si7013, otpMemoryMap.edlTestAddress + byte, otpData.inByte[byte], overWriteDetected) == (uint8_t)EdaCorrection::Status::SUCCESS)
					{
						otpMemoryMap.testDataWritten[byte] = true;
					}
					else
					{
						if (overWriteDetected)
						{
							triedRegOverwrite = true;
						}
						else
						{
							otpWriteFailed = true;
						}
						overWriteAddr = otpMemoryMap.edlTestAddress + byte;
						progress = Progress::FINISH;
						return EdaCorrection::Status::SUCCESS;
					}
				}
			}
#endif
			//_mode = EdaCorrection::Mode::NORMAL;
			progress = Progress::FINISH;
			powerCycled = false;
			return EdaCorrection::Status::SUCCESS;
		}
	}
	progress = Progress::FINISH;
	return EdaCorrection::Status::FAILURE;
}


EdaCorrection::Status EdaCorrection::readFromOtp(Si7013* si7013, bool isOtpOperation)
{
	if (!dummyWrite)
	{
		if (checkSensorConnection(si7013) == false)
		{
			isSensorConnected = false;
			//readOtpValues = true;
			isOtpValid = false;
			return EdaCorrection::Status::FAILURE;
		}
#ifdef ACCESS_MAIN_ADDRESS
		if(si7013->readRegister8(otpMemoryMap.dataVersionAddr, isOtpOperation) == 255)
		{
			isOtpValid = false;
			//readOtpValues = true;
			return EdaCorrection::Status::FAILURE;
		}
		otpDataFormat = (EdaCorrection::OtpDataFormat)si7013->readRegister8(otpMemoryMap.dataVersionAddr, isOtpOperation);
		_emotiBitVersion = (uint8_t)si7013->readRegister8(otpMemoryMap.emotiBitVersionAddr, isOtpOperation);

		for (uint8_t j = 0; j < BYTES_PER_FLOAT; j++)
		{
			otpData.inByte[j] = si7013->readRegister8(otpMemoryMap.edrAddresses + j, isOtpOperation);
		}
		correctionData.vRef2 = otpData.inFloat;
		for (uint8_t i = 0; i < NUM_EDL_READINGS; i++)
		{
			for (uint8_t j = 0; j < BYTES_PER_FLOAT; j++)
			{
				otpData.inByte[j] = si7013->readRegister8(otpMemoryMap.edlAddresses[i] + j, isOtpOperation);
			}
			correctionData.edlReadings[i] = otpData.inFloat;
		}
#else
		for (uint8_t i = 0; i < NUM_EDL_READINGS; i++)
		{
			for (uint8_t j = 0; j < BYTES_PER_FLOAT; j++)
			{
				otpData.inByte[j] = si7013->readRegister8(otpMemoryMap.edlTestAddress + j, isOtpOperation);
			}
			testData = otpData.inFloat;
		}
#endif
	}
	else
	{
		// do nothing here.
	}
	//readOtpValues = true;
	return EdaCorrection::Status::SUCCESS;
}


void EdaCorrection::displayCorrections()
{
	Serial.println("EDL readings stored:");
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		Serial.print("edlReadings["); Serial.print(i); Serial.print("]: "); Serial.println(correctionData.edlReadings[i], FLOAT_PRECISION);
	}
	Serial.println("### Calculating values ####\n");
	Serial.print("Vref1: "); Serial.println(correctionData.vRef1, FLOAT_PRECISION);
	Serial.print("Vref2: "); Serial.println(correctionData.vRef2, FLOAT_PRECISION);
	Serial.print("Rfb: "); Serial.println(correctionData.Rfb, FLOAT_PRECISION);
}


void EdaCorrection::calcEdaCorrection()
{
	correctionData.vRef1 = correctionData.edlReadings[0];
	// correctionData.vRef2 is updated in getFloatFromString while writing to OTP or from readFromOtp while reading
	correctionData.Rfb = 0;
	// using 1M to find Rfeedback
	correctionData.Rfb += (correctionData.trueRskin[3] / ((correctionData.edlReadings[3] / correctionData.vRef1) - 1));
	if (dummyWrite)
	{
		displayCorrections();
		//correctionDataReady = true;
	}
	else
	{
#ifdef ACCESS_MAIN_ADDRESS

		displayCorrections();
		//correctionDataReady = true;
#else
		Serial.print("No calculations performed. Just printing values read from the test Address space ");
#ifdef USE_ALT_SI7013
		Serial.println("of the alternate(external) sensor");
#else
		Serial.println("of the main EmotiBit sensor");
#endif
		
		Serial.print("correctionData.edlReadings[0]: "); Serial.println(correctionData.edlReadings[0], 6);
		Serial.println("Uncomment ACCESS_MAIN_ADDRESS in code to use emotibit with calibration.");
#endif
	}
}
/*
void EdaCorrection::OtpMemoryMap_V0::echoWriteCount()
{
	if (writeCount.dataVersion)// only echo if an attempt has been made to write to the OTP
	{
		Serial.println("\n::::Attempted write count::::");
		Serial.print("DataFormat:"); Serial.println(writeCount.dataVersion);
		Serial.print("EmotiBitVersion:"); Serial.println(writeCount.emotiBitVersion);
		Serial.println("edrVoltage(4 bytes):");
		for (int i = 0; i < 4; i++)
		{
			Serial.print("Byte "); Serial.print(i); Serial.print(":"); Serial.println(writeCount.edrData[i]);
		}
		Serial.println("edlVoltage(20Bytes):");
		for (int j = 0; j < NUM_EDL_READINGS; j++)
		{
			Serial.print("EdldataPoint "); Serial.print(j); Serial.println(":");
			for (int i = 0; i < 4; i++)
			{
				Serial.print("Byte "); Serial.print(i); Serial.print(":"); Serial.println(writeCount.edlData[j][i]);
			}
		}
	}
}
*/