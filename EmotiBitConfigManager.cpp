#include <EmotiBitConfigManager.h>

bool EmotiBitConfigManager::init(uint8_t sdCsPin)
{
#if defined ARDUINO_FEATHER_ESP32
	if (SD.begin(sdCsPin, SPI, 10000000)) // 10MHz works with 40MHz CPU, 20Mhz does NOT
#else
	if (SD.begin(sdCsPin))
#endif
	{
		return true;
	}
	else
	{
		return false;
	}
}

void EmotiBitConfigManager::updateWiFiCredentials(String emotibitFwVersion, String configFilename, uint8_t csPin, const uint8_t maxCreds)
{
	Serial.println("\nSuccessfully entered config file edit mode.");
	Serial.print("Initializing SD ...");
	if(init(csPin))
	{
		Serial.println("Initialized");
	}
	else
	{
		Serial.println("Could not initialize SD card. Make sure battery and SD card are present");
	}
	// Send EmotiBit Firmware version to host
	EmotiBitSerial::sendMessage(EmotiBitFactoryTest::TypeTag::FIRMWARE_VERSION, emotibitFwVersion);
	// instructions
	Serial.println("Options available:");
	Serial.println("1. Add a wifi credential. Usage: @WA,{\"ssid\":\"SSSS\",\"password\" : \"PPPP\"}~");
	Serial.println("  - Replace SSSS with network name and PPPP with network password");
	Serial.println("2. List wifi credentials. Usage: @LS,~");
	Serial.println("3. Delete Wifi Credential. Usage: @WD,<credential number>~");
	Serial.println("  - use List option to get a wifi list. Then use the number in the list to delete that credential");
	Serial.println("4. Reset device. Usage @RS,~");
	Serial.println("  - Restart the device after editing config file");

	while (1) 
	{
		if (Serial.available())
		{
			String msg = Serial.readStringUntil('~'); // is ~ is not present, readStringUntil() break after timeout
			msg += "~";
			String typetag, payload;
			bool serialParsed = EmotiBitSerial::parseSerialMessage(msg, typetag, payload);
			if(serialParsed)
			{
				File file;
				StaticJsonDocument<1024> configAsJson;
				DeserializationError parseError;
				bool fileExists = false, fileParses = false;
				fileExists = SD.exists(configFilename);
				if(fileExists)
				{
					file = SD.open(configFilename);
					parseError = deserializeJson(configAsJson, file); 
					file.close();
				}

				if (typetag.compareTo(EmotiBitPacket::TypeTag::WIFI_ADD) == 0)
				{
					String ssid, password, username, userid;
					// config file is not present. Create file and add cred.
					StaticJsonDocument<256> inputJson;
					DeserializationError error = deserializeJson(inputJson, payload);
					if(error)
					{
						// user input cannot be parsed
						Serial.println("Try again. Cannot parse JSON input.");
						Serial.println("Expected format: WA,{\"ssid\":\"SSSS\",\"password\" : \"PPPP\"}");
						EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_ADD);
					}
					else
					{
						// user input parsed correctly	
						ssid = inputJson["ssid"] | "";
						password = inputJson["password"] | "";
						username = inputJson["username"] | "";
						userid = inputJson["userid"] | "";
						//Serial.println("Parsed output: ");
						Serial.print("Adding:: ");
						Serial.print("ssid: " + ssid); Serial.print(" | ");
						Serial.print("password: " + password);
						username.compareTo("")!= 0 ? Serial.println("username: " + username) : Serial.println();
						userid.compareTo("")!= 0 ? Serial.println("userid: " + userid) : Serial.println();
						
						if(fileExists)
						{
							// config file present!
							Serial.println("config file exists. Appending to file.");
							if(!parseError)
							{
								// file parsed successfully
								if(configAsJson["WifiCredentials"].size() >= maxCreds)
								{
									Serial.println("Config file already has max num allowed creds. Please delete before adding");
									EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_ADD);
									continue;
								}
								else
								{
									JsonObject cred = configAsJson["WifiCredentials"].createNestedObject();
									cred["ssid"] = ssid;
									cred["password"] = password;
									if(userid.compareTo("") != 0)
									{
										cred["userid"] = userid;
									}
									if(username.compareTo("") != 0)
									{
										cred["username"] = username;
									}
								}
							}
							else
							{
								// existing file parse failed
								// remove existing file. A new file will be created with the creds added
								Serial.println("existing config file parse failed. Removing file.");
								SD.remove(configFilename);
							}				
						}
						if(!fileExists || parseError)
						{
							Serial.println("config file does not exist. creating new file");
							// create new file
							configAsJson.clear();
							JsonArray wifiCreds = configAsJson.createNestedArray("WifiCredentials");
							JsonObject cred = wifiCreds.createNestedObject();
							cred["ssid"] = ssid;
							cred["password"] = password;
							if (userid.compareTo("") != 0)
							{
								cred["userid"] = userid;
							}
							if (username.compareTo("") != 0)
							{
								cred["username"] = username;
							}
						}
						// write updated JSON to file
#if defined ARDUINO_FEATHER_ESP32
						file = SD.open(configFilename, FILE_WRITE);
#else 
						file = SD.open(configFilename, O_CREAT | O_WRITE);
#endif
						if(file)
						{
							if( serializeJsonPretty(configAsJson, file) == 0)
							{
								Serial.println("Failed to write to file");
							}
							file.close();
							EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::ACK, EmotiBitPacket::TypeTag::WIFI_ADD);
						}
						else
						{
							Serial.println("Failed to open file");
							EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_ADD);
						}
						// read file contents
						Serial.println("Updated file contents:");
						file = SD.open(configFilename);
						configAsJson.clear();
						error = deserializeJson(configAsJson, file);
						serializeJsonPretty(configAsJson, Serial);
						file.close();
						Serial.println();
					}
				}
				// Replace string with actual type tag
				else if (typetag.compareTo(EmotiBitPacket::TypeTag::WIFI_DELETE) == 0)
				{
					if(!fileExists)
					{
						Serial.println("config file does not exist. Aborting delete");
						EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
						continue;
					}
					if(parseError)
					{
						Serial.println("Cannot parse JSON file. Aborting delete");
						EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
						continue;
					}
					else
					{
						// file can be parsed!
						if(payload.compareTo("") != 0)
						{
							// delete requested credential
							int deleteIndex = payload.toInt();
							if(deleteIndex == 0 && payload.compareTo("0") != 0)
							{
								// toInt() tried to convert invalid string and returned 0 by default
								Serial.println("invalid option. Please be careful to avoid blank spaces.");
								EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
								continue;
							}
							
							if(deleteIndex < configAsJson["WifiCredentials"].size())
							{
								Serial.print("Deleting entry: " + String(deleteIndex) + ". "); 
								Serial.println(configAsJson["WifiCredentials"][deleteIndex]["ssid"].as<String>());
								configAsJson["WifiCredentials"].remove(deleteIndex);
								SD.remove(configFilename);
#if defined ARDUINO_FEATHER_ESP32
								file = SD.open(configFilename, FILE_WRITE);
#else 
								file = SD.open(configFilename, O_CREAT | O_WRITE);
#endif
								serializeJsonPretty(configAsJson, file);
								file.close();
								EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::ACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
							}
							else
							{
								Serial.println("Out of bounds delete. Enter a valid choice");
								EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
							}
						}
						else
						{
							// send list of existing credentials
							Serial.println("Delete needs a parameter. Call LS to get current credential list.");
							EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
						}
					}
				}
				else if (typetag.compareTo(EmotiBitPacket::TypeTag::LIST) == 0)
				{
					if(!fileExists)
					{
						Serial.println("config file does not exist.");
						EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::LIST);
						continue;
					}
					if(parseError)
					{
						Serial.println("Cannot parse config file. Aborting LIST.");
						EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::LIST);
						continue;
					}
					if(configAsJson["WifiCredentials"].size())
					{
						Serial.println("##################################");
						Serial.println("config file credetials:");
						for(int i = 0; i < configAsJson["WifiCredentials"].size(); i++ )
						{
							Serial.print(i); Serial.print(". " + configAsJson["WifiCredentials"][i]["ssid"].as<String>());
							Serial.print(" : "); Serial.print(configAsJson["WifiCredentials"][i]["password"].as<String>());
							Serial.println();
						}
						Serial.println("##################################");
					}
					else
					{
						Serial.println("config file has no credentials. Try WA to add credential first.");
					}
					EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::ACK, EmotiBitPacket::TypeTag::LIST);

				}
				else if (typetag.compareTo(EmotiBitPacket::TypeTag::RESET) == 0)
				{
					EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::ACK, EmotiBitPacket::TypeTag::RESET);
					Serial.println("Restarting MCU");
					delay(1000);
					#ifdef ARDUINO_FEATHER_ESP32
						ESP.restart();
					#elif defined ADAFRUIT_FEATHER_M0
						NVIC_SystemReset();
					#endif
				} 
			}
			else
			{
				Serial.println("not a valid serial message. Expecting @TYPETAG,PAYLOAD~");
			}
			
		}
	}
}