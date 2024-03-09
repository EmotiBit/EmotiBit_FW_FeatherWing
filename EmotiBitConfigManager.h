#ifndef EMOTIBIT_CONFIG_MANAGER_H
#define EMOTIBIT_CONFIG_MANAGER_H
#include <Arduino.h>
#include <EmotiBitSerial.h>
#include <EmotiBitFactoryTest.h>
#include <ArduinoJson.h>
#include <EmotiBitPacket.h>
#ifdef ARDUINO_FEATHER_ESP32
#include <SD.h>
#else
#include <SdFat.h>
#endif

// ToDo: This may not be the best place for this functionality. Consider a better place in the codebase.
class EmotiBitConfigManager
{
public:
#ifdef ARDUINO_FEATHER_ESP32
	// SD is already defined in ESP32 
#else
    // ToDo: conside passing pointer from EmotiBit.
	SdFat SD;	// Use SdFat library
#endif
    /*!
    * @brief initialize sd card
    * @param sdCsPin Pin assigned as CS on EmotiBit
    * @return true is initialization successful
    */
    bool init(uint8_t sdCsPin);
    /*! 
    * @brief Function to update WiFi credentials over Serial.
    * @param emotibitFwVersion Current EmotiBit firmware version. Added for future compatibility for software-firmware handshaking
    * @param configFilename name of the config file on the SD card
    * @param maxCreds Maximum allowed credentials on the SD card
    */
    void updateWiFiCredentials(String emotibitFwVersion, String configFilename, uint8_t csPin, const uint8_t maxCreds);
    // ToDo: In the future, we may want to update creds through WiFi, so this functio can act as a wrapper to call other functions that handle WiFi/serial process.
};
#endif