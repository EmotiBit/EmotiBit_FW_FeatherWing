//#define DEBUG_FEAT_EDA_CTRL
//#define DEBUG_THERM_PROCESS
//#define DEBUG_THERM_UPDATE
//#define DEBUG_BUFFER
//#define TEST_OVERFLOW

#ifndef _EMOTIBIT_H_
#define _EMOTIBIT_H_

#include "Arduino.h"
#include <Wire.h>
#include <EmotiBit_Si7013.h>
#include <BMI160Gen.h>
#include <MAX30105.h>
#include <EmotiBit_NCP5623.h>
#include <SparkFun_MLX90632_Arduino_Library.h>
#include "DoubleBufferFloat.h"
#include <ArduinoJson.h>
#include "EmotiBitWiFi.h"
#include <SPI.h>
#ifdef ARDUINO_FEATHER_ESP32
#include <SD.h>
#include "driver/adc.h"
#include <esp_bt.h>
#else
#include <SdFat.h>
#include <ArduinoLowPower.h>
#include "wiring_private.h"
#endif
#include "AdcCorrection.h"
#include "EmotiBitVersionController.h"
#include "DigitalFilter.h"
#include "EmotiBitFactoryTest.h"
#include "EmotiBitEda.h"
#include "EmotiBitVariants.h"
#include "EmotiBitNvmController.h"
#include "heartRate.h"
#include "EmotiBitConfigManager.h"
#include "FileTransferManager.h"

class EmotiBit {
  
public:
	enum class TestingMode
	{
		NONE,
		CHRONIC,
		ACUTE,
		FACTORY_TEST,
		length
	};



  String firmware_version = "1.9.0.feat-sdCardWiFiCredentials.16.feat-FtpServer.1";



	TestingMode testingMode = TestingMode::NONE;
	const bool DIGITAL_WRITE_DEBUG = false;
#if defined (ARDUINO_FEATHER_ESP32)
	const uint8_t DEBUG_OUT_PIN_0 = 26;
	const uint8_t DEBUG_OUT_PIN_1 = 33;
	const uint8_t DEBUG_OUT_PIN_2 = 15;
#elif defined (ADAFRUIT_FEATHER_M0)
	const uint8_t DEBUG_OUT_PIN_0 = 14;
	const uint8_t DEBUG_OUT_PIN_1 = 10;
	const uint8_t DEBUG_OUT_PIN_2 = 9;
#endif

	const bool DC_DO_V2 = true;

	bool _debugMode = false;
	bool dummyIsrWithDelay = false;
	uint32_t targetFileSyncDelay = 1;
	struct ReadSensorsDurationMax
	{
		uint32_t total = 0;
		uint32_t led = 0;
		uint32_t eda = 0;
		uint32_t ppg = 0;
		uint32_t tempPpg = 0;
		uint32_t tempHumidity = 0;
		uint32_t thermopile = 0;
		uint32_t imu = 0;
		uint32_t battery = 0;
		uint32_t nvm = 0;
	} readSensorsDurationMax;
	uint32_t readSensorsIntervalMin = 1000000;
	uint32_t readSensorsIntervalMax = 0;


	enum class SensorTimer {
		MANUAL
	};

	//struct IMUSettings {
	//int gyroResolution = 250;
	//int accResolution = 2;
	//	uint8_t IF_CONF = 0x6B; //BMI160_IF_CONF
	//uint8_t FIFO_CONF = 0x47;//BMI160_RA_FIFO_CONFIG_1
	//uint8_t MAG_IF_0 = 0x4B;//BMI160_MAG_IF_0
	//uint8_t MAG_IF_1 = 0x4C;//BMI160_MAG_IF_1
	//uint8_t MAG_IF_2 = 0x4D;//BMI160_MAG_IF_2
	//uint8_t MAG_IF_3 = 0x4E;//BMI160_MAG_IF_3
	//uint8_t MAG_IF_4 = 0x4F;//BMI160_MAG_IF_4
	//uint8_t DATA_T_L = 0x20;//BMI160_RA_TEMP_L 
	//uint8_t DATA_T_M = 0x21;//BMI160_RA_TEMP_M 
	//uint8_t DATA_MAG_X_L = 0x04;//BMI160_RA_MAG_X_L
	//uint8_t DATA_MAG_X_M = 0x05;//BMI160_RA_MAG_X_M
	//uint8_t MAG_CONF = 0x44;
	//uint8_t FIFO_L = 0x46;
	//uint8_t FIFO_M = 0x47;
	//uint8_t STATUS = 0x1B;
	//uint8_t I2C_MAG = 0x20;
	//uint8_t MAG_DATA_8BYTE = 0x03;
	//uint8_t ADD_BMM_MEASURE = 0x4C; 
	//uint8_t ADD_BMM_DATA = 0x42;
	//};
	struct DeviceAddress {
		uint8_t MLX = 0x3A;
		uint8_t EEPROM_FLASH_34AA02 = 0x50; // 7 bit address
	};
	struct BMM150TrimData {
		int8_t dig_x1;
		int8_t dig_y1;
		int8_t dig_x2;
		int8_t dig_y2;
		uint16_t dig_z1;
		int16_t dig_z2;
		int16_t dig_z3;
		int16_t dig_z4;
		uint8_t dig_xy1;
		int8_t dig_xy2;
		uint16_t dig_xyz1;
	};
  
#if defined(EMOTIBIT_PPG_100HZ)
	struct PPGSettings {
		uint8_t ledPowerLevel = 0x2F; //Options: 0=Off to 255=50mA
		uint16_t sampleAverage = 8;   //Options: 1, 2, 4, 8, 16, 32
		uint8_t ledMode = 3;          //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
		uint16_t sampleRate = 800;    //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
		uint16_t pulseWidth = 118;     //Options: 69, 118, 215, 411
		uint16_t adcRange = 4096;     //Options: 2048, 4096, 8192, 16384
	};
#else
	struct PPGSettings {
		uint8_t ledPowerLevel = 0x2F; //Options: 0=Off to 255=50mA
		uint16_t sampleAverage = 16;   //Options: 1, 2, 4, 8, 16, 32
		uint8_t ledMode = 3;          //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
		uint16_t sampleRate = 400;    //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
		uint16_t pulseWidth = 215;     //Options: 69, 118, 215, 411
		uint16_t adcRange = 4096;     //Options: 2048, 4096, 8192, 16384
	};
#endif

	struct IMUSettings {
		uint8_t acc_odr = BMI160AccelRate::BMI160_ACCEL_RATE_25HZ;
		uint8_t acc_bwp = BMI160_DLPF_MODE_NORM;
		uint8_t acc_us = BMI160_DLPF_MODE_NORM;
		uint8_t gyr_odr = BMI160GyroRate::BMI160_GYRO_RATE_25HZ;
		uint8_t gyr_bwp = BMI160_DLPF_MODE_NORM;
		uint8_t gyr_us = BMI160_DLPF_MODE_NORM;
		uint8_t mag_odr = BMI160MagRate::BMI160_MAG_RATE_25HZ;
	};
	
	struct EnableDigitalFilter {
		bool mx = false;
		bool my = false;
		bool mz = false;
		bool eda = true;
	};

	struct SamplingRates {
		float eda = 0.f;
		float accelerometer = 0.f;
		float gyroscope = 0.f;
		float magnetometer = 0.f;
		float humidity = 0.f;
		float temperature = 0.f;
		float temperature_1 = 0.f;
		float thermopile = 0.f;
		float ppg = 0.f;
	};

	struct SamplesAveraged {
		uint8_t eda = 1;
		uint8_t humidity = 1;
		uint8_t temperature = 1;
		uint8_t temperature_1 = 1;
		uint8_t thermopile = 1;
		uint8_t battery = 1;
	};

	enum class Error {
		NONE = 0,
		BUFFER_OVERFLOW = -1,
		DATA_CLIPPING = -2,
		INDEX_OUT_OF_BOUNDS = -4,
		SENSOR_NOT_READY = -8
	};

  enum class DataType{
  	DATA_OVERFLOW,
		DATA_CLIPPING,
		PPG_INFRARED,
		PPG_RED,
		PPG_GREEN,
		EDA,
		EDL,
		EDR,
		TEMPERATURE_0,
		TEMPERATURE_1,
		THERMOPILE,
		HUMIDITY_0,
		ACCELEROMETER_X,
		ACCELEROMETER_Y,
		ACCELEROMETER_Z,
		GYROSCOPE_X,
		GYROSCOPE_Y,
		GYROSCOPE_Z,
		MAGNETOMETER_X,
		MAGNETOMETER_Y,
		MAGNETOMETER_Z,
		BATTERY_VOLTAGE,
		BATTERY_PERCENT,
		DEBUG,
		length
  };

  enum class DebugTags{
  	WIFI_CONNHISTORY = 1,  // To print wifi connection history till point of do
  	WIFI_DISCONNECT,  // Add message at wifi disconnect 
  	WIFI_TIMELOSTCONN,  // add message indicating time of disconnect
  	WIFI_CONNECT,  // add message indicating established connection
  	WIFI_UPDATECONNRECORDTIME,// NO LONGER IN USE
  	TIME_PARSEINCOMINGMSG = 6,  // time taken to parse incoming message
  	TIME_TIMESTAMPSYNC,  // time taken for time stamp syncing
  	TIME_MSGGENERATION = 8,  // time taken for message generation
  	TIME_MSGTX,  // time taken for comlpete transmission(udp_sdcard)
  	TIME_UDPTX,  // time taken for udp transmission
  	TIME_SDCARDTX,  // time taken for sdcard transmission
  	TIME_FILEOPEN,  // time taken for file opening
  	TIME_FILEWRITES,  // time taken for file write
  	TIME_FILECLOSE,  // time taken for file close
  	TIME_FILESYNC  //time taken for file syncing
  };

  //TODO: Make enum classs for led's
  enum class Led {
	  RED = 1,
	  BLUE,
	  YELLOW
  };

  enum class BattLevel {
	  THRESHOLD_HIGH = 50, // Set thresholds for changing led indication on board for battery
	  //THRESHOLD_MED  = 15,
	  THRESHOLD_LOW  = 20,
	  //INDICATION_SEQ_HIGH = 1,
	  //INDICATION_SEQ_MED,
	  //INDICATION_SEQ_LOW
  };

	enum class PowerMode {
		HIBERNATE,
		WIRELESS_OFF,				// fully shutdown wireless
		MAX_LOW_POWER,			// data not sent, time-syncing accuracy low
		LOW_POWER,					// data not sent, time-syncing accuracy high
		NORMAL_POWER,				// data sending, time-syncing accuracy high
		length
	};

	Si7013 tempHumiditySensor;
	DeviceAddress deviceAddress;
	uint8_t buttonPin;
	PPGSettings ppgSettings;
	IMUSettings imuSettings;
	MAX30105 ppgSensor;
	NCP5623 led;
	MLX90632 thermopile;
	EmotiBitEda emotibitEda;
	EmotiBitNvmController _emotibitNvmController;
	EmotiBitConfigManager _emotibitConfigManager;
	FileTransferManager _fileTransferManager;

	int _emotiBitSystemConstants[(int)SystemConstants::length];
	float adcRes;
	BMM150TrimData bmm150TrimData;
	bool bmm150XYClipped = false;
	bool bmm150ZHallClipped = false;
	bool thermopileBegun = false;
	int thermopileFs = 8; // *** NOTE *** changing this may wear out the Melexis flash
	uint8_t thermopileMode = MODE_STEP;		// If changing to MODE_CONTINUOUS besure to adjust SAMPLING_DIV to match thermopile rate
	volatile unsigned long int _thermReadFinishedTime;

	// ---------- BEGIN ino refactoring --------------
	static const uint16_t OUT_MESSAGE_RESERVE_SIZE = 2048;
	static const uint16_t OUT_MESSAGE_TARGET_SIZE = 1024;
	static const uint16_t DATA_SEND_INTERVAL = 100;
	static const uint16_t MAX_SD_WRITE_LEN = 512; // 512 is the size of the sdFat buffer
	static const uint16_t MAX_DATA_BUFFER_SIZE = 48;
	static const uint16_t NORMAL_POWER_MODE_PACKET_INTERVAL = 200;
	static const uint16_t LOW_POWER_MODE_PACKET_INTERVAL = 1000;
	uint16_t modePacketInterval = NORMAL_POWER_MODE_PACKET_INTERVAL;

	// Timer constants
#define TIMER_PRESCALER_DIV 1024
#if defined(ARDUINO_FEATHER_ESP32)
	const uint32_t CPU_HZ = 80000000; // 80MHz has been tested working to save battery life
#elif defined(ADAFRUIT_FEATHER_M0)
	const uint32_t CPU_HZ = 48000000; // In Hz
#endif
	
	
	/*
	* Dev notes on understanding sampling rates
	* If a sampling rate can be set for a sensor, it is done so in the sensor settigns.
	* The Firmware reads data from the sensor @BASE_SAMPLING_FREQ/{SENSOR}_SAMPLING_DIV Hz
	* further, the recorded samples may be avaraged, as set by EmotiBit::SamplesAveraged struct.
	* So, net, the sampling rate of the data channel is BASE_SAMPLING_FREQ/{DATATYPE}_SAMPLING_DIV/SamplesAveraged.{DATATYPE}
	*/ 
	// ToDo: Make sampling variables changeable
#define BASE_SAMPLING_FREQ 150
#define LED_REFRESH_DIV 10
#define EDA_SAMPLING_DIV 2
#define PPG_SAMPLING_DIV 2
#define TEMPERATURE_1_SAMPLING_DIV 20
#define TEMPERATURE_0_SAMPLING_DIV 10
#define THERMOPILE_SAMPLING_DIV 20 	// TODO: This should change according to the rate set on the thermopile begin function 
#define IMU_SAMPLING_DIV 2
#define BATTERY_SAMPLING_DIV 50
#define DUMMY_ISR_DIV 10

	struct TimerLoopOffset
	{
		uint8_t led = 4;
		uint8_t eda = 1;
		uint8_t ppg = 1;
		uint8_t bottomTemp = 2;
		uint8_t tempHumidity = 2;
		uint8_t thermopile = 0;
		uint8_t imu = 1;
		uint8_t battery = 6;
	} timerLoopOffset;	// Sets initial value of sampling counters


//// ToDo: Make sampling variables changeable
//#define BASE_SAMPLING_FREQ 120
//#define EDA_SAMPLING_DIV 1
//#define IMU_SAMPLING_DIV 4
//#define PPG_SAMPLING_DIV 4
//#define LED_REFRESH_DIV 8
//#define THERMOPILE_SAMPLING_DIV 16	// TODO: This should change according to the rate set on the thermopile begin function 
//#define TEMPERATURE_0_SAMPLING_DIV 4
//#define BATTERY_SAMPLING_DIV 20

	//struct TimerLoopOffset
	//{
	//	uint8_t eda = 0;
	//	uint8_t imu = 3;
	//	uint8_t ppg = 1;
	//	uint8_t led = 2;
	//	uint8_t thermopile = 6;
	//	uint8_t tempHumidity = 0;
	//	uint8_t battery = 0;
	//} timerLoopOffset;	// Sets initial value of sampling counters

	struct AcquireData {
		bool eda = true;
		bool tempHumidity = true;
		bool thermopile = true;
		bool imu = true;
		bool ppg = true;
		bool tempPpg = true;
		bool debug = false;
		bool battery = true;
		bool heartRate = true; // Note: we may want to move this to a separarte flag someday, for controlling derivative signals
		bool edrMetrics = true;
	} acquireData;

	struct ChipBegun {
		bool SI7013 = false;
		bool MAX30101 = false;
		bool BMI160 = false;
		bool BMM150 = false;
		bool NCP5623 = false;
		bool MLX90632 = false;
	} chipBegun;
	

	const char *typeTags[(uint8_t)EmotiBit::DataType::length];
	bool _newDataAvailable[(uint8_t)EmotiBit::DataType::length];
	uint8_t _printLen[(uint8_t)EmotiBit::DataType::length];
	bool _sendData[(uint8_t)EmotiBit::DataType::length];
	bool _sendSerialData[(uint8_t)EmotiBit::DataType::length];

#ifdef ARDUINO_FEATHER_ESP32
	// SD is already defined in ESP32 
#else
	SdFat SD;	// Use SdFat library
#endif

	volatile uint8_t battIndicationSeq = 0;
	volatile uint16_t BattLedDuration = 65535;

	EmotiBitWiFi _emotiBitWiFi; 
	TwoWire* _EmotiBit_i2c = nullptr;
#ifdef ARDUINO_FEATHER_ESP32
	uint32_t i2cClkMain = 433000;	// Adjust for empirically slow I2C clock
#else 
	uint32_t i2cClkMain = 400000;
#endif
	String _outDataPackets;		// Packets that will be sent over wireless (if enabled) and written to SD card (if recording)
	uint16_t _outDataPacketCounter = 0;
	//String _outSdPackets;		// Packts that will be written to SD card (if recording) but not sent over wireless
	//String _inControlPackets;	// Control packets recieved over wireless
	String _sdCardFilename = "datalog.csv";
#if defined ARDUINO_FEATHER_ESP32
	const char *_configFilename = "/config.txt";
#else
	const char *_configFilename = "config.txt";
#endif
	File _dataFile;
	volatile bool _sdWrite;
	PowerMode _powerMode;
	bool _sendTestData = false;
	DataType _serialData = DataType::length;
	volatile bool buttonPressed = false;
	bool startBufferOverflowTest = false;

	void setupFailed(const String failureMode, int buttonPin = -1, bool configFileError = false);
	bool setupSdCard(bool loadConfig = true);
	void updateButtonPress();
	void sleep(bool i2cSetupComplete = true);
	void startTimer(int frequencyHz);
	void setTimerFrequency(int frequencyHz);
	//void TC3_Handler();
	void stopTimer();
	void(*onShortPressCallback)(void);
	void(*onLongPressCallback)(void);
	void(*onDataReadyCallback)(void);
	void attachShortButtonPress(void(*shortButtonPressFunction)(void));
	void attachLongButtonPress(void(*longButtonPressFunction)(void));
	PowerMode getPowerMode();
	void setPowerMode(PowerMode mode);
	bool writeSdCardMessage(const String &s);
	int freeMemory();
	bool loadConfigFile(const String &filename);
	bool addPacket(uint32_t timestamp, const String typeTag, float * data, size_t dataLen, uint8_t precision = 0, bool printToSerial = false);
	bool addPacket(uint32_t timestamp, EmotiBit::DataType t, float * data, size_t dataLen, uint8_t precision = 4);
	bool addPacket(EmotiBit::DataType t);
	void parseIncomingControlPackets(String &controlPackets, uint16_t &packetNumber);
	void readSensors();
	void processHeartRate();
	void processData();
	void sendData();
	bool processThermopileData();	// placeholder until separate EmotiBitThermopile controller is implemented
	void writeSerialData(EmotiBit::DataType t);
	void printEmotiBitInfo();
	

	/**
	 * Copies data buffer of the specified DataType into the passed array
	 *
	 * @param t an EmotiBit::DataType to read
	 * @data a pre-allocated array of size dataSize to copy data into
	 * @dataSize a size_t specifying the size of the passed data array
	 * @return the size of the present data buffer
	 */
	size_t readData(EmotiBit::DataType t, float *data, size_t dataSize);
	
	/**
	 * Copies data buffer and timestamp of the specified DataType into the passed parameters
	 *
	 * @param t an EmotiBit::DataType to read
	 * @data a pre-allocated array of size dataSize to copy data into
	 * @dataSize a size_t specifying the size of the passed data array
	 * @timestamp returns with the most recent data timestamp
	 * @return the size of the present data buffer
	 */
	size_t readData(EmotiBit::DataType t, float *data, size_t dataSize, uint32_t &timestamp);		
	
	/**
	 * Points to the data buffer of the specified type into the passed parameter
	 *		CAUTION: this pointer becomes stale after calling EmotiBit::update()
	 *
	 * @param t an EmotiBit::DataType to read
	 * @data returns with a pointer to the existing (double buffered) float[]
	 * @return the size of the present data buffer 
	 */
	size_t readData(EmotiBit::DataType t, float **data);
	
	/**
	 * Points to the data buffer and reads the timestamp of the specified type into the passed parameters
	 *		CAUTION: this pointer becomes stale after calling EmotiBit::update()
	 *
	 * @param t an EmotiBit::DataType to read
	 * @data returns with a pointer to the existing (double buffered) float[]
	 * @timestamp returns with the most recent timestamp in the present data array
	 * @return the size of the present data buffer
	 */
	size_t readData(EmotiBit::DataType t, float **data, uint32_t &timestamp);	

	void updateBatteryIndication(float battPercent);
	void appendTestData(String &dataMessage, uint16_t &packetNumber);
	bool createModePacket(String &modePacket, uint16_t &packetNumber);
	void sendModePacket(String &sentModePacket, uint16_t &packetNumber);
	void processDebugInputs(String &debugPackets, uint16_t &packetNumber);
	void processFactoryTestMessages();
	/**
	* Test to assess buffer overflow duration limits
	* @param maxTestDuration Total duration (in mS) of test to assess overflows
	* @param delayInterval Interval (in mS) to poll buffers and print results
	* @param humanReadable Boolean to switch between commas and carriage returns for human readability
	*/
	void bufferOverflowTest(unsigned int maxTestDuration = 5000, unsigned int delayInterval = 100, bool humanReadable = true);
	String getFeatherMacAddress();
	String getHardwareVersion();
	int detectEmotiBitVersion();
	/*!
	 * @brief Function to perform a software reset on the MCU
	 */
	void restartMcu();
	
	
	// ----------- END ino refactoring ---------------

	
  EmotiBit();
	
	/*!
		@brief Setup of all things EmotiBit
		@param String firmwareVariant typically passes name of .ino file for traceability
		@return 0 if successful, otherwise error code [ToDo: Add setup error codes]
	*/
  uint8_t setup(String firmwareVariant = "");   /**< Setup all the sensors */
	uint8_t update();
  void setAnalogEnablePin(uint8_t i); /**< Power on/off the analog circuitry */
  int8_t updateIMUData();                /**< Read any available IMU data from the sensor FIFO into the inputBuffers */
	int8_t updatePPGData();                /**< Read any available PPG data from the sensor FIFO into the inputBuffers */
	int8_t updateTempHumidityData();       /**< Read any available temperature and humidity data into the inputBuffers */
	int8_t updatePpgTempData();			/**< Read any available temp data from PPG into buffer*/
	int8_t updateThermopileData();         /**< Read Thermopile data into the buffers*/
	int8_t updateBatteryData();     /**< Reads battery voltage and passes it to different buffers for update */
	int8_t updateBatteryVoltageData(float battVolt);     /**< updates battery voltage inputBuffer */
	int8_t updateBatteryPercentData(float battPercent);     /**< updates battery percentage inputBuffer */
	float convertRawGyro(int16_t gRaw);
	float convertRawAcc(int16_t aRaw);
	float convertMagnetoX(int16_t mag_data_x, uint16_t data_rhall);
	float convertMagnetoY(int16_t mag_data_y, uint16_t data_rhall);
	float convertMagnetoZ(int16_t mag_data_z, uint16_t data_rhall);
	bool getBit(uint8_t num, uint8_t bit);
	void bmm150GetRegs(uint8_t address, uint8_t* dest, uint16_t len);
	void bmm150ReadTrimRegisters();
	// https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150.c#L90-L99
	// https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150_defs.h

  
  size_t getData(DataType t, float** data, uint32_t * timestamp = nullptr); /**< Swap buffers and read out the data */
  size_t getDataThermopile(float** data, uint32_t * timestamp = nullptr);
	//size_t dataAvailable(DataType t);
	float readBatteryVoltage();
	int8_t getBatteryPercent(float bv);
	bool setSensorTimer(SensorTimer t);
	bool printConfigInfo(File &file, const String &datetimeString);
	bool setSamplingRates(SamplingRates s);
	bool setSamplesAveraged(SamplesAveraged s);
	void scopeTimingTest();

private:
	float average(BufferFloat &b);
	int8_t checkIMUClipping(EmotiBit::DataType type, int16_t data, uint32_t timestamp);
	int8_t pushData(EmotiBit::DataType type, float data, uint32_t * timestamp = nullptr); // Deprecated. BUFFER_OVERFLOW count built into DoubleBuffer now.

	SensorTimer _sensorTimer = SensorTimer::MANUAL;
	SamplingRates _samplingRates;
	EnableDigitalFilter _enableDigitalFilter;
	SamplesAveraged _samplesAveraged;
	uint8_t _batteryReadPin;
	float _vcc;
	uint8_t _adcBits;
	float _accelerometerRange; // supported values: 2, 4, 8, 16 (G)
	float _gyroRange; // supported values: 125, 250, 500, 1000, 2000 (degrees/second)
	EmotiBitVersionController::EmotiBitVersion _hwVersion;
#if defined(ARDUINO_FEATHER_ESP32)
	String _featherVersion = "Adafruit Feather HUZZAH32";
#elif defined(ADAFRUIT_FEATHER_M0)
	String _featherVersion = "Adafruit Feather M0 WiFi";
#else
	String _featherVersion = "UNKNOWN";
#endif
	String _sourceId = "EmotiBit FeatherWing";
	String emotiBitSku;
	String emotibitDeviceId = "";
	String firmware_variant = "";
	uint32_t emotibitSerialNumber;
	uint8_t _imuFifoFrameLen = 0; // in bytes
	const uint8_t _maxImuFifoFrameLen = 40; // in bytes
	uint8_t _imuBuffer[40];

	// ToDo: Utilize on-chip PPG 32 sample buffer
	// Adjust buffer sizes for different PPG sampling rates & MCUs
	// See EmotiBit Buffer Size RAM Calculator google sheet
	// ToDo: Consider how to better manange stack memory allocation
#if !defined(EMOTIBIT_PPG_100HZ) && !defined(ARDUINO_FEATHER_ESP32)
	// 2.4 seconds data buffering
	const uint16_t EDA_BUFFER_SIZE = 36;
	const uint16_t PPG_BUFFER_SIZE = 60;
	const uint16_t TEMP_BUFFER_SIZE = 18;
	const uint16_t IMU_BUFFER_SIZE = 9;
#endif
#if !defined(EMOTIBIT_PPG_100HZ) && defined(ARDUINO_FEATHER_ESP32)
	// 4.8 seconds data buffering
	const uint16_t EDA_BUFFER_SIZE = 72;
	const uint16_t PPG_BUFFER_SIZE = 120;
	const uint16_t TEMP_BUFFER_SIZE = 36;
	const uint16_t IMU_BUFFER_SIZE = 69;
#endif
#if defined(EMOTIBIT_PPG_100HZ) && !defined(ARDUINO_FEATHER_ESP32)
	// 1.07 seconds data buffering
	const uint16_t EDA_BUFFER_SIZE = 16;
	const uint16_t PPG_BUFFER_SIZE = 107;
	const uint16_t TEMP_BUFFER_SIZE = 8;
	const uint16_t IMU_BUFFER_SIZE = 8;
#endif
#if defined(EMOTIBIT_PPG_100HZ) && defined(ARDUINO_FEATHER_ESP32)
	// 3.6 seconds data buffering
	const uint16_t EDA_BUFFER_SIZE = 54;
	const uint16_t PPG_BUFFER_SIZE = 360;
	const uint16_t TEMP_BUFFER_SIZE = 27;
	const uint16_t IMU_BUFFER_SIZE = 39;
#endif

	DoubleBufferFloat eda			 = DoubleBufferFloat(EDA_BUFFER_SIZE);
	DoubleBufferFloat edl			 = DoubleBufferFloat(EDA_BUFFER_SIZE);
	DoubleBufferFloat edr			 = DoubleBufferFloat(EDA_BUFFER_SIZE);
	DoubleBufferFloat ppgInfrared	 = DoubleBufferFloat(PPG_BUFFER_SIZE);
	DoubleBufferFloat ppgRed		 = DoubleBufferFloat(PPG_BUFFER_SIZE);
	DoubleBufferFloat ppgGreen		 = DoubleBufferFloat(PPG_BUFFER_SIZE);
	DoubleBufferFloat temp0			 = DoubleBufferFloat(TEMP_BUFFER_SIZE);
	DoubleBufferFloat temp1		     = DoubleBufferFloat(TEMP_BUFFER_SIZE); // To store the thermistor Object Temp
	DoubleBufferFloat therm0		 = DoubleBufferFloat(TEMP_BUFFER_SIZE); // To store the thermistor Object Temp
	DoubleBufferFloat therm0AMB		 = DoubleBufferFloat(TEMP_BUFFER_SIZE); // To store the raw value AMB from the thermistor
	DoubleBufferFloat therm0Sto		 = DoubleBufferFloat(TEMP_BUFFER_SIZE); // To store the raw Value Sto from the thermistor
	DoubleBufferFloat humidity0		 = DoubleBufferFloat(TEMP_BUFFER_SIZE);
	DoubleBufferFloat accelX		 = DoubleBufferFloat(IMU_BUFFER_SIZE); // Account for IMU_CHIP_BUFFER_SIZE when creating BUFFER_SIZE_FACTOR > 1
	DoubleBufferFloat accelY		 = DoubleBufferFloat(IMU_BUFFER_SIZE);
	DoubleBufferFloat accelZ		 = DoubleBufferFloat(IMU_BUFFER_SIZE);
	DoubleBufferFloat gyroX			 = DoubleBufferFloat(IMU_BUFFER_SIZE);
	DoubleBufferFloat gyroY			 = DoubleBufferFloat(IMU_BUFFER_SIZE);
	DoubleBufferFloat gyroZ			 = DoubleBufferFloat(IMU_BUFFER_SIZE);
	DoubleBufferFloat magX			 = DoubleBufferFloat(IMU_BUFFER_SIZE);
	DoubleBufferFloat magY			 = DoubleBufferFloat(IMU_BUFFER_SIZE);
	DoubleBufferFloat magZ			 = DoubleBufferFloat(IMU_BUFFER_SIZE);
	DoubleBufferFloat batteryVoltage = DoubleBufferFloat(1);
	DoubleBufferFloat batteryPercent = DoubleBufferFloat(1);
	DoubleBufferFloat dataOverflow	 = DoubleBufferFloat(1);	// ToDo: Refactor with nullptr checks for removal
	DoubleBufferFloat dataClipping   = DoubleBufferFloat(1);	// ToDo: Refactor with nullptr checks for removal
#ifdef DEBUG_BUFFER
	DoubleBufferFloat debugBuffer    = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE);
#else
	DoubleBufferFloat debugBuffer = DoubleBufferFloat(1);			// ToDo: Refactor for nullptr checks
#endif

	DoubleBufferFloat * dataDoubleBuffers[(uint8_t)DataType::length];

	// Oversampling buffers
	// Single buffered arrays must only be accessed from ISR functions, not in the main loop
	// ToDo: add assignment for dynamic allocation;
	// 	**** WARNING **** THIS MUST MATCH THE SAMPLING DIVS ETC
	BufferFloat edlBuffer = BufferFloat(5);
	BufferFloat edrBuffer = BufferFloat(5);	
	BufferFloat temperatureBuffer = BufferFloat(2);	
	BufferFloat humidityBuffer = BufferFloat(2);
	BufferFloat batteryVoltageBuffer = BufferFloat(15);
	BufferFloat batteryPercentBuffer = BufferFloat(15);

	const uint8_t SCOPE_TEST_PIN = A0;
	bool scopeTestPinOn = false;

};

void attachEmotiBit(EmotiBit*e = nullptr);
#ifdef ADAFRUIT_FEATHER_M0
void attachToInterruptTC3(void(*readFunction)(void), EmotiBit*e = nullptr);
void ReadSensors();
#elif defined ARDUINO_FEATHER_ESP32
void onTimer();
void attachToCore(void(*readFunction)(void*), EmotiBit*e = nullptr);
void ReadSensors(void* pvParameters);
#endif


#endif
