#include "EmotiBitCalibration.h"
//#include "EmotiBit.h"
EmotiBitCalibration::EmotiBitCalibration()
{
	calibrateSensor[(uint8_t)EmotiBitCalibration::SensorType::GSR] = false;
	// TODO : set it to false for all other sensors
	// calibrationTags[(uint8_t)SensorType::GSR] = "GSR_C\0";

	// TODO: for all other sensor types
}
EmotiBitCalibration::GsrCalibration::GsrCalibration()
{
	edlCumSum = 0;
	edlAvgCount = 1;
	calibrationTag = "GC\0";
	finishedSensorCalibration = false;
}

void EmotiBitCalibration::GsrCalibration::finishedCalibration()
{
	finishedSensorCalibration = true;
}

void EmotiBitCalibration::GsrCalibration::performCalibration(float newVal)
{
	if (edlAvgCount < MAX_EDL_AVG_COUNT)
	{
		// add the cummulative sum
		edlCumSum += newVal;
		edlAvgCount++;
		Serial.print("updating:");
		Serial.println(edlAvgCount);
	}
	else if (edlAvgCount == MAX_EDL_AVG_COUNT)
	{
		// take the average
		edlCumSum = edlCumSum / edlAvgCount;
		edlAvgCount++;
		Serial.println("updated");
	}
	else
	{
		// send this data over to the computer
		// set "calibrated tag"
		// Serial.println("done with data");
		finishedCalibration();
	}
}

//void EmotiBitCalibration::sendCalibrationPacket()
//{
//	uint8_t precision = 10;
//	// write code to generate and send the apcket over Wifi
//	EmotiBitPacket::Header header;
//	header = EmotiBitPacket::createHeader(gsrcalibration.calibrationTag, millis(), 0/*Find a way to share packetCunter*/, 1);
//	_outCalibrationPacket = "";
//	_outCalibrationPacket += EmotiBitPacket::headerToString(header);
//	_outCalibrationPacket += ',';
//	_outCalibrationPacket += String(gsrcalibration.edlCumSum, precision);
//	myEmotiBitWiFi.sendData(_outCalibrationPacket);
//	Serial.println("Sending the data:");
//	Serial.println(gsrcalibration.edlCumSum);
//	Serial.println("Ending Execution");
//	while (1);
//	//_emotiBitWiFi.sendData(_outDataPackets);
//}

//void EmotiBitCalibration::calibrateSensor(EmotiBitCalibration::SensorType sensor, float edlVal)
//{
//	if (sensor == EmotiBitCalibration::SensorType::GSR)
//	{
//		gsrcalibration.performCalibration();
//	}
//}

void EmotiBitCalibration::setSensorToCalibrate(EmotiBitCalibration::SensorType sensor)
{
	if (sensor == EmotiBitCalibration::SensorType::GSR)
	{
		calibrateSensor[(uint8_t)sensor] = true;
	}
	// TODO: other statements for other sensor calibrations
}

bool EmotiBitCalibration::getSensorToCalibrate(EmotiBitCalibration::SensorType sensor)
{
	return calibrateSensor[(uint8_t)sensor];
}