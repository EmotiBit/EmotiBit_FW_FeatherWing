#include "EmotiBitWiFi.h"
#include <driver/source/nmasic.h>

uint8_t EmotiBitWiFi::begin(uint16_t timeout, uint16_t attemptDelay)
{
	uint8_t status = WiFi.status();
	uint32_t startBegin = millis();

	if (status == WL_NO_SHIELD) {
		Serial.println("No WiFi shield found. Try WiFi.setPins().");
		return WL_NO_SHIELD;
	}

	checkWiFi101FirmwareVersion();

	while (status != WL_CONNECTED)
	{
		if (numCredentials == 0)
		{
			Serial.println("NO WIFI CREDENTIALS FOUND");
		}
		else
		{
			Serial.println("<<<<<<< Switching WiFi Networks >>>>>>>");
			status = begin(credentials[currentCredential].ssid, credentials[currentCredential].pass, 1, attemptDelay);
			if (status == WL_CONNECTED) {
				break;
			}
		}
		if (millis() - startBegin > timeout)
		{
			Serial.println("*********** EmotiBitWiFi.begin() Timeout ***********");
			break;
		}
		currentCredential = (currentCredential + 1) % numCredentials;
	}
	return status;
}

//uint8_t EmotiBitWiFi::begin(uint8_t credentialIndex, uint8_t maxAttempts, uint16_t attemptDelay)
//{
//	if (credentialIndex < numCredentials) {
//		return begin(credentials->ssid, credentials->password);
//	}
//	return WL_CONNECT_FAILED;
//}

uint8_t EmotiBitWiFi::begin(const String &ssid, const String &pass, uint8_t maxAttempts, uint16_t attemptDelay)
{

	int8_t status = WiFi.status();
	int8_t attempt = 0;

	if (status == WL_NO_SHIELD) {
		Serial.println("No WiFi shield found. Try WiFi.setPins().");
		return WL_NO_SHIELD;
	}

	while (status != WL_CONNECTED) 
	{
		if (attempt > maxAttempts)
		{
			return status;
		}
		Serial.print("Attempting to connect to SSID: ");
		Serial.println(ssid);
		// ToDo: Add WEP support
		_wifiOff = false;
		status = WiFi.begin(ssid, pass);
		_needsAdvertisingBegin = true;
		delay(attemptDelay);
		attempt++;
	}
	WiFi.setTimeout(25);
	wifiBeginStart = millis();
	Serial.println("Connected to WiFi");
	printWiFiStatus();

	Serial.print("Starting EmotiBit advertising connection on port ");
	Serial.println(_advertisingPort);
	_advertisingCxn.begin(_advertisingPort);
	_needsAdvertisingBegin = false;

	return status;
}

void EmotiBitWiFi::end()
{
	if (_isConnected)
	{
		Serial.println("Disconnecting EmotiBitWiFi");
		disconnect();
	}
	if (WiFi.status() == WL_CONNECTED) {
		Serial.println("Disconnecting WiFi...");
		WiFi.disconnect();
	}
	if (!_wifiOff)
	{
		Serial.println("Ending WiFi...");
		_wifiOff = true;
		WiFi.end();
	}
}

bool EmotiBitWiFi::isOff()
{
	return _wifiOff;
}

int8_t EmotiBitWiFi::updateWiFi()
{
	if (_wifiOff)
	{
		return WL_DISCONNECTED;
	}

	int wifiStatus = WiFi.status();
	if (wifiStatus != WL_CONNECTED)
	{
		static bool getLostWifiTime = true;
		static uint32_t lostWifiTime;

		if (getLostWifiTime) {
			lostWifiTime = millis();
			getLostWifiTime = false;
			//DBTAG1
			//addDebugPacket((uint8_t)EmotiBit::DebugTags::WIFI_TIMELOSTCONN, lostWifiTime);// for reporting Wifi Lost moment
		}

		static uint8_t wifiReconnectAttempts = 0;
		if (millis() - wifiBeginStart > WIFI_BEGIN_ATTEMPT_DELAY)
		{
			bool switchCredentials = false;
			if ((millis() - lostWifiTime > WIFI_BEGIN_SWITCH_CRED) && (wifiReconnectAttempts >= MAX_WIFI_RECONNECT_ATTEMPTS))
			{
				switchCredentials = true;
				wifiReconnectAttempts = 0;
				//gotIP = false;
				//sendResetPacket = true;
			}

			if (switchCredentials && numCredentials > 0)
			{
				currentCredential = (currentCredential + 1) % numCredentials;
				Serial.println("<<<<<<< Switching WiFi Networks >>>>>>>");
			}
			Serial.print("WIFI_BEGIN_ATTEMPT_DELAY: ");
			Serial.println(WIFI_BEGIN_ATTEMPT_DELAY);
			//Serial.println(lostWifiTime);               //uncomment for debugging
			wifiStatus = begin(credentials[currentCredential].ssid, credentials[currentCredential].pass, 1, 0);
			wifiReconnectAttempts++;
			wifiBeginStart = millis();
		}
		if (wifiStatus == WL_CONNECTED) {
			getLostWifiTime = true;
			//For case where begin works immediately in this loop, but disconnects again within
			// the attempt delay. Without changing nBS, the code is blocked from attempting to reconnect
			wifiBeginStart = millis() - WIFI_BEGIN_ATTEMPT_DELAY - 1;
		}
	}
}

int8_t EmotiBitWiFi::readUdp(WiFiUDP &udp, String &message)
{
	int8_t wifiStatus = status();
	if (wifiStatus != WL_CONNECTED)
	{
		return wifiStatus;
	}

	// ToDo: Consider need for a while loop here handle packet backlog
	int packetSize = udp.parsePacket();
	if (packetSize)
	{

		// read the packet into packetBufffer
		int len = udp.read(_inPacketBuffer, EmotiBitPacket::MAX_TO_EMOTIBIT_PACKET_LEN);
		if (len > 0)
		{
			_inPacketBuffer[len] = '\0';
			message = String(_inPacketBuffer);
			return SUCCESS;
		}
	}
	return FAIL;
}

int8_t EmotiBitWiFi::processAdvertising()
{
	int8_t wifiStatus = status();
	if (wifiStatus != WL_CONNECTED)
	{
		return wifiStatus;
	}

	if (_needsAdvertisingBegin)
	{
		_advertisingCxn.begin(_advertisingPort);
		_needsAdvertisingBegin = false;
	}

	EmotiBitPacket::Header outPacketHeader;
	String outMessage = "";

	// NOTE: Currently only one packet per message (datagram) is processed
	// ToDo: Parse delimiters to split up multi-packet messages
	// ToDo: Consider need for a while loop here handle packet backlog
	int8_t status = readUdp(_advertisingCxn, _receivedAdvertisingPacket);
	if (status == SUCCESS)
	{
		if (DEBUG_PRINT) Serial.print("Received: ");
		if (DEBUG_PRINT) Serial.print(_receivedAdvertisingPacket);
		int16_t dataStartChar = EmotiBitPacket::getHeader(_receivedAdvertisingPacket, _packetHeader);
		if (dataStartChar != EmotiBitPacket::MALFORMED_HEADER)
		{
			bool sendMessage = false;
			IPAddress senderIp = _advertisingCxn.remoteIP();
			uint16_t senderPort = _advertisingCxn.remotePort();

			if (_packetHeader.typeTag.equals(EmotiBitPacket::TypeTag::HELLO_EMOTIBIT))
			{
				EmotiBitPacket::Header outPacketHeader = EmotiBitPacket::createHeader(EmotiBitPacket::TypeTag::HELLO_HOST, millis(), advertisingPacketCounter++);
				outMessage += EmotiBitPacket::headerToString(outPacketHeader);
				outMessage += ",";
				outMessage += EmotiBitPacket::PayloadLabel::DATA_PORT;
				outMessage += ",";
				outMessage += _dataPort;
				outMessage += EmotiBitPacket::PACKET_DELIMITER_CSV;
				sendMessage = true;
			}
			else if (_packetHeader.typeTag.equals(EmotiBitPacket::TypeTag::EMOTIBIT_CONNECT) && dataStartChar > 0)
			{
				//_controlCxn.setTimeout(50);
				//_dataCxn.setTimeout(50);

				if (!_isConnected)
				{
					connect(_advertisingCxn.remoteIP(), _receivedAdvertisingPacket.substring(dataStartChar));
				}
				if (_isConnected)
				{
					// Send a message to tell the host that the connection worked
					outMessage += createPongPacket();
					sendMessage = true;

					connectionTimer = millis();
				}
			}
			else if (_packetHeader.typeTag.equals(EmotiBitPacket::TypeTag::PING) && dataStartChar > 0)
			{
				if (_isConnected)
				{
					String value;
					int16_t valueChar = EmotiBitPacket::getPacketKeyedValue(_receivedAdvertisingPacket,
						EmotiBitPacket::PayloadLabel::DATA_PORT, value, dataStartChar);
					if (valueChar > -1)
					{
						// We found DATA_PORT in the payload
						if (senderIp == _hostIp && value.toInt() == _dataPort)
						{
							// PING is from our host, reply with PONG to tell the host that we're connected
							connectionTimer = millis();	// Refresh connection timer
							outMessage += createPongPacket();
							sendMessage = true;
						}
					}
				}
			}
			if (sendMessage)
			{
				if (DEBUG_PRINT) Serial.print("Sending: ");
				if (DEBUG_PRINT) Serial.print(outMessage);
				//syncPackets + outMessage;
				sendAdvertising(outMessage, senderIp, senderPort);
			}
		}
	}

	if (_isConnected)
	{
		if (millis() - connectionTimer > connectionTimeout)
		{
			disconnect();
		}
	}

	// ToDo: Consider adding _controlCxn repairing mechanism
	return SUCCESS;
}

int8_t EmotiBitWiFi::sendAdvertising(const String& message, const IPAddress& ip, uint16_t port)
{
	sendUdp(_advertisingCxn, message, ip, port);
}

int8_t EmotiBitWiFi::sendData(const String &message)
{
	if (_isConnected)
	{
		// ToDo: Consider adding _controlCxn.connected() conditional
		sendUdp(_dataCxn, message, _hostIp, _dataPort);
	}
}

int8_t EmotiBitWiFi::sendUdp(WiFiUDP& udp, const String& message, const IPAddress& ip, uint16_t port)
{
	int8_t wifiStatus = status();
	if (wifiStatus != WL_CONNECTED)
	{
		return wifiStatus;
	}

	static int16_t firstIndex;
	firstIndex = 0;
	while (firstIndex < message.length()) {
		static int16_t lastIndex;
		lastIndex = message.length() - 1; // message.length() - 1 to handle \n 
		while (lastIndex - firstIndex > MAX_SEND_LEN - 1) {
			lastIndex = message.lastIndexOf(EmotiBitPacket::PACKET_DELIMITER_CSV, lastIndex - 1);
			//Serial.println(lastIndex);
		}
		//Serial.println(outputMessage.substring(firstIndex, lastIndex));
		for (uint8_t n = 0; n < _nDataSends; n++) {
			udp.beginPacket(ip, port);
			udp.print(message.substring(firstIndex, lastIndex + 1));	// use lastIndex + 1 to get \n back
			udp.endPacket();
		}
		firstIndex = lastIndex + 1;	// increment substring indexes for breaking up sends
	}
	
	return SUCCESS;
}

uint8_t EmotiBitWiFi::readControl(String& packet) 
{
	uint8_t numPackets = 0;
	if (_isConnected) 
	{
		if (!_controlCxn.connected())
		{
			// We lost our Control connection
			disconnect();
		}
		else
		{
			while (_controlCxn.available())
			{
				int c = _controlCxn.read();

				if (c == (int)EmotiBitPacket::PACKET_DELIMITER_CSV)
				{
					numPackets++;
					packet = "";
					packet += _receivedControlMessage;
					_receivedControlMessage = "";
					return numPackets;
				}
				else
				{
					if (c == 0) {
						// Throw out null term
						// ToDo: handle this more properly
					}
					else
					{
						//Serial.print((char)c);
						_receivedControlMessage += (char)c;
					}
				}
			}
		}
	}
	return numPackets;
}

int8_t EmotiBitWiFi::connect(const IPAddress &hostIp, const String& connectPayload) {

	int8_t wifiStatus = status();
	if (wifiStatus != WL_CONNECTED)
	{
		return wifiStatus;
	}

	bool gotControlPort = false;
	bool gotDataPort = false;
	String value;
	int16_t valueChar;
	uint16_t controlPort;
	uint16_t dataPort;
	valueChar = EmotiBitPacket::getPacketKeyedValue(connectPayload,	EmotiBitPacket::PayloadLabel::CONTROL_PORT, value);
	if (valueChar > -1)
	{
		controlPort = value.toInt();
		gotControlPort = true;
	}
	valueChar = EmotiBitPacket::getPacketKeyedValue(connectPayload, EmotiBitPacket::PayloadLabel::DATA_PORT, value);
	if (valueChar > -1)
	{
		dataPort = value.toInt();
		gotDataPort = true;
	}

	if (gotControlPort & gotDataPort)
	{
		return connect(hostIp, controlPort, dataPort);
	}
	else
	{
		return FAIL;
	}
}
int8_t EmotiBitWiFi::connect(const IPAddress &hostIp, uint16_t controlPort, uint16_t dataPort) {

	int8_t wifiStatus = status();
	if (wifiStatus != WL_CONNECTED)
	{
		return wifiStatus;
	}

	if (_isConnected)
	{
		disconnect();
	}

	Serial.print("\nStarting control connection to server: ");
	Serial.print(hostIp);
	Serial.print(" : ");
	Serial.print(controlPort);
	Serial.print(" ... ");
	if (_controlCxn.connect(hostIp, controlPort))
	{
		_isConnected = true;
		_controlCxn.flush();
		Serial.println("connected");

		_hostIp = hostIp;
		_dataPort = dataPort;
		_controlPort = controlPort;

		// ToDo: Send a message to host to confirm connection

		Serial.print("Starting data connection to server: ");
		Serial.print(hostIp);
		Serial.print(" : ");
		Serial.println(dataPort);
		_dataCxn.begin(dataPort);
		return SUCCESS;
	}
	return FAIL;
}

int8_t EmotiBitWiFi::disconnect() {
	Serial.println("Disconnect()");
	int8_t retStatus = FAIL;

	int8_t wifiStatus = status();
	if (wifiStatus != WL_CONNECTED)
	{
		retStatus = wifiStatus;
	}

	if (_isConnected) {
		if (_controlCxn.connected())
		{
			Serial.println("Disconnecting... ");
			Serial.println("Stopping Control Cxn... ");
			_controlCxn.flush();
			_controlCxn.stop();
		}
		Serial.println("Stopping Data Cxn... ");
		_dataCxn.flush();
		_dataCxn.stop();
		Serial.println("Stopped... ");

		retStatus = SUCCESS;
	}

	_isConnected = false;
	_dataPort = -1;
	_controlPort = -1;

	return retStatus;
}

String EmotiBitWiFi::createPongPacket()
{
	String outMessage;
	EmotiBitPacket::Header outPacketHeader = EmotiBitPacket::createHeader(EmotiBitPacket::TypeTag::PONG, millis(), advertisingPacketCounter++, 2);
	outMessage += EmotiBitPacket::headerToString(outPacketHeader);
	outMessage += ",";
	outMessage += EmotiBitPacket::PayloadLabel::DATA_PORT;
	outMessage += ",";
	outMessage += _dataPort;
	outMessage += EmotiBitPacket::PACKET_DELIMITER_CSV;
	return outMessage;
}

int8_t EmotiBitWiFi::update(String &syncPackets, uint16_t &syncPacketCounter)
{
	updateWiFi();

	processAdvertising();

	if (_isConnected)
	{
		// ToDo:: Consider imposing a post-data-send delay to ensure host isn't bogged down when time sync arrives
		static uint32_t timeSyncTimer = millis();
		if (millis() - timeSyncTimer > TIME_SYNC_INTERVAL)
		{
			timeSyncTimer = millis();
			processTimeSync(syncPackets, syncPacketCounter);
		}
	}
}

int8_t EmotiBitWiFi::processTimeSync(String &syncPackets, uint16_t &syncPacketCounter)
{
	// ToDo: Consider whether time syncing should be performed on the advertising channel
	if (_isConnected)
	{
		String syncMessage;
		EmotiBitPacket::Header header;
		int8_t status;
		int16_t dataStartChar;

		//makes sure no old messages are in the read buffer
		status = readUdp(_dataCxn, syncMessage);
		while (status == SUCCESS) {
			dataStartChar = EmotiBitPacket::getHeader(syncMessage, header);
			syncPackets += EmotiBitPacket::createPacket(header.typeTag, syncPacketCounter++,
				syncMessage.substring(dataStartChar, syncMessage.length() - 1), header.dataLength);
			status = readUdp(_dataCxn, syncMessage);
		}

		int16_t rdPacketNumber = syncPacketCounter;

		String payload;
		payload += EmotiBitPacket::TypeTag::TIMESTAMP_LOCAL;
		payload += ",";
		payload += EmotiBitPacket::TypeTag::TIMESTAMP_UTC;
		String packet = EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::REQUEST_DATA, syncPacketCounter++, payload, 2);
		sendData(packet);
		if (DEBUG_PRINT) Serial.print("Sync Sent: ");
		if (DEBUG_PRINT) Serial.print(packet);
		syncPackets += packet;

		uint32_t syncWaitTimer = millis();

		while (millis() - syncWaitTimer < MAX_SYNC_WAIT_INTERVAL)
		{
			status = readUdp(_dataCxn, syncMessage);
			// NOTE: Currently only one packet per message (datagram) is processed
			// ToDo: Parse delimiters to split up multi-packet messages
			if (status == SUCCESS)
			{
				if (DEBUG_PRINT) Serial.print("Sync Received: ");
				if (DEBUG_PRINT) Serial.print(syncMessage);
				dataStartChar = EmotiBitPacket::getHeader(syncMessage, header);
				if (dataStartChar > 0)
				{
					if (header.typeTag.equals(EmotiBitPacket::TypeTag::TIMESTAMP_LOCAL))
					{
						syncPackets += EmotiBitPacket::createPacket(header.typeTag, syncPacketCounter++,
							syncMessage.substring(dataStartChar, syncMessage.length() - 1), header.dataLength);
						//syncPackets += EmotiBitPacket::headerToString(EmotiBitPacket::createHeader(header.typeTag, millis(), syncPacketCounter++, header.dataLength));
						//syncPackets += syncMessage.substring(dataStartChar);
					}
					else if (header.typeTag.equals(EmotiBitPacket::TypeTag::TIMESTAMP_UTC))
					{
						syncPackets += EmotiBitPacket::createPacket(header.typeTag, syncPacketCounter++,
							syncMessage.substring(dataStartChar, syncMessage.length() - 1), header.dataLength);
					}
					else if (header.typeTag.equals(EmotiBitPacket::TypeTag::ACK))
					{
						syncPackets += EmotiBitPacket::createPacket(header.typeTag, syncPacketCounter++,
							syncMessage.substring(dataStartChar, syncMessage.length() - 1), header.dataLength);
						String ackPacketNumber;
						EmotiBitPacket::getPacketElement(syncMessage, ackPacketNumber, dataStartChar);
						if (rdPacketNumber == ackPacketNumber.toInt()) {
							return SUCCESS;
						}
					}
				}
			}
		}
	}
	return FAIL;
}

void EmotiBitWiFi::printWiFiStatus() {
	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print your WiFi shield's IP address:
	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: ");
	Serial.println(ip);

	// print the received signal strength:
	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):");
	Serial.print(rssi);
	Serial.println(" dBm");
}

int8_t EmotiBitWiFi::addCredential(const String &ssid, const String &password)
{
	if (numCredentials < MAX_CREDENTIALS)
	{
		credentials[numCredentials].ssid = ssid;
		credentials[numCredentials].pass = password;
		numCredentials++;
	}
}

uint8_t EmotiBitWiFi::listNetworks() {
	if (_wifiOff)
	{
		return 0;
	}

	// scan for nearby networks:
	Serial.println("xxxxxxx Scan Networks xxxxxxx");
	Serial.println(millis());
	int numSsid = WiFi.scanNetworks();
	if (numSsid == -1)
	{
		Serial.println("Couldn't get a wifi connection");
		//while (true);
	}
	Serial.println(millis());

	// print the list of networks seen:
	Serial.print("number of available networks:");
	Serial.println(numSsid);

	// print the network number and name for each network found:
	for (int thisNet = 0; thisNet < numSsid; thisNet++) {
		Serial.print(thisNet);
		Serial.print(") ");
		Serial.print(WiFi.SSID(thisNet));
		Serial.print("\tSignal: ");
		Serial.print(WiFi.RSSI(thisNet));
		Serial.print(" dBm");
		Serial.print(" \n");
		//Serial.print("\tEncryption: ");
		//printEncryptionType(WiFi.encryptionType(thisNet));
		//Serial.flush();
	}
	Serial.println(millis());
	Serial.println("xxxxxxx  xxxxxxx");

	return numSsid;
}

bool EmotiBitWiFi::isConnected()
{
	return _isConnected;
}

int8_t EmotiBitWiFi::status()
{
	if (_wifiOff)
	{
		return WL_DISCONNECTED;
	}
	return (int8_t) WiFi.status();
}

void EmotiBitWiFi::checkWiFi101FirmwareVersion()
{
	// Print a welcome message
	Serial.println("WiFi101 firmware check.");

	// Check for the presence of the shield
	Serial.print("WiFi101 shield: ");
	if (WiFi.status() == WL_NO_SHIELD) {
		Serial.println("NOT PRESENT");
		return; // don't continue
	}
	Serial.println("DETECTED");

	// Print firmware version on the shield
	String fv = WiFi.firmwareVersion();
	String latestFv;
	Serial.print("Firmware version installed: ");
	Serial.println(fv);

	if (REV(GET_CHIPID()) >= REV_3A0) {
		// model B
		latestFv = WIFI_FIRMWARE_LATEST_MODEL_B;
	}
	else {
		// model A
		latestFv = WIFI_FIRMWARE_LATEST_MODEL_A;
	}

	// Print required firmware version
	Serial.print("Latest firmware version available : ");
	Serial.println(latestFv);
}