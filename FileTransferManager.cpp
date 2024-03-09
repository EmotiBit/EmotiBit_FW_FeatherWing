/**************************************************************************/
/*!
    @file     FileTransferManager.cpp
    @author   Nitin Nair (EmotiBit)

    @mainpage File transfer manager for EmotiBit

    @section intro_sec Introduction

    This is a library to file transfers on EmotiBit.

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

#include "FileTransferManager.h"

bool FileTransferManager::begin()
{
    Serial.println("Setting Mode to File Transfer");
  	setMode(FileTransferManager::Mode::FILE_TRANSFER);
    if(getProtocol() == Protocol::FTP)
    {
        if(WiFi.status() != WL_CONNECTED)
        {
            Serial.println("FTP server needs WiFi connection. Please connect to a WiFi network before starting FTP server");
            Serial.println("Please restart MCU, connect to WiFi and then start FTP server");
            return false;
        }
        else
        {
            // in FTP
            // create FTP server
            Serial.print("On network: "); Serial.println(WiFi.SSID());
            Serial.print("FTP server started at IP: "); Serial.println(WiFi.localIP());
            Serial.println("Use a FTP client (Example FileZilla) to access EmotiBit file system.");

            // start FTP server
            ftpServer.begin(_ftpCreds.username.c_str(), _ftpCreds.password.c_str());
            return true;
        }
    }
    else
    {
        // implement other protocol
        return false;
    }
    return false;
}

void FileTransferManager::setFtpAuth(String username, String password)
{
    _ftpCreds.username = username;
    _ftpCreds.password = password;
}

bool FileTransferManager::setProtocol(Protocol protocol)
{
    _protocol = protocol;
    // ToDo: Handle all conditions that may result in a return false
    return true;
}

FileTransferManager::Protocol FileTransferManager::getProtocol()
{
    return _protocol;
}

bool FileTransferManager::setMode(Mode mode)
{
    _mode = mode;
    return true;
}

FileTransferManager::Mode FileTransferManager::getMode()
{
    return _mode;
}

void FileTransferManager::handleTransfer()
{
    if(getMode() == Mode::FILE_TRANSFER)
    {
        if(getProtocol() == Protocol::FTP)
        {
            if(WiFi.status() != WL_CONNECTED)
            {
                Serial.println("FTP server needs WiFi connection. Please connect to a WiFi network before starting FTP server");
                Serial.println("Please restart MCU, connect to WiFi and then start FTP server");
                while(1);
            }
            else
            {
                /*
                uint32_t timeNow = millis(); // use to keep serial active
                const uint16_t PRINT_DELAY = 7000;  // time between serial prints. A way to keep serial active to represent on-going process 

                if( (millis() - timeNow) > PRINT_DELAY)
                {
                    // print somehting on the serial every 10 secs
                    Serial.print("FTP server active on "); Serial.println(WiFi.localIP());
                    timeNow = millis();
                }
                */
                // handleFTP requests. To get back to normal mode, press reset
                ftpServer.handleFTP();
                return;
            }
        }
        else
        {
            Serial.println("Protocol not Implemented. Please check FileTransferManager::handleTransfer for more details");
            setMode(Mode::NORMAL);
            Serial.println("Reset MCU to start again");
            while(1);
        }
    }
    else
    {
        // not in file transfer mode. Do nothing.
        return;
    }

}
