/**************************************************************************/
/*!
    @file     EmotiBitLedController.cpp
    @author   Nitin Nair (EmotiBit)

    @mainpage LED handler for EmotiBit

    @section intro_sec Introduction

    This class handles LEDs on EmotiBit.

		EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    @section author Author

    Written by Nitin Nair for EmotiBit.

    @section  HISTORY

    v0.1  - First release

    @section license License

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/
#include "EmotiBitLedController.h"


bool EmotiBitLedController::begin(TwoWire* emotibitI2c, EmotiBitVersionController::EmotiBitVersion hwVersion)
{
    bool status;
    _hwVersion = hwVersion;
    
    #ifdef TEST_KTD2026
        _hwVersion = EmotiBitVersionController::EmotiBitVersion::V07A;
    #endif
    if((uint8_t) _hwVersion > (uint8_t)EmotiBitVersionController::EmotiBitVersion::V06A)
    {
        Serial.println("initializing KTD2026");
        if(ktd2026b != nullptr)
        {
            delete ktd2026b;
            ktd2026b = nullptr;
        }
        ktd2026b = new KTD2026(KTD2026B_I2C_ADDRESS, emotibitI2c);
        status = ktd2026b->begin();
        ktd2026b->setEnable();
        ktd2026b->setChannelCurrent(KTD2026::Channel::CH1, settingsKTD2026.iOut);
        ktd2026b->setChannelCurrent(KTD2026::Channel::CH2, settingsKTD2026.iOut);
        ktd2026b->setChannelCurrent(KTD2026::Channel::CH3, settingsKTD2026.iOut);
    }
    else
    {
        if((uint8_t) hwVersion > (uint8_t)EmotiBitVersionController::EmotiBitVersion::V05C)
        {
            
            // EmotiBit HW v6
            status = ncp5623.begin(*emotibitI2c, NCP5623_C_DEFAULT_ADDR);
            
        }
        else
        {
            status = ncp5623.begin(*emotibitI2c, NCP5623_B_DEFAULT_ADDR);
        }
        // ToDo: version control this setup specific to EmotiBit HW with NCP
        // set NCP5623 settings
        // ToDo: expose NCP driver settings to function signature
        if (settingsNCP5623.driverCurrent > 0)
        {
            // ToDo: NCP driver update to avoid I2C communication in setCurrent
            ncp5623.setCurrent(settingsNCP5623.driverCurrent);
        }
        ncp5623.setLEDpwm((uint8_t)Led::RED, settingsNCP5623.pwmVal);
        ncp5623.setLEDpwm((uint8_t)Led::BLUE, settingsNCP5623.pwmVal);
        ncp5623.setLEDpwm((uint8_t)Led::YELLOW, settingsNCP5623.pwmVal);
        setState(Led::RED, false);
        setState(Led::BLUE, false);
        setState(Led::YELLOW, false);
        update();
    }
    // ToDo: expose other elements of the NCP settings to the LedController
    return status;
}

bool EmotiBitLedController::setState(Led led, bool state, bool updateNow)
{
    if((uint8_t) _hwVersion > (uint8_t)EmotiBitVersionController::EmotiBitVersion::V06A)
    {
        KTD2026::LedMode mode;
        KTD2026::Channel channel = ledToKtdMap[(uint8_t)led]; // use the map to convert EmotiBit LED to KTD2026 channel
        if(state)
        {
            mode = KTD2026::LedMode::ALWAYS_ON;
        }
        else
        {
            mode = KTD2026::LedMode::ALWAYS_OFF;
        }
        //Serial.print("Channel: "); Serial.print((uint8_t)channel); Serial.print("\tmode: "); Serial.println((uint8_t)mode);
        bool status = ktd2026b->updateChannelMode(channel, mode);
        if(!status)
        {
            return false;
        }
    }
    else
    {
        _ledStatus[(int)led].state = state;
        _ledStatus[(int)led].isChanged = true;
    }
    if(updateNow)
    {
        return update();
    }
    return true;
}

bool EmotiBitLedController::getState(Led led)
{
    if((uint8_t) _hwVersion > (uint8_t)EmotiBitVersionController::EmotiBitVersion::V06A)
    {
        uint8_t mask = 0b00000011;  // defaults to mask for Channel 1
        uint8_t channelModeReg = ktd2026b->getChannelControl();
        //Serial.print("Channel control: 0x"); Serial.println(channelModeReg, HEX);
        switch(led)
        {
            case Led::BLUE:  // shift by 2; 0b00001100, for channel 2
                mask = mask << 2;
                break;
            case Led::YELLOW: // shift by 4; 0b00110000, for channel 3
                mask = mask << 4;
                break;
        }
        //Serial.print("getState mask: 0x"); Serial.println(mask, HEX);
        if((channelModeReg & mask))
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
        return _ledStatus[(int)led].state;
    }    
}

bool EmotiBitLedController::_updateNcp()
{
    // update the internal class of the driver
    for (uint8_t i = 0;i<Led::length;i++)
    {
        if(_ledStatus[i].isChanged)
        {
            // ToDo: implement bounds check when referencing array
            ncp5623.setLED(ledToNcpMap[i], _ledStatus[i].state);
        }
    }
    
    // Communicate with the driver and write the changes to chip 
    ncp5623.send();
    // ToDo: change the function signature of NCP5623::send() to return i2c error so that it can be propagated upstream.
    return true;
}

bool EmotiBitLedController::_updateKtd2026()
{
    uint8_t status = ktd2026b->sendChannelControl();
    if(status)
    {
        return false;
    }
    return true;
}

bool EmotiBitLedController::update()
{
    // this will become version controlled if we change the LED driver on EmotiBit
    if((uint8_t) _hwVersion > (uint8_t)EmotiBitVersionController::EmotiBitVersion::V06A)
    {
        return _updateKtd2026();
    }
    else
    {
        return _updateNcp();
    }
}