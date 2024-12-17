/**************************************************************************/
/*!
    @file     EmotiBitLedController.h

    This class handles LEDs on EmotiBit.

    EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    Written by Nitin Nair for EmotiBit.

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/
#ifndef EMOTIBIT_LED_CONTROLLER
#define EMOTIBIT_LED_CONTROLLER
#include <Arduino.h>
#include "EmotiBitVersionController.h"
#include "EmotiBit_NCP5623.h"
#include "KTD2026.h"

class EmotiBitLedController
{
public:
    /*!
     * @brief enumerator for LEDs on EmotiBit 
    */
    enum Led
    {
        RED = 0,
        BLUE, 
        YELLOW,
        length 
    };

    /*!
     * @brief struct to hold current state of LEDs on EmotiBit
    */
    struct LedStatus
    {
        bool state = false;
        bool isChanged = false;
    } _ledStatus[Led::length];
    
    /*!
     * @brief Map LedController to NCP led numbers.
              The NCP led number are taken from the driver implementation.
    */
    const uint8_t ledToNcpMap[Led::length] = 
                                    { 
                                        1,  // Red
                                        2,  // Blue
                                        3   // Yellow
                                    }; 

    struct SettingsNCP5623{
        uint8_t driverCurrent = 1;
        uint8_t pwmVal = 8;
    } settingsNCP5623;

    /*!
        @brief Map EmotiBit LEDs to KTD2026 channels. Refer hardware schematic for mapping.
    */
    const KTD2026::Channel ledToKtdMap[Led::length] =
                                    {
                                        KTD2026::Channel::CH1, // RED
                                        KTD2026::Channel::CH2, // BLUE
                                        KTD2026::Channel::CH3  // YELLOW
                                    };

    struct SettingsKTD2026{
        // ToDo: consider if we want 3 iOut settings, 1 for each channel. 
        //Since each channel has a different LED color, different currents may be required to produce same luminosity.
        uint8_t iOut = 0x10;  // setting current to 2mA. value = 2mA/24mA * 192 steps = 16 steps = 0x10
    }settingsKTD2026;

    NCP5623 ncp5623; //<! instance of NCP5623
    KTD2026* ktd2026b = new KTD2026();  //<! instance of KTD2026

public:

    /*!
     * @brief Setup the LedController
     * @param emotibitI2c i2c instance on EmotiBit
     * @param hwVersion EmotiBit Hardware Version
     * @return true if setup successful, else false
    */
   bool begin(TwoWire* emotibitI2c, EmotiBitVersionController::EmotiBitVersion hwVersion);

    /*!
     * @brief set the state of the LED
     * @param led the led to set
     * @param state State LED is set to
     * @param updateNow Specify if the physical state should be updated immediately. Warning: EmotiBit LED driver should only be communicated with in the ISR if running stock EmotiBit FW.
     * @return true if success, else false
    */
    bool setState(Led led, bool state, bool updateNow = false);

    /*!
     * @brief Function to get the State of the LED
     * @param led Get state of this led
     * @return state of the LED can be true (ON) or false (OFF)
    */
    bool getState(Led led);
    
    /*!
     * @brief Function to update the LEDs with the internal LED state. Call with caution as it may involve communicating with the driver.
              EmotiBit stock FW only allows I2C communication ONLY DURING ISR. Communicating outside ISR may cause collision on the I2C line.
     * @return true if successful, else false
    */
    bool update();

    /*!
        @brief Set HW version in the LedController class
        @param version EmotiBit HW version
        @return True is set successfully, else false
    */
    bool setHwVersion(EmotiBitVersionController::EmotiBitVersion version)
    {
        _hwVersion = version;
        return true;
    }

    /*!
        @brief Get the HW version set in the LedController class
        @return HW version
    */
    EmotiBitVersionController::EmotiBitVersion getHeWVersion()
    {
        return _hwVersion;
    }

private:
    /*!
     * @brief Function to communicate with the NCP5623 driver
     * @return true if successful, else false
    */
    bool _updateNcp();

    /*!
        @brief Function to communicate with the KTD2026 driver
        @return True if successful, else false
    */
    bool _updateKtd2026();
    EmotiBitVersionController::EmotiBitVersion _hwVersion;
};
#endif