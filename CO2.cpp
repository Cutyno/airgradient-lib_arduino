/*
  CO2.cpp - Library for AirGradient sensor kit - implementation
  Copyright (c) 2006 John Doe.  All right reserved.
*/


// include this library's description file
#include "CO2.h"

// include description files for other libraries used (if any)
#include <SoftwareSerial.h>
#include "Arduino.h"
#include <Wire.h>
#include <math.h>

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

CO2::CO2(int rx_pin,int tx_pin,int baudRate, bool displayMsg){
    _debugMsg = displayMsg;
    if (_debugMsg) {
        Serial.println("Initializing CO2...");
    }
    _SoftSerial_CO2 = new SoftwareSerial(rx_pin,tx_pin);
    _SoftSerial_CO2->begin(baudRate);
    if(getCO2_Raw() == -1){
        if (_debugMsg) {
            Serial.println("CO2 Sensor Failed to Initialize ");
        }
    }
    else
    {
        Serial.println("CO2 Successfully Initialized. Heating up for 10s");
        delay(10000);
    }
}

// START CO2 FUNCTIONS //
const char *CO2::getCO2(int retryLimit)
{
    int ctr = 0;
    int result_CO2 = getCO2_Raw();
    while (result_CO2 == -1)
    {
        result_CO2 = getCO2_Raw();
        if ((ctr == retryLimit) || (result_CO2 == -1))
        {
            Char_CO2[0] = 'N';
            Char_CO2[1] = 'U';
            Char_CO2[2] = 'L';
            Char_CO2[3] = 'L';
            Char_CO2[4] = '\0';
            return Char_CO2;
        }
        ctr++;
    }
    sprintf(Char_CO2, "%d", result_CO2);
    return Char_CO2;
}

int CO2::getCO2_Raw()
{
    const byte CO2Command[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};
    byte CO2Response[] = {0, 0, 0, 0, 0, 0, 0};

    _SoftSerial_CO2->write(CO2Command, 7);
    delay(100); // give the sensor a bit of time to respond

    if (_SoftSerial_CO2->available())
    {
        for (int i = 0; i < 7; i++)
        {
            int byte = _SoftSerial_CO2->read();
            CO2Response[i] = byte;
            if (CO2Response[0] != 254)
            {
                return -1; // error code for debugging
            }
        }
        unsigned long val = CO2Response[3] * 256 + CO2Response[4];
        return val;
    }
    else
    {
        return -2; // error code for debugging
    }
}

// END CO2 FUNCTIONS //