/*
  AirGradient.cpp - Library for AirGradient sensor kit - implementation
  Copyright (c) 2006 John Doe.  All right reserved.
*/


// include this library's description file
#include "SHT.h"

// include description files for other libraries used (if any)
#include <SoftwareSerial.h>
#include "Arduino.h"
#include <Wire.h>
#include <math.h>

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

SHT::SHT(uint8_t address, bool displayMsg)
{
    _debugMsg = displayMsg;
    if (_debugMsg)
    {
        Serial.println("Initializing TMP_RH...");
    }
    _address = address;
    periodicStart(SHT3XD_REPEATABILITY_HIGH, SHT3XD_FREQUENCY_10HZ);
}

//START TMP_RH FUNCTIONS//

TMP_RH_ErrorCode SHT::reset()
{
    return  softReset();
}

TMP_RH SHT::periodicFetchData()
{
    TMP_RH result;
    TMP_RH_raw sensor;
    TMP_RH_ErrorCode error = writeCommand(SHT3XD_CMD_FETCH_DATA);
    if (error == SHT3XD_NO_ERROR)
    {
        sensor = readTemperatureAndHumidity();
        sprintf(result.t_char,"%d", sensor.t);
        sprintf(result.rh_char,"%f", sensor.rh);

        return result;
    }
    return returnError(error);
}

TMP_RH_raw SHT::periodicFetchData_raw()
{
    TMP_RH_raw result;
    TMP_RH_ErrorCode error = writeCommand(SHT3XD_CMD_FETCH_DATA);
    if (error == SHT3XD_NO_ERROR)
    {
        result = readTemperatureAndHumidity();
        return result;
    }
    return returnError_raw(error);
}

TMP_RH_ErrorCode SHT::periodicStop(void)
{
    return writeCommand(SHT3XD_CMD_STOP_PERIODIC);
}

TMP_RH_ErrorCode SHT::periodicStart(TMP_RH_Repeatability repeatability, TMP_RH_Frequency frequency) //
{
    TMP_RH_ErrorCode error;

    switch (repeatability)
    {
        case SHT3XD_REPEATABILITY_LOW:
            switch (frequency)
            {
                case SHT3XD_FREQUENCY_HZ5:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_HALF_L);
                break;
                case SHT3XD_FREQUENCY_1HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_1_L);
                break;
                case SHT3XD_FREQUENCY_2HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_2_L);
                break;
                case SHT3XD_FREQUENCY_4HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_4_L);
                break;
                case SHT3XD_FREQUENCY_10HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_10_L);
                break;
                default:
                    error = SHT3XD_PARAM_WRONG_FREQUENCY;
                break;
            }
        break;
        case SHT3XD_REPEATABILITY_MEDIUM:
            switch (frequency)
            {
                case SHT3XD_FREQUENCY_HZ5:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_HALF_M);
                break;
                case SHT3XD_FREQUENCY_1HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_1_M);
                break;
                case SHT3XD_FREQUENCY_2HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_2_M);
                break;
                case SHT3XD_FREQUENCY_4HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_4_M);
                break;
                case SHT3XD_FREQUENCY_10HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_10_M);
                break;
                default:
                    error = SHT3XD_PARAM_WRONG_FREQUENCY;
                break;
            }
        break;
        case SHT3XD_REPEATABILITY_HIGH:
            switch (frequency)
            {
                case SHT3XD_FREQUENCY_HZ5:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_HALF_H);
                break;
                case SHT3XD_FREQUENCY_1HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_1_H);
                break;
                case SHT3XD_FREQUENCY_2HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_2_H);
                break;
                case SHT3XD_FREQUENCY_4HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_4_H);
                break;
                case SHT3XD_FREQUENCY_10HZ:
                    error = writeCommand(SHT3XD_CMD_PERIODIC_10_H);
                break;
                default:
                    error = SHT3XD_PARAM_WRONG_FREQUENCY;
                break;
            }
        break;
        default:
            error = SHT3XD_PARAM_WRONG_REPEATABILITY;
        break;
    }
    delay(100);
    return error;
}


TMP_RH_ErrorCode SHT::writeCommand(TMP_RH_Commands command)
{
    Wire.beginTransmission(_address);
    Wire.write(command >> 8);
    Wire.write(command & 0xFF);
    return (TMP_RH_ErrorCode)(-10 * Wire.endTransmission());
}

TMP_RH_ErrorCode SHT::softReset() {
    return writeCommand(SHT3XD_CMD_SOFT_RESET);
}


uint32_t SHT::readSerialNumber()
{
    uint32_t result = SHT3XD_NO_ERROR;
    uint16_t buf[2];
    if (writeCommand(SHT3XD_CMD_READ_SERIAL_NUMBER) == SHT3XD_NO_ERROR)
    {
        if (read_TMP_RH(buf, 2) == SHT3XD_NO_ERROR)
        {
            result = (buf[0] << 16) | buf[1];
        }
    }
    else if(writeCommand(SHT3XD_CMD_READ_SERIAL_NUMBER) != SHT3XD_NO_ERROR)
    {
        if (_debugMsg)
        {
            Serial.println("TMP_RH Failed to Initialize.");
        }
    }
    return result;
}

uint32_t SHT::testTMP_RH()
{
    uint32_t result = SHT3XD_NO_ERROR;
    uint16_t buf[2];

    if (writeCommand(SHT3XD_CMD_READ_SERIAL_NUMBER) == SHT3XD_NO_ERROR)
    {
        if (read_TMP_RH(buf, 2) == SHT3XD_NO_ERROR)
        {
            result = (buf[0] << 16) | buf[1];
        }
        if (_debugMsg)
        {
            Serial.print("TMP_RH successfully initialized with serial number: ");
            Serial.println(result);
        }
    }
    else if(writeCommand(SHT3XD_CMD_READ_SERIAL_NUMBER) != SHT3XD_NO_ERROR)
    {
        if (_debugMsg)
        {
            Serial.println("TMP_RH Failed to Initialize.");
        }
    }
    return result;
}

TMP_RH_ErrorCode SHT::clearAll() {
    return writeCommand(SHT3XD_CMD_CLEAR_STATUS);
}


TMP_RH_raw SHT::readTemperatureAndHumidity()
{
    TMP_RH_raw result;
    result.t = 0;
    result.rh = 0;
    TMP_RH_ErrorCode error;
    uint16_t buf[2];

    if (error == SHT3XD_NO_ERROR)
        error = read_TMP_RH(buf, 2);

    if (error == SHT3XD_NO_ERROR)
    {
        result.t = calculateTemperature(buf[0]);
        result.rh = calculateHumidity(buf[1]);
    }
    result.error = error;
    return result;
}

TMP_RH_ErrorCode SHT::read_TMP_RH(uint16_t* data, uint8_t numOfPair)//
{
    uint8_t buf[2];
    uint8_t checksum;

    const uint8_t numOfBytes = numOfPair * 3;
    Wire.requestFrom(_address, numOfBytes);
    for (int counter = 0; counter < numOfPair; counter++)
    {
        Wire.readBytes(buf, (uint8_t)2);
        checksum = Wire.read();
        if (checkCrc(buf, checksum) != 0)
            return SHT3XD_CRC_ERROR;
        data[counter] = (buf[0] << 8) | buf[1];
    }
    return SHT3XD_NO_ERROR;
}


uint8_t SHT::checkCrc(uint8_t data[], uint8_t checksum)
{
    return calculateCrc(data) != checksum;
}

float SHT::calculateTemperature(uint16_t rawValue)
{
    float value = 175.0f * (float)rawValue / 65535.0f - 45.0f;
    return round(value*10)/10;
}


float SHT::calculateHumidity(uint16_t rawValue)
{
    return 100.0f * rawValue / 65535.0f;
}

uint8_t SHT::calculateCrc(uint8_t data[])
{
    uint8_t bit;
    uint8_t crc = 0xFF;
    for (uint8_t dataCounter = 0; dataCounter < 2; dataCounter++)
    {
        crc ^= (data[dataCounter]);
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x131;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

TMP_RH SHT::returnError(TMP_RH_ErrorCode error)
{
    TMP_RH result;

    result.t_char[0] = 'N';
    result.t_char[1] = 'U';
    result.t_char[2] = 'L';
    result.t_char[3] = 'L';
    result.t_char[4] = '\0';

    result.rh_char[0] = 'N';
    result.rh_char[1] = 'U';
    result.rh_char[2] = 'L';
    result.rh_char[3] = 'L';
    result.rh_char[4] = '\0';

    result.error = error;
    return result;
}

TMP_RH_raw SHT::returnError_raw(TMP_RH_ErrorCode error)
{
    TMP_RH_raw result;
    result.t = NULL;
    result.rh = NULL;

    result.error = error;
    return result;
}

//END TMP_RH FUNCTIONS //