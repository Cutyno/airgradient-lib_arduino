/*
  SHT.h - Library for SHT sensor kit - description
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// ensure this library description is only included once
#ifndef SHT_h
#define SHT_h

#include <SoftwareSerial.h>
#include <Print.h>
#include "Stream.h"

//ENUMS AND STRUCT FOR TMP_RH START
typedef enum
{
    SHT3XD_CMD_READ_SERIAL_NUMBER = 0x3780,

    SHT3XD_CMD_READ_STATUS = 0xF32D,
    SHT3XD_CMD_CLEAR_STATUS = 0x3041,

    SHT3XD_CMD_HEATER_ENABLE = 0x306D,
    SHT3XD_CMD_HEATER_DISABLE = 0x3066,

    SHT3XD_CMD_SOFT_RESET = 0x30A2,

    SHT3XD_CMD_CLOCK_STRETCH_H = 0x2C06,
    SHT3XD_CMD_CLOCK_STRETCH_M = 0x2C0D,
    SHT3XD_CMD_CLOCK_STRETCH_L = 0x2C10,

    SHT3XD_CMD_POLLING_H = 0x2400,
    SHT3XD_CMD_POLLING_M = 0x240B,
    SHT3XD_CMD_POLLING_L = 0x2416,

    SHT3XD_CMD_ART = 0x2B32,

    SHT3XD_CMD_PERIODIC_HALF_H = 0x2032,
    SHT3XD_CMD_PERIODIC_HALF_M = 0x2024,
    SHT3XD_CMD_PERIODIC_HALF_L = 0x202F,
    SHT3XD_CMD_PERIODIC_1_H = 0x2130,
    SHT3XD_CMD_PERIODIC_1_M = 0x2126,
    SHT3XD_CMD_PERIODIC_1_L = 0x212D,
    SHT3XD_CMD_PERIODIC_2_H = 0x2236,
    SHT3XD_CMD_PERIODIC_2_M = 0x2220,
    SHT3XD_CMD_PERIODIC_2_L = 0x222B,
    SHT3XD_CMD_PERIODIC_4_H = 0x2334,
    SHT3XD_CMD_PERIODIC_4_M = 0x2322,
    SHT3XD_CMD_PERIODIC_4_L = 0x2329,
    SHT3XD_CMD_PERIODIC_10_H = 0x2737,
    SHT3XD_CMD_PERIODIC_10_M = 0x2721,
    SHT3XD_CMD_PERIODIC_10_L = 0x272A,

    SHT3XD_CMD_FETCH_DATA = 0xE000,
    SHT3XD_CMD_STOP_PERIODIC = 0x3093,

    SHT3XD_CMD_READ_ALR_LIMIT_LS = 0xE102,
    SHT3XD_CMD_READ_ALR_LIMIT_LC = 0xE109,
    SHT3XD_CMD_READ_ALR_LIMIT_HS = 0xE11F,
    SHT3XD_CMD_READ_ALR_LIMIT_HC = 0xE114,

    SHT3XD_CMD_WRITE_ALR_LIMIT_HS = 0x611D,
    SHT3XD_CMD_WRITE_ALR_LIMIT_HC = 0x6116,
    SHT3XD_CMD_WRITE_ALR_LIMIT_LC = 0x610B,
    SHT3XD_CMD_WRITE_ALR_LIMIT_LS = 0x6100,

    SHT3XD_CMD_NO_SLEEP = 0x303E,
} TMP_RH_Commands;

typedef enum
{
    SHT3XD_REPEATABILITY_HIGH,
    SHT3XD_REPEATABILITY_MEDIUM,
    SHT3XD_REPEATABILITY_LOW,
} TMP_RH_Repeatability;

typedef enum
{
    SHT3XD_MODE_CLOCK_STRETCH,
    SHT3XD_MODE_POLLING,
} TMP_RH_Mode;

typedef enum
{
    SHT3XD_FREQUENCY_HZ5,
    SHT3XD_FREQUENCY_1HZ,
    SHT3XD_FREQUENCY_2HZ,
    SHT3XD_FREQUENCY_4HZ,
    SHT3XD_FREQUENCY_10HZ
} TMP_RH_Frequency;

typedef enum
{
    SHT3XD_NO_ERROR = 0,

    SHT3XD_CRC_ERROR = -101,
    SHT3XD_TIMEOUT_ERROR = -102,

    SHT3XD_PARAM_WRONG_MODE = -501,
    SHT3XD_PARAM_WRONG_REPEATABILITY = -502,
    SHT3XD_PARAM_WRONG_FREQUENCY = -503,
    SHT3XD_PARAM_WRONG_ALERT = -504,

    // Wire I2C translated error codes
    SHT3XD_WIRE_I2C_DATA_TOO_LOG = -10,
    SHT3XD_WIRE_I2C_RECEIVED_NACK_ON_ADDRESS = -20,
    SHT3XD_WIRE_I2C_RECEIVED_NACK_ON_DATA = -30,
    SHT3XD_WIRE_I2C_UNKNOW_ERROR = -40
} TMP_RH_ErrorCode;

typedef union
{
    uint16_t rawData;
    struct
    {
        uint8_t WriteDataChecksumStatus : 1;
        uint8_t CommandStatus : 1;
        uint8_t Reserved0 : 2;
        uint8_t SystemResetDetected : 1;
        uint8_t Reserved1 : 5;
        uint8_t T_TrackingAlert : 1;
        uint8_t RH_TrackingAlert : 1;
        uint8_t Reserved2 : 1;
        uint8_t HeaterStatus : 1;
        uint8_t Reserved3 : 1;
        uint8_t AlertPending : 1;
    };
} TMP_RH_RegisterStatus;

struct TMP_RH_raw
{
    float t;
    int rh;
    TMP_RH_ErrorCode error;
};

struct TMP_RH
{
    char t_char[10];
    char rh_char[10];
    TMP_RH_ErrorCode error;
};
struct TMP_RH_Char
{
    TMP_RH_ErrorCode error;
};
// ENUMS AND STRUCTS FOR TMP_RH END

class SHT
{
    // user-accessible "public" interface
    public:
        SHT(uint8_t address=0x44, bool displayMsg=false);

        bool _debugMsg;

        uint16_t BAUD_RATE = 9600;

        //TMP_RH VARIABLES PUBLIC START
        void ClosedCube_TMP_RH(void);
        TMP_RH_ErrorCode TMP_RH_Init(uint8_t address);
        TMP_RH_ErrorCode clearAll(void);

        TMP_RH_ErrorCode softReset(void);
        TMP_RH_ErrorCode reset(void); // same as softReset

        uint32_t readSerialNumber(void);
        uint32_t testTMP_RH(void);

        TMP_RH_ErrorCode periodicStart(TMP_RH_Repeatability repeatability, TMP_RH_Frequency frequency);
        TMP_RH periodicFetchData(void);
        TMP_RH_raw periodicFetchData_raw(void);
        TMP_RH_ErrorCode periodicStop(void);

        //TMP_RH VARIABLES PUBLIC END

    // library-accessible "private" interface
    private:
        int value;

        //TMP_RH VARIABLES PRIVATE START
        uint8_t _address;
        TMP_RH_RegisterStatus _status;

        TMP_RH_ErrorCode writeCommand(TMP_RH_Commands command);
        TMP_RH_ErrorCode writeAlertData(TMP_RH_Commands command, float temperature, float humidity);

        uint8_t checkCrc(uint8_t data[], uint8_t checksum);
        uint8_t calculateCrc(uint8_t data[]);

        float calculateHumidity(uint16_t rawValue);
        float calculateTemperature(uint16_t rawValue);

        TMP_RH_raw readTemperatureAndHumidity();
        TMP_RH_ErrorCode read_TMP_RH(uint16_t* data, uint8_t numOfPair);

        TMP_RH returnError(TMP_RH_ErrorCode command);
        TMP_RH_raw returnError_raw(TMP_RH_ErrorCode command);
        //TMP_RH VARIABLES PRIVATE END
};

#endif // SHT_h
