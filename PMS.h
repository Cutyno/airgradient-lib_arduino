/*
  PMS.h - Library for AirGradient particular matter sensor kit - description
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// ensure this library description is only included once
#ifndef PMS_h
#define PMS_h

#include <SoftwareSerial.h>
#include <Print.h>
#include "Stream.h"

class PMS
{
public:
    PMS(bool displayMsg = false, int rx_pin = D5, int tx_pin = D6, uint16 baudRate = 9600);

    bool _debugMsg;

    // PMS VARIABLES PUBLIC_START
#define PMS_SINGLE_RESPONSE_TIME 1000
#define PMS_TOTAL_RESPONSE_TIME 10000    // = 1000 * 10
#define PMS_STEADY_RESPONSE_TIME = 30000 // = 1000 * 30

    uint16_t BAUD_RATE = 9600;

    struct DATA
    {
        // Standard Particles, CF=1
        uint16_t PM_SP_UG_1_0;
        uint16_t PM_SP_UG_2_5;
        uint16_t PM_SP_UG_10_0;

        // Atmospheric environment
        uint16_t PM_AE_UG_1_0;
        uint16_t PM_AE_UG_2_5;
        uint16_t PM_AE_UG_10_0;
    };

    void sleep();
    void wakeUp();
    void activeMode();
    void passiveMode();

    void requestRead();
    bool read_PMS(DATA &data);
    bool readUntil(DATA &data, uint16_t timeout = PMS_SINGLE_RESPONSE_TIME);
    const char *getPM1();
    const char *getPM2();
    const char *getPM10();
    int getPM1_Raw();
    int getPM2_Raw();
    int getPM10_Raw();

    // PMS VARIABLES PUBLIC_END

    // library-accessible "private" interface
private:
    // PMS VARIABLES PRIVATE START
    enum STATUS
    {
        STATUS_WAITING,
        STATUS_OK
    };
    enum MODE
    {
        MODE_ACTIVE,
        MODE_PASSIVE
    };

    uint8_t _payload[12];
    Stream *_stream;
    DATA *_data;
    STATUS _PMSstatus;
    MODE _mode = MODE_ACTIVE;

    uint8_t _index = 0;
    uint16_t _frameLen;
    uint16_t _checksum;
    uint16_t _calculatedChecksum;
    SoftwareSerial *_SoftSerial_PMS;
    void loop();
    char Char_PM[10];
    // PMS VARIABLES PRIVATE END
};

#endif // PMS_h