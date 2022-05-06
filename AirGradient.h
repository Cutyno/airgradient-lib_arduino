/*
  AirGradient.h - Library for AirGradient sensor kit - description
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// ensure this library description is only included once
#ifndef AirGradient_h
#define AirGradient_h

#include <SoftwareSerial.h>
#include <Print.h>
#include "Stream.h"

#include "PMS.h"
#include "SHT.h"

//MHZ19 CONSTANTS START
// types of sensors.
extern const int MHZ14A;
extern const int MHZ19B;

// status codes
extern const int STATUS_NO_RESPONSE;
extern const int STATUS_CHECKSUM_MISMATCH;
extern const int STATUS_INCOMPLETE;
extern const int STATUS_NOT_READY;
//MHZ19 CONSTANTS END

//ENUMS STRUCTS FOR CO2 START
    struct CO2_READ_RESULT {
    int co2 = -1;
    bool success = false;
};
//ENUMS STRUCTS FOR CO2 END

// library interface description
class AirGradient
{
  // user-accessible "public" interface
  public:
    AirGradient(bool displayMsg=false,int baudRate=9600);

    static void setOutput(Print& debugOut, bool verbose = true);

    void PMS_Init(bool displayMsg=false, int rx_pin=D5, int tx_pin=D6, uint16 baudRate=9600);
    void SHT_Init(uint8_t adress, bool displayMsg=false);
    void beginCO2(void);
    void beginCO2(int,int);

    bool _debugMsg;

    PMS pms;
    SHT sht;

    static const uint16_t BAUD_RATE = 9600;


    //CO2 VARIABLES PUBLIC START
    void CO2_Init();
    void CO2_Init(int,int);
    void CO2_Init(int,int,int);
    const char* getCO2(int retryLimit = 5);
    int getCO2_Raw();
    SoftwareSerial *_SoftSerial_CO2;

    //CO2 VARIABLES PUBLIC END

    //MHZ19 VARIABLES PUBLIC START
    void MHZ19_Init(uint8_t);
    void MHZ19_Init(int,int,uint8_t);
    void MHZ19_Init(int,int,int,uint8_t);
    void setDebug_MHZ19(bool enable);
    bool isPreHeating_MHZ19();
    bool isReady_MHZ19();

    int readMHZ19();

    //MHZ19 VARIABLES PUBLIC END



  // library-accessible "private" interface
  private:
    int value;

    //CO2 VARABLES PUBLIC START
    char Char_CO2[10];

    //CO2 VARABLES PUBLIC END
    //MHZ19 VARABLES PUBLIC START

    int readInternal_MHZ19();

    uint8_t _type_MHZ19, temperature_MHZ19;
    bool debug_MHZ19 = false;

    Stream * _serial_MHZ19;
    SoftwareSerial *_SoftSerial_MHZ19;
    uint8_t getCheckSum_MHZ19(unsigned char *packet);

    //MHZ19 VARABLES PUBLIC END
};


#endif

