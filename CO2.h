/*
  CO2.h - Library for AirGradient sensor kit - description
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// ensure this library description is only included once
#ifndef CO2_h
#define CO2_h

#include <SoftwareSerial.h>
#include <Print.h>
#include "Stream.h"

//ENUMS STRUCTS FOR CO2 START
    struct CO2_READ_RESULT {
    int co2 = -1;
    bool success = false;
};
//ENUMS STRUCTS FOR CO2 END

class CO2
{
    // user-accessible "public" interface
    public:
        CO2(int rx_pin=D4,int tx_pin=D3,int baudRate=9600, bool displayMsg=false);

        bool _debugMsg;

        //CO2 VARIABLES PUBLIC START
        const char* getCO2(int retryLimit = 5);
        int getCO2_Raw();
        SoftwareSerial *_SoftSerial_CO2;
        //CO2 VARIABLES PUBLIC END
    
    // library-accessible "private" interface
    private:
        //CO2 VARABLES PUBLIC START
        char Char_CO2[10];
        //CO2 VARABLES PUBLIC END
};

#endif // CO2_h