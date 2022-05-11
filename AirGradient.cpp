/*
  AirGradient.cpp - Library for AirGradient sensor kit - implementation
  Copyright (c) 2006 John Doe.  All right reserved.
*/


// include this library's description file
#include "AirGradient.h"

// include description files for other libraries used (if any)
#include <SoftwareSerial.h>
#include "Arduino.h"
#include <Wire.h>
#include <math.h>


// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

const int MHZ14A = 14;
const int MHZ19B = 19; // this one we use for AQI whatever

const int MHZ14A_PREHEATING_TIME = 3 * 60 * 1000;
const int MHZ19B_PREHEATING_TIME = 3 * 60 * 1000;

const int MHZ14A_RESPONSE_TIME = 60 * 1000;
const int MHZ19B_RESPONSE_TIME = 120 * 1000;  

const int STATUS_NO_RESPONSE = -2;
const int STATUS_CHECKSUM_MISMATCH = -3;
const int STATUS_INCOMPLETE = -4;
const int STATUS_NOT_READY = -5;
const int STATUS_PWM_NOT_CONFIGURED = -6;
const int STATUS_serial_MHZ19_NOT_CONFIGURED = -7;

unsigned long lastRequest = 0;

bool SerialConfigured = true;
bool PwmConfigured = true;


AirGradient::AirGradient(bool displayMsg,int baudRate)
{
  _debugMsg = displayMsg;
  Wire.begin();
  Serial.begin(baudRate);
  if (_debugMsg) {
    Serial.println("AirGradiant Library instantiated successfully.");
  }
}

void AirGradient::PMS_Init(bool displayMsg, int rx_pin, int tx_pin, uint16 baudRate)
{
  pms = PMS(displayMsg, rx_pin, tx_pin, baudRate);
}

void AirGradient::SHT_Init(uint8_t adress, bool displayMsg)
{
  sht = SHT(adress, displayMsg);
}

void AirGradient::CO2_Init(int rx_pin, int tx_pin, int baudRate, bool displayMsg)
{
  SoftwareSerial S8_serial = SoftwareSerial(rx_pin, tx_pin);
  S8_serial.begin(baudRate);
  co2 = new S8_UART(S8_serial);

  if(displayMsg)
  {
    // Check if S8 is available
    co2->get_firmware_version(CO2sensor.firm_version);
    int len = strlen(CO2sensor.firm_version);
    if (len == 0) {
        Serial.println("SenseAir S8 CO2 sensor not found!");
    }
    // Show basic S8 sensor info
    Serial.println(">>> SenseAir S8 NDIR CO2 sensor <<<");
    Serial.printf("Firmware version: %s\n", CO2sensor.firm_version);
    CO2sensor.sensor_id = co2->get_sensor_ID();
    Serial.print("Sensor ID: 0x"); 
    printIntToHex(CO2sensor.sensor_id, 4); 
    Serial.println("");
    Serial.flush();
  }
}

//START MHZ19 FUNCTIONS //
void AirGradient::MHZ19_Init(uint8_t type) {
  MHZ19_Init(9,10,type);
}
void AirGradient::MHZ19_Init(int rx_pin,int tx_pin, uint8_t type) {
  MHZ19_Init(rx_pin,tx_pin,9600,type);
}
void AirGradient::MHZ19_Init(int rx_pin,int tx_pin, int baudRate, uint8_t type) {
  if (_debugMsg) {
      Serial.println("Initializing MHZ19...");
      }
    _SoftSerial_MHZ19 = new SoftwareSerial(rx_pin,tx_pin);
    _SoftSerial_MHZ19->begin(baudRate);

    if(readMHZ19() == -1){
      if (_debugMsg) {
      Serial.println("MHZ19 Sensor Failed to Initialize ");
      }
    }
    else{
      Serial.println("MHZ19 Successfully Initialized. Heating up for 10s");
      delay(10000);
    }

  _type_MHZ19 = type;

  PwmConfigured = false;
}

/**
 * Enables or disables the debug mode (more logging).
 */
void AirGradient::setDebug_MHZ19(bool enable) {
  debug_MHZ19 = enable;
  if (debug_MHZ19) {
    Serial.println(F("MHZ: debug mode ENABLED"));
  } else {
    Serial.println(F("MHZ: debug mode DISABLED"));
  }
}

bool AirGradient::isPreHeating_MHZ19() {
  if (_type_MHZ19 == MHZ14A) {
    return millis() < (MHZ14A_PREHEATING_TIME);
  } else if (_type_MHZ19 == MHZ19B) {
    return millis() < (MHZ19B_PREHEATING_TIME);
  } else {
    Serial.println(F("MHZ::isPreheating_MHZ19() => UNKNOWN SENSOR"));
    return false;
  }
}

bool AirGradient::isReady_MHZ19() {
  if (isPreHeating_MHZ19()) return false;
  if (_type_MHZ19 == MHZ14A)
    return lastRequest < millis() - MHZ14A_RESPONSE_TIME;
  else if (_type_MHZ19 == MHZ19B)
    return lastRequest < millis() - MHZ19B_RESPONSE_TIME;
  else {
    Serial.print(F("MHZ::isReady_MHZ19() => UNKNOWN SENSOR \""));
    Serial.print(_type_MHZ19);
    Serial.println(F("\""));
    return true;
  }
}


int AirGradient::readMHZ19() { 

  int firstRead = readInternal_MHZ19();
  int secondRead = readInternal_MHZ19();

  if (abs(secondRead - firstRead) > 50) {
      // we arrive here sometimes when the CO2 sensor is not connected
      // could possibly also be fixed with a pull-up resistor on Rx but if we forget this then ...
      Serial.println("MHZ::read() inconsistent values");
      return -1;
  }

  Serial.println("MHZ::read(1) " + String(firstRead));
  Serial.println("MHZ::read(2) " + String(secondRead));

  // TODO: return average?
  return secondRead;
}

int AirGradient::readInternal_MHZ19() {
  if (!SerialConfigured) {
    if (debug_MHZ19) Serial.println(F("-- serial is not configured"));
    return STATUS_serial_MHZ19_NOT_CONFIGURED;
  }
  // if (!isReady_MHZ19()) return STATUS_NOT_READY;
  if (debug_MHZ19) Serial.println(F("-- read CO2 uart ---"));
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  unsigned char response[9];  // for answer

  if (debug_MHZ19) Serial.print(F("  >> Sending CO2 request"));
  _SoftSerial_MHZ19->write(cmd, 9);  // request PPM CO2
  lastRequest = millis();

  // clear the buffer
  memset(response, 0, 9);

  int waited = 0;
  while (_SoftSerial_MHZ19->available() == 0) {
    if (debug_MHZ19) Serial.print(".");
    delay(100);  // wait a short moment to avoid false reading
    if (waited++ > 10) {
      if (debug_MHZ19) Serial.println(F("No response after 10 seconds"));
      _SoftSerial_MHZ19->flush();
      return STATUS_NO_RESPONSE;
    }
  }
  if (debug_MHZ19) Serial.println();

  // The serial stream can get out of sync. The response starts with 0xff, try
  // to resync.
  // TODO: I think this might be wrong any only happens during initialization?
  boolean skip = false;
  while (_SoftSerial_MHZ19->available() > 0 && (unsigned char)_SoftSerial_MHZ19->peek() != 0xFF) {
    if (!skip) {
      Serial.print(F("MHZ: - skipping unexpected readings:"));
      skip = true;
    }
    Serial.print(" ");
    Serial.print(_SoftSerial_MHZ19->peek(), HEX);
    _SoftSerial_MHZ19->read();
  }
  if (skip) Serial.println();

  if (_SoftSerial_MHZ19->available() > 0) {
    int count = _SoftSerial_MHZ19->readBytes(response, 9);
    if (count < 9) {
      _SoftSerial_MHZ19->flush();
      return STATUS_INCOMPLETE;
    }
  } else {
    _SoftSerial_MHZ19->flush();
    return STATUS_INCOMPLETE;
  }

  if (debug_MHZ19) {
    // print out the response in hexa
    Serial.print(F("  << "));
    for (int i = 0; i < 9; i++) {
      Serial.print(response[i], HEX);
      Serial.print(F("  "));
    }
    Serial.println(F(""));
  }

  // checksum
  byte check = getCheckSum_MHZ19(response);
  if (response[8] != check) {
    Serial.println(F("MHZ: Checksum not OK!"));
    Serial.print(F("MHZ: Received: "));
    Serial.println(response[8], HEX);
    Serial.print(F("MHZ: Should be: "));
    Serial.println(check, HEX);
    temperature_MHZ19 = STATUS_CHECKSUM_MISMATCH;
    _SoftSerial_MHZ19->flush();
    return STATUS_CHECKSUM_MISMATCH;
  }

  int ppm_uart = 256 * (unsigned int)response[2] + (unsigned int)response[3];

  temperature_MHZ19 = response[4] - 44;  // - 40;

  byte status = response[5];
  if (debug_MHZ19) {
    Serial.print(F(" # PPM UART: "));
    Serial.println(ppm_uart);
    Serial.print(F(" # temperature_MHZ19? "));
    Serial.println(temperature_MHZ19);
  }

  // Is always 0 for version 14a  and 19b
  // Version 19a?: status != 0x40
  if (debug_MHZ19 && status != 0) {
    Serial.print(F(" ! Status maybe not OK ! "));
    Serial.println(status, HEX);
  } else if (debug_MHZ19) {
    Serial.print(F(" Status  OK: "));
    Serial.println(status, HEX);
  }

  _SoftSerial_MHZ19->flush();
  return ppm_uart;
}

uint8_t AirGradient::getCheckSum_MHZ19(unsigned char* packet) {
  if (!SerialConfigured) {
    if (debug_MHZ19) Serial.println(F("-- serial is not configured"));
    return STATUS_serial_MHZ19_NOT_CONFIGURED;
  }
  if (debug_MHZ19) Serial.println(F("  getCheckSum_MHZ19()"));
  byte i;
  unsigned char checksum = 0;
  for (i = 1; i < 8; i++) {
    checksum += packet[i];
  }
  checksum = 0xff - checksum;
  checksum += 1;
  return checksum;
}

//END MHZ19 FUNCTIONS //
