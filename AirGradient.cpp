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

PMS AirGradient::PMS_Init(bool displayMsg, int rx_pin, int tx_pin, uint16 baudRate)
{
  pms = PMS(displayMsg, rx_pin, tx_pin, baudRate);
}

//START TMP_RH FUNCTIONS//

TMP_RH_ErrorCode AirGradient::TMP_RH_Init(uint8_t address) {
  if (_debugMsg) {
    Serial.println("Initializing TMP_RH...");
    }
  TMP_RH_ErrorCode error = SHT3XD_NO_ERROR;
  _address = address;
  periodicStart(SHT3XD_REPEATABILITY_HIGH, SHT3XD_FREQUENCY_10HZ);
  return error;
}

TMP_RH_ErrorCode AirGradient::reset()
{
  return  softReset();
}

TMP_RH AirGradient::periodicFetchData() //
{
  TMP_RH result;
  TMP_RH_ErrorCode error = writeCommand(SHT3XD_CMD_FETCH_DATA);
  if (error == SHT3XD_NO_ERROR){
    result = readTemperatureAndHumidity();
    sprintf(result.t_char,"%d", result.t);
    sprintf(result.rh_char,"%f", result.rh);

    return result;
  }
  else
    return returnError(error);
}

TMP_RH_ErrorCode AirGradient::periodicStop() {
  return writeCommand(SHT3XD_CMD_STOP_PERIODIC);
}

TMP_RH_ErrorCode AirGradient::periodicStart(TMP_RH_Repeatability repeatability, TMP_RH_Frequency frequency) //
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


TMP_RH_ErrorCode AirGradient::writeCommand(TMP_RH_Commands command)
{
  Wire.beginTransmission(_address);
  Wire.write(command >> 8);
  Wire.write(command & 0xFF);
  return (TMP_RH_ErrorCode)(-10 * Wire.endTransmission());
}

TMP_RH_ErrorCode AirGradient::softReset() {
  return writeCommand(SHT3XD_CMD_SOFT_RESET);
}


uint32_t AirGradient::readSerialNumber()
{
  uint32_t result = SHT3XD_NO_ERROR;
  uint16_t buf[2];

  if (writeCommand(SHT3XD_CMD_READ_SERIAL_NUMBER) == SHT3XD_NO_ERROR) {
    if (read_TMP_RH(buf, 2) == SHT3XD_NO_ERROR) {
      result = (buf[0] << 16) | buf[1];
    }
  }
  else if(writeCommand(SHT3XD_CMD_READ_SERIAL_NUMBER) != SHT3XD_NO_ERROR){
    if (_debugMsg) {
    Serial.println("TMP_RH Failed to Initialize.");
    }

  }

  return result;
}
uint32_t AirGradient::testTMP_RH()
{
  uint32_t result = SHT3XD_NO_ERROR;
  uint16_t buf[2];

  if (writeCommand(SHT3XD_CMD_READ_SERIAL_NUMBER) == SHT3XD_NO_ERROR) {
    if (read_TMP_RH(buf, 2) == SHT3XD_NO_ERROR) {
      result = (buf[0] << 16) | buf[1];
    }
    if (_debugMsg) {
    Serial.print("TMP_RH successfully initialized with serial number: ");
    Serial.println(result);
    }

  }
  else if(writeCommand(SHT3XD_CMD_READ_SERIAL_NUMBER) != SHT3XD_NO_ERROR){
    if (_debugMsg) {
    Serial.println("TMP_RH Failed to Initialize.");
    }

  }

  return result;
}

TMP_RH_ErrorCode AirGradient::clearAll() {
  return writeCommand(SHT3XD_CMD_CLEAR_STATUS);
}


TMP_RH AirGradient::readTemperatureAndHumidity()//
{
  TMP_RH result;

  result.t = 0;
  result.rh = 0;

  TMP_RH_ErrorCode error;
  uint16_t buf[2];

  if (error == SHT3XD_NO_ERROR)
    error = read_TMP_RH(buf, 2);

  if (error == SHT3XD_NO_ERROR) {
    result.t = calculateTemperature(buf[0]);
    result.rh = calculateHumidity(buf[1]);
  }
  result.error = error;

  return result;
}

TMP_RH_ErrorCode AirGradient::read_TMP_RH(uint16_t* data, uint8_t numOfPair)//
{
  uint8_t buf[2];
  uint8_t checksum;

  const uint8_t numOfBytes = numOfPair * 3;
  Wire.requestFrom(_address, numOfBytes);

  int counter = 0;

  for (counter = 0; counter < numOfPair; counter++) {
    Wire.readBytes(buf, (uint8_t)2);
    checksum = Wire.read();

    if (checkCrc(buf, checksum) != 0)
      return SHT3XD_CRC_ERROR;

    data[counter] = (buf[0] << 8) | buf[1];
  }

  return SHT3XD_NO_ERROR;
}


uint8_t AirGradient::checkCrc(uint8_t data[], uint8_t checksum)//
{
  return calculateCrc(data) != checksum;
}

float AirGradient::calculateTemperature(uint16_t rawValue)//
{
  float value = 175.0f * (float)rawValue / 65535.0f - 45.0f;
  return round(value*10)/10;
}


float AirGradient::calculateHumidity(uint16_t rawValue)//
{
  return 100.0f * rawValue / 65535.0f;
}

uint8_t AirGradient::calculateCrc(uint8_t data[])
{
  uint8_t bit;
  uint8_t crc = 0xFF;
  uint8_t dataCounter = 0;

  for (; dataCounter < 2; dataCounter++)
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

TMP_RH AirGradient::returnError(TMP_RH_ErrorCode error) {
  TMP_RH result;
  result.t = NULL;
  result.rh = NULL;

  result.t_char[0] = 'N';
  result.t_char[1] = 'U';
  result.t_char[2] = 'L';
  result.t_char[3] = 'L';

  result.rh_char[0] = 'N';
  result.rh_char[1] = 'U';
  result.rh_char[2] = 'L';
  result.rh_char[3] = 'L';

  result.error = error;
  return result;
}

//END TMP_RH FUNCTIONS //

//START CO2 FUNCTIONS //
void AirGradient::CO2_Init(){
  CO2_Init(D4,D3);
}
void AirGradient::CO2_Init(int rx_pin,int tx_pin){
  CO2_Init(rx_pin,tx_pin,9600);
  
}
void AirGradient::CO2_Init(int rx_pin,int tx_pin,int baudRate){
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
  else{
    Serial.println("CO2 Successfully Initialized. Heating up for 10s");
    delay(10000);
  }
}
const char* AirGradient::getCO2(int retryLimit) {
  int ctr = 0;
  int result_CO2 = getCO2_Raw();
  while(result_CO2 == -1){
    result_CO2 = getCO2_Raw();
    if((ctr == retryLimit) || (result_CO2 == -1)){
      Char_CO2[0] = 'N';
      Char_CO2[1] = 'U';
      Char_CO2[2] = 'L';
      Char_CO2[3] = 'L';
      return Char_CO2;
    }
    ctr++;
  }
  sprintf(Char_CO2,"%d", result_CO2);
  return Char_CO2;
}
int AirGradient::getCO2_Raw(){
  const byte CO2Command[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};
  byte CO2Response[] = {0,0,0,0,0,0,0};

  _SoftSerial_CO2->write(CO2Command, 7);
  delay(100);  //give the sensor a bit of time to respond

  if (_SoftSerial_CO2->available()){
    for (int i=0; i < 7; i++) {
      int byte = _SoftSerial_CO2->read();
      CO2Response[i] = byte;
      if (CO2Response[0] != 254) {
        return -1;  //error code for debugging
      }
    }
    unsigned long val = CO2Response[3]*256 + CO2Response[4];
    return val;
  }
  else
  {
  return -2; //error code for debugging
  }
}

//END CO2 FUNCTIONS //

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
  }//
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
