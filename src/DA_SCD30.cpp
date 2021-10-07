/**
 *  @file    DA_SCD30.cpp
 *  @author  peter c
 *  @date    7/1/2018
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *
 **/

#include "DA_SCD30.h"
#include <Streaming.h>

DA_SCD30::DA_SCD30(Stream &s) : DA_Input(IO_TYPE::SDC30), serialPort(s) {
  setPollingInterval(SCD30_POLLING_INTERVAL);
}

void  DA_SCD30::startContiousMeasurement() {
  node.writeSingleRegister(SCD30_CONTINUOUS_MEASUREMENT, 1);
}

void DA_SCD30::stopContiousMeasurement() {
    node.writeSingleRegister(SCD30_STOP_CONTINUOUS_MEASUREMENT, 1);
}
void DA_SCD30::init() {
  node.begin(SCD30_ADDRESS, serialPort);
//stopContiousMeasurement();
startContiousMeasurement();
 }

void DA_SCD30::serialize(Stream *aOutputStream, bool includeCR) {

  *aOutputStream << "{SDC30-CO2:" << curCO2 << " Temperature:" << curTemperature
                 << " Humidity:" << curHumidity << " }";

  if (includeCR)
    *aOutputStream << endl;
}

float DA_SCD30::getSensorReading(uint8_t aIndex) {
  union {
    uint16_t regsf[2];
    float val;
  } bfconvert;

  bfconvert.regsf[0] = node.getResponseBuffer(aIndex + 1); // data[j + 1];
  bfconvert.regsf[1] = node.getResponseBuffer(aIndex);     // data[j];
  return (bfconvert.val);
}

// Read CO2, Temperature, Humidity
void DA_SCD30::onRefresh() {

  uint8_t result;

  result = node.readInputRegisters(SCD30_READ_MEASUREMENT, SCD30_READ_SZ);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess) {

    curCO2 = getSensorReading(0);
    curTemperature = getSensorReading(2);
    curHumidity = getSensorReading(4);

  } else
  {
    curCO2 = SCD30_NO_RESPONSE;
    curTemperature = SCD30_NO_RESPONSE;
    curHumidity = SCD30_NO_RESPONSE;
  }
}

void DA_SCD30::refreshAll() {}
