/**
 *  @file    DA_SCD30.h
 *  @author  peter c
 *  @date    07/1/2018
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Communicate with SCD30 CO2/Humidity/Temperature Sensor
 *  via modbus
 *
 *https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/CO2/Sensirion_CO2_Sensors_SCD30_Preliminary-Datasheet.pdf
 *
 */

#ifndef DA_SCD30_H
#define DA_SCD30_H
#include <DA_Input.h>
#include <ModbusMaster.h>

// hardcoded in sensor
#define SCD30_BAUD 19200
#define SCD30_BITS 8
#define SC30S_TARTSTOP 1
#define SCD30_PARITY
#define SCD30_ADDRESS 0x61
#define SCD30_POLLING_INTERVAL 5000 // 5 s
// trigger continuous measurement write 0 no pressure compensation
// 700-1200 mBar oteherwise
#define SCD30_CONTINUOUS_MEASUREMENT 0x36

// send a 1 using function code 6 ad this address to disable continuous
//  measurement
#define SCD30_STOP_CONTINUOUS_MEASUREMENT 0x37
// get/set altitude compensation in meters
#define SCD30_ALTITUDE_COMPENSATION 0x38

// get/set measurement interval in seconds
#define SCD30_MEASURE_INTERVAL 0x25

// (de)-activate self-calibration 1=activate
#define SCD30_AUTOMATIC_SELF_CALIBRATION 0x40
// =1 via function code 4 at this address when data is ready to read
#define SCD30_DATA_READY_STATUS 0x27

// read via function code 4 the CO2, humidity, and temperature
#define SCD30_READ_MEASUREMENT 0x28
#define SCD30_READ_SZ 6 // 3 floats

#define SCD30_NO_RESPONSE -126
#define SCD30_DISABLED -127

class DA_SCD30 : public DA_Input {
public:
  DA_SCD30(Stream &s);
  void init();
  inline float getCachedCO2() __attribute__((always_inline)) { return curCO2; }

  inline float getCachedHumidity() __attribute__((always_inline)) {
    return curHumidity;
  }

  inline float getCachedTemperature() __attribute__((always_inline)) {
    return curTemperature;
  }

  void startContiousMeasurement();
  void stopContiousMeasurement();
  // void receiveRaw( char* aResult );
  void serialize(Stream *aOutputStream, bool includeCR);

protected:
  void onRefresh();
  void refreshAll();
  float getSensorReading(uint8_t aIndex);

private:
  ModbusMaster node;
  float curCO2 = 0;
  float curTemperature = 0;
  float curHumidity = 0;

  Stream &serialPort;
};

#endif // DA_SCD30_H
