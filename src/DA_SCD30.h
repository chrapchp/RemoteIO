/**
 *  @file    DA_SCD30.h
 *  @author  peter c
 *  @date    07/1/2018
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Communicate with SDC30 CO2/Humidity/Temperature Sensor
 *  via modbus
 *
 *https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/CO2/Sensirion_CO2_Sensors_SCD30_Preliminary-Datasheet.pdf
 *
 */

 #ifndef DA_SDC30_H
 #define DA_SDC30_H
 #include <DA_Input.h>
 #include <ModbusMaster.h>

// hardcoded in sensor
#define SCD30_BAUD 19200
#define SCD30_BITS 8
#define SC30S_TARTSTOP 1
#define SCD30_PARITY
#define SCD30_ADDRESS 0x61

// trigger continuous measurement write 0 no pressure compensation
// 700-1200 mBar oteherwise
#define SDC30_CONTINUOUS_MEASUREMENT 0x36

// send a 1 using function code 6 ad this address to disable continuous
//  measurement
#define SDC30_STOP_CONTINUOUS_MEASUREMENT 0x37
// get/set altitude compensation in meters
#define SDC30_ALTITUDE_COMPENSATION 0x38

// get/set measurement interval in seconds
#define SDC30_MEASURE_INTERVAL 0x25

// (de)-activate self-calibration 1=activate
#define SDC30_AUTOMATIC_SELF_CALIBRATION 0x40
// =1 via function code 4 at this address when data is ready to read
#define SDC30_DATA_READY_STATUS 0x27

// read via function code 4 the CO2, humidity, and temperature
#define SDC30_READ_MEASUREMENT 0x28
#define SDC30_READ_SZ 18 // bytes to read 0x28 to 0x33


#define SDC30_NO_RESPONSE -127


class DA_SDC30 : public DA_Input {
public:

  DA_SDC30(Stream& s);
  void init();
  inline float getCachedCO2() __attribute__((always_inline))
  {
    return curCO2;
  }

  inline float getCachedHumidity() __attribute__((always_inline))
  {
    return curHumidity;
  }

  inline float getCachedRemperature() __attribute__((always_inline))
  {
    return curTemperature;
  }

  // void receiveRaw( char* aResult );
  void serialize(Stream *aOutputStream,
                 bool    includeCR);

protected:

  void onRefresh();
  void refreshAll();

private:
  ModbusMaster node;
  float curCO2         = SDC30_NO_RESPONSE;
  float curHumidity    = SDC30_NO_RESPONSE;
  float curTemperature = SDC30_NO_RESPONSE;

  Stream& serialPort;
};

  #endif // DA_SDC30_H
