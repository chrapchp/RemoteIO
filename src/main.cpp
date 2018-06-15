/**
 *  @file    main.c
 *  @author  peter c
 *  @date    2018Mar23
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  A remote I/O module to run under a Arduino Mega, Controlino Maxi Automation
 *  Communicates over Modbus TCP/IP
 *  7 x 1-wire temperatures
 *  8 x Analog Inputs
 *  2 x Analog Outputs
 *  Discrite Inputs/Outputs
 */

#include <avr/wdt.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <MgsModbus.h> // cchange memory size here


#include <Streaming.h>


#include <OneWire.h>
#include <DallasTemperature.h>

#include <DA_Flowmeter.h>
#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_AnalogOutput.h>
#include <DA_OneWireDallasMgr.h>
#include <DA_NonBlockingDelay.h>
#include <DA_AtlasMgr.h>

#include "DA_TCPCommandHandler.h"
#include "remoteIO.h"
#include "Controllino.h"


char atlasrxBuff[DA_ATLAS_RX_BUF_SZ];


// Atlas serial 8:1 control pins
#if defined(NC_BUILD)
DA_AtlasMgr atlasSensorMgr = DA_AtlasMgr(Serial2,
                                         CONTROLLINO_PIN_HEADER_DIGITAL_OUT_12,
                                         CONTROLLINO_PIN_HEADER_DIGITAL_OUT_13,
                                         CONTROLLINO_PIN_HEADER_DIGITAL_OUT_14);
#endif // if defined(NC_BUILD)
DA_TCPCommandHandler remoteCommandHandler = DA_TCPCommandHandler();


MgsModbus MBSlave;


// Ethernet settings (depending on MAC and Local network)
// byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0x94, 0xB5 };
//

// Organizationally Unique Identifier made up
////  0xDE 0xAD 0xBE 0x?? 0x?? 0x??
// https://macvendorlookup.com/search
// https://www.miniwebtool.com/mac-address-generator/


IPAddress currentIP(INADDR_NONE);
IPAddress currentGateway(INADDR_NONE);
IPAddress currentSubnet(INADDR_NONE);
byte currentMAC[] = { DEFAULT_PENDING_MAC_ADDRESS };

IPAddress pendingIP(INADDR_NONE);
IPAddress pendingGateway(INADDR_NONE);
IPAddress pendingSubnet(INADDR_NONE);

IPAddress defaultIP(DEFAULT_IP_ADDRESS);
IPAddress defaultGateway(DEFAULT_GATEWAY);
IPAddress defaultSubnet(DEFAULT_SUBNET_MASK);
byte defaultMAC[] = { DEFAULT_MAC_ADDRESS };

byte pendingMAC[] = { DEFAULT_PENDING_MAC_ADDRESS };


// #define IO_DEBUG 2

// forward Declarations
void onXT_006_PulseIn();
void onXT_007_PulseIn();
void processHostWrites();
void refreshHostReads();
void refreshModbusRegisters();
void refreshAnalogs();
void refreshDiscreteInputs();

void onRestoreDefaults(bool aValue,
                       int  aPin);
void onHeartBeat();
void refreshTemperatureUUID(uint16_t aModbusAddressLow,
                            uint16_t aModbusAddressHigh,
                            uint64_t aUUID);
void onFlowCalc();
void doIPMACChange();
void doCheckIPMACChange();
bool detectTransition(bool aDirection,
                      bool aState,
                      bool aPreviousState);
void doCheckRestoreDefaults();
void doCheckForRescanOneWire();
void EEPROMWriteCurrentIPs();
void EEPROMLoadConfig();
void EEPROMWriteDefaultConfig();
void doCheckMACChange();
void doCheckRebootDevice();
void rebootDevice();

// remote command handlers
void remoteAtlasCommandHandler(uint8_t argc,
                               char  **argv,
                               Stream *aOutputStream);
void remoteDeviceCommandHandler(uint8_t argc,
                                char  **argv,
                                Stream *aOutputStream);

void remoteHelpCommandHandler(uint8_t argc,
                              char  **argv,
                              Stream *aOutputStream);

void remoteOneWireCommandHandler(uint8_t argc,
                                 char  **argv,
                                 Stream *aOutputStream);

void printByteArray(uint8_t anArray[],
                    uint8_t aSize,
                    Stream *aOutputStream);


// IP address from class needs bytes flipped for modbus TBOX
// did not want to modify class
uint32_t byteSwap32(uint32_t aValue);
uint16_t byteSwap16(uint16_t aValue);


DA_FlowMeter XT_006(XT006_SENSOR_INTERUPT_PIN, FLOW_CALC_PERIOD_SECONDS);
DA_FlowMeter XT_007(XT007_SENSOR_INTERUPT_PIN, FLOW_CALC_PERIOD_SECONDS);

// Discrete Inputs
///


DA_DiscreteInput DI_000 = DA_DiscreteInput(CONTROLLINO_DI0,
                                           DA_DiscreteInput::None,
                                           false);
DA_DiscreteInput DI_001 = DA_DiscreteInput(CONTROLLINO_DI1,
                                           DA_DiscreteInput::None,
                                           false);
DA_DiscreteInput DI_002 = DA_DiscreteInput(CONTROLLINO_DI2,
                                           DA_DiscreteInput::None,
                                           false);
DA_DiscreteInput DI_003 = DA_DiscreteInput(CONTROLLINO_DI3,
                                           DA_DiscreteInput::None,
                                           false);
DA_DiscreteInput DI_004 = DA_DiscreteInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_08,
  DA_DiscreteInput::None,
  false);
DA_DiscreteInput DI_005 = DA_DiscreteInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_09,
  DA_DiscreteInput::None,
  false);
DA_DiscreteInput DI_006 = DA_DiscreteInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_10,
  DA_DiscreteInput::None,
  false);
DA_DiscreteInput DI_007 = DA_DiscreteInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_11,
  DA_DiscreteInput::None,
  false);
DA_DiscreteInput DI_008 = DA_DiscreteInput(CONTROLLINO_A12,
                                           DA_DiscreteInput::None,
                                           false);
DA_DiscreteInput DI_009 = DA_DiscreteInput(CONTROLLINO_A13,
                                           DA_DiscreteInput::None,
                                           false);


// reset IP to defaults
DA_DiscreteInput CI_001 = DA_DiscreteInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_07,
  DA_DiscreteInput::FallingEdgeDetect,
  false);


// Relay Outputs
//
DA_DiscreteOutput DY_000 = DA_DiscreteOutput(CONTROLLINO_RELAY_00,  HIGH);
DA_DiscreteOutput DY_001 = DA_DiscreteOutput(CONTROLLINO_RELAY_01,  HIGH);
DA_DiscreteOutput DY_002 = DA_DiscreteOutput(CONTROLLINO_RELAY_02,  HIGH);
DA_DiscreteOutput DY_003 = DA_DiscreteOutput(CONTROLLINO_RELAY_03,  HIGH);
DA_DiscreteOutput DY_004 = DA_DiscreteOutput(CONTROLLINO_RELAY_04,  HIGH);
DA_DiscreteOutput DY_005 = DA_DiscreteOutput(CONTROLLINO_RELAY_05,  HIGH);
DA_DiscreteOutput DY_006 = DA_DiscreteOutput(CONTROLLINO_RELAY_06,  HIGH);
DA_DiscreteOutput DY_007 = DA_DiscreteOutput(CONTROLLINO_RELAY_07,  HIGH);
DA_DiscreteOutput DY_008 = DA_DiscreteOutput(CONTROLLINO_RELAY_08,  HIGH);
DA_DiscreteOutput DY_009 = DA_DiscreteOutput(CONTROLLINO_RELAY_09,  HIGH);
DA_DiscreteOutput DY_010 = DA_DiscreteOutput(CONTROLLINO_DO0,  HIGH);
DA_DiscreteOutput DY_011 = DA_DiscreteOutput(CONTROLLINO_DO1,  HIGH);
DA_DiscreteOutput DY_012 = DA_DiscreteOutput(CONTROLLINO_DO2,  HIGH);
DA_DiscreteOutput DY_013 = DA_DiscreteOutput(CONTROLLINO_DO3,  HIGH);
DA_DiscreteOutput DY_014 = DA_DiscreteOutput(CONTROLLINO_DO4,  HIGH);
DA_DiscreteOutput DY_015 = DA_DiscreteOutput(CONTROLLINO_DO5,  HIGH);
DA_DiscreteOutput DY_016 = DA_DiscreteOutput(CONTROLLINO_DO6,  HIGH);
DA_DiscreteOutput DY_017 = DA_DiscreteOutput(CONTROLLINO_DO7,  HIGH);
DA_DiscreteOutput DY_018 = DA_DiscreteOutput(
  CONTROLLINO_PIN_HEADER_DIGITAL_OUT_14,
  HIGH);

#if not defined(NC_BUILD)
DA_DiscreteOutput DY_019 = DA_DiscreteOutput(
  CONTROLLINO_PIN_HEADER_DIGITAL_OUT_13,
  HIGH);
DA_DiscreteOutput DY_020 = DA_DiscreteOutput(
  CONTROLLINO_PIN_HEADER_DIGITAL_OUT_12,
  HIGH);
DA_DiscreteOutput DY_021 = DA_DiscreteOutput(
  CONTROLLINO_PIN_HEADER_DIGITAL_OUT_15,
  HIGH);
#endif // if not defined(NC_BUILD)

// 0-24V
DA_AnalogInput AI_000 =  DA_AnalogInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_00,
  0.0,
  1024.0);
DA_AnalogInput AI_001 =  DA_AnalogInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_01,
  0.0,
  1024.0);
DA_AnalogInput AI_002 =  DA_AnalogInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_02,
  0.0,
  1024.0);
DA_AnalogInput AI_003 =  DA_AnalogInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_03,
  0.0,
  1024.0);
DA_AnalogInput AI_004 =  DA_AnalogInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_04,
  0.0,
  1024.0);
DA_AnalogInput AI_005 =  DA_AnalogInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_05,
  0.0,
  1024.0);
DA_AnalogInput AI_006 =  DA_AnalogInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_06,
  0.0,
  1024.0);

// 0-10 V analog outputs
DA_AnalogOutput AY_000 = DA_AnalogOutput(CONTROLLINO_AO0);
DA_AnalogOutput AY_001 = DA_AnalogOutput(CONTROLLINO_AO1);

// App Version
uint16_t KI_003 = APP_MAJOR << 8 | APP_MINOR << 4 | APP_PATCH;
uint16_t KI_005 = DEVICE_TYPE;


// timer for heart beat and potentially trigger for other operations
DA_NonBlockingDelay KI_001 = DA_NonBlockingDelay(HEART_BEAT_PERIOD, onHeartBeat);
uint16_t KI_001_CV         = 0;

// timer for flow calcs
DA_NonBlockingDelay KI_004 = DA_NonBlockingDelay(FLOW_CALC_PERIOD_SECONDS * 1000,
                                                 onFlowCalc);


DA_OneWireDallasMgr temperatureMgr = DA_OneWireDallasMgr(WIRE_BUS_PIN);

// Debug Serial port
Stream *aOutputStream = &Serial;

// commands from host that need to be onshots
bool CY_006 = false; // update IP
bool CY_001 = false; // restore defaults
bool CY_002 = false; // rescan one wire temperatures devices
bool CY_004 = false; // reboot remote I/O

    #ifdef IO_DEBUG
void onTemperatureRead()
{
  *aOutputStream << "New Sample" << endl;

  for (int i = 0; i < DA_MAX_ONE_WIRE_SENSORS; i++)
  {
    // temperatureMgr.enableSensor(i);
    *aOutputStream << "idx:" << i << "temp:" <<
      temperatureMgr.getTemperature(i) <<
      endl;
  }
}

#endif // ifdef IO_DEBUG


void setup()
{
  MCUSR = 0; // clear existing watchdog timer presets

  temperatureMgr.setPollingInterval(DEFAULT_1WIRE_POLLING_INTERVAL);
  temperatureMgr.setBlockingRead(false);
  temperatureMgr.scanSensors();
    #ifdef IO_DEBUG
  Serial.begin(9600);

  //  aOutputStream->begin(9600);

  temperatureMgr.serialize(aOutputStream, true);
  temperatureMgr.setOnPollCallBack(onTemperatureRead);
      #endif // ifdef PROCESS_TERMINAL

  temperatureMgr.init();
#if  defined(NC2_BUILD)
  temperatureMgr.disableMgr();
#endif // if not defined(NC2_BUILD)
  Serial2.begin(9600);
  #if defined(NC_BUILD)
  atlasSensorMgr.init();
  atlasSensorMgr.setPollingInterval(10000); // ms
  atlasSensorMgr.setEnabled(false);
#endif // if defined(NC_BUILD)

  EEPROMLoadConfig();
  Ethernet.begin(currentMAC, currentIP, currentGateway, currentSubnet);
  remoteCommandHandler.init();
  remoteCommandHandler.addCommandHandler(  DA_TCP_COMMAND_GROUP_ATLAS,
                                           remoteAtlasCommandHandler);
  remoteCommandHandler.addCommandHandler( DA_TCP_COMMAND_GROUP_REMOTE,
                                          remoteDeviceCommandHandler);
  remoteCommandHandler.addCommandHandler(   DA_TCP_COMMAND_GROUP_HELP,
                                            remoteHelpCommandHandler);
  remoteCommandHandler.addCommandHandler(DA_TCP_COMMAND_GROUP_ONEWIRE,
                                         remoteOneWireCommandHandler);
  CI_001.setOnEdgeEvent(&onRestoreDefaults);
  CI_001.setEnabled(true);

  // DI Debounce times // default is 100ms
  DI_000.setDebounceTime(DEFAULT_DI_DEBOUNCE_TIME);
  DI_001.setDebounceTime(DEFAULT_DI_DEBOUNCE_TIME);
  DI_002.setDebounceTime(DEFAULT_DI_DEBOUNCE_TIME);
  DI_003.setDebounceTime(DEFAULT_DI_DEBOUNCE_TIME);
  DI_004.setDebounceTime(DEFAULT_DI_DEBOUNCE_TIME);
  DI_005.setDebounceTime(DEFAULT_DI_DEBOUNCE_TIME);
  DI_006.setDebounceTime(DEFAULT_DI_DEBOUNCE_TIME);
  DI_007.setDebounceTime(DEFAULT_DI_DEBOUNCE_TIME);
  DI_008.setDebounceTime(DEFAULT_DI_DEBOUNCE_TIME);
  DI_009.setDebounceTime(DEFAULT_DI_DEBOUNCE_TIME);


  // Enable/Disable DOs from master values
  AI_000.setEnabled(true);
  AI_001.setEnabled(true);
  AI_002.setEnabled(true);
  AI_003.setEnabled(true);
  AI_004.setEnabled(true);
  AI_005.setEnabled(true);
  AI_006.setEnabled(true);


  // Enable/Disable DOs from master values
  // Relays

  DY_000.setEnabled(true);
  DY_001.setEnabled(true);
  DY_002.setEnabled(true);
  DY_003.setEnabled(true);
  DY_004.setEnabled(true);
  DY_005.setEnabled(true);
  DY_006.setEnabled(true);
  DY_007.setEnabled(true);
  DY_008.setEnabled(true);
  DY_009.setEnabled(true);
  DY_010.setEnabled(true);
  DY_011.setEnabled(true);
  DY_012.setEnabled(true);
  DY_013.setEnabled(true);
  DY_014.setEnabled(true);
  DY_015.setEnabled(true);
  DY_016.setEnabled(true);
  DY_017.setEnabled(true);
  DY_018.setEnabled(true);
  #if not defined(NC_BUILD)
  DY_019.setEnabled(true);
  DY_020.setEnabled(true);
  DY_021.setEnabled(true);
#endif // if not defined(NC_BUILD)


  // DIs
  DI_000.setEnabled(true);
  DI_001.setEnabled(true);
  DI_002.setEnabled(true);
  DI_003.setEnabled(true);
  DI_004.setEnabled(true);
  DI_005.setEnabled(true);
  DI_006.setEnabled(true);
  DI_007.setEnabled(true);
  DI_008.setEnabled(true);
  DI_009.setEnabled(true);


  // DI_003.serialize(aOutputStream, true);
  // AOs
  AY_000.setEnabled(true);
  AY_001.setEnabled(true);


  // FT_001.setUnits(true);

  ENABLE_XT006_SENSOR_INTERRUPTS();
  ENABLE_XT007_SENSOR_INTERRUPTS();
}

void loop()
{
  MBSlave.MbsRun();

  refreshHostReads();
  processHostWrites();

  temperatureMgr.refresh();
  refreshAnalogs();
  refreshDiscreteInputs();
  KI_001.refresh();
  KI_004.refresh();
    #if defined(NC_BUILD)
  atlasSensorMgr.refresh();
  #endif // if defined(NC_BUILD)
  remoteCommandHandler.refresh();
}

void refreshAnalogs()
{
  AI_000.refresh();
  AI_001.refresh();
  AI_002.refresh();
  AI_003.refresh();
  AI_004.refresh();
  AI_005.refresh();
  AI_006.refresh();
}

void refreshDiscreteInputs()
{
  DI_000.refresh();
  DI_001.refresh();
  DI_002.refresh();
  DI_003.refresh();
  DI_004.refresh();
  DI_005.refresh();
  DI_006.refresh();
  DI_007.refresh();
  DI_008.refresh();
  DI_009.refresh();

  CI_001.refresh();
}

void onFlowCalc()
{
  DISABLE_XT006_SENSOR_INTERRUPTS();
  DISABLE_XT007_SENSOR_INTERRUPTS();

  XT_006.end();
  XT_006.begin();

  XT_007.end();
  XT_007.begin();
  ENABLE_XT006_SENSOR_INTERRUPTS();
  ENABLE_XT007_SENSOR_INTERRUPTS();
}

void onXT_006_PulseIn()
{
  XT_006.handleFlowDetection();
}

void onXT_007_PulseIn()
{
  XT_007.handleFlowDetection();
}

void onHeartBeat()
{
  KI_001_CV++;
}

/**
 * [onRestoreDefaults restore defaults in EEPROM]
 *                    used as callback in hard DI or
 *                    invoked directly
 * @param aValue [  don't care if invoked directly ]
 * @param aPin   [ don't care if invoked directly ]
 */
void onRestoreDefaults(bool aValue, int aPin)
{
    #ifdef IO_DEBUG
  *aOutputStream << "onRestoreDefaults()" << endl;
    #endif // ifdef IO_DEBUG


  EEPROMWriteDefaultConfig();
  EEPROMLoadConfig();
  Ethernet.begin(currentMAC, currentIP, currentGateway, currentSubnet);
}

/**
 * [byteSwap16 swap bytes in 16 bit word]
 *             AAAABBBB -> BBBBAAAA
 * @param  aValue [16 bit work perform swapping against]
 * @return        [16 bit swapped results]
 */
uint16_t byteSwap16(uint16_t aValue)
{
  uint16_t b1, b2;

  b2 = aValue >> 8;
  b1 = aValue & 0x00FF;


  return b1 << 8 | b2;
}

/**
 * [byteSwap32 swap upper and lower bytes in uppper and lower word of 32 bit
 *             AAAABBBBCCCCDDDD -> BBBBAAAADDDDCCCC ]
 * @param  aValue [32 bit value to perform byte swap]
 * @return        [swaped bytes in 32 bit word]
 */
uint32_t byteSwap32(uint32_t aValue)
{
  uint32_t w1, w2;

  w2 = byteSwap16((uint16_t)(aValue >> 16));
  w1 = byteSwap16((uint16_t)(aValue & 0x0000FFFF));

  return w1 << 16 | w2;
}

/**
 * [refreshTemperatureUUID refesh 1-wire UUID values to host]
 * @param aModbusAddressLow  [modbus address for lower 16 bits]
 * @param aModbusAddressHigh [modbus address for upper 16 bits]
 * @param aUUID              [UUID represented as 64 bits]
 */
void refreshTemperatureUUID(uint16_t aModbusAddressLow,
                            uint16_t aModbusAddressHigh,
                            uint64_t aUUID)
{
  // Temperature 1 - UUID

  blconvert.val                         = byteSwap32((uint32_t)(aUUID >> 32));
  MBSlave.MbData[aModbusAddressLow]     = blconvert.regsl[1];
  MBSlave.MbData[aModbusAddressLow + 1] = blconvert.regsl[0];

  blconvert.val =
    byteSwap32((uint32_t)(aUUID & 0x00000000FFFFFFFF));
  MBSlave.MbData[aModbusAddressHigh]     = blconvert.regsl[1];
  MBSlave.MbData[aModbusAddressHigh + 1] = blconvert.regsl[0];
}

/**
 * Check for a transition from a coil/digital
 * @param  aCurrentState  [current bit value]
 * @param  aPreviousState [previous bit value]
 * @return                [return the BIT_NO_CHANGE, BIT_RISING_EDGE,
 * BIT_FALLING_EDGE]
 */
uint8_t detectTransition(bool aCurrentState, bool aPreviousState)
{
  uint8_t retVal = BIT_NO_CHANGE;

  if (aCurrentState != aPreviousState)
  {
    if (aCurrentState && !aPreviousState) retVal = BIT_RISING_EDGE;
    else if (!aCurrentState && aPreviousState) retVal = BIT_FALLING_EDGE;
  }
  return retVal;
}

void doIPMACChange()
{
  currentIP      = pendingIP;
  currentGateway = pendingGateway;
  currentSubnet  = pendingSubnet;
  memcpy(currentMAC, pendingMAC, sizeof(currentMAC));
  EEPROMWriteCurrentIPs();
  Ethernet.begin(currentMAC, currentIP, currentGateway, currentSubnet);
}

void doCheckIPMACChange()
{
  uint8_t bitState = detectTransition(MBSlave.GetBit(CW_CY_006), CY_006);

  if (bitState == BIT_RISING_EDGE)
  {
    doIPMACChange();
  }
  CY_006 = MBSlave.GetBit(CW_CY_006);
}

void doCheckRestoreDefaults()
{
  uint8_t bitState = detectTransition(MBSlave.GetBit(CW_CY_001), CY_001);

  if (bitState == BIT_RISING_EDGE)
  {
    onRestoreDefaults(false, 0);
  }
  CY_001 = MBSlave.GetBit(CW_CY_001);
}

void doCheckForRescanOneWire()
{
  uint8_t bitState = detectTransition(MBSlave.GetBit(CW_CY_002), CY_002);

  if (bitState == BIT_RISING_EDGE)
  {
    temperatureMgr.scanSensors();
    #ifdef IO_DEBUG
    temperatureMgr.serialize(aOutputStream, true);
    #endif // ifdef IO_DEBUG
  }
  CY_002 = MBSlave.GetBit(CW_CY_002);
}

void rebootDevice()
{
  #ifdef IO_DEBUG
  *aOutputStream << "rebooting..." << endl;
  #endif // ifdef IO_DEBUG
  wdt_enable(WDTO_15MS); // turn on the WatchDog

  for (;;) {}
}

void doCheckRebootDevice()
{
  uint8_t bitState = detectTransition(MBSlave.GetBit(CW_CY_004), CY_004);

  if (bitState == BIT_RISING_EDGE)
  {
    rebootDevice();
  }
  CY_004 = MBSlave.GetBit(CW_CY_004);
}

void refreshHostReads()
{
  MBSlave.MbData[HR_TI_001] =     (int)(temperatureMgr.getTemperature(0) * 10.0);
  MBSlave.MbData[HR_TI_002] =     (int)(temperatureMgr.getTemperature(1) * 10.0);
  MBSlave.MbData[HR_TI_003] =     (int)(temperatureMgr.getTemperature(2) * 10.0);
  MBSlave.MbData[HR_TI_004] =     (int)(temperatureMgr.getTemperature(3) * 10.0);
  MBSlave.MbData[HR_TI_005] =     (int)(temperatureMgr.getTemperature(4) * 10.0);
  MBSlave.MbData[HR_TI_006] =     (int)(temperatureMgr.getTemperature(5) * 10.0);
  MBSlave.MbData[HR_TI_007] =     (int)(temperatureMgr.getTemperature(6) * 10.0);
  MBSlave.MbData[HR_AI_000] =     AI_000.getRawSample();
  MBSlave.MbData[HR_AI_001] =     AI_001.getRawSample();
  MBSlave.MbData[HR_AI_002] =     AI_002.getRawSample();
  MBSlave.MbData[HR_AI_003] =     AI_003.getRawSample();
  MBSlave.MbData[HR_AI_004] =     AI_004.getRawSample();
  MBSlave.MbData[HR_AI_005] =     AI_005.getRawSample();
  MBSlave.MbData[HR_AI_006] =     AI_006.getRawSample();

#if defined(NC_BUILD)
  MBSlave.MbData[HR_XT_001] = atlasSensorMgr.getCachedValue(DA_ATLAS_PH);
  MBSlave.MbData[HR_XT_002] = atlasSensorMgr.getCachedValue(DA_ATLAS_EC);
  MBSlave.MbData[HR_XT_003] = atlasSensorMgr.getCachedValue(DA_ATLAS_ORB);
  MBSlave.MbData[HR_XT_004] = atlasSensorMgr.getCachedValue(DA_ATLAS_DO);
  MBSlave.MbData[HR_XT_005] = atlasSensorMgr.getCachedValue(DA_ATLAS_RTD);
#endif // if defined(NC_BUILD)

  MBSlave.SetBit(CS_DI_000, DI_000.getSample());
  MBSlave.SetBit(CS_DI_001, DI_001.getSample());
  MBSlave.SetBit(CS_DI_002, DI_002.getSample());
  MBSlave.SetBit(CS_DI_003, DI_003.getSample());
  MBSlave.SetBit(CS_DI_004, DI_004.getSample());
  MBSlave.SetBit(CS_DI_005, DI_005.getSample());
  MBSlave.SetBit(CS_DI_006, DI_006.getSample());
  MBSlave.SetBit(CS_DI_007, DI_007.getSample());
  MBSlave.SetBit(CS_DI_008, DI_008.getSample());
  MBSlave.SetBit(CS_DI_009, DI_009.getSample());


  MBSlave.MbData[HR_XT_006_RW] = XT_006.getCurrentPulses();
  MBSlave.MbData[HR_XT_007_RW] = XT_007.getCurrentPulses();

  // watchdog current value
  MBSlave.MbData[HR_KI_001] = KI_001_CV;

  // App major/minor/patch
  MBSlave.MbData[HR_KI_003] = KI_003;

  MBSlave.MbData[HR_KI_005] = KI_005;

  // app build date
  blconvert.val                 = APP_BUILD_DATE;
  MBSlave.MbData[HR_KI_004]     = blconvert.regsl[1];
  MBSlave.MbData[HR_KI_004 + 1] = blconvert.regsl[0];

  // Current IP
  blconvert.val                    =  byteSwap32(currentIP);
  MBSlave.MbData[HR_CI_006_CV]     = blconvert.regsl[1];
  MBSlave.MbData[HR_CI_006_CV + 1] = blconvert.regsl[0];


  // Current gateway
  blconvert.val                    = byteSwap32(currentGateway);
  MBSlave.MbData[HR_CI_007_CV]     = blconvert.regsl[1];
  MBSlave.MbData[HR_CI_007_CV + 1] = blconvert.regsl[0];

  // Current subnet mask
  blconvert.val                    = byteSwap32(currentSubnet);
  MBSlave.MbData[HR_CI_008_CV]     = blconvert.regsl[1];
  MBSlave.MbData[HR_CI_008_CV + 1] = blconvert.regsl[0];

  // current MAC
  //


  memcpy(bmacconvert.boardMAC, currentMAC, sizeof(currentMAC));

  blconvert.val = byteSwap32(
    bmacconvert.val >> 16 & 0xFFFFFFFF);
  MBSlave.MbData[HR_CI_009_CV_L]     = blconvert.regsl[1];
  MBSlave.MbData[HR_CI_009_CV_L + 1] = blconvert.regsl[0];


  blconvert.val                      = byteSwap16(bmacconvert.val & 0xFFFF);
  MBSlave.MbData[HR_CI_009_CV_H]     = blconvert.regsl[1];
  MBSlave.MbData[HR_CI_009_CV_H + 1] = blconvert.regsl[0];


  // refresh 1-wire UUID
  refreshTemperatureUUID(HR_TI_001_ID_L, HR_TI_001_ID_H,
                         temperatureMgr.getUIID(0));
  refreshTemperatureUUID(HR_TI_002_ID_L, HR_TI_002_ID_H,
                         temperatureMgr.getUIID(1));
  refreshTemperatureUUID(HR_TI_003_ID_L, HR_TI_003_ID_H,
                         temperatureMgr.getUIID(2));
  refreshTemperatureUUID(HR_TI_004_ID_L, HR_TI_004_ID_H,
                         temperatureMgr.getUIID(3));
  refreshTemperatureUUID(HR_TI_005_ID_L, HR_TI_005_ID_H,
                         temperatureMgr.getUIID(4));
  refreshTemperatureUUID(HR_TI_006_ID_L, HR_TI_006_ID_H,
                         temperatureMgr.getUIID(5));
  refreshTemperatureUUID(HR_TI_007_ID_L, HR_TI_007_ID_H,
                         temperatureMgr.getUIID(6));
}

void processHostWrites()
{
  // 1-wire Temperatures
  //
  temperatureMgr.setEnabled(MBSlave.GetBit(CW_TI_001_EN), 0);
  temperatureMgr.setEnabled(MBSlave.GetBit(CW_TI_002_EN), 1);
  temperatureMgr.setEnabled(MBSlave.GetBit(CW_TI_003_EN), 2);
  temperatureMgr.setEnabled(MBSlave.GetBit(CW_TI_004_EN), 3);
  temperatureMgr.setEnabled(MBSlave.GetBit(CW_TI_005_EN), 4);
  temperatureMgr.setEnabled(MBSlave.GetBit(CW_TI_006_EN), 5);
  temperatureMgr.setEnabled(MBSlave.GetBit(CW_TI_007_EN), 6);

  // drive DOs from master values
  // Relays

  DY_000.write(MBSlave.GetBit(CW_DY_000));
  DY_001.write(MBSlave.GetBit(CW_DY_001));
  DY_002.write(MBSlave.GetBit(CW_DY_002));
  DY_003.write(MBSlave.GetBit(CW_DY_003));
  DY_004.write(MBSlave.GetBit(CW_DY_004));
  DY_005.write(MBSlave.GetBit(CW_DY_005));
  DY_006.write(MBSlave.GetBit(CW_DY_006));
  DY_007.write(MBSlave.GetBit(CW_DY_007));
  DY_008.write(MBSlave.GetBit(CW_DY_008));
  DY_009.write(MBSlave.GetBit(CW_DY_009));
  DY_010.write(MBSlave.GetBit(CW_DY_010));
  DY_011.write(MBSlave.GetBit(CW_DY_011));
  DY_012.write(MBSlave.GetBit(CW_DY_012));
  DY_013.write(MBSlave.GetBit(CW_DY_013));
  DY_014.write(MBSlave.GetBit(CW_DY_014));
  DY_015.write(MBSlave.GetBit(CW_DY_015));
  DY_016.write(MBSlave.GetBit(CW_DY_016));
  DY_017.write(MBSlave.GetBit(CW_DY_017));
  DY_018.write(MBSlave.GetBit(CW_DY_018));
  #if not defined(NC_BUILD)
  DY_019.write(MBSlave.GetBit(CW_DY_019));
  DY_020.write(MBSlave.GetBit(CW_DY_020));
  DY_021.write(MBSlave.GetBit(CW_DY_021));
#endif // if not defined(NC_BUILD)


  // Drive AOs from master values
  AY_000.writeAO(MBSlave.MbData[HW_AY_000]);

  // AY_000.serialize( aOutputStream, true);
  AY_001.writeAO(MBSlave.MbData[HW_AY_001]);

  // DO TImer presets controllino: Active High reverse


  // capture pending IP
  blconvert.regsl[1] =   MBSlave.MbData[HW_CI_006_PV];
  blconvert.regsl[0] =   MBSlave.MbData[HW_CI_006_PV + 1];
  pendingIP          = byteSwap32(blconvert.val);

  // capture pending gateway
  blconvert.regsl[1] =   MBSlave.MbData[HW_CI_007_PV];
  blconvert.regsl[0] =   MBSlave.MbData[HW_CI_007_PV + 1];
  pendingGateway     = byteSwap32(blconvert.val);

  // capture pending subnet
  blconvert.regsl[1] =   MBSlave.MbData[HW_CI_008_PV];
  blconvert.regsl[0] =   MBSlave.MbData[HW_CI_008_PV + 1];
  pendingSubnet      = byteSwap32(blconvert.val);

  // capture pending MAC
  uint32_t t32;
  blconvert.regsl[1] = MBSlave.MbData[HW_CI_009_PV_L];
  blconvert.regsl[0] = MBSlave.MbData[HW_CI_009_PV_L + 1];
  t32                = byteSwap32(blconvert.val);

  memcpy(pendingMAC + 2, &t32, 4);
  blconvert.regsl[1] = MBSlave.MbData[HW_CI_009_PV_H];
  blconvert.regsl[0] = MBSlave.MbData[HW_CI_009_PV_H + 1];
  t32                = byteSwap16(blconvert.val);
  memcpy(pendingMAC, &t32, 2);


  doCheckIPMACChange();

  // doCheckMACChange();
  doCheckRestoreDefaults();
  doCheckForRescanOneWire();
  doCheckRebootDevice();
}

void EEPROMWriteCurrentIPs()
{
  uint32_t temp32 = currentIP;

  EEPROM.put(     EEPROM_IP_ADDR, temp32);
  temp32 = currentGateway;
  EEPROM.put(EEPROM_GATEWAY_ADDR, temp32);
  temp32 = currentSubnet;
  EEPROM.put( EEPROM_SUBNET_ADDR, temp32);


  EEPROM.put(    EEPROM_MAC_ADDR, currentMAC);
}

void EEPromWriteOneWireMaps()
{
  EEPROM.put(EEPROM_ONE_WIRE_MAP, temperatureMgr.oneWireTemperatureMap);
}

void printByteArray(uint8_t anArray[], uint8_t aSize, Stream *aOutputStream)
{
  *aOutputStream << "{";

  for (uint8_t i = 0; i < aSize; i++)
  {
    *aOutputStream << "0x";

    if (anArray[i] < 16) *aOutputStream << '0';

    *aOutputStream << _HEX(anArray[i]);

    if (i < aSize - 1) *aOutputStream << ",";
  }
  *aOutputStream << "}";
}

void EEPROMLoadConfig()

{
  uint8_t configFlag;

  EEPROM.get(EEPROM_CONFIG_FLAG_ADDR, configFlag);

  if (configFlag != EEPROM_CONFIGURED)
  {
    EEPROMWriteDefaultConfig();
  }

  uint32_t temp32;

  EEPROM.get(     EEPROM_IP_ADDR, temp32);
  currentIP = temp32;

  EEPROM.get(EEPROM_GATEWAY_ADDR, temp32);
  currentGateway = temp32;

  EEPROM.get( EEPROM_SUBNET_ADDR, temp32);
  currentSubnet = temp32;

  EEPROM.get(    EEPROM_MAC_ADDR, currentMAC);
  EEPROM.get(EEPROM_ONE_WIRE_MAP, temperatureMgr.oneWireTemperatureMap);

    #ifdef IO_DEBUG
  *aOutputStream << "currentIP:" << currentIP << endl;
  *aOutputStream << "currentGateway:" << currentGateway << endl;
  *aOutputStream << "currentSubnet:" << currentSubnet << endl;

  printByteArray(currentMAC, 6, aOutputStream);

    #endif // ifdef IO_DEBUG
}

void EEPROMWriteDefaultConfig()
{
  uint8_t configFlag = EEPROM_CONFIGURED;

  EEPROM.update(EEPROM_CONFIG_FLAG_ADDR, configFlag);
  uint32_t temp32 = defaultIP;

  EEPROM.put(     EEPROM_IP_ADDR, temp32);
  temp32 = defaultGateway;
  EEPROM.put(EEPROM_GATEWAY_ADDR, temp32);
  temp32 = defaultSubnet;
  EEPROM.put( EEPROM_SUBNET_ADDR, temp32);

    EEPROM.put(    EEPROM_MAC_ADDR, defaultMAC);
  temperatureMgr.resetMaps();
  EEPromWriteOneWireMaps();
}

void remoteAtlasCommandHandler(uint8_t argc, char **argv, Stream *aOutputStream)
{
    #if defined(NC_BUILD)
  char command = argv[0][0];



  switch (command) {
  case 's':

    if (argc == 3)
    {
      uint8_t channel = atoi(argv[1]);

      if ((channel >= 0) && (channel < (DA_ATLAS_MAX_CHANNELS)))
      {
        char atlasrxBuff[DA_ATLAS_RX_BUF_SZ];
        atlasSensorMgr.sendRaw(channel, argv[2], atlasrxBuff, 800);
        *aOutputStream << F("Reply:")  << atlasrxBuff << endl;
      }
      else *aOutputStream << F("Invalid Channel:") << channel << endl;
    }
    else *aOutputStream << F("Unrecongized format for command")  << endl;
    break;

  case 'd':

    if (argc == 1)
    {
      for (int i = 0; i < DA_ATLAS_MAX_CHANNELS;
           i++) *aOutputStream << "Sensor[" << i << "] Type:" <<
          atlasSensorMgr.getSensorType(i) << " cached value:" <<
          atlasSensorMgr.getCachedValue(i) << endl;
    }
    else *aOutputStream << F("Unrecongized format for command")  << endl;
    break;

  default:
    *aOutputStream << F("Invalid Command for Atlas")  << endl;
  }
  #else // if defined(NC_BUILD)
  *aOutputStream << F("Atlas not Implemented on this Remote I/O") << endl;
  #endif // if defined(NC_BUILD)
}

void remoteDeviceCommandHandler(uint8_t argc, char  **argv, Stream *aOutputStream)
{
  char command = argv[0][0];

  switch (command) {
  case 'i':

    *aOutputStream << F("TODO set IP:")  << endl;

    break;

  case 'r':

    if (argc == 1)
    {
      *aOutputStream << F("reseting to defaults. Closing Client Connection.") <<
        endl;
      onRestoreDefaults(CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_07, 0);
      rebootDevice();
    }
    else *aOutputStream << F("Unrecongized format for command")  << endl;
    break;

  case 'd':

    if (argc == 1)
    {
      *aOutputStream << F("Version:") << APP_MAJOR << "." << APP_MINOR << "." <<
        APP_PATCH << endl;
      *aOutputStream << F("Device Type:") << DEVICE_TYPE << endl;
      *aOutputStream << F("currentIP:") << currentIP << endl;
      *aOutputStream << F("currentGateway:") << currentGateway << endl;
      *aOutputStream << F("currentSubnet:") << currentSubnet << endl;
      *aOutputStream << F("MAC Address:");
      printByteArray(currentMAC, 6, aOutputStream);
      *aOutputStream << endl;
    }
    else *aOutputStream << F("Unrecognized format for command")  << endl;
    break;

  default:
    *aOutputStream << F("Invalid Command for Remote")  << endl;
  }

  /*
     for (int i = 0; i < argc; i++)
     {
      aOutputStream->print("Arg ");
      aOutputStream->print(     i);
      aOutputStream->print(  ": ");
      aOutputStream->println(argv[i]);
     }
   */
}

void remoteHelpCommandHandler(uint8_t argc,
                              char  **argv,
                              Stream *aOutputStream)
{
  *aOutputStream << F("Atlas Group") << endl;
  *aOutputStream << F("  Send Raw Commands to Altas Sensor:");
  *aOutputStream << F(" atlas s <channel> <raw sensor command>") << endl;
  *aOutputStream << F("  Display Current Sensor Cached Values:");
  *aOutputStream << F(" atlas d ") << endl;

  *aOutputStream << F("Remote Group") << endl;
  *aOutputStream << F("  Display General Remote Device Info:");
  *aOutputStream << F(" remote d") << endl;
  *aOutputStream << F("  Change Serial Port Baud Rate:");
  *aOutputStream << F("remote b <baud> TODO ") << endl;
  *aOutputStream << F("  Change IP:");
  *aOutputStream << F("remote i <IP> TODO ") << endl;
  *aOutputStream << F("  Change Gateway:");
  *aOutputStream << F("remote g <gateway> TODO ") << endl;
  *aOutputStream << F("  Change Subnet Mask:");
  *aOutputStream << F("remote s <Subnet Mask> TODO ") << endl;
  *aOutputStream << F("  Change MAC Address:");
  *aOutputStream << F("remote m <MAC> TODO ") << endl;
  *aOutputStream << F("  Reset to Defaults:");
  *aOutputStream << F("remote r") << endl;

  *aOutputStream << F("1-Wire Group") << endl;
  *aOutputStream << F("  Display Current 1-Wire Info:");
  *aOutputStream << F(" 1wire d ") << endl;
  *aOutputStream << F("  Map 1-Wire Temperature x to sensore position y:");
  *aOutputStream << F(" 1wire m <x> <y>") << endl;
}

void remoteOneWireCommandHandler(uint8_t argc, char  **argv,
                                 Stream *aOutputStream)
{

  #if  not defined(NC2_BUILD)
  char command = argv[0][0];

  switch (command) {
  case 'm':

    if (argc == 3)
    {
      uint8_t x = atoi(argv[1]);
      uint8_t y = atoi(argv[2]);

      if (temperatureMgr.mapSensor(x, y))
      {
        EEPromWriteOneWireMaps();
        temperatureMgr.serialize(aOutputStream, true);
      }
      else *aOutputStream << F("x and|or y out of range:") << " x:" << x <<
          " y:" << y << endl;
    }
    else *aOutputStream << F("Unrecongized format for command")  << endl;
    break;

  case 'd':

    if (argc == 1)
    {
      temperatureMgr.serialize(aOutputStream, true);
    }
    else *aOutputStream << F("Unrecognized format for command")  << endl;
    break;

  default:
    *aOutputStream << F("Invalid Command for 1Wire")  << endl;
  }

  /*
     for (int i = 0; i < argc; i++)
     {
      aOutputStream->print("Arg ");
      aOutputStream->print(     i);
      aOutputStream->print(  ": ");
      aOutputStream->println(argv[i]);
     }
   */
  #else
       *aOutputStream << F("1-Wire not Implemented on this Remote I/O")  << endl;
  #endif
}
