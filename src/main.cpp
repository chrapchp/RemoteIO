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

#include <HardwareSerial.h>
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

#include "remoteIO.h"


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
void onFT_001_PulseIn();
void processHostWrites();
void refreshHostReads();
void refreshModbusRegisters();
void refreshAnalogs();
void refreshDiscreteInputs();
void refreshTimerOutputs();
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
#ifdef IO_DEBUG
void printByteArray(uint8_t anArray[],
                    uint8_t aSize);
#endif // ifdef IO_DEBUG


// IP address from class needs bytes flipped for modbus TBOX
// did not want to modify class
uint32_t byteSwap32(uint32_t aValue);
uint16_t byteSwap16(uint16_t aValue);


DA_FlowMeter FT_001(FT001_SENSOR_INTERUPT_PIN, FLOW_CALC_PERIOD_SECONDS);

// Discrete Inputs
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

// reset IP to defaults
DA_DiscreteInput CI_001 = DA_DiscreteInput(
  CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_11,
  DA_DiscreteInput::FallingEdgeDetect,
  false);

// Relay Outputs
//
DA_DiscreteOutput DY_000 = DA_DiscreteOutput(CONTROLLINO_RELAY_00, HIGH);
DA_DiscreteOutput DY_001 = DA_DiscreteOutput(CONTROLLINO_RELAY_01, HIGH);
DA_DiscreteOutput DY_002 = DA_DiscreteOutput(CONTROLLINO_RELAY_02, HIGH);
DA_DiscreteOutput DY_003 = DA_DiscreteOutput(CONTROLLINO_RELAY_03, HIGH);
DA_DiscreteOutput DY_004 = DA_DiscreteOutput(CONTROLLINO_RELAY_04, HIGH);
DA_DiscreteOutput DY_005 = DA_DiscreteOutput(CONTROLLINO_RELAY_05, HIGH);
DA_DiscreteOutput DY_006 = DA_DiscreteOutput(CONTROLLINO_RELAY_06, HIGH);

// Timer based relay outputs
DA_DiscreteOutputTmr DY_007 = DA_DiscreteOutputTmr(CONTROLLINO_RELAY_07,
                                                   HIGH,
                                                   DEFAULT_TMR_ON_DURATION,
                                                   DEFAULT_TMR_OFF_DURATION);
DA_DiscreteOutputTmr DY_008 = DA_DiscreteOutputTmr(CONTROLLINO_RELAY_08,
                                                   HIGH,
                                                   DEFAULT_TMR_ON_DURATION,
                                                   DEFAULT_TMR_OFF_DURATION);

// Solid State Outputs
DA_DiscreteOutput DY_009 = DA_DiscreteOutput(CONTROLLINO_DO0, HIGH);
DA_DiscreteOutput DY_010 = DA_DiscreteOutput(CONTROLLINO_DO1, HIGH);
DA_DiscreteOutput DY_011 = DA_DiscreteOutput(CONTROLLINO_DO2, HIGH);

// Status LED
DA_DiscreteOutput SI_001 = DA_DiscreteOutput(CONTROLLINO_DO7, HIGH );



// 0-24V
DA_AnalogInput AI_000 =  DA_AnalogInput(CONTROLLINO_A0, 0.0, 1024.0);
DA_AnalogInput AI_001 =  DA_AnalogInput(CONTROLLINO_A1, 0.0, 1024.0);
DA_AnalogInput AI_002 =  DA_AnalogInput(CONTROLLINO_A2, 0.0, 1024.0);
DA_AnalogInput AI_003 =  DA_AnalogInput(CONTROLLINO_A3, 0.0, 1024.0);
DA_AnalogInput AI_004 =  DA_AnalogInput(CONTROLLINO_A4, 0.0, 1024.0);
DA_AnalogInput AI_005 =  DA_AnalogInput(CONTROLLINO_A5, 0.0, 1024.0);

// 0-10 V
DA_AnalogInput AI_006 =  DA_AnalogInput(CONTROLLINO_A12, 0.0, 1024.0);
DA_AnalogInput AI_007 =  DA_AnalogInput(CONTROLLINO_A13, 0.0, 1024.0);


// 0-10 V analog outputs
DA_AnalogOutput AY_000 = DA_AnalogOutput(CONTROLLINO_AO0);
DA_AnalogOutput AY_001 = DA_AnalogOutput(CONTROLLINO_AO1);

// App Version
uint16_t KI_003 = APP_MAJOR << 8 | APP_MINOR << 4 | APP_PATCH;


// timer for heart beat and potentially trigger for other operations
DA_NonBlockingDelay KI_001 = DA_NonBlockingDelay(HEART_BEAT_PERIOD, onHeartBeat);
uint16_t KI_001_CV         = 0;

// timer for flow calcs
DA_NonBlockingDelay KI_004 = DA_NonBlockingDelay(FLOW_CALC_PERIOD_SECONDS * 1000,
                                                 onFlowCalc);


DA_OneWireDallasMgr temperatureMgr = DA_OneWireDallasMgr(WIRE_BUS_PIN);

// Debug Serial port
HardwareSerial *tracePort = &Serial;

// commands from host that need to be onshots
bool CY_006 = false; // update IP
bool CY_001 = false; // restore defaults
bool CY_002 = false; // rescan one wire temperatures devices
bool CY_004 = false; // reboot remote I/O

    #ifdef IO_DEBUG
void onTemperatureRead()
{
  *tracePort << "New Sample" << endl;

  for (int i = 0; i < DA_MAX_ONE_WIRE_SENSORS; i++)
  {
    // temperatureMgr.enableSensor(i);
    *tracePort << "idx:" << i << "temp:" << temperatureMgr.getTemperature(i) <<
      endl;
  }
}

#endif // ifdef IO_DEBUG


void setup()
{

  MCUSR = 0; // clear existing watchdog timer presets
  temperatureMgr.setPollingInterval(5000);
  temperatureMgr.setBlockingRead(false);
  temperatureMgr.scanSensors();
    #ifdef IO_DEBUG
  tracePort->begin(9600);

  temperatureMgr.serialize(tracePort, true);
  temperatureMgr.setOnPollCallBack(onTemperatureRead);
      #endif // ifdef PROCESS_TERMINAL

  temperatureMgr.init();

  EEPROMLoadConfig();

  // currentIP.fromString("192.168.1.51");
  Ethernet.begin(currentMAC, currentIP, currentGateway, currentSubnet);

  CI_001.setOnEdgeEvent(&onRestoreDefaults);
  CI_001.setEnabled(true);

  // FT_001.setUnits(true);
  ENABLE_FT001_SENSOR_INTERRUPTS();
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
  refreshTimerOutputs();
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
  AI_007.refresh();
}

void refreshDiscreteInputs()
{
  DI_000.refresh();
  DI_001.refresh();
  DI_002.refresh();
  DI_003.refresh();
  CI_001.refresh();
}

void refreshTimerOutputs()
{
  DY_007.refresh();
  DY_008.refresh();
}

void onFlowCalc()
{
  DISABLE_FT001_SENSOR_INTERRUPTS();

  FT_001.end();
  SI_001.toggle();
  // FT_001.serialize(tracePort, true);
  FT_001.begin();
  ENABLE_FT001_SENSOR_INTERRUPTS();
}

void onFT_001_PulseIn()
{
  FT_001.handleFlowDetection();
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
  *tracePort << "onRestoreDefaults()" << endl;
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
    temperatureMgr.serialize(tracePort, true);
    #endif // ifdef IO_DEBUG
  }
  CY_002 = MBSlave.GetBit(CW_CY_002);
}

void doCheckRebootDevice()
{
  uint8_t bitState = detectTransition(MBSlave.GetBit(CW_CY_004), CY_004);

  if (bitState == BIT_RISING_EDGE)
  {

    #ifdef IO_DEBUG
    *tracePort << "rebooting..." << endl;
    #endif // ifdef IO_DEBUG
    wdt_enable(WDTO_15MS); // turn on the WatchDog
    for(;;) { }
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
  MBSlave.MbData[HR_AI_007] =     AI_007.getRawSample();


  uint32_t onVal;
  uint32_t offVal;

  onVal =
    (DY_007.isActiveLow() ==
     true ? DY_007.getCurrentOnDuration() : DY_007.getCurrentOffDuration());
  offVal =
    (DY_007.isActiveLow() ==
     true ? DY_007.getCurrentOffDuration() : DY_007.getCurrentOnDuration());


  MBSlave.MbData[HR_DY_007_OFCV] = (uint16_t)(offVal / 1000);
  MBSlave.MbData[HR_DY_007_ONCV] = (uint16_t)(onVal / 1000);

  onVal =
    (DY_008.isActiveLow() ==
     true ? DY_008.getCurrentOnDuration() : DY_008.getCurrentOffDuration());
  offVal =
    (DY_008.isActiveLow() ==
     true ? DY_008.getCurrentOffDuration() : DY_008.getCurrentOnDuration());
  MBSlave.MbData[HR_DY_008_OFCV] = (uint16_t)(offVal / 1000);
  MBSlave.MbData[HR_DY_008_ONCV] = (uint16_t)(onVal / 1000);

  // MBSlave.SetBit(CS_DI_000, digitalRead(CONTROLLINO_DI0));

  MBSlave.SetBit(CS_DI_000, DI_000.getSample());
  MBSlave.SetBit(CS_DI_001, DI_001.getSample());
  MBSlave.SetBit(CS_DI_002, DI_002.getSample());
  MBSlave.SetBit(CS_DI_003, DI_003.getSample());

  MBSlave.MbData[HR_FI_001_RW] = FT_001.getCurrentPulses();

  // watchdog current value
  MBSlave.MbData[HR_KI_001] = KI_001_CV;

  // App major/minor/patch
  MBSlave.MbData[HR_KI_003] = KI_003;

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
  // Enable/Disable DOs from master values
  AI_000.setEnabled(MBSlave.GetBit(CW_AI_000_EN));
  AI_001.setEnabled(MBSlave.GetBit(CW_AI_001_EN));
  AI_002.setEnabled(MBSlave.GetBit(CW_AI_002_EN));
  AI_003.setEnabled(MBSlave.GetBit(CW_AI_003_EN));
  AI_004.setEnabled(MBSlave.GetBit(CW_AI_004_EN));
  AI_005.setEnabled(MBSlave.GetBit(CW_AI_005_EN));
  AI_006.setEnabled(MBSlave.GetBit(CW_AI_006_EN));
  AI_007.setEnabled(MBSlave.GetBit(CW_AI_007_EN));

  // Enable/Disable DOs from master values
  // Relays

  DY_000.setEnabled(MBSlave.GetBit(CW_DY_000_EN));
  DY_001.setEnabled(MBSlave.GetBit(CW_DY_001_EN));
  DY_003.setEnabled(MBSlave.GetBit(CW_DY_003_EN));
  DY_004.setEnabled(MBSlave.GetBit(CW_DY_004_EN));
  DY_005.setEnabled(MBSlave.GetBit(CW_DY_005_EN));
  DY_006.setEnabled(MBSlave.GetBit(CW_DY_006_EN));

  // DO Timers
  DY_007.setEnabled(MBSlave.GetBit(CW_DY_007_EN));
  DY_008.setEnabled(MBSlave.GetBit(CW_DY_008_EN));

  // Solid state
  DY_009.setEnabled(MBSlave.GetBit(CW_DY_009_EN));
  DY_010.setEnabled(MBSlave.GetBit(CW_DY_010_EN));
  DY_011.setEnabled(MBSlave.GetBit(CW_DY_011_EN));

  // DIs
  DI_000.setEnabled(MBSlave.GetBit(CW_DI_000_EN));
  DI_001.setEnabled(MBSlave.GetBit(CW_DI_001_EN));
  DI_002.setEnabled(MBSlave.GetBit(CW_DI_002_EN));
  DI_003.setEnabled(MBSlave.GetBit(CW_DI_003_EN));

  // DI Debounce times // default is 100ms
  DI_000.setDebounceTime(MBSlave.MbData[HW_DI_000_DT]);
  DI_001.setDebounceTime(MBSlave.MbData[HW_DI_001_DT]);
  DI_002.setDebounceTime(MBSlave.MbData[HW_DI_002_DT]);
  DI_003.setDebounceTime(MBSlave.MbData[HW_DI_003_DT]);

  // DI_003.serialize(tracePort, true);
  // AOs
  AY_000.setEnabled(MBSlave.GetBit(CW_AY_000_EN));
  AY_001.setEnabled(MBSlave.GetBit(CW_AY_001_EN));

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


  // Solid state
  DY_009.write(MBSlave.GetBit(CW_DY_009));
  DY_010.write(MBSlave.GetBit(CW_DY_010));
  DY_011.write(MBSlave.GetBit(CW_DY_011));

  // Drive AOs from master values
  AY_000.writeAO(MBSlave.MbData[HW_AY_000]);

  // AY_000.serialize( tracePort, true);
  AY_001.writeAO(MBSlave.MbData[HW_AY_001]);

  // DO TImer presets controllino: Active High reverse
  uint16_t onVal;
  uint16_t offVal;

  onVal =
    (DY_007.isActiveLow() ==
     true ? MBSlave.MbData[HW_DY_007_ONSP] : MBSlave.MbData[HW_DY_007_OFSP]);
  offVal =
    (DY_007.isActiveLow() ==
     true ? MBSlave.MbData[HW_DY_007_OFSP] : MBSlave.MbData[HW_DY_007_ONSP]);

  DY_007.setOffDuration(offVal);
  DY_007.setOnDuration(onVal);

  onVal =
    (DY_008.isActiveLow() ==
     true ? MBSlave.MbData[HW_DY_008_ONSP] : MBSlave.MbData[HW_DY_008_OFSP]);
  offVal =
    (DY_008.isActiveLow() ==
     true ? MBSlave.MbData[HW_DY_008_OFSP] : MBSlave.MbData[HW_DY_008_ONSP]);
  DY_008.setOffDuration(offVal);
  DY_008.setOnDuration(onVal);

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

#ifdef IO_DEBUG
void printByteArray(uint8_t anArray[], uint8_t aSize)
{
  *tracePort << "{";

  for (uint8_t i = 0; i < aSize; i++)
  {
    *tracePort << "0x";

    if (anArray[i] < 16) *tracePort << '0';

    *tracePort << _HEX(anArray[i]);

    if (i < aSize - 1) *tracePort << ",";
  }
  *tracePort << "}";
}

#endif // ifdef IO_DEBUG


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

    #ifdef IO_DEBUG
  *tracePort << "6...";
  *tracePort << "currentIP:" << currentIP << endl;
  *tracePort << "currentGateway:" << currentGateway << endl;
  *tracePort << "currentSubnet:" << currentSubnet << endl;

  printByteArray(currentMAC, 6);

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
}
