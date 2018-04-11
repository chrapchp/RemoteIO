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


#include <Ethernet.h>
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
#include <DA_OneWireDallasMgr.h>

#include "remoteIO.h"


MgsModbus MBSlave;


// Ethernet settings (depending on MAC and Local network)
// byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0x94, 0xB5 };
//

// Organizationally Unique Identifier made up
////  0xDE 0xAD 0xBE 0x?? 0x?? 0x??
// https://macvendorlookup.com/search
// https://www.miniwebtool.com/mac-address-generator/


byte boardMAC[] = { DEFAULT_MAC_ADDRESS };
IPAddress currentIP(DEFAULT_IP_ADDRESS);
IPAddress currentGateway(DEFAULT_GATEWAY);
IPAddress currentSubnet(DEFAULT_SUBNET_MASK);


// #define IO_DEBUG 2

// forward Declarations
void onFT_001_PulseIn();
void processModbusCommands();
void refreshModbusToHost();
void refreshModbusRegisters();
void refreshAnalogs();
void refreshDiscreteInputs();
void refreshTimerOutputs();
void onRestoreDefaults(bool aValue, int aPin);


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
DA_DiscreteInput CI_001 = DA_DiscreteInput(CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_11, DA_DiscreteInput::FallingEdgeDetect, false);

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
                                                   LOW,
                                                   2* DEFAULT_TMR_ON_DURATION ,
                                                   DEFAULT_TMR_OFF_DURATION);
DA_DiscreteOutputTmr DY_008 = DA_DiscreteOutputTmr(CONTROLLINO_RELAY_08,
                                                   LOW,
                                                   DEFAULT_TMR_ON_DURATION,
                                                   2* DEFAULT_TMR_OFF_DURATION);

// Solid State Outputs
DA_DiscreteOutput DY_009 = DA_DiscreteOutput(CONTROLLINO_DO0, HIGH);
DA_DiscreteOutput DY_010 = DA_DiscreteOutput(CONTROLLINO_DO1, HIGH);
DA_DiscreteOutput DY_011 = DA_DiscreteOutput(CONTROLLINO_DO2, HIGH);


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


DA_OneWireDallasMgr temperatureMgr = DA_OneWireDallasMgr(WIRE_BUS_PIN);
HardwareSerial     *tracePort      = &Serial;

    #ifdef IO_DEBUG
void onTemperatureRead()
{
  *tracePort << "New Sample" << endl;

  for (int i = 0; i < DA_MAX_ONE_WIRE_SENSORS; i++)
  {
    temperatureMgr.enableSensor(i);
    *tracePort << "idx:" << i << "temp:" << temperatureMgr.getTemperature(i) <<
      endl;
  }
}

#endif // ifdef IO_DEBUG


void setup()
{
  // currentIP.fromString("192.168.1.51");
  Ethernet.begin(boardMAC, currentIP, currentGateway, currentSubnet);
  temperatureMgr.setPollingInterval(5000);
  temperatureMgr.setBlockingRead(false);
  temperatureMgr.scanSensors();
    #ifdef IO_DEBUG
  tracePort->begin(9600);

  temperatureMgr.serialize(tracePort, true);
      #endif // ifdef PROCESS_TERMINAL
  temperatureMgr.setOnPollCallBack(onTemperatureRead);
  CI_001.setOnEdgeEvent(&onRestoreDefaults);
  CI_001.setEnabled(true);



  temperatureMgr.init();
}

void loop()
{
  MBSlave.MbsRun();
  refreshModbusToHost();
  processModbusCommands();
//  temperatureMgr.refresh();
//  refreshAnalogs();
  refreshDiscreteInputs();



  //refreshTimerOutputs();


  // AI_006.refresh();
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

void onFT_001_PulseIn()

{
  FT_001.handleFlowDetection();
}


void onRestoreDefaults( bool aValue, int aPin)
{
    #ifdef IO_DEBUG
    *tracePort << "TODO:onRestoreDefaults" << endl;
    #endif
}
void refreshModbusToHost()
{
  /*
     MBSlave.SetBit(CS_DS_000, DS_000.getSample());
     MBSlave.SetBit(CS_DS_001, DS_001.getSample());
     MBSlave.SetBit(CS_DS_002, DS_002.getSample());
     MBSlave.MbData[HR_FI_001_RAW] = FI_001.getCurrentPulses();
   */

  /*

     blconvert.val                    = uint32_t(currentIP);
     MBSlave.MbData[HR_DI_002_IP]     = blconvert.regsl[0];
     MBSlave.MbData[HR_DI_002_IP + 1] = blconvert.regsl[1];


     blconvert.val                     = uint32_t(currentGateway);
     MBSlave.MbData[HR_DI_002_IPG]     = blconvert.regsl[0];
     MBSlave.MbData[HR_DI_002_IPG + 1] = blconvert.regsl[1];

     blconvert.val                    = uint32_t(currentSubnet);
     MBSlave.MbData[HR_DI_002_SM]     = blconvert.regsl[0];
     MBSlave.MbData[HR_DI_002_SM + 1] = blconvert.regsl[1];

   */
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

  //MBSlave.SetBit(CS_DI_000, digitalRead(CONTROLLINO_DI0));

  MBSlave.SetBit(CS_DI_000, DI_000.getSample());
  MBSlave.SetBit(CS_DI_001, DI_001.getSample());
  MBSlave.SetBit(CS_DI_002, DI_002.getSample());
  MBSlave.SetBit(CS_DI_003, DI_003.getSample());

  // MBSlave.MbData[HR_AI_008] =        AI_008.getRawSample();
  // MBSlave.MbData[HR_AI_009] =        AI_009.getRawSample();


  /*
     bfconvert.val                 = temperatureMgr.getTemperature( 0 );
     MBSlave.MbData[HR_TI_001]     = bfconvert.regsf[0];
     MBSlave.MbData[HR_TI_001 + 1] = bfconvert.regsf[1];

     bfconvert.val                 = temperatureMgr.getTemperature( 1 );
     MBSlave.MbData[HR_TI_002]     = bfconvert.regsf[0];
     MBSlave.MbData[HR_TI_002 + 1] = bfconvert.regsf[1];

     bfconvert.val                 = temperatureMgr.getTemperature( 2 );
     MBSlave.MbData[HR_TI_003]     = bfconvert.regsf[0];
     MBSlave.MbData[HR_TI_003 + 1] = bfconvert.regsf[1];

     bfconvert.val                 = temperatureMgr.getTemperature( 3 );
     MBSlave.MbData[HR_TI_004]     = bfconvert.regsf[0];
     MBSlave.MbData[HR_TI_004 + 1] = bfconvert.regsf[1];

     bfconvert.val                 = temperatureMgr.getTemperature( 4 );
     MBSlave.MbData[HR_TI_005]     = bfconvert.regsf[0];
     MBSlave.MbData[HR_TI_005 + 1] = bfconvert.regsf[1];
   */
}

void processModbusCommands()
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

  // drive DOs from master values
  // Relays

DY_000.write( MBSlave.GetBit(CW_DY_000));

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


  //  MBSlave.GetBit(CW_FI_001_EN);
}

void refreshModbusRegisters()
{}


/*

   void EEPROMWriteDefaultConfig()
   {
    unsigned short configFlag = EEPROM_CONFIGURED;

    eeprom_write_block((const void *)&configFlag,
                       (void *)EEPROM_CONFIG_FLAG_ADDR,
                       sizeof(configFlag));

    time_t epoch = alarmTimeToUTC(DEFAULT_LIGHTS_ON_ALARM_TIME);
    EEPROMWriteAlarmEntry(epoch, EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
    EEPROMWriteAlarmEntry(epoch, EEPROM_SEEDING_AREA_ON_TIME_ADDR);

    epoch = alarmTimeToUTC(DEFAULT_LIGHTS_OFF_ALARM_TIME);
    EEPROMWriteAlarmEntry(epoch, EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
    EEPROMWriteAlarmEntry(epoch, EEPROM_SEEDING_AREA_OFF_TIME_ADDR);

    EEPROMWriteDuration(              DEFAULT_FAN_ON_DURATION,
                                      EEPROM_FAN_ON_DURATION_ADDR);
    EEPROMWriteDuration(             DEFAULT_FAN_OFF_DURATION,
                                     EEPROM_FAN_OFF_DURATION_ADDR);

    EEPROMWriteDuration( DEFAULT_CIRCULATION_PUMP_ON_DURATION,
                         EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR);
    EEPROMWriteDuration(DEFAULT_CIRCULATION_PUMP_OFF_DURATION,
                        EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR);
   }

   void EEPROMWriteDuration(unsigned int duration, unsigned int atAddress)
   {
    eeprom_write_block((const void *)&duration, (void *)atAddress,
                       sizeof(duration));
   }

   unsigned int EEPROMReadDuration(unsigned int atAddress)
   {
    unsigned int duration = 0;

    eeprom_read_block((void *)&duration, (void *)atAddress, sizeof(duration));
    return duration;
   }

   void EEPROMWriteAlarmEntry(time_t epoch, unsigned int atAddress)
   {
    eeprom_write_block((const void *)&epoch, (void *)atAddress, sizeof(epoch));
   }

   time_t EEPROMReadAlarmEntry(unsigned int atAddress)
   {
    time_t epoch = 0;

    eeprom_read_block((void *)&epoch, (void *)atAddress, sizeof(epoch));
    return epoch;
   }

   unsigned int isEEPROMConfigured()
   {
    unsigned short configFlag;

    eeprom_read_block((void *)&configFlag, (void *)EEPROM_CONFIG_FLAG_ADDR,
                      sizeof(configFlag));
    return configFlag;
   }

   void EEPROMLoadConfig()
   {
    growingChamberLights.offEpoch = EEPROMReadAlarmEntry(
      EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
    growingChamberLights.onLightsOff =  doGrowingChamberLightsOff;

    growingChamberLights.onEpoch = EEPROMReadAlarmEntry(
      EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
    growingChamberLights.onLightsOn =  doGrowingChamberLightsOn;

    seedingAreaLights.offEpoch = EEPROMReadAlarmEntry(
      EEPROM_SEEDING_AREA_OFF_TIME_ADDR);
    seedingAreaLights.onLightsOff = doSeedingAreaLightsOff;

    seedingAreaLights.onEpoch = EEPROMReadAlarmEntry(
      EEPROM_SEEDING_AREA_ON_TIME_ADDR);
    seedingAreaLights.onLightsOn = doSeedingAreaLightsOn;

    PY_001.setOnDuration(EEPROMReadDuration(
                           EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR));
    PY_001.setOffDuration(EEPROMReadDuration(
                            EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR));

    MY_101.setOnDuration(EEPROMReadDuration(EEPROM_FAN_ON_DURATION_ADDR));
    MY_101.setOffDuration(EEPROMReadDuration(EEPROM_FAN_OFF_DURATION_ADDR));


   }

 */
