#include <Ethernet.h>
#include <MgsModbus.h> // cchange memory size here

#include <HardwareSerial.h>
#include <Streaming.h>
#include <Controllino.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <DA_Flowmeter.h>
#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_OneWireDallasMgr.h>

#include "remoteIO.h"

#define CONTROLLINO_MAXI_AUTOMATION

MgsModbus MBSlave;
int inByte = 0; // incoming serial byte

// Ethernet settings (depending on MAC and Local network)
// byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0x94, 0xB5 };
//

// Organizationally Unique Identifier made up
////  0xDE 0xAD 0xBE 0x?? 0x?? 0x??
// https://macvendorlookup.com/search
// https://www.miniwebtool.com/mac-address-generator/


byte boardMAC[] = { 0xDE, 0xAD, 0xBE, 0x0E, 0x94, 0xB5 };
IPAddress currentIP(192, 168, 1, 50);
IPAddress currentGateway(192, 168, 1, 1);
IPAddress currentSubnet(255, 255, 255, 0);


#define WIRE_BUS_PIN 20 // pin
#define ONE_TEMPERATURE_PRECISION 9


//#define DEBUG 2

// forward Declarations
void onFI_002_PulseIn();
void processModbusCommands();
void refreshModbusToHost();
void refreshModbusRegisters();



#define FLOW_CALC_PERIOD_SECONDS 1 // flow rate calc period
#define FI001_SENSOR_INTERUPT_PIN 3

#define ENABLE_FI0012_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(        \
                                                          FI001_SENSOR_INTERUPT_PIN), \
                                                        onFT_001_PulseIn,             \
                                                        RISING)
#define DISABLE_FI001_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt( \
                                                          FI002_SENSOR_INTERUPT_PIN))

// interrupt pin,calculation period in seconds
/*
FlowMeter FI_001(FI001_SENSOR_INTERUPT_PIN, FLOW_CALC_PERIOD_SECONDS);

DA_DiscreteInput DS_000 = DA_DiscreteInput(11, DA_DiscreteInput::None, true);
DA_DiscreteInput DS_001 = DA_DiscreteInput(43, DA_DiscreteInput::None, true);
DA_DiscreteInput DS_002 = DA_DiscreteInput(45, DA_DiscreteInput::None, true);

// Relay DO
DA_DiscreteOutput DY_000 = DA_DiscreteOutput(22, LOW);
DA_DiscreteOutput DY_001 = DA_DiscreteOutput(23, LOW);
DA_DiscreteOutput DY_003 = DA_DiscreteOutput(24, LOW);
DA_DiscreteOutput DY_004 = DA_DiscreteOutput(25, LOW);
DA_DiscreteOutput DY_005 = DA_DiscreteOutput(26, LOW);
DA_DiscreteOutput DY_006 = DA_DiscreteOutput(27, LOW);
DA_DiscreteOutput DY_007 = DA_DiscreteOutput(30, LOW);
DA_DiscreteOutput DY_008 = DA_DiscreteOutput(31, LOW);

// Solidate DO
DA_DiscreteOutput DY_009 = DA_DiscreteOutput(2, LOW);
DA_DiscreteOutput DY_010 = DA_DiscreteOutput(3, LOW);
DA_DiscreteOutput DY_011 = DA_DiscreteOutput(4, LOW);
*/
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


/*
   void onTemperatureRead()
   {
 * tracePort << "New Sample" << endl;
   for (int i = 0; i < DA_MAX_ONE_WIRE_SENSORS; i++)
   {
    temperatureMgr.enableSensor(i);
 * tracePort << "idx:" << i << "temp:" << temperatureMgr.getTemperature(i) <<
    endl;
   }
   }
 */
    #ifdef DEBUG
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

#endif // ifdef DEBUG


void setup()
{
  Ethernet.begin(boardMAC, currentIP, currentGateway, currentSubnet);
  temperatureMgr.setPollingInterval(5000);
    temperatureMgr.setBlockingRead(false);
  temperatureMgr.scanSensors();
    #ifdef DEBUG
  tracePort->begin(9600);
  temperatureMgr.serialize(tracePort, true);

  temperatureMgr.setOnPollCallBack(onTemperatureRead);

    #endif // ifdef PROCESS_TERMINAL


  temperatureMgr.init();
}

void loop()
{
  MBSlave.MbsRun();
  refreshModbusToHost();
  temperatureMgr.refresh();
  AI_000.refresh();
  //AI_006.refresh();
}

void onFI_002_PulseIn()

{
  //FI_001.handleFlowDetection();
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
MBSlave.MbData[HR_TI_001] = 	(int) (temperatureMgr.getTemperature( 0 ) * 10.0);
MBSlave.MbData[HR_TI_002] = 	(int) (temperatureMgr.getTemperature( 1 ) * 10.0);
MBSlave.MbData[HR_TI_003] = 	(int) (temperatureMgr.getTemperature( 2 ) * 10.0);
MBSlave.MbData[HR_TI_004] = 	(int) (temperatureMgr.getTemperature( 3 ) * 10.0);
MBSlave.MbData[HR_TI_005] = 	(int) (temperatureMgr.getTemperature( 4 ) * 10.0);
MBSlave.MbData[HR_TI_006] = 	(int) (temperatureMgr.getTemperature( 5 ) * 10.0);
MBSlave.MbData[HR_TI_007] = 	(int) (temperatureMgr.getTemperature( 6 ) * 10.0);
MBSlave.MbData[HR_AI_000] = 	AI_000.getRawSample();
MBSlave.MbData[HR_AI_001] = 	AI_001.getRawSample();
MBSlave.MbData[HR_AI_002] = 	AI_002.getRawSample();
MBSlave.MbData[HR_AI_003] = 	AI_003.getRawSample();
MBSlave.MbData[HR_AI_004] = 	AI_004.getRawSample();
MBSlave.MbData[HR_AI_005] = 	AI_005.getRawSample();
MBSlave.MbData[HR_AI_006] = 	AI_006.getRawSample();
MBSlave.MbData[HR_AI_007] = 	AI_007.getRawSample();
//MBSlave.MbData[HR_AI_008] = 	AI_008.getRawSample();
//MBSlave.MbData[HR_AI_009] = 	AI_009.getRawSample();


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
  /*
     MBSlave.GetBit(CW_AI_000_EN);
     MBSlave.GetBit(CW_AI_001_EN);
     MBSlave.GetBit(CW_AI_002_EN);
     MBSlave.GetBit(CW_AI_003_EN);
     MBSlave.GetBit(CW_AI_004_EN);
     MBSlave.GetBit(CW_AI_005_EN);
     MBSlave.GetBit(CW_AI_006_EN);
     MBSlave.GetBit(CW_AI_007_EN);
     MBSlave.GetBit(CW_AY_000_EN);
     MBSlave.GetBit(CW_AY_001_EN);
     MBSlave.GetBit(CW_DS_000_EN);
     MBSlave.GetBit(CW_DS_001_EN);
     MBSlave.GetBit(CW_DS_009_EN);
   */

  MBSlave.GetBit(CW_DY_000);
  MBSlave.GetBit(CW_DY_000_EN);
  MBSlave.GetBit(CW_DY_001);
  MBSlave.GetBit(CW_DY_001_EN);
  MBSlave.GetBit(CW_DY_003);
  MBSlave.GetBit(CW_DY_003_EN);
  MBSlave.GetBit(CW_DY_004);
  MBSlave.GetBit(CW_DY_004_EN);
  MBSlave.GetBit(CW_DY_005);
  MBSlave.GetBit(CW_DY_005_EN);
  MBSlave.GetBit(CW_DY_006);
  MBSlave.GetBit(CW_DY_006_EN);
  MBSlave.GetBit(CW_DY_007);
  MBSlave.GetBit(CW_DY_007_EN);
  MBSlave.GetBit(CW_DY_008);
  MBSlave.GetBit(CW_DY_008_EN);
  MBSlave.GetBit(CW_DY_009);
  MBSlave.GetBit(CW_DY_009_EN);
  MBSlave.GetBit(CW_DY_010);
  MBSlave.GetBit(CW_DY_010_EN);
  MBSlave.GetBit(CW_DY_011);
  MBSlave.GetBit(CW_DY_012_EN);

  //  MBSlave.GetBit(CW_FI_001_EN);
}

void refreshModbusRegisters()
{}
