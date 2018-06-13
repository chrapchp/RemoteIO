/**
 * @file        remoteIO.h
 * @version     0.0.1
 * @date        20180309
 * @author      pjc

 *
 * @description
 *  Modbus addresses for remote I/O
 *
 * Using arduino modbus implementation @
 * https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
 ****************************************************************************
 * History: 2018Mar9 - created (pjc)
 */


// MaX allowed - 15.15.15
// End up encoded as follows:
// XXXXMMMMNNNNPPPP
// i.e. 1.2.3 -> 0000 0001 0010 0011 B -> 291 D -> 123 H
#define APP_MAJOR 0
#define APP_MINOR 0
#define APP_PATCH 1


#if defined(GC_BUILD)
  #define DEVICE_TYPE '1' // NC=1, GC=2, 3 = NC 1 remote I/O 2
#elif defined(NC_BUILD)
  #define DEVICE_TYPE '2' // NC=1, GC=2, 3 = NC 1 remote I/O 2
#else
  #define DEVICE_TYPE '3' // NC=1, GC=2, 3 = NC 1 remote I/O 2

#endif // if defined(GC_BUILD)

#define APP_BUILD_DATE 1523558276L

// detecting modbuss coil  change
#define BIT_NO_CHANGE 0
#define BIT_RISING_EDGE 1
#define BIT_FALLING_EDGE 2


// IP addressing used whent nothing stored in EEPROM
#define DEFAULT_IP_ADDRESS   192, 168, 1, 253 // 192.168.1.253
#define DEFAULT_GATEWAY    192, 168, 1, 1     // 192.168.1.1
#define DEFAULT_SUBNET_MASK  255, 255, 255, 0 // 255.255.255.0
#define DEFAULT_MAC_ADDRESS 0xDE, 0xAD, 0xBE, 0x0E, 0x94, 0xB5

// #define DEFAULT_MAC_ADDRESS 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF
#define DEFAULT_PENDING_MAC_ADDRESS 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define DEFAULT_MODBUSBUS_PORT 502
#define DEFAULT_ANALOG_POLL_RATE 2 // seconds
#define DEFAULT_DI_DEBOUNCE_TIME 250 // ms

// EEPROM addresses
#define EEPROM_CONFIGURED 2        // this value stored at address
                                   // CONFIG_FLAG_ADDR
#define EEPROM_CONFIG_FLAG_ADDR 30 // is 0 if nothing was written
#define EEPROM_IP_ADDR EEPROM_CONFIG_FLAG_ADDR  + sizeof(uint8_t)
#define EEPROM_GATEWAY_ADDR EEPROM_IP_ADDR  + sizeof(uint32_t)
#define EEPROM_SUBNET_ADDR EEPROM_GATEWAY_ADDR + sizeof(uint32_t)
#define EEPROM_MAC_ADDR EEPROM_SUBNET_ADDR + sizeof(uint32_t)
#define EEPROM_ONE_WIRE_MAP EEPROM_MAC_ADDR + sizeof(uint32_t)

#define HEART_BEAT_PERIOD 5000     // ms

// flow meter constants
#define FLOW_CALC_PERIOD_SECONDS 1 // flow rate calc period s

#define XT006_SENSOR_INTERUPT_PIN CONTROLLINO_SCREW_TERMINAL_INT_00

#define ENABLE_XT006_SENSOR_INTERRUPTS() attachInterrupt(digitalPinToInterrupt(        \
                                                           XT006_SENSOR_INTERUPT_PIN), \
                                                         onXT_006_PulseIn,             \
                                                         RISING)
#define DISABLE_XT006_SENSOR_INTERRUPTS() detachInterrupt(digitalPinToInterrupt( \
                                                            XT006_SENSOR_INTERUPT_PIN))

#define XT007_SENSOR_INTERUPT_PIN CONTROLLINO_SCREW_TERMINAL_INT_01
#define ENABLE_XT007_SENSOR_INTERRUPTS() attachInterrupt(digitalPinToInterrupt(        \
                                    XT007_SENSOR_INTERUPT_PIN), \
                                    onXT_007_PulseIn,             \
                                              RISING)
#define DISABLE_XT007_SENSOR_INTERRUPTS() detachInterrupt(digitalPinToInterrupt( XT006_SENSOR_INTERUPT_PIN))


// one wire constants
#define WIRE_BUS_PIN 20 // pin
#define ONE_TEMPERATURE_PRECISION 9

// discrete oupute timer defaults
#define DEFAULT_TMR_ON_DURATION  5  // sec
#define DEFAULT_TMR_OFF_DURATION 10 // sec


#define CS_CI_001   85        // Restore to Defaults (hard)
#define CS_DI_000   97        // Discrete Input 0  (0-24V)
#define CS_DI_001   98        // Discrete Input 1  (0-24V)
#define CS_DI_002   99        // Discrete Input 2  (0-24V)
#define CS_DI_003   100        // Discrete Input 3  (0-24V)
#define CS_DI_004   101        // Discrete Input 4
#define CS_DI_005   102        // Discrete Input 5
#define CS_DI_006   103        // Discrete Input 6
#define CS_DI_007   104        // Discrete Input 7
#define CS_DI_008   105        // Discrete Input 8
#define CS_DI_009   106        // Discrete Input 9
#define CS_XT_006   107        // Flow Indicator (0-24V) Interrupt Not useful on its own
#define CS_XT_007   108        // Flow Indicator (0-24V) Interrupt Not useful on its own


#define CW_DY_000   0        // Relay Output 0
#define CW_DY_001   1        // Relay Output 1
#define CW_DY_002   2        // Relay Output 1
#define CW_DY_003   3        // Relay Output 3
#define CW_DY_004   4        // Relay Output 4
#define CW_DY_005   5        // Relay Output 5
#define CW_DY_006   6        // Relay Output 6
#define CW_DY_007   7        // Relay Output 7
#define CW_DY_008   8        // Relay Output 8
#define CW_DY_009   9        // Relay Output 9
#define CW_DY_010   10        // Discrete Output  0 Solid State (2A)
#define CW_DY_011   11        // Discrete Output  1 Solid State (2A)
#define CW_DY_012   12        // Discrete Output  2 Solid State (2A)
#define CW_DY_013   13        // Discrete Output  3 Solid State (2A)
#define CW_DY_014   14        // Discrete Output  4 Solid State (2A)
#define CW_DY_015   15        // Discrete Output  5 Solid State (2A)
#define CW_DY_016   16        // Discrete Output  6 Solid State (2A)
#define CW_DY_017   17        // Discrete Output  7 Solid State (2A)
#define CW_DY_018   18        // Discrete Output Header 1 (20mA)
#define CW_DY_019   19        // Discrete Output Header 2 (20mA)
#define CW_DY_020   20        // Discrete Output Header 3 (20mA)
#define CW_DY_021   21        // Discrete Output  Header 0 (20mA)
#define CW_AI_000_EN   22        // Analog Input 0 Enabled
#define CW_AI_001_EN   23        // Analog Input 1 Enabled
#define CW_AI_002_EN   24        // Analog Input 2 Enabled
#define CW_AI_003_EN   25        // Analog Input 3 Enabled
#define CW_AI_004_EN   26        // Analog Input 4 Enabled
#define CW_AI_005_EN   27        // Analog Input 5 Enabled
#define CW_AI_006_EN   28        // Analog Input 6 Enabled
#define CW_AY_000_EN   29        // Analog Output 0 Enabled
#define CW_AY_001_EN   30        // Analog Output 1 Enabled
#define CW_CY_001   31        // Restore to Defaults (soft)
#define CW_CY_002   32        // Re-Scan 1-Wire Temperatures
#define CW_CY_003   33        // Use DHCP IP Addressing
#define CW_CY_004   34        // Reboot Remote I/O
#define CW_CY_005   35        // Persist settings to EEPROM
#define CW_CY_006   36        // Change IP using pending values
#define CW_DI_000_EN   37        // Discrete Input 0 Enabled
#define CW_DI_001_EN   38        // Discrete Input 1 Enabled
#define CW_DI_002_EN   39        // Discrete Input 2 Enabled
#define CW_DI_003_EN   40        // Discrete Input 3 Enabled
#define CW_DI_004_EN   41        // Discrete Input 4 Enabled
#define CW_DI_005_EN   42        // Discrete Input 5 Enabled
#define CW_DI_006_EN   43        // Discrete Input 6 Enabled
#define CW_DI_007_EN   44        // Discrete Input 7 Enabled
#define CW_DI_008_EN   45        // Analog Input 8 Enabled
#define CW_DI_009_EN   46        // Analog Input 9 Enabled
#define CW_DY_000_EN   47        // Relay Output 0 Enabled
#define CW_DY_001_EN   48        // Relay Output 1 Enabled
#define CW_DY_002_EN   49        // Relay Output 1 Enabled
#define CW_DY_003_EN   50        // Relay Output 3 Enabled
#define CW_DY_004_EN   51        // Relay Output 4 Enabled
#define CW_DY_005_EN   52        // Relay Output 5 Enabled
#define CW_DY_006_EN   53        // Relay Output 6 Enabled
#define CW_DY_007_EN   54        // Relay Output 7 Enabled
#define CW_DY_008_EN   55        // Relay Output 8 Enabled
#define CW_DY_009_EN   56        // Relay Output 9 Enabled
#define CW_DY_010_EN   57        // Discrete Output  0 Solid State Enabled
#define CW_DY_011_EN   58        // Discrete Output  1 Solid State Enabled
#define CW_DY_012_EN   59        // Discrete Output  2 Solidate State Enabled
#define CW_DY_013_EN   60        // Discrete Output  3 Solid State Enabled
#define CW_DY_014_EN   61        // Discrete Output  4 Solid State Enabled
#define CW_DY_015_EN   62        // Discrete Output  5 Solid State Enabled
#define CW_DY_016_EN   63        // Discrete Output  6 Solid State Enabled
#define CW_DY_017_EN   64        // Discrete Output  7 Solid State Enabled
#define CW_DY_018_EN   65        // Discrete Output Header 1 Enabled
#define CW_DY_019_EN   66        // Discrete Output Header 2 Enabled
#define CW_DY_020_EN   67        // Discrete Output Header 3 Enabled
#define CW_DY_021_EN   68        // Discrete Output Header 0 Enabled
#define CW_FI_001_EN   69        // Flow 1 Indicator Enabled
#define CW_FI_002_EN   70        // Flow 2 Indicator Enabled
#define CW_TI_001_EN   71        // 1-Wire Temperature 1 Enabled
#define CW_TI_002_EN   72        // 1-Wire Temperature 2 Enabled
#define CW_TI_003_EN   73        // 1-Wire Temperature 3 Enabled
#define CW_TI_004_EN   74        // 1-Wire Temperature 4 Enabled
#define CW_TI_005_EN   75        // 1-Wire Temperature 5 Enabled
#define CW_TI_006_EN   76        // 1-Wire Temperature 6 Enabled
#define CW_TI_007_EN   77        // 1-Wire Temperature 7 Enabled

#define HR_TI_001   20        // 1-Wire Temperature 1
#define HR_TI_002   21        // 1-Wire Temperature 2
#define HR_TI_003   22        // 1-Wire Temperature 3
#define HR_TI_004   23        // 1-Wire Temperature 4
#define HR_TI_005   24        // 1-Wire Temperature 5
#define HR_TI_006   25        // 1-Wire Temperature 6
#define HR_TI_007   26        // 1-Wire Temperature 7
#define HR_AI_000   27        // Analog Input 0 Value Raw/Scaled  (0-24V)
#define HR_AI_001   28        // Analog Input 1 Value Raw/Scaled  (0-24V)
#define HR_AI_002   29        // Analog Input 2 Value Raw/Scaled  (0-24V)
#define HR_AI_003   30        // Analog Input 3 Value Raw/Scaled  (0-24V)
#define HR_AI_004   31        // Analog Input 4 Value Raw/Scaled  (0-24V)
#define HR_AI_005   32        // Analog Input 5 Value Raw/Scaled  (0-24V)
#define HR_AI_006   33        // Analog Input 6 Value Raw/Scaled  (0-24V)
#define HR_FI_001_RW   34        // Flow Indicator RAW Pulse Per Second
#define HR_FI_002_RW   35        // Flow Indicator RAW Pulse Per Second
#define HR_KI_001   36        // Hearbeat Counter ( every 5 seconds)
#define HR_KI_002   37        // Remote I/O Status register
#define HR_KI_003   38        // Firmware Version in BCD format
#define HR_XT_001   39        // Serial Place holder 1
#define HR_XT_002   40        // Serial Place holder 2
#define HR_XT_003   41        // Serial Place holder 3
#define HR_XT_004   42        // Serial Place holder 4
#define HR_XT_005   43        // Serial Place holder 5
#define HR_KI_005   44        // Device Type 1=NC, 2=GC, 3=NC Remote IO 2
#define HR_CI_006_CV   82        // Current IP Address (decimal format)
#define HR_CI_007_CV   84        // Current IP Gateway (decimal format)
#define HR_CI_008_CV   86        // Current IP Subnet Mask (decimal format)
#define HR_CI_009_CV_H   88        // Current MAC Address High (decimal format)
#define HR_CI_009_CV_L   90        // Current MAC Address Low (decimal format)
#define HR_KI_004   92        // App Build date Unix EPOCH
#define HR_TI_001_ID_H   94        //  1-Wire Temperature 1 (UID) High
#define HR_TI_001_ID_L   96        //  1-Wire Temperature 1 (UID) Low
#define HR_TI_002_ID_H   98        //  1-Wire Temperature 2 (UID) High
#define HR_TI_002_ID_L   100        //  1-Wire Temperature 2 (UID) Low
#define HR_TI_003_ID_H   102        //  1-Wire Temperature 3 (UID) High
#define HR_TI_003_ID_L   104        //  1-Wire Temperature 3(UID) Low
#define HR_TI_004_ID_H   106        //  1-Wire Temperature 4 (UID) High
#define HR_TI_004_ID_L   108        //  1-Wire Temperature 4 (UID) Low
#define HR_TI_005_ID_H   110        //  1-Wire Temperature 5 (UID) High
#define HR_TI_005_ID_L   112        //  1-Wire Temperature 5 (UID) Low
#define HR_TI_006_ID_H   114        //  1-Wire Temperature 6 (UID) High
#define HR_TI_006_ID_L   116        //  1-Wire Temperature 6 (UID) Low
#define HR_TI_007_ID_H   118        //  1-Wire Temperature 7 (UID) High
#define HR_TI_007_ID_L   120        //  1-Wire Temperature 7 (UID) Low

#define HW_AY_000   10        // Analog Output 0 Value (0-10V)
#define HW_AY_001   11        // Analog Output 1 Value (0-10V)
#define HW_CI_006_PV   50        // Change  IP Address (decimal format)
#define HW_CI_007_PV   52        // Change IP Gateway (decimal format)
#define HW_CI_008_PV   54        // Change IP Subnet Mask (decimal format)
#define HW_CI_009_PV_H   56        // Change MAC Address High (decimal format)
#define HW_CI_009_PV_L   58        // Change MAC Address Low (decimal format)


// for sending and recieve long via modbus
union
{
  uint16_t regsf[2];
  float    val;
}
bfconvert;

// for sending and recieve floats via modbus
union
{
  uint16_t regsl[2];
  long     val;
}
blconvert;

// for managing MAC addresses
union
{
  // uint16_t b64convert[4];
  byte     boardMAC[6];
  uint64_t val;
  uint32_t val32[2];
}
bmacconvert;
