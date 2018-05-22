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
#define DEVICE_TYPE 'N' // Nutrient Center N or Growing Chamber G
#define APP_BUILD_DATE 1523558276L

#define BIT_NO_CHANGE 0
#define BIT_RISING_EDGE 1
#define BIT_FALLING_EDGE 2

#ifdef CONTROLLINO_BUILD
  # define CONTROLLINO_MAXI_AUTOMATION
  # include <Controllino.h>
  // Atlas serial 8:1 control pins
  # define S1 CONTROLLINO_PIN_HEADER_DIGITAL_OUT_12
  # define S2 CONTROLLINO_PIN_HEADER_DIGITAL_OUT_13
  # define S3 CONTROLLINO_PIN_HEADER_DIGITAL_OUT_14
#else
  // Atlas serial 8:1 control pins
  # define S1 7
  # define S2 8
  # define S3 9
#endif // ifdef CONTROLLINO_BUILD

const uint8_t s1 = S1;
const uint8_t s2 = S2;
const uint8_t s3 = S3;

// IP addressing used whent nothing stored in EEPROM
#define DEFAULT_IP_ADDRESS   192, 168, 1, 253 // 192.168.1.253
#define DEFAULT_GATEWAY    192, 168, 1, 1     // 192.168.1.1
#define DEFAULT_SUBNET_MASK  255, 255, 255, 0 // 255.255.255.0
#define DEFAULT_MAC_ADDRESS 0xDE, 0xAD, 0xBE, 0x0E, 0x94, 0xB5

// #define DEFAULT_MAC_ADDRESS 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF
#define DEFAULT_PENDING_MAC_ADDRESS 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define DEFAULT_MODBUSBUS_PORT 502
#define DEFAULT_ANALOG_POLL_RATE 2 // seconds


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

#define FT001_SENSOR_INTERUPT_PIN CONTROLLINO_SCREW_TERMINAL_INT_00

#define ENABLE_FT001_SENSOR_INTERRUPTS() attachInterrupt(digitalPinToInterrupt(        \
                                                           FT001_SENSOR_INTERUPT_PIN), \
                                                         onFT_001_PulseIn,             \
                                                         RISING)
#define DISABLE_FT001_SENSOR_INTERRUPTS() detachInterrupt(digitalPinToInterrupt( \
                                                            FT001_SENSOR_INTERUPT_PIN))

// one wire constants
#define WIRE_BUS_PIN 20 // pin
#define ONE_TEMPERATURE_PRECISION 9

// discrete oupute timer defaults
#define DEFAULT_TMR_ON_DURATION  5  // sec
#define DEFAULT_TMR_OFF_DURATION 10 // sec


#define CS_DI_000   80              // Discrete Input 0  (0-24V)
#define CS_DI_001   81              // Discrete Input 1  (0-24V)
#define CS_DI_002   82              // Discrete Input 2  (0-24V)
#define CS_DI_003   83              // Discrete Input 3  (0-24V)
#define CS_FT_001   84              // Flow Indicator (0-24V) Interrupt Not
                                    // useful on its own
#define CS_CI_001   85              // Restore to Defaults (hard)
#define CW_AI_000_EN   0            // Analog Input 0 Enabled
#define CW_AI_001_EN   1            // Analog Input 1 Enabled
#define CW_AI_002_EN   2            // Analog Input 2 Enabled
#define CW_AI_003_EN   3            // Analog Input 3 Enabled
#define CW_AI_004_EN   4            // Analog Input 4 Enabled
#define CW_AI_005_EN   5            // Analog Input 5 Enabled
#define CW_AI_006_EN   6            // Analog Input 6 Enabled
#define CW_AI_007_EN   7            // Analog Input 7 Enabled
#define CW_AY_000_EN   8            // Analog Output 0 Enabled
#define CW_AY_001_EN   9            // Analog Output 1 Enabled
#define CW_CY_001   10              // Restore to Defaults (soft)
#define CW_DY_000   11              // Relay Output 0
#define CW_DY_000_EN   12           // Relay Output 0 Enabled
#define CW_DY_001   13              // Relay Output 1
#define CW_DY_001_EN   14           // Relay Output 1 Enabled
#define CW_DY_002   15              // Relay Output 1
#define CW_DY_002_EN   16           // Relay Output 1 Enabled
#define CW_DY_003   17              // Relay Output 3
#define CW_DY_003_EN   18           // Relay Output 3 Enabled
#define CW_DY_004   19              // Relay Output 4
#define CW_DY_004_EN   20           // Relay Output 4 Enabled
#define CW_DY_005   21              // Relay Output 5
#define CW_DY_005_EN   22           // Relay Output 5 Enabled
#define CW_DY_006   23              // Relay Output 6
#define CW_DY_006_EN   24           // Relay Output 6 Enabled
#define CW_DY_007   25              // Relay Timer Output 1
#define CW_DY_007_EN   26           // Relay Timer Output 1 Enabled
#define CW_DY_007_OFF   27          // Relay Timer Output 1 Force Off
#define CW_DY_007_ON   28           // Relay Timer Output 1 Force On
#define CW_DY_008   29              // Relay Timer Output 2
#define CW_DY_008_EN   30           // Relay Timer Output 2 Enabled
#define CW_DY_008_OFF   31          // Relay Timer Output 2 Force Off
#define CW_DY_008_ON   32           // Relay Timer Output 2 Force On
#define CW_DY_009   33              // Discrete Output  0 Solid State (2A)
#define CW_DY_009_EN   34           // Discrete Output  0 Solid State Enabled
#define CW_DY_010   35              // Discrete Output  1 Solid State (2A)
#define CW_DY_010_EN   36           // Discrete Output  1 Solid State Enabled
#define CW_DY_011   37              // Discrete Output  2 Solid State (2A)
#define CW_DY_011_EN   38           // Discrete Output  2 Solidate State Enabled
#define CW_FI_001_EN   39           // Flow Indicator Enabled
#define CW_DI_000_EN   40           // Discrete Input 0 Enabled
#define CW_DI_001_EN   41           // Discrete Input 1 Enabled
#define CW_DI_002_EN   42           // Discrete Input 2 Enabled
#define CW_DI_003_EN   43           // Discrete Input 3 Enabled
#define CW_CY_002   44              // Re-Scan 1-Wire Temperatures
#define CW_CY_003   45              // Use DHCP IP Addressing
#define CW_CY_004   46              // Reboot Remote I/O
#define CW_CY_005   47              // Persist settings to EEPROM
#define CW_TI_001_EN   48           // 1-Wire Temperature 1 Enabled
#define CW_TI_002_EN   49           // 1-Wire Temperature 2 Enabled
#define CW_TI_003_EN   50           // 1-Wire Temperature 3 Enabled
#define CW_TI_004_EN   51           // 1-Wire Temperature 4 Enabled
#define CW_TI_005_EN   52           // 1-Wire Temperature 5 Enabled
#define CW_TI_006_EN   53           // 1-Wire Temperature 6 Enabled
#define CW_TI_007_EN   54           // 1-Wire Temperature 7 Enabled
#define CW_AI_008_EN   55           // Chamber Humidity Enable
#define CW_AI_009_EN   56           // Chamber CO2 Enable
#define CW_AI_010_EN   57           // pH Enable
#define CW_AI_011_EN   58           // EC Enable
#define CW_CY_006   59              // Change IP using pending values
#define CW_SI_001   60              // Remote I/O Status LED


#define HR_TI_001   40              // 1-Wire Temperature 1
#define HR_TI_002   41              // 1-Wire Temperature 2
#define HR_TI_003   42              // 1-Wire Temperature 3
#define HR_TI_004   43              // 1-Wire Temperature 4
#define HR_TI_005   44              // 1-Wire Temperature 5
#define HR_TI_006   45              // 1-Wire Temperature 6
#define HR_TI_007   46              // 1-Wire Temperature 7
#define HR_AI_000   47              // Analog Input 0 Value Raw/Scaled  (0-24V)
#define HR_AI_001   48              // Analog Input 1 Value Raw/Scaled  (0-24V)
#define HR_AI_002   49              // Analog Input 2 Value Raw/Scaled  (0-24V)
#define HR_AI_003   50              // Analog Input 3 Value Raw/Scaled  (0-24V)
#define HR_AI_004   51              // Analog Input 4 Value Raw/Scaled  (0-24V)
#define HR_AI_005   52              // Analog Input 5 Value Raw/Scaled  (0-24V)
#define HR_AI_006   53              // Analog Input 6 Value Raw/Scaled  (0-10V)
#define HR_AI_007   54              // Analog Input 7 Value Raw/Scaled  (0-10V)
#define HR_AI_008   55              // Chamber Humidity
#define HR_AI_009   56              // Chamber CO2
#define HR_AI_010   57              // pH
#define HR_AI_011   58              // EC
#define HR_DY_007_OFCV   59         // Relay Timer 1 Off  Duration  Current
                                    // Value(s)
#define HR_DY_007_ONCV   60         // Relay Timer 1  On Duration  Current
                                    // Value(s)
#define HR_DY_008_OFCV   61         // Relay Timer 2 Off  Duration  Current
                                    // Value(s)
#define HR_DY_008_ONCV   62         // Relay Timer 2  On Duration  Current
                                    // Value(s)
#define HR_FI_001_RW   63           // Flow Indicator RAW Pulse Per Second
#define HR_KI_001   64              // Hearbeat Counter ( every 5 seconds)
#define HR_KI_002   65              // Remote I/O Status register
#define HR_KI_003   66              // Firmware Version in BCD format
#define HR_CI_006_CV   196          // Current IP Address (decimal format)
#define HR_CI_007_CV   198          // Current IP Gateway (decimal format)
#define HR_CI_008_CV   200          // Current IP Subnet Mask (decimal format)
#define HR_CI_009_CV_H   202        // Current MAC Address High (decimal format)
#define HR_CI_009_CV_L   204        // Current MAC Address Low (decimal format)
#define HR_TI_001_ID_H   206        //  1-Wire Temperature 1 (UID) High
#define HR_TI_001_ID_L   208        //  1-Wire Temperature 1 (UID) Low
#define HR_TI_002_ID_H   210        //  1-Wire Temperature 2 (UID) High
#define HR_TI_002_ID_L   212        //  1-Wire Temperature 2 (UID) Low
#define HR_TI_003_ID_H   214        //  1-Wire Temperature 3 (UID) High
#define HR_TI_003_ID_L   216        //  1-Wire Temperature 3(UID) Low
#define HR_TI_004_ID_H   218        //  1-Wire Temperature 4 (UID) High
#define HR_TI_004_ID_L   220        //  1-Wire Temperature 4 (UID) Low
#define HR_TI_005_ID_H   222        //  1-Wire Temperature 5 (UID) High
#define HR_TI_005_ID_L   224        //  1-Wire Temperature 5 (UID) Low
#define HR_TI_006_ID_H   226        //  1-Wire Temperature 6 (UID) High
#define HR_TI_006_ID_L   228        //  1-Wire Temperature 6 (UID) Low
#define HR_TI_007_ID_H   230        //  1-Wire Temperature 7 (UID) High
#define HR_TI_007_ID_L   232        //  1-Wire Temperature 7 (UID) Low
#define HR_KI_004   234             // App Build date Unix EPOCH
#define HW_DI_000_DT   10           // Discrete Input 0 Debounce Time (ms)
#define HW_DI_001_DT   11           // Discrete Input 1 Debounce Time (ms)
#define HW_DI_002_DT   12           // Discrete Input 2 Debounce Time (ms)
#define HW_AI_000_PR   13           // Analog Input 0 Poll Rate (ms)
#define HW_AI_001_PR   14           // Analog Input 1 Poll Rate (ms)
#define HW_AI_002_PR   15           // Analog Input 2 Poll Rate (ms)
#define HW_AI_003_PR   16           // Analog Input 3 Poll Rate (ms)
#define HW_AI_004_PR   17           // Analog Input 4 Poll Rate (ms)
#define HW_AI_005_PR   18           // Analog Input 5 Poll Rate (ms)
#define HW_AI_006_PR   19           // Analog Input 6 Poll Rate (ms)
#define HW_AI_007_PR   20           // Analog Input 7 Poll Rate (ms)
#define HW_AY_000   21              // Analog Output 0 Value (0-10V)
#define HW_AY_001   22              // Analog Output 1 Value (0-10V)
#define HW_DY_007_OFSP   23         // Relay Timer 1  Off Duration Setpoint (s)
#define HW_DY_007_ONSP   24         // Relay Timer 1  On Duration Setpoint (s)
#define HW_DY_008_OFSP   25         // Relay Timer 2  Off Duration Setpoint (s)
#define HW_DY_008_ONSP   26         // Relay Timer 2  On Duration Setpoint (s)
#define HW_DI_003_DT   27           // Discrete Input 3 Debounce Time (ms)
#define HW_CI_006_PV   90           // Change  IP Address (decimal format)
#define HW_CI_007_PV   92           // Change IP Gateway (decimal format)
#define HW_CI_008_PV   94           // Change IP Subnet Mask (decimal format)
#define HW_CI_009_PV_H   96         // Change MAC Address High (decimal format)
#define HW_CI_009_PV_L   98         // Change MAC Address Low (decimal format)


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
