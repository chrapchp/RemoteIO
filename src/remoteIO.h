/**
 * @file        remoteIO.h
 * @version     0.1
 * @date        20180309
 * @author      pjc

 *
 * @description
 *  Modbus addresses for remote I/O
 *
 * Using arduino modbus implementation @
 * https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
 */


#define DEFAULT_IP_ADDRESS  3232235826  // 192.168.1.50
#define DEFAULT_GATEWAY    3232235777   // 192.168.1.1
#define DEFAULT_SUBNET_MASK  4294967040 // 255.255.255.0
#define DEFAULT_MODBUSBUS_PORT 502
#define DEFAULT_ANALOG_POLL_RATE 5      // seconds


#define ONE_WIRE_PIN 20
#define ONE_TEMPERATURE_PRECISION 9

#define CS_DS_000   80       // Discrete Input 0  (0-24V)
#define CS_DS_001   81       // Discrete Input 1  (0-24V)
#define CS_DS_002   82       // Discrete Input 2  (0-24V)
#define CS_FI_001   83       // Flow Indicator (0-24V) Interrupt Not useful on
                             // its own
#define CW_AI_000_EN   0     // Analog Input 0 Enabled
#define CW_AI_001_EN   1     // Analog Input 1 Enabled
#define CW_AI_002_EN   2     // Analog Input 2 Enabled
#define CW_AI_003_EN   3     // Analog Input 3 Enabled
#define CW_AI_004_EN   4     // Analog Input 4 Enabled
#define CW_AI_005_EN   5     // Analog Input 5 Enabled
#define CW_AI_006_EN   6     // Analog Input 6 Enabled
#define CW_AI_007_EN   7     // Analog Input 7 Enabled
#define CW_AY_000_EN   8     // Analog Output 0 Enabled
#define CW_AY_001_EN   9     // Analog Output 1 Enabled
#define CW_DS_000_EN   10    // Discrete Input 0 Enabled
#define CW_DS_001_EN   11    // Discrete Input 1 Enabled
#define CW_DS_002_EN   12    // Discrete Input 2 Enabled
#define CW_DY_000   13       // Relay Output 0
#define CW_DY_000_EN   14    // Relay Output 0 Enabled
#define CW_DY_001   16       // Relay Output 1
#define CW_DY_001_EN   17    // Relay Output 1 Enabled
#define CW_DY_003   18       // Relay Output 3
#define CW_DY_003_EN   19    // Relay Output 3 Enabled
#define CW_DY_004   20       // Relay Output 4
#define CW_DY_004_EN   21    // Relay Output 4 Enabled
#define CW_DY_005   22       // Relay Output 5
#define CW_DY_005_EN   23    // Relay Output 5 Enabled
#define CW_DY_006   24       // Relay Output 6
#define CW_DY_006_EN   25    // Relay Output 6 Enabled
#define CW_DY_007   26       // Relay Timer Output 1
#define CW_DY_007_EN   27    // Relay Timer Output 1 Enabled
#define CW_DY_008   28       // Relay Timer Output 2
#define CW_DY_008_EN   29    // Relay Timer Output 2 Enabled
#define CW_DY_009   30       // Discrete Output  0 Solid State (2A)
#define CW_DY_009_EN   32    // Discrete Output  0 Solid State Enabled
#define CW_DY_010   31       // Discrete Output  1 Solid State (2A)
#define CW_DY_010_EN   32    // Discrete Output  1 Solid State Enabled
#define CW_DY_011   33       // Discrete Output  2 Solid State (2A)
#define CW_DY_012_EN   34    // Discrete Output  2 Solidate State Enabled
#define CW_FI_001_EN   35    // Flow Indicator Enabled
#define HR_TI_001   30       // Spray Bin 1 Temperature
#define HR_TI_002   31       // Spray Bin 1 Temperature
#define HR_TI_003   32       // Spray Bin 1 Temperature
#define HR_TI_004   33       // Spray Bin 1 Temperature
#define HR_TI_005   34       // Spray Bin 1 Temperature
#define HR_TI_006   35       // Spray Bin 1 Temperature
#define HR_TI_007   36       // Chamber Temperature
#define HR_AI_000   37       // Analog Input 0 Value Raw/Scaled  (0-24V)
#define HR_AI_001   38       // Analog Input 1 Value Raw/Scaled  (0-24V)
#define HR_AI_002   39       // Analog Input 2 Value Raw/Scaled  (0-24V)
#define HR_AI_003   40       // Analog Input 3 Value Raw/Scaled  (0-24V)
#define HR_AI_004   41       // Analog Input 4 Value Raw/Scaled  (0-24V)
#define HR_AI_005   42       // Analog Input 5 Value Raw/Scaled  (0-24V)
#define HR_AI_006   43       // Analog Input 6 Value Raw/Scaled  (0-10V)
#define HR_AI_007   44       // Analog Input 7 Value Raw/Scaled  (0-10V)
#define HR_AI_008   45       // Chamber Humidity
#define HR_AI_009   46       // Chamber CO2
#define HR_DI_002_MP   47    // Default IP Modbus Port
#define HR_DI_003_PR   48    // Default Analog Poll Rate
#define HR_DI_004_OFD   49   // Relay Timer 1 Default Off Duration (s)
#define HR_DI_004_OND   50   // Relay Timer 1 Default On Duration (s)
#define HR_DI_005_OFD   51   // Relay Timer 2 Default Off Duration (s)
#define HR_DI_005_OND   52   // Relay Timer 2 Default On Duration (s)
#define HR_DI_009   53       // Hearbeat Counter ( every 5 seconds)
#define HR_DY_007_CV   54    // Relay Timer Output 1 Current Value
#define HR_DY_007_OFD   55   // Relay Timer Output 1 Off Duration (s)
#define HR_DY_007_OND   56   // Relay Timer Output 1 On Duration (s)
#define HR_DY_008_CV   57    // Relay Timer Output 2 Current Value
#define HR_DY_008_OFD   58   // Relay Timer Output 2 Off Duration (s)
#define HR_DY_008_OND   59   // Relay Timer Output 2 On Duration (s)
#define HR_FI_001_RAW   60   // Flow Indicator Pulse Per Second
#define HR_DI_002_IP   196   // Default IP
#define HR_DI_002_IPG   198  // Default Gateway
#define HR_DI_002_SM   200   // Default Subnet Mask
#define HW_AI_000_EUMAX   10 // Analog Input 0 EU max
#define HW_AI_000_EUMIN   11 // Analog Input 0 EU min
#define HW_AI_001_EUMAX   12 // Analog Input 1 EU max
#define HW_AI_001_EUMIN   13 // Analog Input 1 EU min
#define HW_AI_002_EUMAX   14 // Analog Input 2 EU max
#define HW_AI_002_EUMIN   15 // Analog Input 2 EU min
#define HW_AI_003_EUMAX   16 // Analog Input 3 EU max
#define HW_AI_003_EUMIN   17 // Analog Input 3 EU min
#define HW_AI_004_EUMAX   18 // Analog Input 4 EU max
#define HW_AI_004_EUMIN   19 // Analog Input 4 EU min
#define HW_AI_005_EUMAX   20 // Analog Input 5 EU max
#define HW_AI_005_EUMIN   21 // Analog Input 5 EU min
#define HW_AI_006_EUMAX   22 // Analog Input 6 EU max
#define HW_AI_006_EUMIN   23 // Analog Input 6 EU min
#define HW_AI_007_EUMAX   24 // Analog Input 7 EU max
#define HW_AI_007_EUMIN   25 // Analog Input 7 EU min
#define HW_AY_000_EUMAX   26 // Analog Output 0 EU Min
#define HW_AY_000_EUMIN   27 // Analog Output 0 EU Min
#define HW_AY_001_EUMAX   28 // Analog Output 1 EU Min
#define HW_AY_001_EUMIN   29 // Analog Output 1 EU Min
#define HW_AI_000_PR   30    // Analog Input 0 Poll Rate
#define HW_AI_001_PR   31    // Analog Input 1 Poll Rate
#define HW_AI_002_PR   32    // Analog Input 2 Poll Rate
#define HW_AI_003_PR   33    // Analog Input 3 Poll Rate
#define HW_AI_004_PR   34    // Analog Input 4 Poll Rate
#define HW_AI_005_PR   35    // Analog Input 5 Poll Rate
#define HW_AI_006_PR   36    // Analog Input 6 Poll Rate
#define HW_AI_007_PR   37    // Analog Input 7 Poll Rate
#define HW_AY_000   38       // Analog Output 0 Value (0-10V)
#define HW_AY_001   39       // Analog Output 1 Value (0-10V)
#define HW_DI_001_MP   40    // Current IP Modbus Port
#define HW_DI_001_IP   90    // Current IP Address
#define HW_DI_001_IPG   92   // Current IP Gateway
#define HW_DI_001_SM   94    // Current IP Subnet Mask
#define HW_DI_006_RD   96    // Restore to Defaults
#define HW_DI_008_CT   98    // Current Time Local Time


union
{
  unsigned int regsf[2];
  float        val;
}
bfconvert;

union
{
  unsigned int regsl[2];
  long         val;
}
blconvert;
