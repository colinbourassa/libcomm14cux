#include <stdint.h>

//! First byte-count threshold for reading
const uint16_t _14CUX_ReadCount0 = 0x0010;
//! Second byte-count threshold for reading
const uint16_t _14CUX_ReadCount1 = 0x0050;
//! Third byte-count threshold for reading
const uint16_t _14CUX_ReadCount2 = 0x0064;
//! Fourth byte-count threshold for reading
const uint16_t _14CUX_ReadCount3 = 0x0190;
//! Fifth byte-count threshold for reading
const uint16_t _14CUX_ReadCount4 = 0x0200;

//! 14CUX's table index for the ReadCount1 quantity
const uint8_t _14CUX_ReadCount1Value = 0x10;
//! 14CUX's table index for the ReadCount2 quantity
const uint8_t _14CUX_ReadCount2Value = 0x11;
//! 14CUX's table index for the ReadCount3 quantity
const uint8_t _14CUX_ReadCount3Value = 0x12;
//! 14CUX's table index for the ReadCount4 quantity
const uint8_t _14CUX_ReadCount4Value = 0x13;

//! Baud rate of the 14CUX serial link, rounded down to an integer
const int _14CUX_Baud = 7812;

//! Starting address of the 14CUX PROM contents in RAM
const uint16_t _14CUX_ROMAddress = 0xC000;
//! Size of the used portion of the 14CUX PROM
const uint16_t _14CUX_ROMSize = 0x4000;

//! Memory location of Port 1
const uint16_t _14CUX_Port1Offset = 0x0002;
//! Memory location of left bank long-term lambda fueling trim
const uint16_t _14CUX_LongTermLambdaFuelingTrimLeftOffset = 0x0042;
//! Memory location of right bank long-term lambda fueling trim
const uint16_t _14CUX_LongTermLambdaFuelingTrimRightOffset = 0x0046;
//! Memory location of left bank short-term lambda fueling trim
const uint16_t _14CUX_ShortTermLambdaFuelingTrimLeftOffset = 0x0065;
//! Memory location of right bank short-term lambda fueling trim
const uint16_t _14CUX_ShortTermLambdaFuelingTrimRightOffset = 0x0067;
//! Memory location of fault code block
const uint16_t _14CUX_FaultCodesOffset = 0x0049;
//! Memory location of minimum throttle position value
const uint16_t _14CUX_ThrottleMinimumPositionOffset = 0x0051;
//! Memory location of main voltage value
const uint16_t _14CUX_MainVoltageOffset = 0x0055;
//! Memory location of (low) airflow mass value
const uint16_t _14CUX_MassAirflowDirectOffset = 0x0057;
//! Memory location of throttle position value
const uint16_t _14CUX_ThrottlePositionOffset = 0x005F;
//! Memory location of coolant temperature value
const uint16_t _14CUX_CoolantTempOffset = 0x006A;
//! Memory location of idle bypass motor position value
const uint16_t _14CUX_IdleBypassPositionOffset = 0x006D;
//! Memory location of instantaneous engine speed value
const uint16_t _14CUX_EngineSpeedInstantaneousOffset = 0x007A;
//! Memory location of filtered engine speed value
const uint16_t _14CUX_EngineSpeedFilteredOffset = 0x007C;
//! Memory location of selected gear value
const uint16_t _14CUX_TransmissionGearOffset = 0x2000;
//! Memory location of road speed value
const uint16_t _14CUX_RoadSpeedOffset = 0x2003;
//! Memory location of fuel temperature value
const uint16_t _14CUX_FuelTempOffset = 0x2006;
//! Memory location of RPM limit (in RAM)
const uint16_t _14CUX_RPMLimitOffset = 0x200C;
//! Memory location of idle mode bit
const uint16_t _14CUX_IdleModeOffset = 0x2047;
//! Memory location of linearized MAF reading
const uint16_t _14CUX_MassAirflowLinearOffset = 0x204D;
//! Memory location of target idle speed
const uint16_t _14CUX_TargetIdleSpeedOffset = 0x2051;

//! Fixed main voltage coefficient 'A' for very old ECUs (<= 1990)
const uint8_t _14CUX_RevAMainVoltageFactorA = 0x64;
//! Fixed main voltage coefficient 'B' for very old ECUs (<= 1990)
const uint8_t _14CUX_RevAMainVoltageFactorB = 0xBD;
//! Fixed main voltage coefficient 'C' for very old ECUs (<= 1990)
const uint16_t _14CUX_RevAMainVoltageFactorC = 0x6180;

//! Memory location of first main voltage computation factor
const uint16_t _14CUX_RevBMainVoltageFactorAOffset = 0xC79B;
//! Memory location of second main voltage computation factor
const uint16_t _14CUX_RevBMainVoltageFactorBOffset = 0xC79C;
//! Memory location of third main voltage computation factor
const uint16_t _14CUX_RevBMainVoltageFactorCOffset = 0xC79D;

//! Memory location of first main voltage computation factor
const uint16_t _14CUX_RevCMainVoltageFactorAOffset = 0xC7C3;
//! Memory location of second main voltage computation factor
const uint16_t _14CUX_RevCMainVoltageFactorBOffset = 0xC7C4;
//! Memory location of third main voltage computation factor
const uint16_t _14CUX_RevCMainVoltageFactorCOffset = 0xC7C5;

//! Memory location of Fuel Map 0
const uint16_t _14CUX_FuelMap0Offset = 0xC000;

//! Memory location of Fuel Map 1
const uint16_t _14CUX_NewFuelMap1Offset = 0xC267;
//! Memory location of Fuel Map 2
const uint16_t _14CUX_NewFuelMap2Offset = 0xC379;
//! Memory location of Fuel Map 3
const uint16_t _14CUX_NewFuelMap3Offset = 0xC48B;
//! Memory location of Fuel Map 4
const uint16_t _14CUX_NewFuelMap4Offset = 0xC59D;
//! Memory location of Fuel Map 5
const uint16_t _14CUX_NewFuelMap5Offset = 0xC6AF;

//! Memory location of Fuel Map 1
const uint16_t _14CUX_OldFuelMap1Offset = 0xC23F;
//! Memory location of Fuel Map 2
const uint16_t _14CUX_OldFuelMap2Offset = 0xC351;
//! Memory location of Fuel Map 3
const uint16_t _14CUX_OldFuelMap3Offset = 0xC463;
//! Memory location of Fuel Map 4
const uint16_t _14CUX_OldFuelMap4Offset = 0xC575;
//! Memory location of Fuel Map 5
const uint16_t _14CUX_OldFuelMap5Offset = 0xC687;

//! Memory location of the tune number (code revision) in BCD
const uint16_t _14CUX_TuneRevisionOffset = 0xFFE9;

//! Size of each fuel map, in bytes
const uint16_t _14CUX_FuelMapSize = 0x80;
//! Memory location of the current fuel map ID
const uint16_t _14CUX_CurrentFuelMapIdOffset = 0x202C;
//! Memory location of the current fuel map row index
const uint16_t _14CUX_FuelMapRowIndexOffset = 0x005B;
//! Memory location of the current fuel map column index
const uint16_t _14CUX_FuelMapColumnIndexOffset = 0x005C;

//! Memory location of the idle air control motor step count
const uint16_t _14CUX_IdleAirControlStepCountOffset = 0x0075;
//! Memory location of the fuel pump timer
const uint16_t _14CUX_FuelPumpTimerOffset = 0x00AF;

