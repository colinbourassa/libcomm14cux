#ifndef COMM14CUX_H
#define COMM14CUX_H

/** \file comm14cux.h
 * Header file defining the libcomm14cux functions, structs, and enums.
 */

#include <stdint.h>
#include <stdbool.h>

#if defined(WIN32)
  #include <windows.h>
#else
  #include <pthread.h>
  #include <errno.h>
#endif

/** Flag used to enable logging (to stdio) of error conditions */
#define DEBUG_ERR  1
/** Flag used to enable logging (to stdio) of warning conditions */
#define DEBUG_WARN 2
/** Flag used to enable logging (to stdio) of information */
#define DEBUG_INFO 4
/** Combination of all the available debug flags */
#define DEBUG_ALL  7

//#define DEBUG_P DEBUG_ALL

#ifdef DEBUG_P
  #include <stdio.h>

  #if (DEBUG_P & DEBUG_ERR)
    #define dprintf_err   printf
  #else
    #define dprintf_err
  #endif

  #if (DEBUG_P & DEBUG_WARN)
    #define dprintf_warn  printf
  #else
    #define dprintf_warn
  #endif

  #if (DEBUG_P & DEBUG_INFO)
    #define dprintf_info  printf
  #else
    #define dprintf_info
  #endif
#else
  /** Placeholder macro for error-condition debug logging */
  #define dprintf_err
  /** Placeholder macro for warning-condition debug logging */
  #define dprintf_warn
  /** Placeholder macro for informational debug logging */
  #define dprintf_info
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Baud rate of the 14CUX serial port */
#define C14CUX_BAUD 7812

/**
 * Defines the possible byte quantities that may be requested
 * via the serial protocol.
 */
enum c14cux_readcount
{
    //! First byte-count threshold for reading
    C14CUX_ReadCount0 = 0x0010,
    //! Second byte-count threshold for reading
    C14CUX_ReadCount1 = 0x0050,
    //! Third byte-count threshold for reading
    C14CUX_ReadCount2 = 0x0064,
    //! Fourth byte-count threshold for reading
    C14CUX_ReadCount3 = 0x0190,
    //! Fifth byte-count threshold for reading
    C14CUX_ReadCount4 = 0x0200
};

/**
 * Defines the software values that correspond to the possible
 * byte quantities that may be requested via the serial protocol.
 */
enum c14cux_readcount_value
{
    //! 14CUX's table index for the ReadCount1 quantity
    C14CUX_ReadCount1Value = 0x10,
    //! 14CUX's table index for the ReadCount2 quantity
    C14CUX_ReadCount2Value = 0x11,
    //! 14CUX's table index for the ReadCount3 quantity
    C14CUX_ReadCount3Value = 0x12,
    //! 14CUX's table index for the ReadCount4 quantity
    C14CUX_ReadCount4Value = 0x13
};

/**
 * Sizes in bytes of certain fixed-size data structures within the ROM.
 */
enum c14cux_data_size
{
    //! Size of the used portion of the 14CUX PROM
    C14CUX_ROMSize = 0x4000,
    //! Size of each fuel map, in bytes
    C14CUX_FuelMapSize = 0x80
};

/**
 * Important memory offsets in the ECU's address space.
 */
enum c14cux_memory_offset
{
    //! Starting address of the 14CUX PROM contents in RAM
    C14CUX_ROMAddress = 0xC000,
    //! Memory location of Port 1
    C14CUX_Port1Offset = 0x0002,
    //! Memory location of left bank long-term lambda fueling trim
    C14CUX_LongTermLambdaFuelingTrimLeftOffset = 0x0042,
    //! Memory location of right bank long-term lambda fueling trim
    C14CUX_LongTermLambdaFuelingTrimRightOffset = 0x0046,
    //! Memory location of left bank short-term lambda fueling trim
    C14CUX_ShortTermLambdaFuelingTrimLeftOffset = 0x0065,
    //! Memory location of right bank short-term lambda fueling trim
    C14CUX_ShortTermLambdaFuelingTrimRightOffset = 0x0067,
    //! Memory location of fault code block
    C14CUX_FaultCodesOffset = 0x0049,
    //! Memory location of minimum throttle position value
    C14CUX_ThrottleMinimumPositionOffset = 0x0051,
    //! Memory location of main voltage value
    C14CUX_MainVoltageOffset = 0x0055,
    //! Memory location of (low) airflow mass value
    C14CUX_MassAirflowDirectOffset = 0x0057,
    //! Memory location of throttle position value
    C14CUX_ThrottlePositionOffset = 0x005F,
    //! Memory location of coolant temperature value
    C14CUX_CoolantTempOffset = 0x006A,
    //! Memory location of idle bypass motor position value
    C14CUX_IdleBypassPositionOffset = 0x006D,
    //! Memory location of instantaneous engine speed value
    C14CUX_EngineSpeedInstantaneousOffset = 0x007A,
    //! Memory location of filtered engine speed value
    C14CUX_EngineSpeedFilteredOffset = 0x007C,
    //! Memory location of purge valve timer value
    C14CUX_PurgeValveStateOffset = 0x0096,
    //! Memory location of selected gear value
    C14CUX_TransmissionGearOffset = 0x2000,
    //! Memory location of road speed value
    C14CUX_RoadSpeedOffset = 0x2003,
    //! Memory location of fuel temperature value
    C14CUX_FuelTempOffset = 0x2006,
    //! Memory location of RPM limit (in RAM)
    C14CUX_RPMLimitOffset = 0x200C,
    //! Memory location of idle mode bit
    C14CUX_IdleModeOffset = 0x2047,
    //! Memory location of linearized MAF reading
    C14CUX_MassAirflowLinearOffset = 0x204D,
    //! Memory location of target idle speed
    C14CUX_TargetIdleSpeedOffset = 0x2051,

    //! Memory location of first main voltage computation factor
    C14CUX_RevBMainVoltageFactorAOffset = 0xC79B,
    //! Memory location of second main voltage computation factor
    C14CUX_RevBMainVoltageFactorBOffset = 0xC79C,
    //! Memory location of third main voltage computation factor
    C14CUX_RevBMainVoltageFactorCOffset = 0xC79D,

    //! Memory location of first main voltage computation factor
    C14CUX_RevCMainVoltageFactorAOffset = 0xC7C3,
    //! Memory location of second main voltage computation factor
    C14CUX_RevCMainVoltageFactorBOffset = 0xC7C4,
    //! Memory location of third main voltage computation factor
    C14CUX_RevCMainVoltageFactorCOffset = 0xC7C5,

    //! Memory location of Fuel Map 0
    C14CUX_FuelMap0Offset = 0xC000,

    //! Memory location of Fuel Map 1
    C14CUX_NewFuelMap1Offset = 0xC267,
    //! Memory location of Fuel Map 2
    C14CUX_NewFuelMap2Offset = 0xC379,
    //! Memory location of Fuel Map 3
    C14CUX_NewFuelMap3Offset = 0xC48B,
    //! Memory location of Fuel Map 4
    C14CUX_NewFuelMap4Offset = 0xC59D,
    //! Memory location of Fuel Map 5
    C14CUX_NewFuelMap5Offset = 0xC6AF,

    //! Memory location of Fuel Map 1
    C14CUX_OldFuelMap1Offset = 0xC23F,
    //! Memory location of Fuel Map 2
    C14CUX_OldFuelMap2Offset = 0xC351,
    //! Memory location of Fuel Map 3
    C14CUX_OldFuelMap3Offset = 0xC463,
    //! Memory location of Fuel Map 4
    C14CUX_OldFuelMap4Offset = 0xC575,
    //! Memory location of Fuel Map 5
    C14CUX_OldFuelMap5Offset = 0xC687,

    //! Memory location of the tune number (code revision) in BCD
    C14CUX_TuneRevisionOffset = 0xFFE9,
    //! Memory location of the current fuel map ID
    C14CUX_CurrentFuelMapIdOffset = 0x202C,
    //! Memory location of the current fuel map row index
    C14CUX_FuelMapRowIndexOffset = 0x005B,
    //! Memory location of the current fuel map column index
    C14CUX_FuelMapColumnIndexOffset = 0x005C,

    //! Memory location of the idle air control motor step count
    C14CUX_IdleAirControlStepCountOffset = 0x0075,
    //! Memory location of the fuel pump timer
    C14CUX_FuelPumpTimerOffset = 0x00AF
};

/**
 * Hardcoded coefficients that are used in calculations based on the
 * measured main voltage.
 */
enum c14cux_main_voltage_factor
{
    //! Fixed main voltage coefficient 'A' for very old ECUs (<= 1990)
    C14CUX_RevAMainVoltageFactorA = 0x64,
    //! Fixed main voltage coefficient 'B' for very old ECUs (<= 1990)
    C14CUX_RevAMainVoltageFactorB = 0xBD,
    //! Fixed main voltage coefficient 'C' for very old ECUs (<= 1990)
    C14CUX_RevAMainVoltageFactorC = 0x6180
};

/**
 * Defines the bit-level contents of the memory locations that
 * contain ECU fault codes in the 14CUX.
 */
typedef struct
{
  // Location 0x0049, mask 0x77
  //! Indicates fault with the ECU memory checksum
  uint8_t PROM_Checksum_Failure : 1;
  //! Indicates fault with the left oxygen sensor
  uint8_t Lambda_Sensor_Left  : 1;
  //! Indicates fault with the right oxygen sensor
  uint8_t Lambda_Sensor_Right : 1;
  //! (Unused)
  uint8_t Spare0              : 1;
  //! Indicates a misfire in the left bank
  uint8_t Misfire_Left_Bank   : 1;
  //! Indicates a misfire in the right bank
  uint8_t Misfire_Right_Bank  : 1;
  //! Indicates a fault with the airflow meter
  uint8_t Airflow_Meter       : 1;
  //! Indicates that the tune resistor is out of range
  uint8_t Tune_Resistor_Out_of_Range : 1;


  // Location 0x004A, mask 0xFD
  //! Indicates fault in the left bank of fuel injectors
  uint8_t Injector_Left_Bank     : 1;
  //! (Unused) */
  uint8_t Spare2                 : 1;
  //! Indicates fault in the right bank of fuel injectors
  uint8_t Injector_Right_Bank    : 1;
  //! Indicates fault with the coolant temperature sensor
  uint8_t Coolant_Temp_Sensor    : 1;
  //! Indicates fault with the throttle potentiometer
  uint8_t Throttle_Pot           : 1;
  //! Indicates that the MAF read low while the throttle pot read high
  uint8_t Throttle_Pot_Hi_MAF_Lo : 1;
  //! Indicates that the MAF read high while the throttle pot read low
  uint8_t Throttle_Pot_Lo_MAF_Hi : 1;
  //! Indicates fault with the carbon filter purge valve
  uint8_t Purge_Valve_Leak       : 1;


  // Location 0x004B, mask 0x00
  //! (Unused)
  uint8_t Spare3                 : 1;
  //! Indicates that the fuel mix is too lean
  uint8_t Mixture_Too_Lean       : 1;
  //! (Unused)
  uint8_t Spare4                 : 1;
  //! Indicates an air leak in the intake path
  uint8_t Intake_Air_Leak        : 1;
  //! (Unused)
  uint8_t Spare5                 : 4;


  // Location 0x004C, mask 0xD0
  //! Indicates a low fuel pressure fault
  uint8_t Low_Fuel_Pressure        : 1;
  //! (Unused)
  uint8_t Spare6                   : 3;
  //! Indicates fault with the idle bypass stepper motor
  uint8_t Idle_Valve_Stepper_Motor : 1;
  //! (Unused)
  uint8_t Spare7                   : 1;
  //! Indicates fault with the road speed sensor
  uint8_t Road_Speed_Sensor        : 1;
  //! Indicates fault with the gearbox neutral switch
  uint8_t Neutral_Switch           : 1;


  // Location 0x004D, mask 0x20
  //! (Unused)
  uint8_t Spare8                        : 4; 
  //! Indicates ambiguity between a low-fuel-pressure fault and an air-leak fault
  uint8_t Low_Fuel_Pressure_or_Air_Leak : 1;
  //! Indicates fault with the fuel temperature sensor
  uint8_t Fuel_Temp_Sensor              : 1;
  //! (Unused)
  uint8_t Spare9                        : 2; 


  // Location 0x004E, mask 0xC0
  //! (Unused)
  uint8_t Spare10              : 6;
  //! Indicates that the main battery had been disconnected
  uint8_t Battery_Disconnected : 1;
  //! Indicates that the battery-backed memory of the 14CUX had been cleared
  uint8_t RAM_Checksum_Failure : 1;

} c14cux_faultcodes;

/**
 * Major/minor/patch version numbers for this build of the library
 */
typedef struct
{
  //! Major version number
  uint8_t major;
  //! Minor version number
  uint8_t minor;
  //! Patch version number
  uint8_t patch;

} c14cux_version;

/**
 * Describes the possible gear selections in an automatic gearbox, plus the
 * third possibility of having a manual gearbox (which does not communicate
 * its selected gear to the ECU.)
 */
enum c14cux_gear
{
    C14CUX_Gear_NoReading = 0x00,
    C14CUX_Gear_ParkOrNeutral = 0x01,
    C14CUX_Gear_DriveOrReverse = 0x02,
    C14CUX_Gear_ManualGearbox = 0x03
};

/**
 * Describes the two engine banks.
 */
enum c14cux_bank
{
    C14CUX_Bank_Left = 0x0,
    C14CUX_Bank_Right = 0x1
};

/**
 * Describes the types of lambda trim for fueling.
 */
enum c14cux_lambda_trim_type
{
    C14CUX_LambdaTrimType_ShortTerm,
    C14CUX_LambdaTrimType_LongTerm
};

/**
 * Describes the two means of reading a value from the MAF.
 */
enum c14cux_airflow_type
{
    C14CUX_AirflowType_Direct,
    C14CUX_AirflowType_Linearized
};

/**
 * Describes the two methods of interpreting a throttle position.
 */
enum c14cux_throttle_pos_type
{
    C14CUX_ThrottlePosType_Absolute,
    C14CUX_ThrottlePosType_Corrected
};

/**
 * Enumerates the revisions of the PROMs, which can have different data
 * offsets (which affects reading the fuel maps.
 */
enum c14cux_data_offset_rev
{
    //! The revision of the connect ECU has not yet been determined.
    C14CUX_DataOffsets_Unset = 0x00,
    //! The connected ECU uses the first revision of data offsets
    C14CUX_DataOffsets_RevA = 0x01,
    //! The connected ECU uses the second revision of data offsets
    C14CUX_DataOffsets_RevB = 0x02,
    //! The connected ECU uses the third revision of data offsets
    C14CUX_DataOffsets_RevC = 0x03
};

/**
 * Enumerates the three possible states of the carbon canister purge valve.
 */
enum c14cux_purge_valve_state
{
    C14CUX_PurgeValveState_Closed = 0,
    C14CUX_PurgeValveState_Toggling = 1,
    C14CUX_PurgeValveState_Open = 2
};

/**
 * Contains information about the state of the current connection to the ECU.
 */
typedef struct
{
    //! Revision of the connected ECU (affecting fuel map locations)
    enum c14cux_data_offset_rev promRev;
    //! The coarse address set during the last read operation
    uint16_t lastReadCoarseAddress;
    //! The number of bytes read during the last read operation
    uint8_t lastReadQuantity;
    //! Flag set when the user wishes to cancel a read operation
    bool cancelRead;
    //! Factor involved in computations with the main voltage
    uint8_t voltageFactorA;
    //! Factor involved in computations with the main voltage
    uint8_t voltageFactorB;
    //! Factor involved in computations with the main voltage
    uint16_t voltageFactorC;

#if defined(WIN32)
    //! Descriptor for the serial port device
    HANDLE sd;
    //! Lock to prevent multiple simultaneous open/close/read/write operations
    HANDLE mutex;
#else
    //! Descriptor for the serial port device
    int sd;
    //! Lock to prevent multiple simultaneous open/close/read/write operations
    pthread_mutex_t mutex;
#endif

} c14cux_info;

uint16_t swapShort(const uint16_t source);

void c14cux_init(c14cux_info* info);
void c14cux_cleanup(c14cux_info* info);
bool c14cux_connect(c14cux_info* info, const char *devPath);
void c14cux_disconnect(c14cux_info* info);
bool c14cux_isConnected(c14cux_info* info);
bool c14cux_readMem(c14cux_info* info, uint16_t addr, uint16_t len, uint8_t* buffer);
bool c14cux_writeMem(c14cux_info* info, uint16_t addr, uint8_t val);
bool c14cux_dumpROM(c14cux_info* info, uint8_t* buffer);
bool c14cux_getFaultCodes(c14cux_info* info, c14cux_faultcodes* faultCodes);

c14cux_version c14cux_getLibraryVersion();

bool c14cux_getRoadSpeed(c14cux_info* info, uint16_t* roadSpeed);
bool c14cux_getCoolantTemp(c14cux_info* info, int16_t* coolantTemp);
bool c14cux_getFuelTemp(c14cux_info* info, int16_t* fuelTemp);
bool c14cux_getMAFReading(c14cux_info* info, const enum c14cux_airflow_type type, float* mafReading);
bool c14cux_getEngineRPM(c14cux_info* info, uint16_t* engineRPM);
bool c14cux_getRPMLimit(c14cux_info* info, uint16_t* rpmLimit);
bool c14cux_getTargetIdle(c14cux_info* info, uint16_t* targetIdleRPM);
bool c14cux_getThrottlePosition(c14cux_info* info, const enum c14cux_throttle_pos_type type, float* throttlePos);
bool c14cux_getGearSelection(c14cux_info* info, enum c14cux_gear* gear);
bool c14cux_getMainVoltage(c14cux_info* info, float* voltage);
bool c14cux_getFuelMap(c14cux_info* info, uint8_t fuelMapId, uint16_t* adjustmentFactor, uint8_t* buffer);
bool c14cux_getCurrentFuelMap(c14cux_info* info, uint8_t* fuelMapId);
bool c14cux_getFuelMapRowIndex(c14cux_info* info, uint8_t* fuelMapRowIndex);
bool c14cux_getFuelMapColumnIndex(c14cux_info* info, uint8_t* fuelMapColIndex);
bool c14cux_getLambdaTrimShort(c14cux_info* info, const enum c14cux_bank bank, int16_t* lambdaTrim);
bool c14cux_getLambdaTrimLong(c14cux_info* info, const enum c14cux_bank bank, int16_t* lambdaTrim);
bool c14cux_getIdleBypassMotorPosition(c14cux_info* info, float* bypassMotorPos);
bool c14cux_getFuelPumpRelayState(c14cux_info* info, bool* fuelPumpRelayState);
bool c14cux_getTuneRevision(c14cux_info* info, uint16_t *tuneNumber);
bool c14cux_getIdleMode(c14cux_info* info, bool* idleMode);
bool c14cux_getPurgeValveState(c14cux_info* info, enum c14cux_purge_valve_state* state);
bool c14cux_isMILOn(c14cux_info* info, bool* milOn);

bool c14cux_clearFaultCodes(c14cux_info* info);
bool c14cux_runFuelPump(c14cux_info* info);
bool c14cux_driveIdleAirControlMotor(c14cux_info* info, uint8_t direction, uint8_t steps);

void c14cux_cancelRead(c14cux_info* info);

#if 0
    void testWrite();
#endif

/* Closing brace for 'extern "C"' */
#ifdef __cplusplus
}
#endif

#endif // COMM14CUX_H

