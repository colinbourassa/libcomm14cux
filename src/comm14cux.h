#ifndef COMM14CUX_H
#define COMM14CUX_H

#include <stdint.h>

#if defined(WIN32)
  #include <windows.h>
#else
  #include <pthread.h>
  #include <errno.h>
#endif

#include "comm14cux_version.h"

// Enhanced debug (bit masks)
#define DEBUG_ERR  1            // print errors
#define DEBUG_WARN 2            // print warnings
#define DEBUG_INFO 4            // information

#define DEBUG_ALL  7            // all

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
  #define dprintf_err
  #define dprintf_warn
  #define dprintf_info
#endif

//! First byte-count threshold for reading
extern const uint16_t _14CUX_ReadCount0;
//! Second byte-count threshold for reading
extern const uint16_t _14CUX_ReadCount1;
//! Third byte-count threshold for reading
extern const uint16_t _14CUX_ReadCount2;
//! Fourth byte-count threshold for reading
extern const uint16_t _14CUX_ReadCount3;
//! Fifth byte-count threshold for reading
extern const uint16_t _14CUX_ReadCount4;

//! 14CUX's table index for the ReadCount1 quantity
extern const uint8_t _14CUX_ReadCount1Value;
//! 14CUX's table index for the ReadCount2 quantity
extern const uint8_t _14CUX_ReadCount2Value;
//! 14CUX's table index for the ReadCount3 quantity
extern const uint8_t _14CUX_ReadCount3Value;
//! 14CUX's table index for the ReadCount4 quantity
extern const uint8_t _14CUX_ReadCount4Value;

//! Baud rate of the 14CUX serial link, rounded down to an integer
extern const int _14CUX_Baud;

//! Starting address of the 14CUX PROM contents in RAM
extern const uint16_t _14CUX_ROMAddress;
//! Size of the used portion of the 14CUX PROM
extern const uint16_t _14CUX_ROMSize;

//! Memory location of Port 1
extern const uint16_t _14CUX_Port1Offset;
//! Memory location of left bank long-term lambda fueling trim
extern const uint16_t _14CUX_LongTermLambdaFuelingTrimLeftOffset;
//! Memory location of right bank long-term lambda fueling trim
extern const uint16_t _14CUX_LongTermLambdaFuelingTrimRightOffset;
//! Memory location of left bank short-term lambda fueling trim
extern const uint16_t _14CUX_ShortTermLambdaFuelingTrimLeftOffset;
//! Memory location of right bank short-term lambda fueling trim
extern const uint16_t _14CUX_ShortTermLambdaFuelingTrimRightOffset;
//! Memory location of fault code block
extern const uint16_t _14CUX_FaultCodesOffset;
//! Memory location of minimum throttle position value
extern const uint16_t _14CUX_ThrottleMinimumPositionOffset;
//! Memory location of main voltage value
extern const uint16_t _14CUX_MainVoltageOffset;
//! Memory location of (low) airflow mass value
extern const uint16_t _14CUX_MassAirflowDirectOffset;
//! Memory location of throttle position value
extern const uint16_t _14CUX_ThrottlePositionOffset;
//! Memory location of coolant temperature value
extern const uint16_t _14CUX_CoolantTempOffset;
//! Memory location of idle bypass motor position value
extern const uint16_t _14CUX_IdleBypassPositionOffset;
//! Memory location of instantaneous engine speed value
extern const uint16_t _14CUX_EngineSpeedInstantaneousOffset;
//! Memory location of filtered engine speed value
extern const uint16_t _14CUX_EngineSpeedFilteredOffset;
//! Memory location of selected gear value
extern const uint16_t _14CUX_TransmissionGearOffset;
//! Memory location of road speed value
extern const uint16_t _14CUX_RoadSpeedOffset;
//! Memory location of fuel temperature value
extern const uint16_t _14CUX_FuelTempOffset;
//! Memory location of RPM limit (in RAM)
extern const uint16_t _14CUX_RPMLimitOffset;
//! Memory location of idle mode bit
extern const uint16_t _14CUX_IdleModeOffset;
//! Memory location of linearized MAF reading
extern const uint16_t _14CUX_MassAirflowLinearOffset;
//! Memory location of target idle speed
extern const uint16_t _14CUX_TargetIdleSpeedOffset;

//! Fixed main voltage coefficient 'A' for very old ECUs (<= 1990)
extern const uint8_t _14CUX_RevAMainVoltageFactorA;
//! Fixed main voltage coefficient 'B' for very old ECUs (<= 1990)
extern const uint8_t _14CUX_RevAMainVoltageFactorB;
//! Fixed main voltage coefficient 'C' for very old ECUs (<= 1990)
extern const uint16_t _14CUX_RevAMainVoltageFactorC;

//! Memory location of first main voltage computation factor
extern const uint16_t _14CUX_RevBMainVoltageFactorAOffset;
//! Memory location of second main voltage computation factor
extern const uint16_t _14CUX_RevBMainVoltageFactorBOffset;
//! Memory location of third main voltage computation factor
extern const uint16_t _14CUX_RevBMainVoltageFactorCOffset;

//! Memory location of first main voltage computation factor
extern const uint16_t _14CUX_RevCMainVoltageFactorAOffset;
//! Memory location of second main voltage computation factor
extern const uint16_t _14CUX_RevCMainVoltageFactorBOffset;
//! Memory location of third main voltage computation factor
extern const uint16_t _14CUX_RevCMainVoltageFactorCOffset;

//! Memory location of Fuel Map 0
extern const uint16_t _14CUX_FuelMap0Offset;

//! Memory location of Fuel Map 1
extern const uint16_t _14CUX_NewFuelMap1Offset;
//! Memory location of Fuel Map 2
extern const uint16_t _14CUX_NewFuelMap2Offset;
//! Memory location of Fuel Map 3
extern const uint16_t _14CUX_NewFuelMap3Offset;
//! Memory location of Fuel Map 4
extern const uint16_t _14CUX_NewFuelMap4Offset;
//! Memory location of Fuel Map 5
extern const uint16_t _14CUX_NewFuelMap5Offset;

//! Memory location of Fuel Map 1
extern const uint16_t _14CUX_OldFuelMap1Offset;
//! Memory location of Fuel Map 2
extern const uint16_t _14CUX_OldFuelMap2Offset;
//! Memory location of Fuel Map 3
extern const uint16_t _14CUX_OldFuelMap3Offset;
//! Memory location of Fuel Map 4
extern const uint16_t _14CUX_OldFuelMap4Offset;
//! Memory location of Fuel Map 5
extern const uint16_t _14CUX_OldFuelMap5Offset;

//! Memory location of the tune number (code revision) in BCD
extern const uint16_t _14CUX_TuneRevisionOffset;

//! Size of each fuel map, in bytes
extern const uint16_t _14CUX_FuelMapSize;
//! Memory location of the current fuel map ID
extern const uint16_t _14CUX_CurrentFuelMapIdOffset;
//! Memory location of the current fuel map row index
extern const uint16_t _14CUX_FuelMapRowIndexOffset;
//! Memory location of the current fuel map column index
extern const uint16_t _14CUX_FuelMapColumnIndexOffset;

//! Memory location of the idle air control motor step count
extern const uint16_t _14CUX_IdleAirControlStepCountOffset;
//! Memory location of the fuel pump timer
extern const uint16_t _14CUX_FuelPumpTimerOffset;

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

} Comm14CUXFaultCodes;

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

} Comm14CUXVersion;

/**
 * Describes the possible gear selections in an automatic gearbox, plus the
 * third possibility of having a manual gearbox (which does not communicate
 * its selected gear to the ECU.)
 */
enum Comm14CUXGear
{
    Comm14CUXGear_NoReading = 0x00,
    Comm14CUXGear_ParkOrNeutral = 0x01,
    Comm14CUXGear_DriveOrReverse = 0x02,
    Comm14CUXGear_ManualGearbox = 0x03
};

/**
 * Describes the two engine banks.
 */
enum Comm14CUXBank
{
    Comm14CUXBank_Left = 0x0,
    Comm14CUXBank_Right = 0x1
};

/**
 * Describes the types of lambda trim for fueling.
 */
enum Comm14CUXLambdaTrimType
{
    Comm14CUXLambdaTrimType_ShortTerm,
    Comm14CUXLambdaTrimType_LongTerm
};

/**
 * Describes the two means of reading a value from the MAF.
 */
enum Comm14CUXAirflowType
{
    Comm14CUXAirflowType_Direct,
    Comm14CUXAirflowType_Linearized
};

/**
 * Describes the two methods of interpreting a throttle position.
 */
enum Comm14CUXThrottlePosType
{
    Comm14CUXThrottlePosType_Absolute,
    Comm14CUXThrottlePosType_Corrected
};

/**
 * Enumerates the revisions of the PROMs, which can have different data
 * offsets (which affects reading the fuel maps.
 */
enum Comm14CUXDataOffsets
{
    //! The revision of the connect ECU has not yet been determined.
    Comm14CUXDataOffsets_Unset = 0x00,
    //! The connected ECU uses the first revision of data offsets
    Comm14CUXDataOffsets_RevA = 0x01,
    //! The connected ECU uses the second revision of data offsets
    Comm14CUXDataOffsets_RevB = 0x02,
    //! The connected ECU uses the third revision of data offsets
    Comm14CUXDataOffsets_RevC = 0x03
};

typedef struct
{
    //! Revision of the connected ECU (affecting fuel map locations)
    enum Comm14CUXDataOffsets promRev;
    //! The coarse address set during the last read operation
    uint16_t lastReadCoarseAddress;
    //! The number of bytes read during the last read operation
    uint8_t lastReadQuantity;
    //! Flag set when the user wishes to cancel a read operation
    uint8_t cancelRead;
    //! Lowest throttle measurement seen so far
    uint16_t lowestThrottleMeasurement;
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
    HANDLE s_mutex;
#else
    //! Descriptor for the serial port device
    int sd;
    //! Lock to prevent multiple simultaneous open/close/read/write operations
    pthread_mutex_t mutex;
#endif

} cuxinfo;

uint16_t swapShort(const uint16_t source);

   void _14cux_init(cuxinfo *info);
uint8_t _14cux_connect(cuxinfo *info, const char *devPath);
   void _14cux_disconnect();
uint8_t _14cux_isConnected();
uint8_t _14cux_readMem(cuxinfo *info, uint16_t addr, uint16_t len, uint8_t* buffer);
uint8_t _14cux_writeMem(cuxinfo *info, uint16_t addr, uint8_t val);
uint8_t _14cux_dumpROM(cuxinfo *info, uint8_t* buffer);
uint8_t _14cux_getFaultCodes(cuxinfo *info, Comm14CUXFaultCodes *faultCodes);

Comm14CUXVersion _14cux_getLibraryVersion();

uint8_t _14cux_getRoadSpeed(cuxinfo *info, uint16_t *roadSpeed);
uint8_t _14cux_getCoolantTemp(cuxinfo *info, int16_t *coolantTemp);
uint8_t _14cux_getFuelTemp(cuxinfo *info, int16_t *fuelTemp);
uint8_t _14cux_getMAFReading(cuxinfo *info, const enum Comm14CUXAirflowType type, float *mafReading);
uint8_t _14cux_getEngineRPM(cuxinfo *info, uint16_t *engineRPM);
uint8_t _14cux_getRPMLimit(cuxinfo *info, uint16_t *rpmLimit);
uint8_t _14cux_getTargetIdle(cuxinfo *info, uint16_t *targetIdleRPM);
uint8_t _14cux_getThrottlePosition(cuxinfo *info, const enum Comm14CUXThrottlePosType type, float *throttlePos);
uint8_t _14cux_getGearSelection(cuxinfo *info, enum Comm14CUXGear *gear);
uint8_t _14cux_getMainVoltage(cuxinfo *info, float *voltage);
uint8_t _14cux_getFuelMap(cuxinfo *info, uint8_t fuelMapId, uint16_t *adjustmentFactor, uint8_t *buffer);
uint8_t _14cux_getCurrentFuelMap(cuxinfo *info, uint8_t *fuelMapId);
uint8_t _14cux_getFuelMapRowIndex(cuxinfo *info, uint8_t *fuelMapRowIndex);
uint8_t _14cux_getFuelMapColumnIndex(cuxinfo *info, uint8_t *fuelMapColIndex);
uint8_t _14cux_getLambdaTrimShort(cuxinfo *info, const enum Comm14CUXBank bank, int16_t *lambdaTrim);
uint8_t _14cux_getLambdaTrimLong(cuxinfo *info, const enum Comm14CUXBank bank, int16_t *lambdaTrim);
uint8_t _14cux_getIdleBypassMotorPosition(cuxinfo *info, float *bypassMotorPos);
uint8_t _14cux_getFuelPumpRelayState(cuxinfo *info, uint8_t *fuelPumpRelayState);
uint8_t _14cux_getTuneRevision(cuxinfo *info, uint16_t *tuneNumber);
uint8_t _14cux_getIdleMode(cuxinfo *info, uint8_t *idleMode);
uint8_t _14cux_isMILOn(cuxinfo *info, uint8_t *milOn);

uint8_t _14cux_clearFaultCodes(cuxinfo *info);
uint8_t _14cux_runFuelPump(cuxinfo *info);
uint8_t _14cux_driveIdleAirControlMotor(cuxinfo *info, uint8_t direction, uint8_t steps);

void _14cux_cancelRead(cuxinfo *info);

#if 0
    void testWrite();
#endif

#endif // COMM14CUX_H

