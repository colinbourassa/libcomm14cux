#ifndef COMM14CUX_H
#define COMM14CUX_H

#include <stdint.h>
#include <string>
#ifdef linux
  #include <pthread.h>
#elif defined(WIN32)
  #include <windows.h>
#endif
#include "comm14cux_version.h"

namespace Serial14CUXParams
{
    //! First byte-count threshold for reading
    const uint16_t ReadCount0 = 0x0010;
    //! Second byte-count threshold for reading
    const uint16_t ReadCount1 = 0x0050;
    //! Third byte-count threshold for reading
    const uint16_t ReadCount2 = 0x0064;
    //! Fourth byte-count threshold for reading
    const uint16_t ReadCount3 = 0x0190;
    //! Fifth byte-count threshold for reading
    const uint16_t ReadCount4 = 0x0200;

    //! 14CUX's table index for the ReadCount1 quantity
    const uint8_t ReadCount1Value = 0x10;
    //! 14CUX's table index for the ReadCount2 quantity
    const uint8_t ReadCount2Value = 0x11;
    //! 14CUX's table index for the ReadCount3 quantity
    const uint8_t ReadCount3Value = 0x12;
    //! 14CUX's table index for the ReadCount4 quantity
    const uint8_t ReadCount4Value = 0x13;

    //! Baud rate of the 14CUX serial link, rounded down to an integer
    const int Baud_14CUX = 7812;

    //! Starting address of the 14CUX PROM contents in RAM
    const uint16_t ROMAddress = 0xC000;
    //! Size of the used portion of the 14CUX PROM
    const uint16_t ROMSize = 0x4000;

    //! Memory location of Port 1
    const uint16_t Port1Offset = 0x0002;
    //! Memory location of left bank lambda-based fueling trim
    const uint16_t LambdaFuelingTrimLeftOffset = 0x0042;
    //! Memory location of right bank lambda-based fueling trim
    const uint16_t LambdaFuelingTrimRightOffset = 0x0046;
    //! Memory location of fault code block
    const uint16_t FaultCodesOffset = 0x0049;
    //! Memory location of main voltage value
    const uint16_t MainVoltageOffset = 0x0055;
    //! Memory location of (low) airflow mass value
    const uint16_t MassAirflowOffset = 0x0057;
    //! Memory location of throttle position value
    const uint16_t ThrottlePositionOffset = 0x005F;
    //! Memory location of coolant temperature value
    const uint16_t CoolantTempOffset = 0x006A;
    //! Memory location of idle bypass motor position value
    const uint16_t IdleBypassPositionOffset = 0x006D;
    //! Memory location of instantaneous engine speed value
    const uint16_t EngineSpeedInstantaneousOffset = 0x007A;
    //! Memory location of filtered engine speed value
    const uint16_t EngineSpeedFilteredOffset = 0x007C;
    //! Memory location of selected gear value
    const uint16_t TransmissionGearOffset = 0x2000;
    //! Memory location of road speed value
    const uint16_t RoadSpeedOffset = 0x2003;
    //! Memory location of fuel temperature value
    const uint16_t FuelTempOffset = 0x2006;
    //! Memory location of target idle speed
    const uint16_t TargetIdleSpeedOffset = 0x2151;

    //! Memory location of Fuel Map 0
    const uint16_t FuelMap0Offset = 0xC000;

    //! Memory location of Fuel Map 1
    const uint16_t NewFuelMap1Offset = 0xC267;
    //! Memory location of Fuel Map 2
    const uint16_t NewFuelMap2Offset = 0xC379;
    //! Memory location of Fuel Map 3
    const uint16_t NewFuelMap3Offset = 0xC48B;
    //! Memory location of Fuel Map 4
    const uint16_t NewFuelMap4Offset = 0xC59D;
    //! Memory location of Fuel Map 5
    const uint16_t NewFuelMap5Offset = 0xC6AF;

    //! Memory location of Fuel Map 1
    const uint16_t OldFuelMap1Offset = 0xC23F;
    //! Memory location of Fuel Map 2
    const uint16_t OldFuelMap2Offset = 0xC351;
    //! Memory location of Fuel Map 3
    const uint16_t OldFuelMap3Offset = 0xC463;
    //! Memory location of Fuel Map 4
    const uint16_t OldFuelMap4Offset = 0xC575;
    //! Memory location of Fuel Map 5
    const uint16_t OldFuelMap5Offset = 0xC687;

    //! Size of each fuel map, in bytes
    const uint16_t FuelMapSize = 0x80;
    //! Memory location of the current fuel map ID
    const uint16_t CurrentFuelMapIdOffset = 0x202C;
    //! Memory location of the current fuel map row index
    const uint16_t FuelMapRowIndexOffset = 0x005B;
    //! Memory location of the current fuel map column index
    const uint16_t FuelMapColumnIndexOffset = 0x005C;

    //! Memory location of the idle air control motor step count
    const uint16_t IdleAirControlStepCountOffset = 0x0075;
    //! Memory location of the fuel pump timer
    const uint16_t FuelPumpTimerOffset = 0x00AF;
}

/**
 * Defines the bit-level contents of the memory locations that
 * contain ECU fault codes in the 14CUX.
 */
typedef struct
{
  // Location 0x0049, mask 0x77
  //! Indicates fault with the ECU memory checksum
  uint8_t ECU_Memory_Check    : 1;
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
  //! (Unused)
  uint8_t Spare1              : 1;


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
  //! Indicates fault with the throttle potentiometer
  uint8_t Throttle_Pot_Hi_MAF_Lo : 1;
  //! Indicates fault with the throttle potentiometer
  uint8_t Throttle_Pot_Lo_MAF_Hi : 1;
  //! Indicates fault with the carbon filter purge valve
  uint8_t Purge_Valve_Leak       : 1;


  // Location 0x004B, mask 0x00
  //! (Unused)
  uint8_t Spare3 : 8;


  // Location 0x004C, mask 0xD0
  //! (Unused)
  uint8_t Spare4                   : 4;
  //! Indicates fault with the idle bypass stepper motor
  uint8_t Idle_Valve_Stepper_Motor : 1;
  //! (Unused)
  uint8_t Spare5                   : 1;
  //! Indicates fault with the road speed sensor
  uint8_t Road_Speed_Sensor        : 1;
  //! Indicates fault with the gearbox neutral switch
  uint8_t Neutral_Switch           : 1;


  // Location 0x004D, mask 0x20
  //! (Unused)
  uint8_t Spare6           : 5; 
  //! Indicates fault with the fuel temperature sensor
  uint8_t Fuel_Temp_Sensor : 1;
  //! (Unused)
  uint8_t Spare7           : 2; 


  // Location 0x004E, mask 0xC0
  //! (Unused)
  uint8_t Spare8               : 6;
  //! Indicates that the main battery had been disconnected
  uint8_t Battery_Disconnected : 1;
  //! Indicates that the battery-backed memory of the 14CUX had been cleared
  uint8_t ECM_Memory_Cleared   : 1;

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
 * Enumerates the revisions of the PROMs, which can have different data
 * offsets (which affects reading the fuel maps.
 */
enum Comm14CUXDataOffsets
{
    //! The revision of the connect ECU has not yet been determined.
    Comm14CUXDataOffsets_Unset = 0x00,
    //! The connected ECU uses the old offsets for fuel map data.
    Comm14CUXDataOffsets_Old = 0x01,
    //! The connected ECU uses the new offsets for fuel map data.
    Comm14CUXDataOffsets_New = 0x02
};

/**
 * Allows communications with a 14CUX ECU via its 7812.5bps serial link.
 * This requires a serial device capable of setting a baud rate close to the
 * desired 7812.5.
 */
class Comm14CUX
{
public:
    Comm14CUX();
    ~Comm14CUX();

    bool connect(std::string devPath);
    void disconnect();
    bool isConnected();
    bool readMem(uint16_t addr, uint16_t len, uint8_t* buffer);
    bool writeMem(uint16_t addr, uint8_t val);
    bool dumpROM(uint8_t* buffer);
    bool getFaultCodes(Comm14CUXFaultCodes &faultCodes);
    Comm14CUXVersion getVersion();

    bool getRoadSpeed(uint16_t &roadSpeed);
    bool getCoolantTemp(int16_t &coolantTemp);
    bool getFuelTemp(int16_t &fuelTemp);
    bool getMAFReading(float &mafReading);
    bool getEngineRPM(uint16_t &engineRPM);
    bool getTargetIdle(uint16_t &targetIdleRPM);
    bool getThrottlePosition(float &throttlePos);
    bool getGearSelection(Comm14CUXGear &gear);
    bool getMainVoltage(float &voltage);
    bool getFuelMap(uint8_t fuelMapId, uint16_t &adjustmentFactor, uint8_t *buffer);
    bool getCurrentFuelMap(uint8_t &fuelMapId);
    bool getFuelMapRowIndex(uint8_t &fuelMapRowIndex);
    bool getFuelMapColumnIndex(uint8_t &fuelMapColIndex);
    bool getLambdaTrim(Comm14CUXBank bank, int16_t &lambdaTrim);
    bool getIdleBypassMotorPosition(float &bypassMotorPos);
    bool getFuelPumpRelayState(bool &fuelPumpRelayState);

    bool clearFaultCodes();
    bool runFuelPump();
    bool driveIdleAirControlMotor(uint8_t direction, uint8_t steps);

    void cancelRead();

private:
    int16_t readSerialBytes(uint8_t *buffer, uint16_t quantity);
    int16_t writeSerialBytes(uint8_t *buffer, uint16_t quantity);
    bool setReadCoarseAddr(uint16_t addr, uint16_t len);
    bool setWriteCoarseAddr(uint16_t addr);
    bool setCoarseAddr(uint16_t addr, uint16_t len);
    uint16_t getByteCountForNextRead(uint16_t len, uint16_t bytesRead);
    bool sendReadCmd(uint16_t addr, uint16_t len, bool lastByteOnly);
    bool openSerial(std::string devPath);
    double hyperbolicOffsetModel(double count);
    double nistHahnModel(double count);

    //! Revision of the connected ECU (affecting fuel map locations)
    Comm14CUXDataOffsets m_promRev;
    //! The coarse address set during the last read operation
    uint16_t m_lastReadCoarseAddress;
    //! The number of bytes read during the last read operation
    uint8_t m_lastReadQuantity;
    //! Flag set when the user wishes to cancel a read operation
    bool m_cancelRead;

#ifdef linux

    //! Descriptor for the serial port device
    int sd;

    //! Lock to prevent multiple simultaneous open/close/read/write operations
    pthread_mutex_t s_mutex;

#elif defined(WIN32)

    //! Descriptor for the serial port device
    HANDLE sd;

    //! Lock to prevent multiple simultaneous open/close/read/write operations
    HANDLE s_mutex;

#endif

};

#endif // COMM14CUX_H

