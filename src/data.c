// libcomm14cux - a communications library for the Lucas 14CUX ECU
//
// data.c: This file contains routines that fetch and format data
//         from the ECU.

#if defined(WIN32) && defined(linux)
#error "Only one of 'WIN32' or 'linux' may be defined."
#endif

#include <math.h>
#include <stdlib.h>

#if defined(WIN32)
  #include <windows.h>
#endif

#include "comm14cux.h"
#include "comm14cux_internal.h"

const int16_t temperature_adc_to_degrees_f[256] =
{ 266,264,260,258,255,253,249,248,244,242,239,237,233,231,228,226,
  222,221,219,217,215,213,212,210,208,206,204,203,201,199,197,195,
  194,192,190,188,186,185,183,181,179,177,176,174,172,170,168,167,
  165,163,161,159,158,158,156,154,152,150,149,147,145,145,143,141,
  140,138,138,136,136,134,132,132,131,131,129,129,127,125,125,123,
  123,122,122,120,120,118,118,116,116,114,114,113,113,111,111,109,
  109,107,107,105,105,104,104,102,102,100,100, 98, 98, 96, 96, 95,
  95, 95, 93, 93, 91, 91, 89, 89, 87, 87, 86, 86, 84, 84, 82, 82,
  82, 82, 80, 80, 78, 78, 78, 77, 77, 75, 75, 73, 73, 73, 71, 71,
  71, 69, 69, 68, 68, 66, 66, 64, 64, 62, 62, 60, 60, 59, 59, 57,
  57, 57, 55, 55, 53, 53, 51, 51, 50, 50, 48, 48, 46, 46, 44, 44,
  44, 44, 42, 42, 42, 41, 41, 41, 39, 39, 37, 37, 37, 35, 35, 35,
  35, 33, 33, 32, 32, 31, 31, 29, 29, 27, 27, 25, 25, 23, 23, 22,
  22, 22, 20, 20, 18, 18, 18, 16, 16, 14, 14, 13, 13, 13, 11, 11,
  11,  9,  9,  7,  7,  5,  5,  4,  4,  2,  2,  0,  0, -2, -2, -4,
  -4, -4, -5, -5, -7, -7, -7, -9, -9, -9,-11,-11,-11,-13,-13,-13 };

/**
 * Dumps the entire contents of the 14CUX ROM and places it in a buffer.
 * @param info State information for the active connection.
 * @param buffer Buffer of at least 16KB.
 * @return True when the ROM was read and the buffer was successfully
 *   populated; false otherwise.
 */
bool c14cux_dumpROM(c14cux_info* info, uint8_t* buffer)
{
    return c14cux_readMem(info, C14CUX_ROMAddress, C14CUX_ROMSize, buffer);
}

/**
 * Gets the measured road speed of the vehicle in miles per hour.
 * @param info State information for the active connection.
 * @param roadSpeed Set to road speed in miles per hour (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool c14cux_getRoadSpeed(c14cux_info* info, uint8_t* roadSpeed)
{
    uint8_t kphSpeed = 0;
    float floatSpeed = 0.0;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_RoadSpeedOffset, 1, &kphSpeed))
    {
        // convert KPH to MPH
        floatSpeed = (float)kphSpeed * 0.621371192;
        *roadSpeed = (uint8_t)floatSpeed;
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the temperature of the engine coolant in degrees Fahrenheit.
 * @param info State information for the active connection.
 * @param coolantTemp Set to the coolant temperature (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool c14cux_getCoolantTemp(c14cux_info* info, int16_t* coolantTemp)
{
    uint8_t count = 0;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_CoolantTempOffset, 1, &count))
    {
        *coolantTemp = (int16_t)(c14cux_hyperbolicOffsetModel(count));
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the temperature of the fuel in degrees Fahrenheit.
 * @param info State information for the active connection.
 * @param fuelTemp Set to the fuel temperature (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool c14cux_getFuelTemp(c14cux_info* info, int16_t* fuelTemp)
{
    uint8_t count = 0;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_FuelTempOffset, 1, &count))
    {
        *fuelTemp = (int16_t)(c14cux_hyperbolicOffsetModel(count));
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the mass airflow reading at the intake.
 * @param info State information for the active connection.
 * @param type Type of reading to take. A "Direct" reading changes linearly
 *   with voltage but logarithmically with airflow, and "Linearized" changes
 *   linearly with airflow.
 * @param mafReading Set to MAF measurement as a percentage of the highest
 *   possible measurement (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool c14cux_getMAFReading(c14cux_info* info, const enum c14cux_airflow_type type, float *mafReading)
{
    uint16_t maf = 0;
    bool retVal = false;

    if ((type == C14CUX_AirflowType_Direct) &&
        c14cux_readMem(info, C14CUX_MassAirflowDirectOffset, 2, (uint8_t*)&maf))
    {
        maf = swapShort(maf);

        // sanity check the reading
        if (maf <= 1023)
        {
            *mafReading = maf / 1023.0;
            retVal = true;
        }
    }
    else if ((type == C14CUX_AirflowType_Linearized) &&
             c14cux_readMem(info, C14CUX_MassAirflowLinearOffset, 2, (uint8_t*)&maf))
    {
        maf = swapShort(maf);

        // sanity check the reading
        if (maf <= 17290)
        {
            *mafReading = maf / 17290.0;
            retVal = true;
        }
    }

    return retVal;
}

/**
 * Gets the idle bypass motor position as a percentage of the widest possible
 * opening.
 * @param info State information for the active connection.
 * @param bypassMotorPos Set to the idle bypass position as a percentage of
 *   wide-open
 * @return True if successfully read; false otherwise
 */
bool c14cux_getIdleBypassMotorPosition(c14cux_info* info, float* bypassMotorPos)
{
    uint8_t pos = 0;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_IdleBypassPositionOffset, 1, &pos))
    {
        // 180 counts is fully closed; it shouldn't be reading any higher than that
        if (pos > 180)
        {
            pos = 180;
        }

        *bypassMotorPos = (180 - pos) / 180.0;
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the engine (crankshaft) speed in revolutions per minute.
 * @param info State information for the active connection.
 * @param engineRPM Set to engine speed in RPM (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool c14cux_getEngineRPM(c14cux_info* info, uint16_t* engineRPM)
{
    uint16_t pulseWidth = 0;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_EngineSpeedFilteredOffset, 2, (uint8_t*)&pulseWidth))
    {
        // when ignition is on but the engine isn't running, the pulsewidth
        // value remains at its initialized state of 0xFFFF; this is used
        // as a special case to indicate 0 RPM.
        if (pulseWidth == 0xFFFF)
        {
            *engineRPM = 0;
        }
        else
        {
            *engineRPM = 7500000 / swapShort(pulseWidth);
        }
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the RPM limit in revolutions per minute.
 * @param info State information for the active connection.
 * @param rpmLimit Set to the RPM limit if read successfully
 * @return True if successfully read; false otherwise
 */
bool c14cux_getRPMLimit(c14cux_info* info, uint16_t* rpmLimit)
{
    uint16_t pulseWidth = 0;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_RPMLimitOffset, 2, (uint8_t*)&pulseWidth))
    {
        *rpmLimit = 7500000 / swapShort(pulseWidth);
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the current target idle speed.
 * @param info State information for the active connection.
 * @param targetIdleRPM Set to the current target idle speed (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool c14cux_getTargetIdle(c14cux_info* info, uint16_t* targetIdleRPM)
{
    uint16_t targetIdle = 0;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_TargetIdleSpeedOffset, 2, (uint8_t*)&targetIdle))
    {
        *targetIdleRPM = swapShort(targetIdle);
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the throttle position as a percentage of WOT.
 * @param info State information for the active connection.
 * @param type Selects either an 'absolute' throttle reading (a simple percentage
 *   of the maximum reading of 1023), or a 'corrected' throttle reading (which is
 *   adjusted so that the lowest value read is shown as 0%)
 * @param throttlePos Set to throttle position as a percentage (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool c14cux_getThrottlePosition(c14cux_info* info, const enum c14cux_throttle_pos_type type, float* throttlePos)
{
    uint16_t throttle = 0;
    uint16_t throttleMinPos = 0;
    uint16_t correctionOffset = 0;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_ThrottlePositionOffset, 2, (uint8_t*)&throttle))
    {
        throttle = swapShort(throttle);

        // sanity check the reading
        if (throttle <= 1023)
        {
            // if we're being asked to correct the measured value (so that the lowest
            // actual measurement is shown as 0%), then read the stored minimum position
            if (type == C14CUX_ThrottlePosType_Corrected)
            {
                if (c14cux_readMem(info, C14CUX_ThrottleMinimumPositionOffset, 2, (uint8_t*)&throttleMinPos))
                {
                    throttleMinPos = swapShort(throttleMinPos);
                    retVal = true;
                }
            }
            else
            {
                retVal = true;
            }

            if (throttle >= throttleMinPos)
            {
                // subtract off the base offset (which is zero for an absolute reading, or
                // the minimum throttle position reading for a corrected reading)
                *throttlePos = (throttle - throttleMinPos) / (1023.0 - throttleMinPos);
            }
            else
            {
                // if, because of some glitch, the current throttle reading is lower than the
                // lowest reading that the ECU has stored, just report the position to zero
                *throttlePos = 0;
            }
        }
    }

    return retVal;
}

/**
 * Gets the transmission gear selection (neutral or drive).
 * @param info State information for the active connection.
 * @param gear Set to the currently-selected gear (park/neutral, drive/reverse,
 *   or manual gearbox.)
 * @return True if successfully read; false otherwise
 */
bool c14cux_getGearSelection(c14cux_info* info, enum c14cux_gear* gear)
{
    uint8_t nSwitch = 0;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_TransmissionGearOffset, 1, &nSwitch))
    {
        // Note that the comparison values used below are the same as those used
        // in the ECU's firmware (at least in tune R3652). Because they're used
        // inline in the firmware rather than being stored at fixed offsets in
        // the data segment, we don't attempt to read them out and instead just
        // hardcode them here as well.

        // if the ADC reading is fairly low, it's indicating park/neutral
        if (nSwitch < 0x4D)
        {
            *gear = C14CUX_Gear_ParkOrNeutral;
        }
        else if (nSwitch > 0xB3) // if the reading is fairly high, it's drive/reverse
        {
            *gear = C14CUX_Gear_DriveOrReverse;
        }
        else
        // manual gearboxes have a resistor fitted to give a midpoint value
        {
            *gear = C14CUX_Gear_ManualGearbox;
        }

        retVal = true;
    }

    return retVal;
}

/**
 * Determines the revision of the code in the connected ECU based on the
 * position of the first fuel map.
 * @param info State information for the active connection.
 */
void c14cux_determineDataOffsets(c14cux_info *info)
{
    // if the revision of the ROM hasn't yet been determined...
    if (info->promRev == C14CUX_DataOffsets_Unset)
    {
        uint8_t fuelMapRowData[FUEL_MAP_COLUMNS];
        uint8_t voltageOffsetA;
        int firstRowOffset = 0;
        bool oldRev = true;

        // read the first row of fuel map data at each of its two (?) possible locations
        if (c14cux_readMem(info, C14CUX_OldFuelMap1Offset, FUEL_MAP_COLUMNS, &fuelMapRowData[0]))
        {
            while ((firstRowOffset < FUEL_MAP_COLUMNS) && oldRev)
            {
                // the first row of fuel map data should never contain values as high
                // as 0x30, so if this data (read from the old Map 1 location) only
                // contains low values, we decide that it must represent real map data
                if (fuelMapRowData[firstRowOffset] > 0x30)
                {
                    oldRev = false;
                }
                firstRowOffset += 1;
            }

            if (oldRev)
            {
                // very old 14CU(X) units (1990ish and earlier) have some of the main-voltage
                // computation factors hardcoded, so we need to check for that as well
                if (c14cux_readMem(info, C14CUX_RevBMainVoltageFactorAOffset, 1, &voltageOffsetA))
                {
                    if (voltageOffsetA == 0xFF)
                    {
                        info->promRev = C14CUX_DataOffsets_RevA;
                    }
                    else
                    {
                        info->promRev = C14CUX_DataOffsets_RevB;
                    }
                }
            }
            else
            {
                info->promRev = C14CUX_DataOffsets_RevC;
            }
        }
    }
}

/**
 * Gets the main voltage being supplied to the ECU.
 * @param info State information for the active connection.
 * @param mainVoltage Set to the main relay voltage (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool c14cux_getMainVoltage(c14cux_info* info, float* mainVoltage)
{
    uint16_t storedVal = 0;
    uint16_t adcCount = 0;
    bool retVal = false;
    bool readCoefficients = false;

    if ((info->voltageFactorA != 0) &&
        (info->voltageFactorB != 0) &&
        (info->voltageFactorC != 0))
    {
        readCoefficients = true;
    }
    else
    {
        c14cux_determineDataOffsets(info);

        // read the coefficients from the ROM so that we can reverse the ADC computation
        if (info->promRev == C14CUX_DataOffsets_RevA)
        {
            info->voltageFactorA = C14CUX_RevAMainVoltageFactorA;
            info->voltageFactorB = C14CUX_RevAMainVoltageFactorB;
            info->voltageFactorC = C14CUX_RevAMainVoltageFactorC;
            readCoefficients = true;
        }
        else if (info->promRev == C14CUX_DataOffsets_RevB)
        {
            if (c14cux_readMem(info, C14CUX_RevBMainVoltageFactorAOffset, 1, &info->voltageFactorA) &&
                c14cux_readMem(info, C14CUX_RevBMainVoltageFactorBOffset, 1, &info->voltageFactorB) &&
                c14cux_readMem(info, C14CUX_RevBMainVoltageFactorCOffset, 2, (uint8_t*)&info->voltageFactorC))
            {
                info->voltageFactorC = swapShort(info->voltageFactorC);
                readCoefficients = true;
            }
        }
        else if (info->promRev == C14CUX_DataOffsets_RevC)
        {
            if (c14cux_readMem(info, C14CUX_RevCMainVoltageFactorAOffset, 1, &info->voltageFactorA) &&
                c14cux_readMem(info, C14CUX_RevCMainVoltageFactorBOffset, 1, &info->voltageFactorB) &&
                c14cux_readMem(info, C14CUX_RevCMainVoltageFactorCOffset, 2, (uint8_t*)&info->voltageFactorC))
            {
                info->voltageFactorC = swapShort(info->voltageFactorC);
                readCoefficients = true;
            }
        }
    }

    if (readCoefficients &&
        c14cux_readMem(info, C14CUX_MainVoltageOffset, 2, (uint8_t*)&storedVal))
    {
        storedVal = swapShort(storedVal);

        // reverse the quadratic math done by the ECU to get back
        // to the originally-read ADC value
        adcCount = -(16 * (sqrt((4 * info->voltageFactorA * storedVal) -
                                (info->voltageFactorA * info->voltageFactorC) +
                                (64 * info->voltageFactorB * info->voltageFactorB)) -
                           (8 * info->voltageFactorB))) / info->voltageFactorA;

        // follow the linear mapping between ADC and voltage to get the main voltage
        *mainVoltage = (0.07 * adcCount) - 0.09;

        retVal = true;
    }

    return retVal;
}

/**
 * Gets the contents of the specified fuel map.
 * @param info State information for the active connection.
 * @param fuelMapId ID of the fuel map, from 0 to 5
 * @param adjustmentFactor Set to the adjustment factor read at the end of the map
 * @param buffer Buffer of at least 128 bytes (16 cols x 8 rows)
 * @return True if the fuel map was successfully read; 0 if an invalid
 *   fuel map ID was given or if reading failed
 */
bool c14cux_getFuelMap(c14cux_info* info, const uint8_t fuelMapId, uint16_t* adjustmentFactor, uint8_t* rowScaler, uint8_t* buffer)
{
    bool retVal = false;
    uint16_t scalerOffset = 0;

    // check that the map ID is valid
    if (fuelMapId <= 5)
    {
        uint16_t offset = 0;

        c14cux_determineDataOffsets(info);

        // Fuel Map 0 is stored at the same location in both
        // the old and new ROM layouts
        if (fuelMapId == 0)
        {
            offset = C14CUX_FuelMap0Offset;
            scalerOffset = C14CUX_Map0RowScalerInitValueOffset;
        }
        else if ((info->promRev == C14CUX_DataOffsets_RevA) || (info->promRev == C14CUX_DataOffsets_RevB))
        {
            switch (fuelMapId)
            {
            case 1:
                offset = C14CUX_OldFuelMap1Offset;
                break;
            case 2:
                offset = C14CUX_OldFuelMap2Offset;
                break;
            case 3:
                offset = C14CUX_OldFuelMap3Offset;
                break;
            case 4:
                offset = C14CUX_OldFuelMap4Offset;
                break;
            case 5:
                offset = C14CUX_OldFuelMap5Offset;
                break;
            default:
                offset = 0;
                break;
            }

            scalerOffset = offset + C14CUX_FuelMapRowScalerOffset;
        }
        else if (info->promRev == C14CUX_DataOffsets_RevC)
        {
            switch (fuelMapId)
            {
            case 1:
                offset = C14CUX_NewFuelMap1Offset;
                break;
            case 2:
                offset = C14CUX_NewFuelMap2Offset;
                break;
            case 3:
                offset = C14CUX_NewFuelMap3Offset;
                break;
            case 4:
                offset = C14CUX_NewFuelMap4Offset;
                break;
            case 5:
                offset = C14CUX_NewFuelMap5Offset;
                break;
            default:
                offset = 0;
                break;
            }

            scalerOffset = offset + C14CUX_FuelMapRowScalerOffset;
        }

        if (offset != 0)
        {
            uint16_t adjFactor = 0;

            // read the fuel map data and the 16-bit adjustment factor at the end
            if (c14cux_readMem(info, offset, C14CUX_FuelMapSize, buffer) &&
                c14cux_readMem(info, offset + C14CUX_FuelMapSize, 2, (uint8_t*)&adjFactor) &&
                c14cux_readMem(info, scalerOffset, 1, rowScaler))
            {
                *adjustmentFactor = swapShort(adjFactor);
                retVal = true;
            }
        }
    }

    return retVal;
}

/**
 * Gets the ID of the fuel map currently being used. This should be between
 * 0 and 5. The fuel map can be determined by a tune resistor in the wiring
 * harness for non-NAS Land Rovers; unmodified NAS LRs have a software
 * lockout that prevents the selection of any map other than Map 5.
 * @param info State information for the active connection.
 * @param fuelMapId Set to the index of the fuel map currently in use (if read
 *  successfully)
 * @return True if successfully read; false otherwise
 */
bool c14cux_getCurrentFuelMap(c14cux_info* info, uint8_t* fuelMapId)
{
    uint8_t id = 0xff;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_CurrentFuelMapIdOffset, 1, &id) &&
        (id <= 5))
    {
        *fuelMapId = id;
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the current fuel map row index.
 * @param info State information for the active connection.
 * @param fuelMapRowIndex Set to the row index of the fuel map
 * @param rowWeighting Set to the row weighting value
 * @return True if successfully read; false otherwise
 */
bool c14cux_getFuelMapRowIndex(c14cux_info* info, uint8_t* fuelMapRowIndex, uint8_t* rowWeighting)
{
    uint8_t rowIndex = 0xff;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_FuelMapRowIndexOffset, 1, &rowIndex) &&
        ((rowIndex >> 4) < FUEL_MAP_ROWS))
    {
        // fuel map starting row index is stored in the high nibble
        *fuelMapRowIndex = (rowIndex >> 4);

        // row weighting is stored in the low nibble
        *rowWeighting = rowIndex & 0x0F;

        retVal = true;
    }

    return retVal;
}

/**
 * Gets the current fuel map column index.
 * @param info State information for the active connection.
 * @param fuelMapColIndex Set to the column index of the fuel map
 * @param colWeighting Set to the column weighting value
 * @return True if successfully read; false otherwise
 */
bool c14cux_getFuelMapColumnIndex(c14cux_info* info, uint8_t* fuelMapColIndex, uint8_t* colWeighting)
{
    uint8_t colIndex = 0xff;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_FuelMapColumnIndexOffset, 1, &colIndex) &&
        ((colIndex >> 4) < FUEL_MAP_COLUMNS))
    {
        // fuel map starting column index is stored in the high nibble
        *fuelMapColIndex = (colIndex >> 4);

        // column weighting is stored in the low nibble
        *colWeighting = colIndex & 0x0F;

        retVal = true;
    }

    return retVal;
}

/**
 * Gets the current short-term lambda-based fueling trim for the specified engine bank.
 * A larger number indicates more fuel being delivered.
 * @param info State information for the active connection.
 * @param bank Bank of the engine (odd or even)
 * @param lambdaTrim Set to the number of counts of lambda-based fueling trim
 *   for the specified bank, from -256 to 255.
 * @return True if successfully read; false otherwise
 */
bool c14cux_getLambdaTrimShort(c14cux_info* info, const enum c14cux_bank bank, int16_t* lambdaTrim)
{
    bool retVal = false;
    uint16_t fuelTrimRaw = 0;
    uint16_t offset = 0;

    if (bank == C14CUX_Bank_Odd)
    {
        offset = C14CUX_ShortTermLambdaFuelingTrimOddOffset;
    }
    else if (bank == C14CUX_Bank_Even)
    {
        offset = C14CUX_ShortTermLambdaFuelingTrimEvenOffset;
    }

    if ((offset != 0) && c14cux_readMem(info, offset, 2, (uint8_t*)&fuelTrimRaw))
    {
        // compute the number of "fueling counts" (will be between -256 and +255)
        *lambdaTrim = ((swapShort(fuelTrimRaw) / 0x80) - 0x100);
        retVal = true;
    }

    return retVal;
}


/**
 * Gets the current long-term lambda-based fueling trim for the specified engine bank.
 * A larger number indicates more fuel being delivered.
 * @param info State information for the active connection.
 * @param bank Bank of the engine (odd or even)
 * @param lambdaTrim Set to the number of counts of lambda-based fueling trim
 *   for the specified bank, from -256 to 255.
 * @return True if successfully read; false otherwise
 */
bool c14cux_getLambdaTrimLong(c14cux_info* info, const enum c14cux_bank bank, int16_t* lambdaTrim)
{
    bool retVal = false;
    uint16_t fuelTrimRaw = 0;
    uint16_t offset = 0;

    if (bank == C14CUX_Bank_Odd)
    {
        offset = C14CUX_LongTermLambdaFuelingTrimOddOffset;
    }
    else if (bank == C14CUX_Bank_Even)
    {
        offset = C14CUX_LongTermLambdaFuelingTrimEvenOffset;
    }

    if ((offset != 0) && c14cux_readMem(info, offset, 2, (uint8_t*)&fuelTrimRaw))
    {
        // compute the number of "fueling counts" (will be between -256 and +255),
        *lambdaTrim = (swapShort(fuelTrimRaw) / 0x80) - 0x100;
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the measured MAF CO trim voltage.
 * @param info State information for the active connection.
 * @param coTrimVoltage Measured CO trim in volts.
 * @return True when the value was successfully read; false otherwise
 */
bool c14cux_getCOTrimVoltage(c14cux_info* info, float* coTrimVoltage)
{
    bool retVal = false;
    uint16_t rawcount = 0;

    if (c14cux_readMem(info, C14CUX_LongTermLambdaFuelingTrimEvenOffset, 2, (uint8_t*)&rawcount))
    {
        rawcount = swapShort(rawcount);
        *coTrimVoltage = 5.0 * ((float)(rawcount >> 7)) / 1024.0;
        retVal = true;
    }

    return retVal;
}

/**
 * Populates the supplied struct with fault code data read from the ECU.
 * @param info State information for the active connection.
 * @param faultCodes Struct to populate with current fault code data
 * @return True when the fault code data was successfully read and
 *   the struct populated; false when an error occurred
 */
bool c14cux_getFaultCodes(c14cux_info* info, c14cux_faultcodes *faultCodes)
{
    return c14cux_readMem(info, C14CUX_FaultCodesOffset, sizeof(c14cux_faultcodes), (uint8_t*)faultCodes);
}

/**
 * Clears the stored fault codes in the ECU
 * @param info State information for the active connection.
 * @return True when the fault code data was successfully cleared;
 *   false when an error occurred
 */
bool c14cux_clearFaultCodes(c14cux_info* info)
{
    bool writeSuccess = true;
    int faultCodeBlockSize = sizeof(c14cux_faultcodes);
    int bytesWritten = 0;

    while (writeSuccess && (bytesWritten < faultCodeBlockSize))
    {
        if (c14cux_writeMem(info, C14CUX_FaultCodesOffset + bytesWritten, 0x00))
        {
            bytesWritten++;
        }
        else
        {
            writeSuccess = false;
        }
    }

    return writeSuccess;
}

/**
 * Gets the state of the line driving the fuel pump relay.
 * @param info State information for the active connection.
 * @param fuelPumpRelayState Set to true when the fuel pump relay is closed,
 *  or false when it's open
 * @return True when the relay state was successfully read; false otherwise
 */
bool c14cux_getFuelPumpRelayState(c14cux_info* info, bool* fuelPumpRelayState)
{
    uint8_t port1Data = 0x00;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_Port1Offset, 1, &port1Data))
    {
        *fuelPumpRelayState = !(port1Data & 0x40);
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the tune revision of the code in the ROM.
 * @param info State information for the active connection.
 * @param tuneRevision Decimal representation of the tune revision
 * @param chksumFixer Checksum fixer byte value
 * @param tuneIdent Tune 'Ident' word value (provides further differentiation after tune revision number)
 * @return True when the read was successful, false otherwise
 */
bool c14cux_getTuneRevision(c14cux_info* info, uint16_t* tuneRevision, uint8_t* chksumFixer, uint16_t* tuneIdent)
{
    uint8_t idinfo[5];
    bool retVal = false;
    int byteIdx = 0;
    uint8_t* tuneIdPtr = (uint8_t*)tuneIdent;

    if (c14cux_readMem(info, C14CUX_TuneRevisionOffset, 5, idinfo))
    {
        // first two bytes in the 4-byte array represent a BCD tune revision number
        *tuneRevision = 0;
        for (byteIdx = 0; byteIdx < 2; ++byteIdx)
        {
            *tuneRevision = (*tuneRevision * 100) +          // from the previous byte
                            (((idinfo[byteIdx] >> 4) * 10) + // high nibble of current byte
                            (idinfo[byteIdx] & 0x0F));       // low nibble of current byte
        }

        // the next byte is the checksum fixer
        *chksumFixer = idinfo[2];

        // and the final two comprise the "Ident" word
        tuneIdPtr[0] = idinfo[3];
        tuneIdPtr[1] = idinfo[4];
        *tuneIdent = swapShort(*tuneIdent);

        retVal = true;
    }

    return retVal;
}

/**
 * Gets a flag indicating whether the ECU is in "idle mode"
 * @param info State information for the active connection.
 * @param idleMode Set to true when the ECU is driving an idle speed,
 *   false otherwise
 * @return True when the read was successful, false otherwise
 */
bool c14cux_getIdleMode(c14cux_info* info, bool* idleMode)
{
    uint8_t idleModeByte;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_IdleModeOffset, 1, &idleModeByte))
    {
        *idleMode = ((idleModeByte & 0x01) == 0x01);
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the state of the purge valve.
 * @param info State information for the active connection.
 * @param state State of the purge valve (closed, toggling, or open)
 * @return True when the read was successful, false otherwise
 */
bool c14cux_getPurgeValveState(c14cux_info* info, enum c14cux_purge_valve_state* state)
{
    uint16_t purgeValveState;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_PurgeValveStateOffset, 2, (uint8_t*)&purgeValveState))
    {
        purgeValveState = swapShort(purgeValveState);
        if (purgeValveState < 4000) // threshold used by the ECU
        {
            *state = C14CUX_PurgeValveState_Closed;
        }
        else if (purgeValveState < 29000) // threshold used by the ECU
        {
            *state = C14CUX_PurgeValveState_Toggling;
        }
        else
        {
            *state = C14CUX_PurgeValveState_Open;
        }
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the state of the screen heater.
 * @param info State information for the active connection.
 * @param state State of the screen heater (true is on, false is off)
 * @return True when the read was successful, false otherwise
 */
bool c14cux_getScreenHeaterState(c14cux_info* info, bool* state)
{
    uint8_t bits = 0;
    bool status = false;

    if (c14cux_readMem(info, C14CUX_Bits_00DD, 1, &bits))
    {
        *state = ((bits & 0x04) == 0x00);
        status = true;
    }

    return status;
}

/**
 * Gets the state of the AC compressor load input.
 * @param info State information for the active connection.
 * @param state State of the AC compressor (true is on, false is off)
 * @return True when the read was successful, false otherwise
 */
bool c14cux_getACCompressorState(c14cux_info* info, bool* state)
{
    uint8_t bits = 0;
    bool status = false;

    if (c14cux_readMem(info, C14CUX_Bits_008A, 1, &bits))
    {
        *state = ((bits & 0x08) == 0x00);
        status = true;
    }

    return status;
}

/**
 * Gets the state of the Malfunction Indicator Lamp (MIL). Note that
 * a MIL implies that at least one fault code is set, but not every
 * fault code will trigger a MIL.
 * @param info State information for the active connection.
 * @param milOn Set to true when the MIL is lit, false otherwise
 * @return True when the read was successful, false otherwise
 */
bool c14cux_isMILOn(c14cux_info* info, bool* milOn)
{
    uint8_t portData;
    bool retVal = false;

    if (c14cux_readMem(info, C14CUX_Port1Offset, 1, &portData))
    {
        *milOn = !(portData & 0x01);
        retVal = true;
    }

    return retVal;
}

/**
 * Reads the table of RPM thresholds that are used to separate engine speeds
 * into the sixteen ranges indexed for fueling.
 * @param info State information for the active connection.
 * @param table Pointer to RPM table structure
 * @return True when the read was successful, false otherwise
 */
bool c14cux_getRpmTable(c14cux_info* info, c14cux_rpmtable *table)
{
  bool success = false;
  uint8_t column = 0;
  uint16_t pw = 0;

  do
  {
    success = c14cux_readMem(info, C14CUX_RPMTableOffset + (column * 4), 2, (uint8_t*)&pw);
    if (success)
    {
      table->rpm[FUEL_MAP_COLUMNS - column - 1] = 7500000 / swapShort(pw);
    }
    column += 1;

  } while (success && (column < FUEL_MAP_COLUMNS));

  return success;
}

/**
 * Reads the injector pulse width that is computed based on all the
 * environmental factors. The location at which it is stored is used for both
 * banks, and the bank being serviced at the time the value is read is
 * nondeterministic. Therefore, this value is probably most useful when running
 * in open-loop mode, because it should be the same for both banks.
 * @param info State information for the active connection.
 * @param pulseWidth Injector pulse width in microseconds
 * @return True when the read was successful, false otherwise
 */
bool c14cux_getInjectorPulseWidth(c14cux_info* info, uint16_t* pulseWidth)
{
    bool status = false;

    if (c14cux_readMem(info, C14CUX_InjectorPulseWidthOffset, 2, (uint8_t*)pulseWidth))
    {
        *pulseWidth = swapShort(*pulseWidth);
        status = true;
    }

    return status;
}

/**
 * Closes the fuel pump relay to run the pump for a single timeout period
 * (approximately two seconds).
 * @param info State information for the active connection.
 * @return True when the port and timer were written correctly; 0
 *   otherwise.
 */
bool c14cux_runFuelPump(c14cux_info* info)
{
    bool retVal = false;
    uint8_t port1State = 0x00;

    if (c14cux_readMem(info, C14CUX_Port1Offset, 1, &port1State) &&
        c14cux_writeMem(info, C14CUX_FuelPumpTimerOffset, 0xFF) &&
        c14cux_writeMem(info, C14CUX_Port1Offset, port1State & 0xBF))
    {
        retVal = true;
    }

    return retVal;
}

/**
 * Commands the idle air control motor to move.
 * @param info State information for the active connection.
 * @param direction Direction of travel for the motor; 0 opens the valve
 *  and 1 closes it
 * @param steps Number of steps to travel in the specified direction
 * @return True when the command was written successfully; false otherwise
 */
bool c14cux_driveIdleAirControlMotor(c14cux_info* info, const uint8_t direction, const uint8_t steps)
{
    // Bit 0 in 0x008A determines the direction of motion;
    //  0 opens the valve and 1 closes it

    bool retVal = false;
    uint8_t iacDirection = 0x00;

    if (c14cux_readMem(info, C14CUX_Bits_008A, 1, &iacDirection))
    {
        if (direction == 0)
        {
            iacDirection &= 0xFE;
        }
        else
        {
            iacDirection |= 0x01;
        }

        c14cux_writeMem(info, C14CUX_Bits_008A, iacDirection);
        c14cux_writeMem(info, C14CUX_IdleAirControlStepCountOffset, steps);
    }
}

/**
 * A lookup table that translates ADC counts from the temperature sensors
 * to degrees Fahrenheit.
 * @param count Value measured by 14CUX ADC
 * @return Corresponding sensor value in degrees Fahrenheit
 */
double c14cux_hyperbolicOffsetModel(const double count)
{
  return (double)temperature_adc_to_degrees_f[(uint8_t)count];
}

