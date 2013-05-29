// libcomm14cux - a communications library for the Lucas 14CUX ECU
//
// data.cpp: This file contains routines that fetch and format data
//           from the ECU.

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

/**
 * Dumps the entire contents of the 14CUX PROM and places it in a buffer.
 * @param buffer Buffer of at least 16KB.
 * @return True when the PROM was read and the buffer was successfully
 *   populated; false otherwise.
 */
bool _14cux_dumpROM(cuxinfo* info, uint8_t* buffer)
{
    return _14cux_readMem(info, _14CUX_ROMAddress, _14CUX_ROMSize, buffer);
}

/**
 * Gets the measured road speed of the vehicle in miles per hour.
 * @param roadSpeed Set to road speed in miles per hour (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getRoadSpeed(cuxinfo* info, uint16_t* roadSpeed)
{
    uint8_t speed = 0;
    float floatSpeed = 0.0;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_RoadSpeedOffset, 1, &speed))
    {
        // convert KPH to MPH
        floatSpeed = speed * 0.621371192;
        *roadSpeed = (uint16_t)floatSpeed;
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the temperature of the engine coolant in degrees Fahrenheit.
 * @param coolantTemp Set to the coolant temperature (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getCoolantTemp(cuxinfo* info, int16_t* coolantTemp)
{
    uint8_t count = 0;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_CoolantTempOffset, 1, &count))
    {
        *coolantTemp = (int16_t)(_14cux_hyperbolicOffsetModel(count));
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the temperature of the fuel in degrees Fahrenheit.
 * @param fuelTemp Set to the fuel temperature (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getFuelTemp(cuxinfo* info, int16_t* fuelTemp)
{
    uint8_t count = 0;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_FuelTempOffset, 1, &count))
    {
        *fuelTemp = (int16_t)(_14cux_hyperbolicOffsetModel(count));
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the mass airflow reading at the intake.
 * @param type Type of reading to take. A "Direct" reading changes linearly
 *   with voltage but logarithmically with airflow, and "Linearized" changes
 *   linearly with airflow.
 * @param mafReading Set to MAF measurement as a percentage of the highest
 *   possible measurement (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getMAFReading(cuxinfo* info, const enum Comm14CUXAirflowType type, float *mafReading)
{
    uint16_t maf = 0;
    bool retVal = false;

    if ((type == Comm14CUXAirflowType_Direct) &&
        _14cux_readMem(info, _14CUX_MassAirflowDirectOffset, 2, (uint8_t*)&maf))
    {
        *mafReading = swapShort(maf) / 1023.0;
        retVal = true;
    }
    else if ((type == Comm14CUXAirflowType_Linearized) &&
             _14cux_readMem(info, _14CUX_MassAirflowLinearOffset, 2, (uint8_t*)&maf))
    {
        *mafReading = swapShort(maf) / 17290.0;
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the idle bypass motor position as a percentage of the widest possible
 * opening.
 * @param bypassMotorPos Set to the idle bypass position as a percentage of
 *   wide-open
 * @return True if successfully read; false otherwise
 */
bool _14cux_getIdleBypassMotorPosition(cuxinfo* info, float* bypassMotorPos)
{
    uint8_t pos = 0;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_IdleBypassPositionOffset, 1, &pos))
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
 * @param engineRPM Set to engine speed in RPM (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getEngineRPM(cuxinfo* info, uint16_t* engineRPM)
{
    uint16_t pulseWidth = 0;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_EngineSpeedFilteredOffset, 2, (uint8_t*)&pulseWidth))
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
 * @param rpmLimit Set to the RPM limit if read successfully
 * @return True if successfully read; false otherwise
 */
bool _14cux_getRPMLimit(cuxinfo* info, uint16_t* rpmLimit)
{
    uint16_t pulseWidth = 0;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_RPMLimitOffset, 2, (uint8_t*)&pulseWidth))
    {
        *rpmLimit = 7500000 / swapShort(pulseWidth);
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the current target idle speed.
 * @param targetIdleRPM Set to the current target idle speed (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getTargetIdle(cuxinfo* info, uint16_t* targetIdleRPM)
{
    uint16_t targetIdle = 0;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_TargetIdleSpeedOffset, 2, (uint8_t*)&targetIdle))
    {
        *targetIdleRPM = swapShort(targetIdle);
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the throttle position as a percentage of WOT.
 * @param type Selects either an 'absolute' throttle reading (a simple percentage
 *   of the maximum reading of 1023), or a 'corrected' throttle reading (which is
 *   adjusted so that the lowest value read is shown as 0%)
 * @param throttlePos Set to throttle position as a percentage (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getThrottlePosition(cuxinfo* info, const enum Comm14CUXThrottlePosType type, float* throttlePos)
{
    uint16_t throttle = 0;
    uint16_t throttleMinPos = 0;
    uint16_t correctionOffset = 0;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_ThrottlePositionOffset, 2, (uint8_t*)&throttle))
    {
        throttle = swapShort(throttle);

        // if we're being asked to correct the measured value (so that the lowest
        // actual measurement is shown as 0%), then read the stored minimum position
        if (type == Comm14CUXThrottlePosType_Corrected)
        {
            if (_14cux_readMem(info, _14CUX_ThrottleMinimumPositionOffset, 2, (uint8_t*)&throttleMinPos))
            {
                throttleMinPos = swapShort(throttleMinPos);

                // if the throttle is currently showing a lower measurement than we've yet seen,
                // update the locally stored measurement with this new one
                if (throttle < info->lowestThrottleMeasurement)
                {
                    info->lowestThrottleMeasurement = throttle;
                }

                // if this library has seen a lower throttle measurement than the one stored in
                // the ECU as the minimum position, use the lower measurement as the minimum
                if (info->lowestThrottleMeasurement < throttleMinPos)
                {
                    correctionOffset = info->lowestThrottleMeasurement;
                }
                // otherwise, use the ECU's stored minimum
                else
                {
                    correctionOffset = throttleMinPos;
                }

                retVal = true;
            }
        }
        else
        {
            retVal = true;
        }

        // subtract off the base offset (which is zero for an absolute reading, or
        // the minimum throttle position reading for a corrected reading)
        *throttlePos = (throttle - correctionOffset) / (1023.0 - correctionOffset);
    }

    return retVal;
}

/**
 * Gets the transmission gear selection (neutral or drive).
 * @param gear Set to the currently-selected gear (park/neutral, drive/reverse,
 *   or manual gearbox.)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getGearSelection(cuxinfo* info, enum Comm14CUXGear* gear)
{
    uint8_t nSwitch = 0;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_TransmissionGearOffset, 1, &nSwitch))
    {
        // if the ADC reading is fairly low, it's indicating park/neutral
        if (nSwitch < 0x30)
        {
            *gear = Comm14CUXGear_ParkOrNeutral;
        }
        else if (nSwitch > 0xD0)    // if the reading is fairly high, it's drive/reverse
        {
            *gear = Comm14CUXGear_DriveOrReverse;
        }
        else
        // manual gearboxes have a resistor fitted to give a midpoint value
        {
            *gear = Comm14CUXGear_ManualGearbox;
        }

        retVal = true;
    }

    return retVal;
}

/**
 * Determines the revision of the code in the connected ECU based on the
 * position of the first fuel map.
 */
void _14cux_determineDataOffsets(cuxinfo *info)
{
    // if the revision of the PROM hasn't yet been determined...
    if (info->promRev == Comm14CUXDataOffsets_Unset)
    {
        uint8_t maxByteToByteChangeOld = 0;
        uint8_t maxByteToByteChangeNew = 0;
        uint8_t testBufferOld[16];
        uint8_t testBufferNew[16];
        uint8_t voltageOffsetA;
        int firstRowOffset = 1;

        // read the first row of fuel map data at each of its two (?) possible locations
        if (_14cux_readMem(info, _14CUX_OldFuelMap1Offset, 16, &testBufferOld[0]) &&
            _14cux_readMem(info, _14CUX_NewFuelMap1Offset, 16, &testBufferNew[0]))
        {
            // find the greatest byte-to-byte difference in each set of data
            for (firstRowOffset = 1; firstRowOffset < 16; firstRowOffset++)
            {
                if (maxByteToByteChangeOld <
                        abs((testBufferOld[firstRowOffset] - testBufferOld[firstRowOffset - 1])))
                {
                    maxByteToByteChangeOld =
                        abs((testBufferOld[firstRowOffset] - testBufferOld[firstRowOffset - 1]));
                }

                if (maxByteToByteChangeNew <
                        abs((testBufferNew[firstRowOffset] - testBufferNew[firstRowOffset - 1])))
                {
                    maxByteToByteChangeNew =
                        abs((testBufferNew[firstRowOffset] - testBufferNew[firstRowOffset - 1]));
                }
            }

            // real fuel map data will only have small changes between
            // consecutive values, so the run of data with the smallest
            // byte-to-byte changes is probably the real fuel map
            if (maxByteToByteChangeOld < maxByteToByteChangeNew)
            {
                // very old 14CU(X) units (1990ish and earlier) have some of the main-voltage
                // computation factors hardcoded, so we need to check for that as well
                if (_14cux_readMem(info, _14CUX_RevBMainVoltageFactorAOffset, 1, &voltageOffsetA))
                {
                    if (voltageOffsetA == 0xFF)
                    {
                        info->promRev = Comm14CUXDataOffsets_RevA;
                    }
                    else
                    {
                        info->promRev = Comm14CUXDataOffsets_RevB;
                    }
                }
            }
            else
            {
                info->promRev = Comm14CUXDataOffsets_RevC;
            }
        }
    }
}

/**
 * Gets the main voltage being supplied to the ECU.
 * @param mainVoltage Set to the main relay voltage (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getMainVoltage(cuxinfo* info, float* mainVoltage)
{
    uint16_t storedVal = 0;
    uint16_t adcCount = 0;
    bool retVal = false;
    uint8_t readCoefficients = 0;

    if ((info->voltageFactorA != 0) &&
        (info->voltageFactorB != 0) &&
        (info->voltageFactorC != 0))
    {
        readCoefficients = 1;
    }
    else
    {
        _14cux_determineDataOffsets(info);

        // read the coefficients from the PROM so that we can reverse the ADC computation
        if (info->promRev == Comm14CUXDataOffsets_RevA)
        {
            info->voltageFactorA = _14CUX_RevAMainVoltageFactorA;
            info->voltageFactorB = _14CUX_RevAMainVoltageFactorB;
            info->voltageFactorC = _14CUX_RevAMainVoltageFactorC;
            readCoefficients = 1;
        }
        else if (info->promRev == Comm14CUXDataOffsets_RevB)
        {
            if (_14cux_readMem(info, _14CUX_RevBMainVoltageFactorAOffset, 1, &info->voltageFactorA) &&
                _14cux_readMem(info, _14CUX_RevBMainVoltageFactorBOffset, 1, &info->voltageFactorB) &&
                _14cux_readMem(info, _14CUX_RevBMainVoltageFactorCOffset, 2, (uint8_t*)&info->voltageFactorC))
            {
                info->voltageFactorC = swapShort(info->voltageFactorC);
                readCoefficients = 1;
            }
        }
        else if (info->promRev == Comm14CUXDataOffsets_RevC)
        {
            if (_14cux_readMem(info, _14CUX_RevCMainVoltageFactorAOffset, 1, &info->voltageFactorA) &&
                _14cux_readMem(info, _14CUX_RevCMainVoltageFactorBOffset, 1, &info->voltageFactorB) &&
                _14cux_readMem(info, _14CUX_RevCMainVoltageFactorCOffset, 2, (uint8_t*)&info->voltageFactorC))
            {
                info->voltageFactorC = swapShort(info->voltageFactorC);
                readCoefficients = 1;
            }
        }
    }

    if (readCoefficients &&
        _14cux_readMem(info, _14CUX_MainVoltageOffset, 2, (uint8_t*)&storedVal))
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
 * @param fuelMapId ID of the fuel map, from 0 to 5
 * @param adjustmentFactor Set to the adjustment factor read at the end of the map
 * @param buffer Buffer of at least 128 bytes (16 cols x 8 rows)
 * @return True if the fuel map was successfully read; 0 if an invalid
 *   fuel map ID was given or if reading failed
 */
bool _14cux_getFuelMap(cuxinfo* info, const uint8_t fuelMapId, uint16_t* adjustmentFactor, uint8_t* buffer)
{
    bool retVal = false;

    // check that the map ID is valid
    if (fuelMapId <= 5)
    {
        uint16_t offset = 0;

        _14cux_determineDataOffsets(info);

        // Fuel Map 0 is stored at the same location in both
        // the old and new PROM layouts
        if (fuelMapId == 0)
        {
            offset = _14CUX_FuelMap0Offset;
        }
        else if ((info->promRev == Comm14CUXDataOffsets_RevA) || (info->promRev == Comm14CUXDataOffsets_RevB))
        {
            switch (fuelMapId)
            {
            case 1:
                offset = _14CUX_OldFuelMap1Offset;
                break;
            case 2:
                offset = _14CUX_OldFuelMap2Offset;
                break;
            case 3:
                offset = _14CUX_OldFuelMap3Offset;
                break;
            case 4:
                offset = _14CUX_OldFuelMap4Offset;
                break;
            case 5:
                offset = _14CUX_OldFuelMap5Offset;
                break;
            default:
                offset = 0;
                break;
            }
        }
        else if (info->promRev == Comm14CUXDataOffsets_RevC)
        {
            switch (fuelMapId)
            {
            case 1:
                offset = _14CUX_NewFuelMap1Offset;
                break;
            case 2:
                offset = _14CUX_NewFuelMap2Offset;
                break;
            case 3:
                offset = _14CUX_NewFuelMap3Offset;
                break;
            case 4:
                offset = _14CUX_NewFuelMap4Offset;
                break;
            case 5:
                offset = _14CUX_NewFuelMap5Offset;
                break;
            default:
                offset = 0;
                break;
            }
        }

        if (offset != 0)
        {
            uint16_t adjFactor = 0;

            // read the fuel map data and the 16-bit adjustment factor at the end
            if (_14cux_readMem(info, offset, _14CUX_FuelMapSize, buffer) &&
                _14cux_readMem(info, offset + _14CUX_FuelMapSize, 2, (uint8_t*)&adjFactor))
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
 * @param fuelMapId Set to the index of the fuel map currently in use (if read
 *  successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getCurrentFuelMap(cuxinfo* info, uint8_t* fuelMapId)
{
    uint8_t id = 0xff;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_CurrentFuelMapIdOffset, 1, &id))
    {
        *fuelMapId = id;
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the current fuel map row index.
 * @param fuelMapRowIndex Set to the row index of the fuel map (if read
 *  successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getFuelMapRowIndex(cuxinfo* info, uint8_t* fuelMapRowIndex)
{
    uint8_t rowIndex = 0xff;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_FuelMapRowIndexOffset, 1, &rowIndex))
    {
        // fuel map starting row index is stored in the high nibble
        *fuelMapRowIndex = (rowIndex >> 4);

        // The 'fuzzy' nature of the fuel map means that the value selected
        // by the upper nibbles of the row-column pair is actually just the
        // upper-left corner of a square of four values, which are partially
        // combined by using the lower nibbles of the row-column indicies as
        // weights. To provide a simple index here, we simply round up to the
        // next row if the lower nibble is >= 8.
        if (((rowIndex & 0x0F) >= 0x08) && (*fuelMapRowIndex < 0x07))
        {
            *fuelMapRowIndex += 1;
        }
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the current fuel map column index.
 * @param fuelMapColIndex Set to the column index of the fuel map (if read
 *  successfully)
 * @return True if successfully read; false otherwise
 */
bool _14cux_getFuelMapColumnIndex(cuxinfo* info, uint8_t* fuelMapColIndex)
{
    uint8_t colIndex = 0xff;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_FuelMapColumnIndexOffset, 1, &colIndex))
    {
        // fuel map starting column index is stored in the high nibble
        *fuelMapColIndex = (colIndex >> 4);

        // The 'fuzzy' nature of the fuel map means that the value selected
        // by the upper nibbles of the row-column pair is actually just the
        // upper-left corner of a square of four values, which are partially
        // combined by using the lower nibbles of the row-column indicies as
        // weights. To provide a simple index here, we simply round up to the
        // next column if the lower nibble is >= 8.
        if (((colIndex & 0x0F) >= 0x08) && (*fuelMapColIndex < 0x0F))
        {
            *fuelMapColIndex += 1;
        }
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the current short-term lambda-based fueling trim for the specified engine bank.
 * A larger number indicates more fuel being delivered.
 * @param bank Bank of the engine (left or right)
 * @param lambdaTrim Set to the number of counts of lambda-based fueling trim
 *   for the specified bank, from -256 to 255.
 * @return True if successfully read; false otherwise
 */
bool _14cux_getLambdaTrimShort(cuxinfo* info, const enum Comm14CUXBank bank, int16_t* lambdaTrim)
{
    bool retVal = false;
    uint16_t fuelTrimRaw = 0;
    uint16_t offset = 0;

    if (bank == Comm14CUXBank_Left)
    {
        offset = _14CUX_ShortTermLambdaFuelingTrimLeftOffset;
    }
    else if (bank == Comm14CUXBank_Right)
    {
        offset = _14CUX_ShortTermLambdaFuelingTrimRightOffset;
    }

    if ((offset != 0) && _14cux_readMem(info, offset, 2, (uint8_t*)&fuelTrimRaw))
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
 * @param bank Bank of the engine (left or right)
 * @param lambdaTrim Set to the number of counts of lambda-based fueling trim
 *   for the specified bank, from -256 to 255.
 * @return True if successfully read; false otherwise
 */
bool _14cux_getLambdaTrimLong(cuxinfo* info, const enum Comm14CUXBank bank, int16_t* lambdaTrim)
{
    bool retVal = false;
    uint16_t fuelTrimRaw = 0;
    uint16_t offset = 0;

    if (bank == Comm14CUXBank_Left)
    {
        offset = _14CUX_LongTermLambdaFuelingTrimLeftOffset;
    }
    else if (bank == Comm14CUXBank_Right)
    {
        offset = _14CUX_LongTermLambdaFuelingTrimRightOffset;
    }

    if ((offset != 0) && _14cux_readMem(info, offset, 2, (uint8_t*)&fuelTrimRaw))
    {
        // compute the number of "fueling counts" (will be between -256 and +255),
        *lambdaTrim = (swapShort(fuelTrimRaw) / 0x80) - 0x100;
        retVal = true;
    }

    return retVal;
}

/**
 * Populates the supplied struct with fault code data read from the ECU.
 * @param faultCodes Struct to populate with current fault code data
 * @return True when the fault code data was successfully read and
 *   the struct populated; 0 when an error occurred
 */
bool _14cux_getFaultCodes(cuxinfo* info, Comm14CUXFaultCodes *faultCodes)
{
    return _14cux_readMem(info, _14CUX_FaultCodesOffset, sizeof(Comm14CUXFaultCodes), (uint8_t*)faultCodes);
}

/**
 * Clears the stored fault codes in the ECU
 * @return True when the fault code data was successfully cleared;
 *   0 when an error occurred
 */
bool _14cux_clearFaultCodes(cuxinfo* info)
{
    uint8_t writeSuccess = 1;
    int faultCodeBlockSize = sizeof(Comm14CUXFaultCodes);
    int bytesWritten = 0;

    while ((writeSuccess == 1) && (bytesWritten < faultCodeBlockSize))
    {
        if (_14cux_writeMem(info, _14CUX_FaultCodesOffset + bytesWritten, 0x00))
        {
            bytesWritten++;
        }
        else
        {
            writeSuccess = 0;
        }
    }

    return writeSuccess;
}

/**
 * Gets the state of the line driving the fuel pump relay.
 * @param fuelPumpRelayState Set to 1 when the fuel pump relay is closed,
 *  or 0 when it's open
 * @return True when the relay state was successfully read; false otherwise
 */
bool _14cux_getFuelPumpRelayState(cuxinfo* info, bool* fuelPumpRelayState)
{
    uint8_t port1Data = 0x00;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_Port1Offset, 1, &port1Data))
    {
        *fuelPumpRelayState = !(port1Data & 0x40);
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the tune revision of the code in the PROM.
 * @param tuneRevision Decimal representation of the tune revision
 * @return True when the read was successful, false otherwise
 */
bool _14cux_getTuneRevision(cuxinfo* info, uint16_t* tuneRevision)
{
    uint8_t tuneRevHex[2];
    char tuneRevStr[4];
    bool retVal = false;
    int byteIdx = 0;

    if (_14cux_readMem(info, _14CUX_TuneRevisionOffset, 2, tuneRevHex))
    {
        *tuneRevision = 0;
        for (byteIdx = 0; byteIdx < 2; ++byteIdx)
        {
            *tuneRevision = (*tuneRevision * 100) +              // from the previous byte
                            (((tuneRevHex[byteIdx] >> 4) * 10) + // high nibble of current byte
                            (tuneRevHex[byteIdx] & 0x0F));       // low nibble of current byte
        }
        retVal = true;
    }

    return retVal;
}

/**
 * Gets a flag indicating whether the ECU is in "idle mode"
 * @param idleMode Set to true when the ECU is driving an idle speed,
 *   false otherwise
 * @return True when the read was successful, false otherwise
 */
bool _14cux_getIdleMode(cuxinfo* info, bool* idleMode)
{
    uint8_t idleModeByte;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_IdleModeOffset, 1, &idleModeByte))
    {
        *idleMode = ((idleModeByte & 0x01) == 0x01);
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the state of the Malfunction Indicator Lamp (MIL). Note that
 * a MIL implies that at least one fault code is set, but not every
 * fault code will trigger a MIL.
 * @param milOn Set to true when the MIL is lit, false otherwise
 * @return True when the read was successful, false otherwise
 */
bool _14cux_isMILOn(cuxinfo* info, bool* milOn)
{
    uint8_t portData;
    bool retVal = false;

    if (_14cux_readMem(info, _14CUX_Port1Offset, 1, &portData))
    {
        *milOn = !(portData & 0x01);
        retVal = true;
    }

    return retVal;
}

/**
 * Closes the fuel pump relay to run the pump for a single timeout period
 * (approximately two seconds).
 * @return True when the port and timer were written correctly; 0
 *   otherwise.
 */
bool _14cux_runFuelPump(cuxinfo* info)
{
    bool retVal = false;
    uint8_t port1State = 0x00;

    if (_14cux_readMem(info, _14CUX_Port1Offset, 1, &port1State) &&
        _14cux_writeMem(info, _14CUX_FuelPumpTimerOffset, 0xFF) &&
        _14cux_writeMem(info, _14CUX_Port1Offset, port1State & 0xBF))
    {
        retVal = true;
    }

    return retVal;
}

/**
 * Commands the idle air control motor to move.
 * @param direction Direction of travel for the motor; 0 opens the valve
 *  and 1 closes it
 * @param steps Number of steps to travel in the specified direction
 * @return True when the command was written successfully; false otherwise
 */
bool _14cux_driveIdleAirControlMotor(cuxinfo* info, const uint8_t direction, const uint8_t steps)
{
    // Bit 0 in 0x008A determines the direction of motion;
    //  0 opens the valve and 1 closes it

    bool retVal = false;
    uint8_t iacDirection = 0x00;

    if (_14cux_readMem(info, 0x008A, 1, &iacDirection))
    {
        if (direction == 0)
        {
            iacDirection &= 0xFE;
        }
        else
        {
            iacDirection |= 0x01;
        }

        _14cux_writeMem(info, 0x008A, iacDirection);
        _14cux_writeMem(info, _14CUX_IdleAirControlStepCountOffset, steps);
    }
}

/**
 * A hyperbolic curve model that describes the impedence response of
 * the 14CUX coolant/fuel temperature sensors.
 * Thanks to ZunZun.com for providing curve-fitting tools.
 * @param count Value measured by 14CUX ADC
 * @return Corresponding sensor value in degrees Fahrenheit
 */
double _14cux_hyperbolicOffsetModel(const double count)
{
    double temp;
    temp = 0.0;
    // coefficients
    double a = -3.6576314003476170E+02;
    double b = 7.9966463239719403E+01;
    double c = 7.9351039566434352E+01;
    double d = -3.6609229160008152E+02;
    double f = 4.8357928134157724E-01;
    double Offset = 2.8843089370081475E+02;
    temp = a * count / (b + count) + c * count / (d + count) + f * count;
    temp += Offset;
    return temp;
}

