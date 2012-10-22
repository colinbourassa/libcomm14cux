// libcomm14cux - a communications library for the Lucas 14CUX ECU
//
// data.cpp: This file contains routines that fetch and format data
//           from the ECU.

#if defined(WIN32) && defined(linux)
#error "Only one of 'WIN32' or 'linux' may be defined."
#endif

// Includes for all platforms
#include <math.h>

#if defined(ARDUINO)
  // Arduino-only includes
  #include <WProgram.h>
#else
  // Non-Arduino includes
  #include <stdlib.h>

  #if defined(WIN32)
    // Windows-only includes
    #include <windows.h>
  #endif
#endif

#include "comm14cux.h"

/**
 * Dumps the entire contents of the 14CUX PROM and places it in a buffer.
 * @param buffer Buffer of at least 16KB.
 * @return True when the PROM was read and the buffer was successfully
 *   populated; false otherwise.
 */
bool Comm14CUX::dumpROM(uint8_t* buffer)
{
    return readMem(Serial14CUXParams::ROMAddress, Serial14CUXParams::ROMSize, buffer);
}

/**
 * Gets the measured road speed of the vehicle in miles per hour.
 * @param roadSpeed Set to road speed in miles per hour (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool Comm14CUX::getRoadSpeed(uint16_t &roadSpeed)
{
    uint8_t speed = 0;
    float floatSpeed = 0.0;
    bool retVal = false;

    if (readMem(Serial14CUXParams::RoadSpeedOffset, 1, &speed))
    {
        // convert KPH to MPH
        floatSpeed = speed * 0.621371192;
        roadSpeed = (uint16_t)floatSpeed;
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the temperature of the engine coolant in degrees Fahrenheit.
 * @param coolantTemp Set to the coolant temperature (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool Comm14CUX::getCoolantTemp(int16_t &coolantTemp)
{
    uint8_t count = 0;
    bool retVal = false;

    if (readMem(Serial14CUXParams::CoolantTempOffset, 1, &count))
    {
        coolantTemp = (int16_t)(hyperbolicOffsetModel(count));
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the temperature of the fuel in degrees Fahrenheit.
 * @param fuelTemp Set to the fuel temperature (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool Comm14CUX::getFuelTemp(int16_t &fuelTemp)
{
    uint8_t count = 0;
    bool retVal = false;

    if (readMem(Serial14CUXParams::FuelTempOffset, 1, &count))
    {
        fuelTemp = (int16_t)(hyperbolicOffsetModel(count));
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
bool Comm14CUX::getMAFReading(Comm14CUXAirflowType type, float &mafReading)
{
    uint16_t maf = 0;
    bool retVal = false;

    if ((type == Comm14CUXAirflowType_Direct) &&
        readMem(Serial14CUXParams::MassAirflowDirectOffset, 2, (uint8_t*)&maf))
    {
        mafReading = swapShort(maf) / 1023.0;
        retVal = true;
    }
    else if ((type == Comm14CUXAirflowType_Linearized) &&
             readMem(Serial14CUXParams::MassAirflowLinearOffset, 2, (uint8_t*)&maf))
    {
        mafReading = swapShort(maf) / 17290.0;
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
bool Comm14CUX::getIdleBypassMotorPosition(float &bypassMotorPos)
{
    uint8_t pos = 0;
    bool retVal = false;

    if (readMem(Serial14CUXParams::IdleBypassPositionOffset, 1, &pos))
    {
        // 180 counts is fully closed; it shouldn't be reading any higher than that
        if (pos > 180)
        {
            pos = 180;
        }

        bypassMotorPos = (180 - pos) / 180.0;
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the engine (crankshaft) speed in revolutions per minute.
 * @param engineRPM Set to engine speed in RPM (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool Comm14CUX::getEngineRPM(uint16_t &engineRPM)
{
    uint16_t pulseWidth = 0;
    bool retVal = false;

    if (readMem(Serial14CUXParams::EngineSpeedFilteredOffset, 2, (uint8_t*)&pulseWidth))
    {
        // when ignition is on but the engine isn't running, the pulsewidth
        // value remains at its initialized state of 0xFFFF; this is used
        // as a special case to indicate 0 RPM.
        if (pulseWidth == 0xFFFF)
        {
            engineRPM = 0;
        }
        else
        {
            pulseWidth = swapShort(pulseWidth);
            engineRPM = 7500000 / pulseWidth;
        }
        retVal = true;
    }

    return retVal;
}

/**
 * Gets the current target idle speed.
 * @param targetIdleRPM Set to the current target idle speed (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool Comm14CUX::getTargetIdle(uint16_t &targetIdleRPM)
{
    uint16_t targetIdle = 0;
    bool retVal = false;

    if (readMem(Serial14CUXParams::TargetIdleSpeedOffset, 2, (uint8_t*)&targetIdle))
    {
        targetIdleRPM = swapShort(targetIdle);
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
bool Comm14CUX::getThrottlePosition(Comm14CUXThrottlePosType type, float &throttlePos)
{
    uint16_t throttle = 0;
    uint16_t throttleMinPos = 0;
    uint16_t correctionOffset = 0;
    bool retVal = false;

    if (readMem(Serial14CUXParams::ThrottlePositionOffset, 2, (uint8_t*)&throttle))
    {
        throttle = swapShort(throttle);

        // if we're being asked to correct the measured value (so that the lowest
        // actual measurement is shown as 0%), then read the stored minimum position
        if (type == Comm14CUXThrottlePosType_Corrected)
        {
            if (readMem(Serial14CUXParams::ThrottleMinimumPositionOffset, 2, (uint8_t*)&throttleMinPos))
            {
                throttleMinPos = swapShort(throttleMinPos);

                // if the throttle is currently showing a lower measurement than we've yet seen,
                // update the locally stored measurement with this new one
                if (throttle < m_lowestThrottleMeasurement)
                {
                    m_lowestThrottleMeasurement = throttle;
                }

                // if this library has seen a lower throttle measurement than the one stored in
                // the ECU as the minimum position, use the lower measurement as the minimum
                if (m_lowestThrottleMeasurement < throttleMinPos)
                {
                    correctionOffset = m_lowestThrottleMeasurement;
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
        throttlePos = (throttle - correctionOffset) / (1023.0 - correctionOffset);
    }

    return retVal;
}

/**
 * Gets the transmission gear selection (neutral or drive).
 * @param gear Set to the currently-selected gear (park/neutral, drive/reverse,
 *   or manual gearbox.)
 * @return True if successfully read; false otherwise
 */
bool Comm14CUX::getGearSelection(Comm14CUXGear &gear)
{
    uint8_t nSwitch = 0;
    bool retVal = false;

    if (readMem(Serial14CUXParams::TransmissionGearOffset, 1, &nSwitch))
    {
        // if the ADC reading is fairly low, it's likely indicating park/neutral
        if (nSwitch < 0x30)
        {
            gear = Comm14CUXGear_ParkOrNeutral;
        }
        else if (nSwitch > 0xD0)    // if the reading is fairly high, it's drive/reverse
        {
            gear = Comm14CUXGear_DriveOrReverse;
        }
        else
        // manual gearboxes have a resistor fitted to give a
        // midpoint value and put the ECU in 'manual mode'
        {
            gear = Comm14CUXGear_ManualGearbox;
        }

        retVal = true;
    }

    return retVal;
}

/**
 * Determines the revision of the code in the connected ECU based on the
 * position of the first fuel map.
 */
void Comm14CUX::determineDataOffsets()
{
    // if the revision of the PROM hasn't yet been determined...
    if (m_promRev == Comm14CUXDataOffsets_Unset)
    {
        uint8_t maxByteToByteChangeOld = 0;
        uint8_t maxByteToByteChangeNew = 0;
        uint8_t testBufferOld[16];
        uint8_t testBufferNew[16];
        uint8_t voltageOffsetA;

        // read the first row of fuel map data at each of its two (?) possible locations
        if (readMem(Serial14CUXParams::OldFuelMap1Offset, 16, &testBufferOld[0]) &&
            readMem(Serial14CUXParams::NewFuelMap1Offset, 16, &testBufferNew[0]))
        {
            // find the greatest byte-to-byte difference in each set of data
            for (int firstRowOffset = 1; firstRowOffset < 16; firstRowOffset++)
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
                if (readMem(Serial14CUXParams::RevBMainVoltageFactorAOffset, 1, &voltageOffsetA))
                {
                    if (voltageOffsetA == 0xFF)
                    {
                        m_promRev = Comm14CUXDataOffsets_RevA;
                    }
                    else
                    {
                        m_promRev = Comm14CUXDataOffsets_RevB;
                    }
                }
            }
            else
            {
                m_promRev = Comm14CUXDataOffsets_RevC;
            }
        }
    }
}

/**
 * Gets the main voltage being supplied to the ECU.
 * @param mainVoltage Set to the main relay voltage (if read successfully)
 * @return True if successfully read; false otherwise
 */
bool Comm14CUX::getMainVoltage(float &mainVoltage)
{
    uint16_t storedVal = 0;
    uint16_t adcCount = 0;
    bool retVal = false;
    bool readCoefficients = false;

    if ((m_voltageFactorA != 0) &&
        (m_voltageFactorB != 0) &&
        (m_voltageFactorC != 0))
    {
        readCoefficients = true;
    }
    else
    {
        determineDataOffsets();

        // read the coefficients from the PROM so that we can reverse the ADC computation
        if (m_promRev == Comm14CUXDataOffsets_RevA)
        {
            m_voltageFactorA = Serial14CUXParams::RevAMainVoltageFactorA;
            m_voltageFactorB = Serial14CUXParams::RevAMainVoltageFactorB;
            m_voltageFactorC = Serial14CUXParams::RevAMainVoltageFactorC;
            readCoefficients = true;
        }
        else if (m_promRev == Comm14CUXDataOffsets_RevB)
        {
            if (readMem(Serial14CUXParams::RevBMainVoltageFactorAOffset, 1, &m_voltageFactorA) &&
                readMem(Serial14CUXParams::RevBMainVoltageFactorBOffset, 1, &m_voltageFactorB) &&
                readMem(Serial14CUXParams::RevBMainVoltageFactorCOffset, 2, (uint8_t*)&m_voltageFactorC))
            {
                m_voltageFactorC = swapShort(m_voltageFactorC);
                readCoefficients = true;
            }
        }
        else if (m_promRev == Comm14CUXDataOffsets_RevC)
        {
            if (readMem(Serial14CUXParams::RevCMainVoltageFactorAOffset, 1, &m_voltageFactorA) &&
                readMem(Serial14CUXParams::RevCMainVoltageFactorBOffset, 1, &m_voltageFactorB) &&
                readMem(Serial14CUXParams::RevCMainVoltageFactorCOffset, 2, (uint8_t*)&m_voltageFactorC))
            {
                m_voltageFactorC = swapShort(m_voltageFactorC);
                readCoefficients = true;
            }
        }
    }

    if (readCoefficients &&
        readMem(Serial14CUXParams::MainVoltageOffset, 2, (uint8_t*)&storedVal))
    {
        storedVal = swapShort(storedVal);

        // reverse the quadratic math done by the ECU to get back
        // to the originally-read ADC value
        adcCount = -(16 * (sqrt((4 * m_voltageFactorA * storedVal) -
                                (m_voltageFactorA * m_voltageFactorC) +
                                (64 * m_voltageFactorB * m_voltageFactorB)) -
                           (8 * m_voltageFactorB))) / m_voltageFactorA;

        // follow the linear mapping between ADC and voltage to get the main voltage
        mainVoltage = (0.07 * adcCount) - 0.09;

        retVal = true;
    }

    return retVal;
}

/**
 * Gets the contents of the specified fuel map.
 * @param fuelMapId ID of the fuel map, from 0 to 5
 * @param adjustmentFactor Set to the adjustment factor read at the end of the map
 * @param buffer Buffer of at least 128 bytes (16 cols x 8 rows)
 * @return True if the fuel map was successfully read; false if an invalid
 *   fuel map ID was given of if reading failed
 */
bool Comm14CUX::getFuelMap(uint8_t fuelMapId, uint16_t &adjustmentFactor, uint8_t* buffer)
{
    bool retVal = false;

    // check that the map ID is valid
    if (fuelMapId <= 5)
    {
        uint16_t offset = 0;

        determineDataOffsets();

        // Fuel Map 0 is stored at the same location in both
        // the old and new PROM layouts
        if (fuelMapId == 0)
        {
            offset = Serial14CUXParams::FuelMap0Offset;
        }
        else if ((m_promRev == Comm14CUXDataOffsets_RevA) || (m_promRev == Comm14CUXDataOffsets_RevB))
        {
            switch (fuelMapId)
            {
            case 1:
                offset = Serial14CUXParams::OldFuelMap1Offset;
                break;
            case 2:
                offset = Serial14CUXParams::OldFuelMap2Offset;
                break;
            case 3:
                offset = Serial14CUXParams::OldFuelMap3Offset;
                break;
            case 4:
                offset = Serial14CUXParams::OldFuelMap4Offset;
                break;
            case 5:
                offset = Serial14CUXParams::OldFuelMap5Offset;
                break;
            default:
                offset = 0;
                break;
            }
        }
        else if (m_promRev == Comm14CUXDataOffsets_RevC)
        {
            switch (fuelMapId)
            {
            case 1:
                offset = Serial14CUXParams::NewFuelMap1Offset;
                break;
            case 2:
                offset = Serial14CUXParams::NewFuelMap2Offset;
                break;
            case 3:
                offset = Serial14CUXParams::NewFuelMap3Offset;
                break;
            case 4:
                offset = Serial14CUXParams::NewFuelMap4Offset;
                break;
            case 5:
                offset = Serial14CUXParams::NewFuelMap5Offset;
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
            if (readMem(offset, Serial14CUXParams::FuelMapSize, buffer) &&
                readMem(offset + Serial14CUXParams::FuelMapSize, 2, (uint8_t*)&adjFactor))
            {
                adjustmentFactor = swapShort(adjFactor);
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
bool Comm14CUX::getCurrentFuelMap(uint8_t &fuelMapId)
{
    uint8_t id = 0xff;
    bool retVal = false;

    if (readMem(Serial14CUXParams::CurrentFuelMapIdOffset, 1, &id))
    {
        fuelMapId = id;
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
bool Comm14CUX::getFuelMapRowIndex(uint8_t &fuelMapRowIndex)
{
    uint8_t rowIndex = 0xff;
    bool retVal = false;

    if (readMem(Serial14CUXParams::FuelMapRowIndexOffset, 1, &rowIndex))
    {
        // fuel map starting row index is stored in the high nibble
        fuelMapRowIndex = (rowIndex >> 4);

        // The 'fuzzy' nature of the fuel map means that the value selected
        // by the upper nibbles of the row-column pair is actually just the
        // upper-left corner of a square of four values, which are partially
        // combined by using the lower nibbles of the row-column indicies as
        // weights. To provide a simple index here, we simply round up to the
        // next row if the lower nibble is >= 8.
        if (((rowIndex & 0x0F) >= 0x08) && (fuelMapRowIndex < 0x07))
        {
            fuelMapRowIndex++;
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
bool Comm14CUX::getFuelMapColumnIndex(uint8_t &fuelMapColIndex)
{
    uint8_t colIndex = 0xff;
    bool retVal = false;

    if (readMem(Serial14CUXParams::FuelMapColumnIndexOffset, 1, &colIndex))
    {
        // fuel map starting column index is stored in the high nibble
        fuelMapColIndex = (colIndex >> 4);

        // The 'fuzzy' nature of the fuel map means that the value selected
        // by the upper nibbles of the row-column pair is actually just the
        // upper-left corner of a square of four values, which are partially
        // combined by using the lower nibbles of the row-column indicies as
        // weights. To provide a simple index here, we simply round up to the
        // next column if the lower nibble is >= 8.
        if (((colIndex & 0x0F) >= 0x08) && (fuelMapColIndex < 0x0F))
        {
            fuelMapColIndex++;
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
bool Comm14CUX::getLambdaTrimShort(Comm14CUXBank bank, int16_t &lambdaTrim)
{
    bool retVal = false;
    uint16_t fuelTrimRaw = 0;
    uint16_t offset = 0;

    if (bank == Comm14CUXBank_Left)
    {
        offset = Serial14CUXParams::ShortTermLambdaFuelingTrimLeftOffset;
    }
    else if (bank == Comm14CUXBank_Right)
    {
        offset = Serial14CUXParams::ShortTermLambdaFuelingTrimRightOffset;
    }

    if ((offset != 0) && readMem(offset, 2, (uint8_t*)&fuelTrimRaw))
    {
        // compute the number of "fueling counts" (will be between -256 and +255)
        lambdaTrim = ((swapShort(fuelTrimRaw) / 0x80) - 0x100);
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
bool Comm14CUX::getLambdaTrimLong(Comm14CUXBank bank, int16_t &lambdaTrim)
{
    bool retVal = false;
    uint16_t fuelTrimRaw = 0;
    uint16_t offset = 0;

    if (bank == Comm14CUXBank_Left)
    {
        offset = Serial14CUXParams::LongTermLambdaFuelingTrimLeftOffset;
    }
    else if (bank == Comm14CUXBank_Right)
    {
        offset = Serial14CUXParams::LongTermLambdaFuelingTrimRightOffset;
    }

    if ((offset != 0) && readMem(offset, 2, (uint8_t*)&fuelTrimRaw))
    {
        // compute the number of "fueling counts" (will be between -256 and +255),
        lambdaTrim = (swapShort(fuelTrimRaw) / 0x80) - 0x100;
        retVal = true;
    }

    return retVal;
}

/**
 * Populates the supplied struct with fault code data read from the ECU.
 * @param faultCodes Struct to populate with current fault code data
 * @return True when the fault code data was successfully read and
 *   the struct populated; false when an error occurred
 */
bool Comm14CUX::getFaultCodes(Comm14CUXFaultCodes &faultCodes)
{
    return readMem(Serial14CUXParams::FaultCodesOffset, sizeof(Comm14CUXFaultCodes), (uint8_t*)&faultCodes);
}

/**
 * Clears the stored fault codes in the ECU
 * @return True when the fault code data was successfully cleared;
 *   false when an error occurred
 */
bool Comm14CUX::clearFaultCodes()
{
    bool writeSuccess = true;
    int faultCodeBlockSize = sizeof(Comm14CUXFaultCodes);
    int bytesWritten = 0;

    while ((writeSuccess == true) && (bytesWritten < faultCodeBlockSize))
    {
        if (writeMem(Serial14CUXParams::FaultCodesOffset + bytesWritten, 0x00))
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
 * @param fuelPumpRelayState Set to true when the fuel pump relay is closed,
 *  or false when it's open
 * @return True when the relay state was successfully read; false otherwise
 */
bool Comm14CUX::getFuelPumpRelayState(bool &fuelPumpRelayState)
{
    uint8_t port1Data = 0x00;
    bool retVal = false;

    if (readMem(Serial14CUXParams::Port1Offset, 1, &port1Data))
    {
        fuelPumpRelayState = !(port1Data & 0x40);
        retVal = true;
    }

    return retVal;
}

/**
 * Closes the fuel pump relay to run the pump for a single timeout period
 * (approximately two seconds).
 * @return True when the port and timer were written correctly; false
 *   otherwise.
 */
bool Comm14CUX::runFuelPump()
{
    bool retVal = false;
    uint8_t port1State = 0x00;

    if (readMem(Serial14CUXParams::Port1Offset, 1, &port1State) &&
        writeMem(Serial14CUXParams::FuelPumpTimerOffset, 0xFF) &&
        writeMem(Serial14CUXParams::Port1Offset, port1State & 0xBF))
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
bool Comm14CUX::driveIdleAirControlMotor(uint8_t direction, uint8_t steps)
{
    // Bit 0 in 0x008A determines the direction of motion;
    //  0 opens the valve and 1 closes it

    bool retVal = false;
    uint8_t iacDirection = 0x00;

    if (readMem(0x008A, 1, &iacDirection))
    {
        if (direction == 0)
        {
            iacDirection &= 0xFE;
        }
        else
        {
            iacDirection |= 0x01;
        }

        writeMem(0x008A, iacDirection);
        writeMem(Serial14CUXParams::IdleAirControlStepCountOffset, steps);
    }
}

/**
 * A hyperbolic curve model that describes the impedence response of
 * the 14CUX coolant/fuel temperature sensors.
 * Thanks to ZunZun.com for providing curve-fitting tools.
 * @param count Value measured by 14CUX ADC
 * @return Corresponding sensor value in degrees Fahrenheit
 */
double Comm14CUX::hyperbolicOffsetModel(double count)
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

