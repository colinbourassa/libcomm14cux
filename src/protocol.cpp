// libcomm14cux - a communications library for the Lucas 14CUX ECU
//
// protocol.cpp: This file contains routines specific to handling
//               the software protocol used by the ECU over its
//               serial link.

#if defined(WIN32) && defined(linux)
#error "Only one of 'WIN32' or 'linux' may be defined."
#endif

#if defined(ARDUINO)
  // Arduino-only includes
  #include <WProgram.h>
#else
  // Non-Arduino includes
  #include <unistd.h>

  #if defined(WIN32)
    // Windows-only includes
    #include <windows.h>
  #endif
#endif

#include "comm14cux.h"

/**
 * Cancels the pending read operation by aborting after the current read
 * command is finished.
 */
void Comm14CUX::cancelRead()
{
    m_cancelRead = true;
}

/**
 * Reads bytes from the serial device using an OS-specific call.
 * @param buffer Buffer into which data should be read
 * @param quantity Number of bytes to read
 * @return Number of bytes read from the device, or -1 if no bytes could be read
 */
int16_t Comm14CUX::readSerialBytes(uint8_t* buffer, uint16_t quantity)
{
    int16_t bytesRead = -1;

    if (isConnected())
    {
#if defined(ARDUINO)
        char c = -1;
        unsigned long timeout = millis() + 100;

        while ((millis() < timeout) && (bytesRead < quantity))
        {
            // try to read a byte
            c = sd->read();

            // if a byte was read...
            if (c != -1)
            {
                // indicate that we've received at least one char
                if (bytesRead < 0)
                {
                    bytesRead = 1;
                }

                // save it to the buffer
                *buffer = (uint8_t)c;
                buffer++;
                bytesRead++;
            }
        }
#elif defined(WIN32)
        DWORD w32BytesRead = 0;
        if ((ReadFile(sd, (UCHAR *) buffer, quantity, &w32BytesRead, NULL) == TRUE) &&
            (w32BytesRead > 0))
        {
            bytesRead = w32BytesRead;
        }
#elif defined(linux) || defined(__FreeBSD__)
        bytesRead = read(sd, buffer, quantity);
#else // other Unix (incl. Mac OS X)
        int sel_nr = 1;         // init to get into the loop
        int read_cnt = 0;

        // There are subtle differences in read(,,timeout), select(), & ARDUINO
        // read() waits till count, or the line has been idle for time, before returning (perhaps some chars)
        // select() waits for time before reading (looping as necessary)
        // ARDUINO requires all chars and stops reading at time

        bytesRead = 0;
        while ((bytesRead < quantity) && (sel_nr > 0))
        {
            comms_timeout.tv_sec = 0;
            comms_timeout.tv_usec = 100000;

            // setsize, read set, write set error set, timeout
            sel_nr = select(sd + 1, &sds, NULL, NULL, &comms_timeout);

            if (sel_nr < 0)
            {
                // quickly using non thread save strerror, perror to std err is alternative
                dprintf_err("14CUX(error): select() error %d %s\n", errno, strerror(errno));
            }

            if (sel_nr == 1)
            {
                // our descriptor is now ready with something; read some bytes
                read_cnt = read(sd, buffer, quantity - bytesRead);

                if (read_cnt > 0)
                {
                    bytesRead += read_cnt;
                    buffer += read_cnt; // move the index
                }

                if (read_cnt <= 0)
                {
                    dprintf_err("14CUX(error): Unexpected read result %d\n", read_cnt);
                    return -1;
                }

                dprintf_info("14CUX(info): Read %d byte(s)\n", read_cnt);
            }

            if (sel_nr > 1)
            {
                dprintf_err("14CUX(error): Error (internal) select() more than 1 descriptor!\n");
                return -1;
            }
        }                       // while
#endif
    }
    else
    {
        dprintf_warn("14CUX(warning): Not connected.\n");
    }

    return bytesRead;
}

/**
 * Writes bytes to the serial device using an OS-specific call
 * @param buffer Buffer from which written data should be drawn
 * @param quantity Number of bytes to write
 * @return Number of bytes written to the device, or -1 if no bytes could be written
 */
int16_t Comm14CUX::writeSerialBytes(uint8_t* buffer, uint16_t quantity)
{
    int16_t bytesWritten = -1;

    if (isConnected())
    {
#if defined(ARDUINO)
        bytesWritten = 0;
        while (bytesWritten < quantity)
        {
            sd->print(*buffer);
            buffer++;
            bytesWritten++;
        }
#elif defined(WIN32)
        DWORD w32BytesWritten = 0;
        if ((WriteFile(sd, (UCHAR *) buffer, quantity, &w32BytesWritten, NULL) == TRUE) &&
            (w32BytesWritten == quantity))
        {
            bytesWritten = w32BytesWritten;
        }
#else
        bytesWritten = write(sd, buffer, quantity);
#endif
    }

    return bytesWritten;
}


/**
 * Writes a test pattern so the baud rate can be validated on an oscilloscope
 */
void Comm14CUX::testWrite(void)
{
    // Serial Comms:-
    // On RS232 data lines, Mark is negative, Space is positive Voltage
    // RS232 controll line, ON is positive, OFF is negative
    // RS232 drivers are usually inverting.
    // UART and other serial ICs like MCUs are designed to work with inverting drivers
    //
    // The 14CUX uses levels between 0 and approximatly 12V
    // The output from the 14CUX MCU is inverting
    // The input to the MCU is direct with voltage limiting (non-inverting)
    //
    // Notation:-
    //     Md = data mark
    //     Sd = data space
    //     Ss = Start space
    //     Ms = Stop mark
    //     Mi = Idle mark (often a 0 or variable length, can be a large multiple of 16 (or 8) x UART BRG clock
    //     PB = Parity Bit
    // Useful patterns:-
    //     0x55     [Mi] Ss Md Sd Md Sd Md Sd Md Sd Ms [Mi]
    //     0xFF     [Mi] Ss Md Md Md Md Md Md Md Md Ms [Mi]
    //
    // Each character is 10 bits.
    // bit time is 1/7812
    // 781.2 characters will last 1 second
    // 3906 will last 5 seconds
    uint8_t c = 0xff;
    int i;

    for (i = 0; i < 3906; i++)
    {
        if (writeSerialBytes(&c, 1) != 1)
        {
            dprintf_err("14CUX(error): Failed to write test pattern. cnt=%d\n", i);
            return;
        }
    }
}


/**
 * Reads the specified number of bytes from memory at the specified address.
 * @param addr Address at which memory should be read.
 * @param len Number of bytes to read.
 * @param buffer Pointer to a buffer of at least len bytes
 * @return True when the operation was successful; false otherwise
 */
bool Comm14CUX::readMem(uint16_t addr, uint16_t len, uint8_t* buffer)
{
#if defined(WIN32) && !defined(ARDUINO)
    if (WaitForSingleObject(s_mutex, INFINITE) != WAIT_OBJECT_0)
    {
        return false;
    }
#elif !defined(ARDUINO)
    pthread_mutex_lock(&s_mutex);
#endif

    uint16_t totalBytesRead = 0;
    uint16_t singleReqQuantity = 0;
    uint16_t singleReqBytesRead = 0;
    bool readSuccess = false;
    int16_t readCallBytesRead = 1;
    bool sendLastByteOnly = false;

    m_cancelRead = false;

    if (isConnected())
    {
        // loop until we've read all the bytes, or we experienced a read error
        while ((totalBytesRead < len) && (readCallBytesRead > 0) && (m_cancelRead == false))
        {
            // read the maximum number of bytes as is reasonable
            singleReqQuantity = getByteCountForNextRead(len, totalBytesRead);

            // if the next address to read is within the 64-byte window
            // created by the last coarse address that was set, then we
            // can just send the final byte of the read command
            sendLastByteOnly =
                ((singleReqQuantity == m_lastReadQuantity) &&
                 ((addr + totalBytesRead) < (m_lastReadCoarseAddress + 64)) &&
                 (m_lastReadCoarseAddress <= (addr + totalBytesRead)));

            dprintf_info
                ("14CUX(info): Sending cmd to read %d bytes at 0x%04X...\n", singleReqQuantity, addr + totalBytesRead);

            // if sending the read command is successful...
            if (sendReadCmd(addr + totalBytesRead, singleReqQuantity, sendLastByteOnly))
            {
                dprintf_info("14CUX(info): Successfully sent read command.\n");

                // reset the number of bytes read during this single read operation
                singleReqBytesRead = 0;

                // loop until we've read all the bytes for this single read operation,
                // or until we time out
                do
                {
                    readCallBytesRead =
                        readSerialBytes(buffer + totalBytesRead +
                                        singleReqBytesRead, singleReqQuantity - singleReqBytesRead);

                    singleReqBytesRead += readCallBytesRead;
                }
                while ((readCallBytesRead > 0) && (singleReqBytesRead < singleReqQuantity));

                // if all the reads were successful, add the total bytes
                // read from this request to the overall total
                if (readCallBytesRead > 0)
                {
                    dprintf_info("14CUX(info): Successfully read %d bytes.\n", singleReqBytesRead);
                    totalBytesRead += singleReqBytesRead;

                    // remember the number of bytes read on this pass, in case we
                    // want to issue an abbreviated command next time (which will
                    // send the same number of bytes)
                    m_lastReadQuantity = singleReqQuantity;
                }
            }
            else
            {
                // if we were unable to even send the read command,
                // stop with failure
                dprintf_err("14CUX(error): Failed to send read command\n");
                readCallBytesRead = -1;
            }
        }
    }

    // if we read as many bytes as were requested, indicate success
    if (totalBytesRead == len)
    {
        dprintf_info("14CUX(info): Successfully read all requested bytes.\n");
        readSuccess = true;
    }
    else
    {
        m_lastReadCoarseAddress = 0;
        m_lastReadQuantity = 0;
    }

#if defined(WIN32) && !defined(ARDUINO)
    ReleaseMutex(s_mutex);
#elif !defined(ARDUINO)
    pthread_mutex_unlock(&s_mutex);
#endif

    return readSuccess;
}

/**
 * Sends the three bytes comprising a read command to the 14CUX.
 * @param addr 16-bit address at which to read
 * @param len Number of bytes to read
 * @param lastByteOnly Send only the last byte of the read command; this will
 *      work only if the coarse address was previously set.
 * @return True when the read command is successfully sent; false otherwise
 */
bool Comm14CUX::sendReadCmd(uint16_t addr, uint16_t len, bool lastByteOnly)
{
    bool success = false;
    uint8_t cmdByte = 0;

    // do the command-agnostic coarse address setup
    // (req'd for both reads and writes)
    if (lastByteOnly || setReadCoarseAddr(addr, len))
    {
        // if we actually did send the command bytes to set a new coarse
        // address, then remember what it was
        if (!lastByteOnly)
        {
            m_lastReadCoarseAddress = addr;
        }

        // build and send the byte for the Read command
        cmdByte = ((addr & 0x003F) | 0xC0);
        dprintf_info("14CUX(info): Sending Read command byte: 0x%02X... (no echo)\n", cmdByte);
        if (writeSerialBytes(&cmdByte, 1) == 1)
        {
            // The 14CUX doesn't echo the Read command byte;
            // it simply starts sending the requested data.
            // Because of this, we can consider the command
            // successfully sent without checking for an echo
            // of the last byte.
            success = true;
        }
        else
        {
            dprintf_err("14CUX(error): Faled to write byte!\n");
        }
    }

    return success;
}

/**
 * Sends command bytes to the 14CUX to set the coarse address and read length
 * for a read operation. The coarse address consists of the upper 10 bits of
 * the 16-bit address.
 * @param addr Memory address at which the read will occur.
 * @param len Number of bytes to retrieve when the read operation is executed.
 * @return True when a valid read length was supplied, the bytes were
 *   successfully sent to the 14CUX, and the same bytes were echoed in reply;
 *   false otherwise.
 */
bool Comm14CUX::setReadCoarseAddr(uint16_t addr, uint16_t len)
{
    return setCoarseAddr(addr, len);
}

/**
 * Sends command bytes to the 14CUX to set the coarse address for a write
 * operation. The coarse address consists of the upper 10 bits of the 16-bit
 * address.
 * @param addr Memory address at which the read will occur.
 * @return True when the bytes were successfully sent to the 14CUX and the
 *   same bytes were echoed in reply; false otherwise.
 */
bool Comm14CUX::setWriteCoarseAddr(uint16_t addr)
{
    return setCoarseAddr(addr, 0x00);
}

/**
 * Sends command bytes to the 14CUX to set the coarse address for either a
 * read or write operation. If the command is ultimately a write operation,
 * the 14CUX will ignore the length.
 * @param addr Memory address at which the read or write will occur.
 * @param len Number of bytes to retrieve when the read operation is executed.
 * @return True when the command bytes were sent and echoed properly; false otherwise
 */
bool Comm14CUX::setCoarseAddr(uint16_t addr, uint16_t len)
{
    uint8_t firstByte = 0x00;
    uint8_t secondByte = 0x00;
    uint8_t readByte = 0x00;
    bool retVal = false;

    dprintf_info("14CUX(info): Sending command to set coarse address...\n");

    if (len == 0)
    {
        // if zero was passed for the length, we assume that this address
        // selection is being done for a write operation (which does not
        // require a quantity)
        firstByte = 0;
    }
    else if (len <= Serial14CUXParams::ReadCount0)
    {
        // if we're reading between 1 and 16 bytes, set the byte value
        // to (len - 1), which is interpreted by the 14CUX as (len)
        firstByte = len - 1;
    }
    else
    {
        // otherwise, we check to ensure that len is one of the allowed
        // preset values; if it isn't, return failure
        switch (len)
        {
        case Serial14CUXParams::ReadCount1:
            firstByte = Serial14CUXParams::ReadCount1Value;
            break;
        case Serial14CUXParams::ReadCount2:
            firstByte = Serial14CUXParams::ReadCount2Value;
            break;
        case Serial14CUXParams::ReadCount3:
            firstByte = Serial14CUXParams::ReadCount3Value;
            break;
        case Serial14CUXParams::ReadCount4:
            firstByte = Serial14CUXParams::ReadCount4Value;
            break;
        default:
            dprintf_err("14CUX(error): Invalid length.\n");
            return false;
        }
    }

    // bit 7 is fixed low; bits 6:2 are the read quantity,
    // and bits 1:0 are the top two bits (15:14) of the address
    firstByte <<= 2;
    firstByte |= (addr >> 14);

    dprintf_info("14CUX(info): Sending byte: 0x%02X...\n", firstByte);

    if ((writeSerialBytes(&firstByte, 1) == 1) &&
        (readSerialBytes(&readByte, 1) == 1) &&
        (readByte == firstByte))
    {
        // bits 13:6 of the address
        secondByte = ((addr >> 6) & 0x00ff);

        dprintf_info("14CUX(info): Sending byte: 0x%02X...\n", secondByte);

        if ((writeSerialBytes(&secondByte, 1) == 1) &&
            (readSerialBytes(&readByte, 1) == 1) &&
            (readByte == secondByte))
        {
            retVal = true;
        }
    }

    return retVal;
}

/**
 * Writes the specified byte to the specified location in memory.
 * @param addr 16-bit address at which to write
 * @param val 8-bit value to write at specified location
 * @return True when byte was written successfully; false otherwise
 */
bool Comm14CUX::writeMem(uint16_t addr, uint8_t val)
{
#if defined(WIN32) && !defined(ARDUINO)
    if (WaitForSingleObject(s_mutex, INFINITE) != WAIT_OBJECT_0)
    {
        return false;
    }
#elif !defined(ARDUINO)
    pthread_mutex_lock(&s_mutex);
#endif

    bool retVal = false;
    uint8_t cmdByte = 0x00;
    uint8_t readByte = 0x00;

    m_lastReadCoarseAddress = 0;
    m_lastReadQuantity = 0;

    if (isConnected() && setWriteCoarseAddr(addr))
    {
        // build the next byte in the command
        cmdByte = ((addr & 0x003F) | 0x80);

        // send (and look for the echo of) the third command byte
        dprintf_info("14CUX(info): Sending byte: 0x%02X...\n", cmdByte);
        if ((writeSerialBytes(&cmdByte, 1) == 1) &&
            (readSerialBytes(&readByte, 1) == 1) &&
            (readByte == cmdByte))
        {
            // send (and look for the echo of) the byte
            // containing the value to write
            dprintf_info("14CUX(info): Sending byte: 0x%02X...\n", val);
            if ((writeSerialBytes(&val, 1) == 1) &&
                (readSerialBytes(&readByte, 1) == 1) &&
                (readByte == val))
            {
                retVal = true;
            }
        }
    }

#if defined(WIN32) && !defined(ARDUINO)
    ReleaseMutex(s_mutex);
#elif !defined(ARDUINO)
    pthread_mutex_unlock(&s_mutex);
#endif

    return retVal;
}

/**
 * Determines the number of bytes that should be requested during the next
 * read operation, based on the total requested, number read thus far, and
 * the quantities allowed by the 14CUX in a single read.
 * @len Total number of bytes being read over multiple read requests
 * @bytesRead Number of bytes read thus far
 * @return Number of bytes to request for the next single read operation
 */
uint16_t Comm14CUX::getByteCountForNextRead(uint16_t len, uint16_t bytesRead)
{
    uint16_t bytesLeft = len - bytesRead;
    uint16_t retCount = 0;

    if (bytesLeft >= Serial14CUXParams::ReadCount4)
    {
        retCount = Serial14CUXParams::ReadCount4;
    }
    else if (bytesLeft >= Serial14CUXParams::ReadCount3)
    {
        retCount = Serial14CUXParams::ReadCount3;
    }
    else if (bytesLeft >= Serial14CUXParams::ReadCount2)
    {
        retCount = Serial14CUXParams::ReadCount2;
    }
    else if (bytesLeft >= Serial14CUXParams::ReadCount1)
    {
        retCount = Serial14CUXParams::ReadCount1;
    }
    else if (bytesLeft >= Serial14CUXParams::ReadCount0)
    {
        retCount = Serial14CUXParams::ReadCount0;
    }
    else
    {
        retCount = bytesLeft;
    }

    return retCount;
}

