// libcomm14cux - a communications library for the Lucas 14CUX ECU
//
// protocol.c: This file contains routines specific to handling
//             the software protocol used by the ECU over its
//             serial link.

#include <time.h>
#include "comm14cux.h"
#include "comm14cux_internal.h"

#ifdef WIN32
  #define CLOCK_MONOTONIC_RAW CLOCK_MONOTONIC
#endif

int get_elapsed_ms(struct timespec* start, struct timespec* end)
{
  const int deltaMs = (end->tv_sec - start->tv_sec) * 1000 +
                      (end->tv_nsec - start->tv_nsec) / 1000000;
  return deltaMs;
}

/**
 * Cancels the pending read operation by aborting after the current read
 * command is finished.
 */
void c14cux_cancel_read(c14cux_info* info)
{
  info->cancelRead = 1;
}

/**
 * Reads bytes from the serial device using an OS-specific call.
 * @param buffer Buffer into which data should be read
 * @param quantity Number of bytes to read
 * @return Number of bytes read from the device, or -1 if reading failed
 */
int16_t c14cux_read_serial_bytes(c14cux_info* info, uint8_t* buf, uint16_t count)
{
  int retVal = -1;
  if (info->connected)
  {
    int readCount = 0;
    int ftdiStatus = 0;
    struct timespec start;
    struct timespec now;
    int elapsedMs = 0;

    if (clock_gettime(CLOCK_MONOTONIC_RAW, &start) == 0)
    {
      do {
        ftdiStatus = ftdi_read_data(&info->ftdi, buf + readCount, count - readCount);
        readCount += (ftdiStatus > 0) ? ftdiStatus : 0;
        if (readCount < count)
        {
          // TODO: experiment with a sleep delay here
          elapsedMs = (clock_gettime(CLOCK_MONOTONIC_RAW, &now) == 0) ? get_elapsed_ms(&start, &now) : C14CUX_TIMEOUT_MS;
        }
      } while ((readCount < count) && (ftdiStatus != -666) && (elapsedMs < C14CUX_TIMEOUT_MS));
      retVal = readCount;
    }
  }
  return retVal;
}

/**
 * Writes bytes to the serial device using an OS-specific call
 * @param buffer Buffer from which written data should be drawn
 * @param quantity Number of bytes to write
 * @return Number of bytes written to the device, or -1 if writing failed
 */
int16_t c14cux_write_serial_bytes(c14cux_info* info, uint8_t* buf, uint16_t count)
{
  int16_t retVal = -1;
  if (info->connected)
  {
    int ftdiStatus = 0;
    int numWritten = 0;
    struct timespec start;
    struct timespec now;
    int elapsedMs = 0;

    if (clock_gettime(CLOCK_MONOTONIC_RAW, &start) == 0)
    {
      do {
        ftdiStatus = ftdi_write_data(&info->ftdi, buf + numWritten, count - numWritten);
        numWritten += (ftdiStatus > 0) ? ftdiStatus : 0;
        if (numWritten < count)
        {
          elapsedMs = (clock_gettime(CLOCK_MONOTONIC_RAW, &now) == 0) ?
                      get_elapsed_ms(&start, &now) : C14CUX_TIMEOUT_MS;
        }
      } while ((numWritten < count) && (ftdiStatus != -666) && (elapsedMs < C14CUX_TIMEOUT_MS));
      retVal = numWritten;
    }
  }

  return retVal;
}

/**
 * Reads the specified number of bytes from memory at the specified address.
 * @param info State information for the active connection.
 * @param addr Address at which memory should be read.
 * @param len Number of bytes to read.
 * @param buffer Pointer to a buffer of at least len bytes
 * @return 1 when the operation was successful; 0 otherwise
 */
bool c14cux_read_mem(c14cux_info* info, uint16_t addr, uint16_t len, uint8_t* buffer)
{
  pthread_mutex_lock(&info->mutex);

  uint16_t totalBytesRead = 0;
  uint16_t singleReqQuantity = 0;
  uint16_t singleReqBytesRead = 0;
  bool readSuccess = false;
  int16_t readCallBytesRead = 1;
  bool sendLastByteOnly = false;

  info->cancelRead = 0;

  if (info->connected)
  {
    // loop until we've read all the bytes, or we experienced a read error
    while ((totalBytesRead < len) && (readCallBytesRead > 0) && (info->cancelRead == 0))
    {
      // read the maximum number of bytes as is reasonable
      singleReqQuantity = c14cux_get_byte_count_for_next_read(len, totalBytesRead);

      // if the next address to read is within the 64-byte window
      // created by the last coarse address that was set, then we
      // can just send the final byte of the read command
      sendLastByteOnly =
        ((singleReqQuantity == info->lastReadQuantity) &&
         ((addr + totalBytesRead) < (info->lastReadCoarseAddress + 64)) &&
         (info->lastReadCoarseAddress <= (addr + totalBytesRead)));

      dprintf_info("14CUX(info): Sending cmd to read %d bytes at 0x%04X...\n",
                   singleReqQuantity, addr + totalBytesRead);

      // if sending the read command is successful...
      if (c14cux_send_read_cmd(info, addr + totalBytesRead, singleReqQuantity, sendLastByteOnly))
      {
        dprintf_info("14CUX(info): Successfully sent read command.\n");

        // reset the number of bytes read during this single read operation
        singleReqBytesRead = 0;

        // loop until we've read all the bytes for this single read operation,
        // or until we time out
        do
        {
          readCallBytesRead =
            c14cux_read_serial_bytes(info, buffer + totalBytesRead + singleReqBytesRead,
                                     singleReqQuantity - singleReqBytesRead);

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
          info->lastReadQuantity = singleReqQuantity;
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
    info->lastReadCoarseAddress = 0;
    info->lastReadQuantity = 0;
  }

  pthread_mutex_unlock(&info->mutex);

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
bool c14cux_send_read_cmd(c14cux_info* info, uint16_t addr, uint16_t len, bool lastByteOnly)
{
  bool success = 0;
  uint8_t cmdByte = 0;

  // do the command-agnostic coarse address setup
  // (req'd for both reads and writes)
  if (lastByteOnly || c14cux_set_read_coarse_addr(info, addr, len))
  {
    // if we actually did send the command bytes to set a new coarse
    // address, then remember what it was
    if (!lastByteOnly)
    {
      info->lastReadCoarseAddress = addr;
    }

    // build and send the byte for the Read command
    cmdByte = ((addr & 0x003F) | 0xC0);
    dprintf_info("14CUX(info): Sending Read command byte: 0x%02X... (no echo)\n", cmdByte);

    if (c14cux_write_serial_bytes(info, &cmdByte, 1) == 1)
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
bool c14cux_set_read_coarse_addr(c14cux_info* info, uint16_t addr, uint16_t len)
{
  return c14cux_set_coarse_addr(info, addr, len);
}

/**
 * Sends command bytes to the 14CUX to set the coarse address for a write
 * operation. The coarse address consists of the upper 10 bits of the 16-bit
 * address.
 * @param addr Memory address at which the read will occur.
 * @return 1 when the bytes were successfully sent to the 14CUX and the
 *   same bytes were echoed in reply; 0 otherwise.
 */
bool c14cux_set_write_coarse_addr(c14cux_info* info, uint16_t addr)
{
  return c14cux_set_coarse_addr(info, addr, 0x00);
}

/**
 * Sends command bytes to the 14CUX to set the coarse address for either a
 * read or write operation. If the command is ultimately a write operation,
 * the 14CUX will ignore the length.
 * @param addr Memory address at which the read or write will occur.
 * @param len Number of bytes to retrieve when the read operation is executed.
 * @return True when the command bytes were sent and echoed properly; false otherwise
 */
bool c14cux_set_coarse_addr(c14cux_info* info, uint16_t addr, uint16_t len)
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
  else if (len <= C14CUX_ReadCount0)
  {
    // if we're reading between 1 and 16 bytes, set the byte value
    // to (len - 1), which is interpreted by the 14CUX as (len)
    firstByte = len - 1;
  }
  else
  {
    // otherwise, we check to ensure that len is one of the allowed
    // preset values; if it isn't, return failure
    if (len == C14CUX_ReadCount1)
    {
      firstByte = C14CUX_ReadCount1Value;
    }
    else if (len == C14CUX_ReadCount2)
    {
      firstByte = C14CUX_ReadCount2Value;
    }
    else if (len == C14CUX_ReadCount3)
    {
      firstByte = C14CUX_ReadCount3Value;
    }
    else if (len == C14CUX_ReadCount4)
    {
      firstByte = C14CUX_ReadCount4Value;
    }
    else
    {
      dprintf_err("14CUX(error): Invalid length.\n");
      return 0;
    }
  }

  // bit 7 is fixed low; bits 6:2 are the read quantity,
  // and bits 1:0 are the top two bits (15:14) of the address
  firstByte <<= 2;
  firstByte |= (addr >> 14);

  dprintf_info("14CUX(info): Sending byte: 0x%02X...\n", firstByte);

  if ((c14cux_write_serial_bytes(info, &firstByte, 1) == 1) &&
      (c14cux_read_serial_bytes(info, &readByte, 1) == 1) &&
      (readByte == firstByte))
  {
    // bits 13:6 of the address
    secondByte = ((addr >> 6) & 0x00ff);

    dprintf_info("14CUX(info): Sending byte: 0x%02X...\n", secondByte);

    if ((c14cux_write_serial_bytes(info, &secondByte, 1) == 1) &&
        (c14cux_read_serial_bytes(info, &readByte, 1) == 1) &&
        (readByte == secondByte))
    {
      retVal = true;
    }
  }

  return retVal;
}

/**
 * Writes the specified byte to the specified location in memory.
 * @param info State information for the active connection.
 * @param addr 16-bit address at which to write
 * @param val 8-bit value to write at specified location
 * @return True when byte was written successfully; false otherwise
 */
bool c14cux_write_mem(c14cux_info* info, uint16_t addr, uint8_t val)
{
  pthread_mutex_lock(&info->mutex);

  bool retVal = false;
  uint8_t cmdByte = 0x00;
  uint8_t readByte = 0x00;

  info->lastReadCoarseAddress = 0;
  info->lastReadQuantity = 0;

  if (info->connected && c14cux_set_write_coarse_addr(info, addr))
  {
    // build the next byte in the command
    cmdByte = ((addr & 0x003F) | 0x80);

    // send (and look for the echo of) the third command byte
    dprintf_info("14CUX(info): Sending byte: 0x%02X...\n", cmdByte);

    if ((c14cux_write_serial_bytes(info, &cmdByte, 1) == 1) &&
        (c14cux_read_serial_bytes(info, &readByte, 1) == 1) &&
        (readByte == cmdByte))
    {
      // send (and look for the echo of) the byte
      // containing the value to write
      dprintf_info("14CUX(info): Sending byte: 0x%02X...\n", val);

      if ((c14cux_write_serial_bytes(info, &val, 1) == 1) &&
          (c14cux_read_serial_bytes(info, &readByte, 1) == 1) &&
          (readByte == val))
      {
        retVal = true;
      }
    }
  }
  pthread_mutex_unlock(&info->mutex);

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
uint16_t c14cux_get_byte_count_for_next_read(uint16_t len, uint16_t bytesRead)
{
  const uint16_t bytesLeft = len - bytesRead;
  uint16_t retCount = 0;

  if (bytesLeft >= C14CUX_ReadCount4)
  {
    retCount = C14CUX_ReadCount4;
  }
  else if (bytesLeft >= C14CUX_ReadCount3)
  {
    retCount = C14CUX_ReadCount3;
  }
  else if (bytesLeft >= C14CUX_ReadCount2)
  {
    retCount = C14CUX_ReadCount2;
  }
  else if (bytesLeft >= C14CUX_ReadCount1)
  {
    retCount = C14CUX_ReadCount1;
  }
  else if (bytesLeft >= C14CUX_ReadCount0)
  {
    retCount = C14CUX_ReadCount0;
  }
  else
  {
    retCount = bytesLeft;
  }

  return retCount;
}

