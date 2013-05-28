// libcomm14cux - a communications library for the Lucas 14CUX ECU
//
// setup.cpp: This file contains routines that perform the
//            setup/initialization of the library and the
//            serial port.

#if defined(WIN32) && defined(linux)
#error "Only one of 'WIN32' or 'linux' may be defined."
#endif

#include <unistd.h>
#include <fcntl.h>

#if defined(WIN32)
  #include <windows.h>
#else
  #include <string.h>
  #include <sys/ioctl.h>
  #include <termios.h>
  #include <arpa/inet.h>
#endif

#if defined(linux)
  #include <linux/serial.h>
#elif defined(__APPLE__)
  #include <IOKit/serial/ioss.h>
#endif

#include "comm14cux.h"
#include "comm14cux_internal.h"

#if defined(WIN32)
uint16_t swapShort(const uint16_t source)
{
    static const uint16_t hibyte = 0xff00;
    static const uint16_t lobyte = 0x00ff;

    return ((source & hibyte) >> 8) | ((source & lobyte) << 8);
}
#else
uint16_t swapShort(const uint16_t source)
{
    return ntohs(source);
}
#endif

void _14cux_init(cuxinfo *info)
{
    info->promRev = Comm14CUXDataOffsets_Unset;
    info->lastReadCoarseAddress = 0x0000;
    info->lastReadQuantity = 0x00;
    info->cancelRead = 0;
    info->lowestThrottleMeasurement = 0xffff;
    info->voltageFactorA = 0;
    info->voltageFactorB = 0;
    info->voltageFactorC = 0;

#if defined(WIN32)
    info->sd = INVALID_HANDLE_VALUE;
    info->mutex = CreateMutex(NULL, TRUE, NULL);
#else
    info->sd = 0;
    pthread_mutex_init(&info->mutex, NULL);
#endif
}

/**
 * Returns version information for this build of the library.
 */
Comm14CUXVersion _14cux_getLibraryVersion()
{
    Comm14CUXVersion ver;

    ver.major = COMM14CUX_VER_MAJOR;
    ver.minor = COMM14CUX_VER_MINOR;
    ver.patch = COMM14CUX_VER_PATCH;

    return ver;
}

/**
 * Closes the serial device.
 */
void _14cux_disconnect(cuxinfo *info)
{
#if defined(WIN32)
    if (WaitForSingleObject(info->mutex, INFINITE) == WAIT_OBJECT_0)
    {
        if (_14cux_isConnected(info))
        {
            CloseHandle(info->sd);
            info->sd = INVALID_HANDLE_VALUE;
        }

        ReleaseMutex(info->mutex);
    }
#else
    pthread_mutex_lock(&info->mutex);

    if (_14cux_isConnected(info))
    {
        close(info->sd);
        info->sd = 0;
    }

    pthread_mutex_unlock(&info->mutex);
#endif
}

/**
 * Opens the serial port (or returns with success if it is already open.)
 * @param devPath Full path to the serial device (e.g. "/dev/ttyUSB0" or "COM2")
 * @return 1 if the serial device was successfully opened and its
 *   baud rate was set; 0 otherwise.
 */
uint8_t _14cux_connect(cuxinfo *info, const char *devPath)
{
    uint8_t result = 0;

#if defined(WIN32)

    if (WaitForSingleObject(info->mutex, INFINITE) == WAIT_OBJECT_0)
    {
        result = _14cux_isConnected(info) || _14cux_openSerial(info, devPath);
        ReleaseMutex(info->mutex);
        dprintf_info("14CUX(info): Connected (Win32)\n");
    }
    else
    {
        dprintf_err("14CUX(error): Connect failed (Win32)\n");
    }

#else // Linux/Unix

    pthread_mutex_lock(&info->mutex);
    result = _14cux_isConnected(info) || _14cux_openSerial(info, devPath);
    pthread_mutex_unlock(&info->mutex);
    if (result)
    {
        dprintf_info("14CUX(info): Connected (Linux/Unix)\n");
    }
    else
    {
        dprintf_err("14CUX(error): Connect failed (Linux/Unix)\n");
    }

#endif

    return result;
}

/**
 * Opens the serial device for the USB<->RS-232 converter and sets the
 * parameters for the link to match those on the 14CUX.
 * Uses OS-specific calls to do this in the correct manner.
 * Note for FreeBSD users: Do not use the ttyX devices, as they block
 * on the open() call while waiting for a carrier detect line, which
 * will never be asserted. Instead, use the equivalent cuaX device.
 * Example: /dev/cuaU0 (instead of /dev/ttyU0)
 * @return 1 if the open/setup was successful, 0 otherwise
 */
uint8_t _14cux_openSerial(cuxinfo *info, const char *devPath)
{
    uint8_t retVal = 0;

#if defined(linux) || defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__) || defined(__APPLE__)
    // Most UNIXes can handle the serial port in a similar fashion (using
    // the termios interface.) The only major difference between them is
    // the assignment of the nonstandard baud rate, which is done directly
    // into the termios struct for BSD, but via ioctls for both Linux and OS X.

    struct termios newtio;
    uint8_t success = 1;

    dprintf_info("14CUX(info): Opening the serial device (%s)...\n", devPath);
    info->sd = open(devPath, O_RDWR | O_NOCTTY);

    if (info->sd > 0)
    {
        dprintf_info("14CUX(info): Opened device successfully.\n");

        if (tcgetattr(info->sd, &newtio) != 0)
        {
            dprintf_err("14CUX(error): Unable to read serial port parameters.\n");
            success = 0;
        }

        if (success)
        {
            // set up the serial port:
            // * enable the receiver, set 8-bit fields, set local mode, disable hardware flow control
            // * set non-canonical mode, disable echos, disable signals
            // * disable all software flow control
            // * disable all output post-processing
            newtio.c_cflag &= ((CREAD | CS8 | CLOCAL) & ~(CRTSCTS));
            newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
            newtio.c_oflag &= ~OPOST;

#if defined(linux) || defined(__APPLE__)
            // when waiting for responses, wait until we haven't received any
            // characters for a period of time before returning with failure
            newtio.c_cc[VTIME] = 1;
            newtio.c_cc[VMIN] = 0;

            // set the baud rate selector to 38400, which, in this case,
            // is simply an indicator that we're using a custom baud rate
            // (set by an ioctl() below)
            cfsetispeed(&newtio, B38400);
            cfsetospeed(&newtio, B38400);

#else // BSD and other UNIXes

            // This is set higher than the 0.1 seconds used by the Linux/OSX
            // code, as values much lower than this cause the first echoed
            // byte to be missed when running under BSD.
            newtio.c_cc[VTIME] = 5;
            newtio.c_cc[VMIN] = 0;

            // set the input and output baud rates to 7812
            cfsetispeed(&newtio, _14CUX_Baud);
            cfsetospeed(&newtio, _14CUX_Baud);
#endif

            // attempt to set the termios parameters
            dprintf_info("14CUX(info): Setting serial port parameters...\n");

            // flush the serial buffers and set the new parameters
            if ((tcflush(info->sd, TCIFLUSH) != 0) ||
                (tcsetattr(info->sd, TCSANOW, &newtio) != 0))
            {
                dprintf_err("14CUX(error): Failure setting up port\n");
                close(info->sd);
                success = 0;
            }
        }

#ifdef linux
        // Linux requires an ioctl() to set a nonstandard baud rate
        if (success)
        {
            struct serial_struct serial_info;
            dprintf_info("14CUX(info): Setting custom baud rate...\n");
            if (ioctl(info->sd, TIOCGSERIAL, &serial_info) != -1)
            {
                serial_info.flags = ASYNC_SPD_CUST | ASYNC_LOW_LATENCY;
                serial_info.custom_divisor = serial_info.baud_base / _14CUX_Baud;

                if (ioctl(info->sd, TIOCSSERIAL, &serial_info) != -1)
                {
                    dprintf_info("14CUX(info): Baud rate setting successful.\n");
                    retVal = 1;
                }
            }
        }
#elif defined(__APPLE__)
        // OS X requires an ioctl() to set a nonstandard baud rate
        if (success)
        {
            speed_t speed = _14CUX_Baud;
            if (ioctl(info->sd, IOSSIOSPEED, &speed) != -1)
            {
                retVal = 1;
            }
            else
            {
                dprintf_err("14CUX(error): Unable to set baud rate.\n");
            }
        }
#else
        retVal = success;
#endif

        // close the device if it couldn't be configured
        if (retVal == 0)
        {
            close(info->sd);
        }
    }
    else // open() returned failure
    {
        dprintf_err("14CUX(error): Error opening device (%s)\n", strerror(errno));
    }

#elif defined(WIN32)

    DCB dcb;
    COMMTIMEOUTS commTimeouts;

    // attempt to open the device
    dprintf_info("14CUX(info): Opening the serial device (Win32) '%s'...\n", devPath);

    // open and get a handle to the serial device
    info->sd = CreateFile(devPath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    // verify that the serial device was opened
    if (info->sd != INVALID_HANDLE_VALUE)
    {
        if (GetCommState(info->sd, &dcb) == TRUE)
        {
            // set the serial port parameters, including the custom baud rate
            dcb.BaudRate = _14CUX_Baud;
            dcb.fParity = FALSE;
            dcb.fOutxCtsFlow = FALSE;
            dcb.fOutxDsrFlow = FALSE;
            dcb.fDtrControl = FALSE;
            dcb.fRtsControl = FALSE;
            dcb.ByteSize = 8;
            dcb.Parity = 0;
            dcb.StopBits = 0;

            if ((SetCommState(info->sd, &dcb) == TRUE) &&
                (GetCommTimeouts(info->sd, &commTimeouts) == TRUE))
            {
                // modify the COM port parameters to wait 100 ms before timing out
                commTimeouts.ReadIntervalTimeout = 100;
                commTimeouts.ReadTotalTimeoutMultiplier = 0;
                commTimeouts.ReadTotalTimeoutConstant = 100;

                if (SetCommTimeouts(info->sd, &commTimeouts) == TRUE)
                {
                    retVal = 1;
                }
            }
        }

        // the serial device was opened, but couldn't be configured properly;
        // close it before returning with failure
        if (!retVal)
        {
            dprintf_err("14CUX(error): Failure setting up port; closing serial device...\n");
            CloseHandle(info->sd);
        }
    }
    else
    {
        dprintf_err("14CUX(error): Error opening device.\n");
    }

#endif

// Send the serial test pattern shortly after attempting a connection
// Enable this when debugging the serial port speed and operation
#if 0
    if (retVal)
    {
        dprintf_warn("14CUX(warn): Sending serial test pattern.\n");
        testWrite(); // send the pattern
        dprintf_warn("14CUX(warn): Serial test pattern done.\n");
    }
#endif
    return retVal;
}

/**
 * Checks the file descriptor for the serial device to determine if it has
 * already been opened.
 * @return 1 if the serial device is open; 0 otherwise.
 */
uint8_t _14cux_isConnected(cuxinfo* info)
{
#if defined(WIN32)
    return (info->sd != INVALID_HANDLE_VALUE);
#else
    return (info->sd > 0);
#endif
}

