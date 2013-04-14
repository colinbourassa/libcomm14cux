// libcomm14cux - a communications library for the Lucas 14CUX ECU
//
// setup.cpp: This file contains routines that perform the
//            setup/initialization of the library and the
//            serial port.

#if defined(WIN32) && defined(linux)
#error "Only one of 'WIN32' or 'linux' may be defined."
#endif

#if defined(ARDUINO)
  // Arduino-only includes
  #include <WProgram.h>
#else
  // Non-Arduino includes
  #include <unistd.h>
  #include <fcntl.h>

  #if defined(WIN32)
    // Windows-only includes
    #include <windows.h>
  #else
    // Non-Windows, Non-Arduino includes
    #include <string.h>
    #include <sys/ioctl.h>
    #include <termios.h>
    #include <arpa/inet.h>
  #endif

  #if defined(linux)
    // Linux-only includes
    #include <linux/serial.h>
  #elif defined(__APPLE__)
    #include <IOKit/serial/ioss.h>
  #endif
#endif

#include "comm14cux.h"

#if defined(WIN32) || defined(ARDUINO)
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

/**
 * Constructor. Initializes device handle and mutex.
 */
Comm14CUX::Comm14CUX() :
    m_promRev(Comm14CUXDataOffsets_Unset),
    m_lastReadCoarseAddress(0x0000),
    m_lastReadQuantity(0x00),
    m_cancelRead(false),
    m_lowestThrottleMeasurement(0xffff),
    m_voltageFactorA(0),
    m_voltageFactorB(0),
    m_voltageFactorC(0)
{
#if defined(ARDUINO)
    sd = 0;
#elif defined(WIN32)
    sd = INVALID_HANDLE_VALUE;
    s_mutex = CreateMutex(NULL, TRUE, NULL);
#else
    sd = 0;
    pthread_mutex_init(&s_mutex, NULL);
#endif
}

/**
 * Closes the serial device.
 */
Comm14CUX::~Comm14CUX()
{
    disconnect();

#if defined(WIN32) && !defined(ARDUINO)
    CloseHandle(s_mutex);
#elif !defined(ARDUINO)
    pthread_mutex_destroy(&s_mutex);
#endif
}

/**
 * Returns version information for this build of the library.
 */
Comm14CUXVersion Comm14CUX::getVersion()
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
void Comm14CUX::disconnect()
{
#if defined(ARDUINO)
    sd = 0;
#elif defined(WIN32)
    if (WaitForSingleObject(s_mutex, INFINITE) == WAIT_OBJECT_0)
    {
        if (isConnected())
        {
            CloseHandle(sd);
            sd = INVALID_HANDLE_VALUE;
        }

        ReleaseMutex(s_mutex);
    }
#else
    pthread_mutex_lock(&s_mutex);

    if (isConnected())
    {
        close(sd);
        sd = 0;
    }

    pthread_mutex_unlock(&s_mutex);
#endif
}

/**
 * Opens the serial port (or returns with success if it is already open.)
 * @param devPath Full path to the serial device (e.g. "/dev/ttyUSB0" or "COM2")
 * @return True if the serial device was successfully opened and its
 *   baud rate was set; false otherwise.
 */
bool Comm14CUX::connect(const char *devPath)
{
    bool result = false;

#if defined(ARDUINO)

    result = isConnected() || openSerial(devPath);
    if result
    {
        dprintf_info("14CUX(info): Connected (Arduino)\n");
    }
    else
    {
        dprintf_err("14CUX(error): Connect failed (Arduino)\n");
    }

#elif defined(WIN32)

    if (WaitForSingleObject(s_mutex, INFINITE) == WAIT_OBJECT_0)
    {
        result = isConnected() || openSerial(devPath);
        ReleaseMutex(s_mutex);
        dprintf_info("14CUX(info): Connected (Win32)\n");
    }
    else
    {
        dprintf_err("14CUX(error): Connect failed (Win32)\n");
    }

#else // Linux/Unix

    pthread_mutex_lock(&s_mutex);
    result = isConnected() || openSerial(devPath);
    pthread_mutex_unlock(&s_mutex);
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
 * @return True if the open/setup was successful, false otherwise
 */
bool Comm14CUX::openSerial(const char *devPath)
{
    bool retVal = false;

#ifdef ARDUINO

    sd->begin(Serial14CUXParams::Baud_14CUX);
    retVal = true;

#elif defined(linux) || defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__) || defined(__APPLE__)
    // Most UNIXes can handle the serial port in a similar fashion (using
    // the termios interface.) The only major difference between them is
    // the assignment of the nonstandard baud rate, which is done directly
    // into the termios struct for BSD, but via ioctls for both Linux and OS X.

	struct termios newtio;
    bool success = true;

    dprintf_info("14CUX(info): Opening the serial device (%s)...\n", devPath);
    sd = open(devPath, O_RDWR | O_NOCTTY);

	if (sd > 0)
	{
		dprintf_info("14CUX(info): Opened device successfully.\n");

        if (tcgetattr(sd, &newtio) != 0)
        {
            dprintf_err("14CUX(error): Unable to read serial port parameters.\n");
            success = false;
        }

        if (success)
        {
            // set up the serial port in non-canonical mode
            newtio.c_cflag = (CREAD | CS8 | CLOCAL);
            newtio.c_lflag = 0;

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
            cfsetispeed(&newtio, Serial14CUXParams::Baud_14CUX);
            cfsetospeed(&newtio, Serial14CUXParams::Baud_14CUX);
#endif

            // attempt to set the termios parameters
            dprintf_info("14CUX(info): Setting serial port parameters...\n");

            // flush the serial buffers and set the new parameters
            if ((tcflush(sd, TCIFLUSH) != 0) ||
                (tcsetattr(sd, TCSANOW, &newtio) != 0))
            {
                dprintf_err("14CUX(error): Failure setting up port\n");
                close(sd);
                success = false;
            }
        }

#ifdef linux
        // Linux requires an ioctl() to set a nonstandard baud rate
        if (success)
        {
            struct serial_struct serial_info;
            dprintf_info("14CUX(info): Setting custom baud rate...\n");
            if (ioctl(sd, TIOCGSERIAL, &serial_info) != -1)
            {
                serial_info.flags = ASYNC_SPD_CUST | ASYNC_LOW_LATENCY;
                serial_info.custom_divisor = serial_info.baud_base / Serial14CUXParams::Baud_14CUX;

                if (ioctl(sd, TIOCSSERIAL, &serial_info) != -1)
                {
                    dprintf_info("14CUX(info): Baud rate setting successful.\n");
                    retVal = true;
                }
            }
        }
#elif defined(__APPLE__)
        // OS X requires an ioctl() to set a nonstandard baud rate
        if (success)
        {
            speed_t speed = Serial14CUXParams::Baud_14CUX;
            if (ioctl(sd, IOSSIOSPEED, &speed) != -1)
            {
                retVal = true;
            }
            else
            {
                dprintf_err("14CUX(error): Unable to set baud rate.\n");
            }
        }
#endif
        // close the device if it couldn't be configured
        if (retVal == false)
        {
            close(sd);
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
    sd = CreateFile(devPath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    // verify that the serial device was opened
    if (sd != INVALID_HANDLE_VALUE)
    {
        if (GetCommState(sd, &dcb) == TRUE)
        {
            // set the serial port parameters, including the custom baud rate
            dcb.BaudRate = Serial14CUXParams::Baud_14CUX;
            dcb.fParity = FALSE;
            dcb.fOutxCtsFlow = FALSE;
            dcb.fOutxDsrFlow = FALSE;
            dcb.fDtrControl = FALSE;
            dcb.fRtsControl = FALSE;
            dcb.ByteSize = 8;
            dcb.Parity = 0;
            dcb.StopBits = 0;

            if ((SetCommState(sd, &dcb) == TRUE) &&
                (GetCommTimeouts(sd, &commTimeouts) == TRUE))
            {
                // modify the COM port parameters to wait 100 ms before timing out
                commTimeouts.ReadIntervalTimeout = 100;
                commTimeouts.ReadTotalTimeoutMultiplier = 0;
                commTimeouts.ReadTotalTimeoutConstant = 100;

                if (SetCommTimeouts(sd, &commTimeouts) == TRUE)
                {
                    retVal = true;
                }
            }
        }

        // the serial device was opened, but couldn't be configured properly;
        // close it before returning with failure
        if (!retVal)
        {
            dprintf_err("14CUX(error): Failure setting up port; closing serial device...\n");
            CloseHandle(sd);
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
 * @return True if the serial device is open; false otherwise.
 */
bool Comm14CUX::isConnected()
{
#if defined(ARDUINO)
    return (sd != 0);
#elif defined(WIN32)
    return (sd != INVALID_HANDLE_VALUE);
#else
    return (sd > 0);
#endif
}

