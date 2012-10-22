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
  #elif !defined(WIN32)
    // Other UNIXes (incl. Mac OS X)

    // There is no serial.h in Mac OS X;
    // to get around all sorts of serial I/O problems, enable use of
    // the select() call to manage IO timeouts
    #define SELECT
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
#if !defined(linux)
    FD_ZERO(&sds);
#endif
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
#if !defined(linux)
        // for other UNIXes
        FD_ZERO(&sds);
#endif
    }

    pthread_mutex_unlock(&s_mutex);
#endif
}

/**
 * Opens the serial port (or returns with success if it is already open.)
 * @param devPath Full path to the serial device (e.g. "/dev/ttyUSB0")
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
        dprintf_info("14CUX(info): Done connect (Arduino)\n");
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
        dprintf_info("14CUX(info): Done connect (WIN32)\n");
    }
    else
    {
        dprintf_err("14CUX(error): Connect failed (WIN32)\n");
    }

#else // Linux/Unix

    pthread_mutex_lock(&s_mutex);
    result = isConnected() || openSerial(devPath);
    pthread_mutex_unlock(&s_mutex);
    if (result)
    {
        dprintf_info("14CUX(info): Done connect (Linux/Unix)\n");
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
 * @return True if the open/setup was successful, false otherwise
 */
bool Comm14CUX::openSerial(const char *devPath)
{
    bool retVal = false;

#ifdef ARDUINO

    sd->begin(Serial14CUXParams::Baud_14CUX);
    retVal = true;

#elif defined(linux)

    struct termios newtio;
    struct serial_struct serial_info;

    // attempt to open the device
    dprintf_info("14CUX(info): Opening the serial device...\n");
    sd = open(devPath, O_RDWR | O_NOCTTY);

    if (sd > 0)
    {
        dprintf_info("14CUX(info): Opened device successfully.\n");
        memset(&newtio, 0, sizeof(newtio));

        newtio.c_cflag = (CREAD | CS8 | CLOCAL);

        // set non-canonical mode
        newtio.c_lflag = 0;

        // when waiting for responses, wait until we haven't received
        // any characters for one-tenth of a second before timing out
        newtio.c_cc[VTIME] = 1;
        newtio.c_cc[VMIN] = 0;

        // set the baud rate selector to 38400, which, in this case,
        // is simply an indicator that we're using a custom baud rate
        // (set by an ioctl() below)
        cfsetispeed(&newtio, B38400);
        cfsetospeed(&newtio, B38400);

        // attempt to set the termios parameters
        dprintf_info("14CUX(info): Setting serial port parameters (except baud)...\n");
        if ((tcflush(sd, TCIFLUSH) == 0) &&
            (tcsetattr(sd, TCSANOW, &newtio) == 0))
        {
            // attempt to set a custom baud rate
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

        // the serial device was opened, but couldn't be configured properly;
        // close it before returning with failure
        if (!retVal)
        {
            dprintf_err("14CUX(error): Failure setting up port; closing serial device...\n");
            close(sd);
        }
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
            dprintf_warn("14CUX(warning): Failure setting up port; closing serial device...\n");
            CloseHandle(sd);
        }
    }
    else
    {
        dprintf_err("14CUX(error): CreateFile() returned INVALID_HANDLE_VALUE (WIN32)\n");
    }

#else
    // Unix (incl. Mac OS X)

    struct termios newtio;
    // struct serial_struct serial_info;

    // attempt to open the device
    dprintf_info("14CUX(info): Opening the serial device (Unix) ...\n");
    // open for read write, not controlling terminal, and ignore DCD line (i.e. no blocking )
    sd = open(devPath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    // 0_NONBLOCK is like O_NDELAY, and lets you open (and maybe read) without DCD
    // But this is reported to disable VTIME on some systems :-(

    if (sd < 0)
    {
        dprintf_err("14CUX(error): USB TTY open() failed, errno %d, %s\n", errno, strerror(errno));
    }
    else
    {
        FD_SET(sd, &sds); // Add the opened file descriptor to our select() set

        dprintf_info("14CUX(info): Opened device successfully.\n");
        memset(&newtio, 0, sizeof(newtio)); // we probably should fill this out with tcgetattr(), then make changes.

        newtio.c_cflag = (CREAD | CS8 | CLOCAL);

        // set non-canonical mode
        newtio.c_lflag = 0; // This probably disables VTIME, below

        // when waiting for responses, wait until we haven't received
        // any characters for one-tenth of a second before timing out
        // Ment to work when ICANON is disabled, OS X ???
        newtio.c_cc[VTIME] = 1; // VTIME may not be honored on Mac OS X read, use select() to read [or alarm()]
        newtio.c_cc[VMIN] = 0;

        // set the baud rate selector to 38400, which, in this case,
        // is simply an indicator that we're using a custom baud rate
        // (set by an ioctl() below), some documents suggest B38400 is in some way an initial magic value?
        cfsetispeed(&newtio, B38400);
        cfsetospeed(&newtio, B38400);

        // attempt to set the termios parameters
        dprintf_info("14CUX(info): Setting serial port parameters (except baud)...\n");

        // Make sure file descriptor is for a TTY device
        if (!isatty(sd))
        {
            dprintf_err("14CUX(error): File descriptor not a TTY device (openSerial()).\n");
        }
        else if (tcflush(sd, TCIFLUSH) != 0)    // flush unread output
        {
            dprintf_err("14CUX(error): tcflush() input failed, errno %d, %s\n", errno, strerror(errno));
        }
        else if (tcsetattr(sd, TCSANOW, &newtio) != 0)  // set the termios struct, term.h (ttycom.h TIOCSETA)
        {
            dprintf_err
                ("14CUX(error): tcsetattr() failed to set termios struct, errno %d, %s\n", errno, strerror(errno));
        }
        else
        {
            // attempt to set a custom baud rate
            dprintf_info("14CUX(info): Setting custom baud rate...\n");

            // speed_t in /Developer//SDKs/MacOSX10.5.sdk/usr/include/sys/termios.h  (unsigned long)
            // _IOW in /usr/include/sys/ioccom.h
            // The below IOCTL fails in x86_64 mode as speed_t goes from 4 to 8 bytes and is lost
            // It would seem the IOCTL requires the speed to be a uint32 to work, even in x86_64 - at least on 10.5.8
            // The typing and sizeof for this is all a bit sloppy, and clarification of the IOSSIOSPEED request would be appreciated.
            // Perhaps it is dependent on the com port hardware driver, but this is at least now working on 10.5.8 with a USB FTDI adapter.
            #define IOSSIOSPEED _IOW('T', 2, int)   //  was ....speed_t), what does 'T' and 2 mean and do?
            int new_baud = static_cast < int >(Serial14CUXParams::Baud_14CUX);
            if (ioctl(sd, IOSSIOSPEED, &new_baud, 1) == -1)
            {
                // return -1 is error, otherwise value depends on call
                dprintf_err("14CUX(error): ioctl(), errno %d, %s\n", errno, strerror(errno));
                dprintf_err("   new_baud %d\n", new_baud);
                dprintf_err("   sizeof(speed_t)=%d sizeof(int)=%d\n", sizeof(speed_t), sizeof(int));
                dprintf_err("   IOS 0x%x\n", IOSSIOSPEED);
            }

            // Now try and verify that the speed was set correctly
            struct termios termAttr;    // clean readback variable to confirm settings
            speed_t baudRate = 0;

            // Obtain a copy of the termios structure for our adjusted port
            tcgetattr(sd, &termAttr);

            // Get the output speed
            baudRate = cfgetospeed(&termAttr);
            dprintf_info("Output speed = %d\n", baudRate);

            // Get the input speed
            baudRate = cfgetispeed(&termAttr);
            dprintf_info("Input speed = %d\n", baudRate);
            dprintf_info("VTIME = %d\n", termAttr.c_cc[VTIME]);

            if (baudRate == (Serial14CUXParams::Baud_14CUX))
            {
                dprintf_info("14CUX(info): Baud rate setting successful.\n");
                retVal = true;
            }
        }

        // the serial device was opened, but couldn't be configured properly;
        // close it before returning with failure
        if (!retVal)
        {
            dprintf_err("14CUX(error): Failure setting up port (Unix); closing serial device...\n");
            close(sd);
            FD_ZERO(&sds);
        }
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

