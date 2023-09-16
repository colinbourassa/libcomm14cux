// libcomm14cux - a communications library for the Lucas 14CUX ECU
//
// setup.c: This file contains routines that perform the
//          setup/initialization of the library and the
//          serial port.

#include "comm14cux.h"
#include "comm14cux_internal.h"
#include "comm14cux_version.h"
#include <stdlib.h>
#include <string.h>
#include <libusb.h>

#ifndef WIN32
#include <arpa/inet.h>
#endif

/**
 * Swaps multibyte big-endian data (from the ECU) into the local endianness.
 */
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

/**
 * Sets initial values in the state-info struct.
 * Note that this routine does not actually open the serial port or attempt
 * to connect to the ECU; that requires c14cux_connect().
 * @param info
 */
void c14cux_init(c14cux_info* info, bool verbose)
{
  info->promRev = C14CUX_DataOffsets_Unset;
  info->lastReadCoarseAddress = 0x0000;
  info->lastReadQuantity = 0x00;
  info->cancelRead = false;
  info->voltageFactorA = 0;
  info->voltageFactorB = 0;
  info->voltageFactorC = 0;
  info->connected = false;
  info->verbose = verbose;
  ftdi_init(&info->ftdi);
  pthread_mutex_init(&info->mutex, NULL);
}

/**
 * Disconnects (if necessary) and closes the mutex handle.
 * @param info State information for the current connection.
 */
void c14cux_cleanup(c14cux_info* info)
{
  if (info->connected)
  {
    ftdi_usb_close(&info->ftdi);
    info->connected = false;
  }
  pthread_mutex_destroy(&info->mutex);
  ftdi_deinit(&info->ftdi);
}

/**
 * Returns version information for this build of the library.
 * @return Version of this build of libcomm14cux
 */
c14cux_version c14cux_get_version()
{
  c14cux_version ver;
  ver.major = COMM14CUX_VER_MAJOR;
  ver.minor = COMM14CUX_VER_MINOR;
  ver.patch = COMM14CUX_VER_PATCH;

  return ver;
}

/**
 * Closes the serial device.
 * @param info State information for the current connection.
 */
void c14cux_disconnect(c14cux_info* info)
{
  pthread_mutex_lock(&info->mutex);
  if (info->connected)
  {
    ftdi_usb_close(&info->ftdi);
    info->connected = false;
  }
  pthread_mutex_unlock(&info->mutex);
}

/**
 * Opens and configures the FTDI device (or returns with success if it is already open.)
 * @param info State information for the current connection.
 * @param vid Vendor ID of the USB-to-serial device
 * @param pid Product ID of the USB-to-serial device
 * @param baud Baud rate, which should be set to C14CUX_BAUD for standard ECUs
 * @return True if the FTDI device was successfully opened and configured; false otherwise.
 */
bool c14cux_connect_by_usb_pid(c14cux_info* info, uint16_t vid, uint16_t pid, unsigned int baud)
{
  pthread_mutex_lock(&info->mutex);
  if (!info->connected &&
      (ftdi_set_interface(&info->ftdi, INTERFACE_A) == 0))
  {
    if (ftdi_usb_open(&info->ftdi, vid, pid) == 0)
    {
      if (c14cux_config_ftdi(info, baud))
      {
        info->connected = true;
      }
      else
      {
        ftdi_usb_close(&info->ftdi);
      }
    }
  }
  pthread_mutex_unlock(&info->mutex);
  dprintf_info(info->connected ? "14CUX(info): Connected\n" : "14CUX(error): Connect failed\n");
  return info->connected;
}

/**
 * Opens and configures the FTDI device (or returns with success if it is already open.)
 * @param info State information for the current connection.
 * @param bus USB bus number on which the FTDI device is connected
 * @param addr USB bus address at which the FTDI device is connected
 * @param baud Baud rate, which should be set to C14CUX_BAUD for standard ECUs
 * @return True if the FTDI device was successfully opened and configured; false otherwise.
 */
bool c14cux_connect_by_usb_addr(c14cux_info* info, uint8_t bus, uint8_t addr, unsigned int baud)
{
  pthread_mutex_lock(&info->mutex);
  if (!info->connected &&
      (ftdi_set_interface(&info->ftdi, INTERFACE_A) == 0))
  {
    if (ftdi_usb_open_bus_addr(&info->ftdi, bus, addr) == 0)
    {
      if (c14cux_config_ftdi(info, baud))
      {
        info->connected = true;
      }
      else
      {
        ftdi_usb_close(&info->ftdi);
      }
    }
  }
  pthread_mutex_unlock(&info->mutex);
  dprintf_info(info->connected ? "14CUX(info): Connected\n" : "14CUX(error): Connect failed\n");
  return (info->connected);
}

/**
 * Sets the baud rate, line properties, and latency for an FTDI device that
 * was previously opened.
 * @return True if the open/setup was successful, false otherwise
 */
bool c14cux_config_ftdi(c14cux_info* info, unsigned int baud)
{
  bool status = false;
  ftdi_tcioflush(&info->ftdi);
  if (ftdi_set_baudrate(&info->ftdi, baud) == 0)
  {
    if (ftdi_set_line_property(&info->ftdi, BITS_8, STOP_BIT_1, NONE) == 0)
    {
      if (ftdi_setflowctrl(&info->ftdi, SIO_DISABLE_FLOW_CTRL) == 0)
      {
        if (ftdi_set_latency_timer(&info->ftdi, 1) == 0)
        {
          status = true;
        }
        else if (info->verbose)
        {
          fprintf(stderr, "Failed to set FTDI latency timer (\"%s\")\n", ftdi_get_error_string(&info->ftdi));
        }
      }
      else if (info->verbose)
      {
        fprintf(stderr, "Failed to disable FTDI flow control (\"%s\")\n", ftdi_get_error_string(&info->ftdi));
      }
    }
    else if (info->verbose)
    {
      fprintf(stderr, "Failed to set FTDI line properties (\"%s\")\n", ftdi_get_error_string(&info->ftdi));
    }
  }
  else if (info->verbose)
  {
    fprintf(stderr, "Failed to set FTDI baud rate (\"%s\")\n", ftdi_get_error_string(&info->ftdi));
  }

  return status;
}

/**
 * Frees memory previously used to store a list of FTDI device info structs.
 */
void c14cux_ftdi_list_free(c14cux_ftdi_info* list)
{
  c14cux_ftdi_info* next = NULL;
  while (list)
  {
    next = list->next;
    free(list);
    list = next;
  }
}

/**
 * Retrieves a list of all FTDI devices (with known USB PIDs) on the system.
 * Note that memory used by the returned list must be freed after use by
 * calling c14cux_ftdi_list_free().
 * @param info libcomm14cux connection context
 * @param returnList Pointer to the head of the linked list of FTDI device info structs
 * @return Number of FTDI devices found
 */
int c14cux_ftdi_enumerate(c14cux_info* info, c14cux_ftdi_info** returnList)
{
  struct ftdi_device_list* libftdiList;

  // Use VID:PID of 0:0 to search for all standard VID:PID
  // combinations known to libftdi.
  const int libftdiCount = ftdi_usb_find_all(&info->ftdi, &libftdiList, 0, 0);
  struct ftdi_device_list** libftdiListHead = &libftdiList;
  int libftdiIndex = 0;

  char manufacturer[USB_STR_LEN];
  char description[USB_STR_LEN];
  int count = 0;
  c14cux_ftdi_info* prev = NULL;
  c14cux_ftdi_info* cur = NULL;
  c14cux_ftdi_info* listHead = NULL;

  while (libftdiList && (libftdiIndex < libftdiCount))
  {
    if (ftdi_usb_get_strings(&info->ftdi, libftdiList->dev,
                             manufacturer, USB_STR_LEN,
                             description, USB_STR_LEN,
                             NULL, 0) == 0)
    {
      const uint8_t bus_num = libusb_get_bus_number(libftdiList->dev);
      const uint8_t device_addr = libusb_get_device_address(libftdiList->dev);
      cur = malloc(sizeof(c14cux_ftdi_info));
      cur->busNumber = bus_num;
      cur->deviceAddress = device_addr;
      strncpy(cur->manufacturer, manufacturer, USB_STR_LEN);
      strncpy(cur->description, description, USB_STR_LEN);
      if (listHead == NULL)
      {
        listHead = cur;
      }
      if (prev)
      {
        prev->next = cur;
      }
      prev = cur;
      count++;
    }
    libftdiIndex++;
    libftdiList = libftdiList->next;
  }

  ftdi_list_free(libftdiListHead);
  *returnList = listHead;
  return count;
}

