#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "comm14cux.h"

void usage(c14cux_version ver, const char* name)
{
  printf("write14cux using libcomm14cux v%d.%d.%d\n", ver.major, ver.minor, ver.patch);
  printf("Usage: %s <serial device> [-b baud-rate] <address> <value>\n", name);
}

int main(int argc, char** argv)
{
  uint16_t addr;
  uint8_t val;
  c14cux_version ver;
  c14cux_info info;
  int retVal = 0;
  unsigned int baud = C14CUX_BAUD;

  ver = c14cux_getLibraryVersion();

  if (argc < 4)
  {
    usage(ver, argv[0]);
    return 0;
  }

  // if the user specified a nonstandard baud rate, grab it from the parameter list
  if (strcmp(argv[2], "-b") == 0)
  {
    if (argc < 6)
    {
      usage(ver, argv[0]);
      return 0;
    }
    else
    {
      baud = strtoul(argv[3], NULL, 10);
      addr = strtoul(argv[4], NULL, 0);
      val = strtoul(argv[5], NULL, 0);
    }
  }
  else
  {
    addr = strtoul(argv[2], NULL, 0);
    val = strtoul(argv[3], NULL, 0);
  }

  c14cux_init(&info);

  if (c14cux_connect(&info, argv[1], baud))
  {
    if (c14cux_writeMem(&info, addr, val))
    {
      printf("Wrote 0x%02X to $%04X.\n", val, addr);
    }
    else
    {
      fprintf(stderr, "Error: failure writing to 14CUX.\n");
      retVal = -3;
    }

    c14cux_disconnect(&info);
  }
  else
  {
    fprintf(stderr, "Error: could not open serial device (%s).\n", argv[1]);
    retVal = -4;
  }

  return retVal;
}

