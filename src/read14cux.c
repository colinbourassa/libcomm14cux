#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "comm14cux.h"

void usage(c14cux_version ver)
{
  printf("read14cux using libcomm14cux v%d.%d.%d\n", ver.major, ver.minor, ver.patch);
  printf("Usage: read14cux [-b baud-rate] <address> <length> [output file]\n");
}

int main(int argc, char** argv)
{
  uint8_t readBuf[0x10000];
  uint16_t addr;
  uint16_t len;
  c14cux_info info;
  FILE* fp;
  int retVal = 0;
  int bytePos = 0;
  unsigned int baud = C14CUX_BAUD;
  int outfileParamPos = -1;

  if (argc < 3)
  {
    usage(c14cux_get_version());
    return 0;
  }

  printf("Usage: read14cux [-b baud-rate] <address> <length> [output file]\n");
  // if the user specified a nonstandard baud rate, grab it from the parameter list
  if (strcmp(argv[1], "-b") == 0)
  {
    if (argc < 5)
    {
      usage(c14cux_get_version());
      return 0;
    }
    else
    {
      if (argc >= 6)
      {
        outfileParamPos = 5;
      }
      baud = strtoul(argv[2], NULL, 10);
      addr = strtoul(argv[3], NULL, 0);
      len = strtoul(argv[4], NULL, 0);
    }
  }
  else
  {
    addr = strtoul(argv[1], NULL, 0);
    len = strtoul(argv[2], NULL, 0);
    if (argc >= 4)
    {
      outfileParamPos = 3;
    }
  }

  c14cux_init(&info, true);

  if (c14cux_connect_by_usb_pid(&info, 0x0403, 0x6001, baud))
  {
    if (c14cux_read_mem(&info, addr, len, readBuf))
    {
      // if a filename was specified, write to the file rather than STDOUT
      if (outfileParamPos > 0)
      {
        fp = fopen(argv[outfileParamPos], "wb");

        if (fp != NULL)
        {
          if (fwrite(readBuf, 1, len, fp) == len)
          {
            printf("File '%s' written.\n", argv[4]);
          }
          else
          {
            printf("Error: failed to write file '%s'.\n", argv[4]);
            retVal = -1;
          }

          if (fclose(fp) != 0)
          {
            printf("Error: failed to close file.\n");
            retVal = -2;
          }
        }
      }
      else
      {
        // since a filename wasn't specified, simply send a text
        // representation of the data to STDOUT
        for (bytePos = 0; bytePos < len; bytePos++)
        {
          if ((bytePos != 0) && (bytePos % 16 == 0))
          {
            printf("\n");
          }

          printf("%02X ", readBuf[bytePos]);
        }

        printf("\n");
      }
    }
    else
    {
      printf("Error: failure reading from 14CUX.\n");
      retVal = -3;
    }

    c14cux_disconnect(&info);
  }
  else
  {
    printf("Error: failed to open FTDI USB device.\n");
    retVal = -4;
  }

  return retVal;
}

