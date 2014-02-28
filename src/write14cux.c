#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "comm14cux.h"

int main(int argc, char** argv)
{
    uint16_t addr;
    uint8_t val;
    c14cux_version ver;
    c14cux_info info;
    int retVal = 0;

    ver = c14cux_getLibraryVersion();

    if (argc < 4)
    {
        printf("write14cux using libcomm14cux v%d.%d.%d\n", ver.major, ver.minor, ver.patch);
        printf("Usage: %s <serial device> <address> <value>\n", argv[0]);
        return 0;
    }

    addr = strtoul(argv[2], NULL, 0);
    val = strtoul(argv[3], NULL, 0);

    c14cux_init(&info);

    if (c14cux_connect(&info, argv[1]))
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

