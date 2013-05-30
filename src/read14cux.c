#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "comm14cux.h"

int main(int argc, char** argv)
{
    uint8_t readBuf[0x10000];
    uint16_t addr;
    uint16_t len;
    c14cux_version ver;
    c14cux_info info;
    FILE *fp;
    int retVal = 0;
    int bytePos = 0;

    ver = c14cux_getLibraryVersion();

    if (argc < 4)
    {
        printf("read14cux using libcomm14cux v%d.%d.%d\n", ver.major, ver.minor, ver.patch);
        printf("Usage: %s <serial device> <address> <length> [output file]\n", argv[0]);
        return 0;
    }

    addr = strtoul(argv[2], NULL, 0);
    len = strtoul(argv[3], NULL, 0);

    c14cux_init(&info);

    if (c14cux_connect(&info, argv[1]))
    {
        if (c14cux_readMem(&info, addr, len, readBuf))
        {
            // if a filename was specified, write to the file rather than STDOUT
            if (argc >= 5)
            {
                fp = fopen(argv[4], "w");
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
        printf("Error: could not open serial device (%s).\n", argv[1]);
        retVal = -4;
    }

    return retVal;
}

