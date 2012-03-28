#include <string>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "comm14cux.h"

int main(int argc, char** argv)
{
    uint8_t readBuf[0x10000];
    uint16_t addr;
    uint16_t len;
    std::string device;
    std::string outputFile;
    Comm14CUX* lucas;
    Comm14CUXVersion ver;
    FILE *fp;
    int retVal;

    retVal = 0;
    lucas = new Comm14CUX();
    ver = lucas->getVersion();

    if (argc < 4)
    {
        printf("read14cux using libcomm14cux v%d.%d.%d\n", ver.major, ver.minor, ver.patch);
        printf("Usage: %s <serial device> <address> <length> [output file]\n", argv[0]);
        delete lucas;
        return 0;
    }

    device = argv[1];
    addr = strtoul(argv[2], NULL, 0);
    len = strtoul(argv[3], NULL, 0);

    if (argc >= 5)
    {
        outputFile = argv[4];
    }

    if (lucas->connect(device))
    {
        if (lucas->readMem(addr, len, readBuf))
        {
            // if a filename was specified, write to the file rather than STDOUT
            if (!outputFile.empty())
            {
                fp = fopen(outputFile.c_str(), "w");
                if (fp != NULL)
                {
                    if (fwrite(readBuf, 1, len, fp) == len)
                    {
                        printf("File '%s' written.\n", outputFile.c_str());
                    }
                    else
                    {
                        printf("Error: failed to write file '%s'.\n", outputFile.c_str());
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
                for (int bytePos = 0; bytePos < len; bytePos++)
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

        lucas->disconnect();
    }
    else
    {
        printf("Error: could not open serial device.\n", argv[0]);
        retVal = -4;
    }

    delete lucas;

    return retVal;
}

