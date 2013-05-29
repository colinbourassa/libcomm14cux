#ifndef COMM14CUX_INTERNAL_H
#define COMM14CUX_INTERNAL_H

#include <stdbool.h>

int16_t _14cux_readSerialBytes(cuxinfo* info, uint8_t *buffer, uint16_t quantity);
int16_t _14cux_writeSerialBytes(cuxinfo* info, uint8_t *buffer, uint16_t quantity);
bool _14cux_setReadCoarseAddr(cuxinfo* info, uint16_t addr, uint16_t len);
bool _14cux_setWriteCoarseAddr(cuxinfo* info, uint16_t addr);
bool _14cux_setCoarseAddr(cuxinfo* info, uint16_t addr, uint16_t len);
uint16_t _14cux_getByteCountForNextRead(uint16_t len, uint16_t bytesRead);
bool _14cux_sendReadCmd(cuxinfo* info, uint16_t addr, uint16_t len, bool lastByteOnly);
bool _14cux_openSerial(cuxinfo* info, const char *devPath);
double _14cux_hyperbolicOffsetModel(const double count);
void _14cux_determineDataOffsets(cuxinfo* info);

#endif // COMM14CUX_INTERNAL_H

