#ifndef COMM14CUX_INTERNAL_H
#define COMM14CUX_INTERNAL_H

#include <stdbool.h>

int16_t c14cux_readSerialBytes(c14cux_info* info, uint8_t *buffer, uint16_t quantity);
int16_t c14cux_writeSerialBytes(c14cux_info* info, uint8_t *buffer, uint16_t quantity);
bool c14cux_setReadCoarseAddr(c14cux_info* info, uint16_t addr, uint16_t len);
bool c14cux_setWriteCoarseAddr(c14cux_info* info, uint16_t addr);
bool c14cux_setCoarseAddr(c14cux_info* info, uint16_t addr, uint16_t len);
uint16_t c14cux_getByteCountForNextRead(uint16_t len, uint16_t bytesRead);
bool c14cux_sendReadCmd(c14cux_info* info, uint16_t addr, uint16_t len, bool lastByteOnly);
bool c14cux_openSerial(c14cux_info* info, const char *devPath, uint32_t baud);
double c14cux_hyperbolicOffsetModel(const double count);
void c14cux_determineDataOffsets(c14cux_info* info);

#endif // COMM14CUX_INTERNAL_H

