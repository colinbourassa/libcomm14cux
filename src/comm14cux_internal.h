#ifndef COMM14CUX_INTERNAL_H
#define COMM14CUX_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

int16_t c14cux_read_serial_bytes(c14cux_info* info, uint8_t* buffer, uint16_t quantity);
int16_t c14cux_write_serial_bytes(c14cux_info* info, uint8_t* buffer, uint16_t quantity);
bool c14cux_set_read_coarse_addr(c14cux_info* info, uint16_t addr, uint16_t len);
bool c14cux_set_write_coarse_addr(c14cux_info* info, uint16_t addr);
bool c14cux_set_coarse_addr(c14cux_info* info, uint16_t addr, uint16_t len);
uint16_t c14cux_get_byte_count_for_next_read(uint16_t len, uint16_t bytesRead);
bool c14cux_send_read_cmd(c14cux_info* info, uint16_t addr, uint16_t len, bool lastByteOnly);
bool c14cux_config_ftdi(c14cux_info* info, unsigned int baud);
void c14cux_determine_data_offsets(c14cux_info* info);

#endif // COMM14CUX_INTERNAL_H

