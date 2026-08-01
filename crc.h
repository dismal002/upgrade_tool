#pragma once
#include <stdint.h>
#include <stddef.h>

void InitCRC32Table();
uint32_t CRC32_Update(uint32_t crc, const uint8_t* buffer, size_t size);
