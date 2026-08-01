#include "crc.h"
static uint32_t gTable_Crc32[256];
static bool g_Crc32Initialized = false;

void InitCRC32Table() {
    if (g_Crc32Initialized) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? (0xEDB88320 ^ (crc >> 1)) : (crc >> 1);
        }
        gTable_Crc32[i] = crc;
    }
    g_Crc32Initialized = true;
}

uint32_t CRC32_Update(uint32_t crc, const uint8_t* buffer, size_t size) {
    InitCRC32Table();
    crc = ~crc;
    for (size_t i = 0; i < size; i++) {
        crc = gTable_Crc32[(crc ^ buffer[i]) & 0xFF] ^ (crc >> 8);
    }
    return ~crc;
}

// ============================================================================
