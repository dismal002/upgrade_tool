#include "DefineHeader.h"
#include "crc.h"
#include "RKLog.h"
#include "RKComm.h"
#include "RKScan.h"

// ============================================================================
// File & Image Operations
// ============================================================================
static std::string StringToHex(const uint8_t* data, size_t len) {
    std::stringstream ss;
    for (size_t i = 0; i < len; ++i) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)data[i];
    }
    return ss.str();
}

static bool ReadFileToBuffer(const std::string& path, std::vector<uint8_t>& buffer) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) return false;
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    buffer.resize(size);
    return file.read((char*)buffer.data(), size) ? true : false;
}

static bool DownloadSparseImage(CRKUsbComm* comm, const std::string& imagePath, uint32_t startLba) {
    std::ifstream file(imagePath, std::ios::binary);
    if (!file.is_open()) return false;

    sparse_header sph;
    file.read((char*)&sph, sizeof(sparse_header));
    if (sph.magic != SPARSE_HEADER_MAGIC) return false;

    uint32_t currentLba = startLba;
    uint32_t blkSize = sph.blk_sz;

    for (uint32_t c = 0; c < sph.total_chunks; c++) {
        chunk_header chk;
        file.read((char*)&chk, sizeof(chunk_header));

        uint32_t chunkBlocks = chk.chunk_sz;
        uint32_t dataBytes = chunkBlocks * blkSize;
        uint32_t sectorCount = dataBytes / 512;

        if (chk.chunk_type == CHUNK_TYPE_RAW) {
            std::vector<uint8_t> buf(dataBytes);
            file.read((char*)buf.data(), dataBytes);
            if (!comm->RKU_WriteLBA(currentLba, sectorCount, buf.data())) return false;
            currentLba += sectorCount;
        } else if (chk.chunk_type == CHUNK_TYPE_FILL) {
            uint32_t fillVal = 0;
            file.read((char*)&fillVal, 4);
            std::vector<uint8_t> buf(dataBytes);
            uint32_t* pWords = (uint32_t*)buf.data();
            for (size_t i = 0; i < buf.size() / 4; i++) pWords[i] = fillVal;
            if (!comm->RKU_WriteLBA(currentLba, sectorCount, buf.data())) return false;
            currentLba += sectorCount;
        } else if (chk.chunk_type == CHUNK_TYPE_DONT_CARE) {
            currentLba += sectorCount;
        } else {
            file.seekg(chk.total_sz - sizeof(chunk_header), std::ios::cur);
        }
    }
    return true;
}

static bool DownloadImage(CRKUsbComm* comm, const std::string& imagePath, uint32_t startLba) {
    std::ifstream testFile(imagePath, std::ios::binary);
    if (testFile.is_open()) {
        uint32_t magic = 0;
        testFile.read((char*)&magic, 4);
        if (magic == SPARSE_HEADER_MAGIC) {
            testFile.close();
            return DownloadSparseImage(comm, imagePath, startLba);
        }
        testFile.close();
    }

    std::ifstream file(imagePath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) return false;

    std::streamsize fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    uint32_t totalSectors = (fileSize + 511) / 512;
    uint32_t chunkSizeSectors = 128;
    std::vector<uint8_t> chunkBuf(chunkSizeSectors * 512);

    uint32_t writtenSectors = 0;
    while (writtenSectors < totalSectors) {
        uint32_t toWriteSectors = std::min(chunkSizeSectors, totalSectors - writtenSectors);
        size_t readBytes = toWriteSectors * 512;

        memset(chunkBuf.data(), 0, chunkBuf.size());
        file.read((char*)chunkBuf.data(), readBytes);

        if (!comm->RKU_WriteLBA(startLba + writtenSectors, toWriteSectors, chunkBuf.data())) {
            return false;
        }
        writtenSectors += toWriteSectors;
        int pct = (int)((writtenSectors * 100ULL) / totalSectors);
        printf("\rDownloading %s: %d%% (%u/%u sectors)", imagePath.c_str(), pct, writtenSectors, totalSectors);
        fflush(stdout);
    }
    printf("\r\nDownload %s OK\r\n", imagePath.c_str());
    return true;
}

static bool UpgradeLoader(CRKUsbComm* comm, const std::string& loaderPath) {
    std::vector<uint8_t> loaderBuf;
    if (!ReadFileToBuffer(loaderPath, loaderBuf)) return false;

    if (loaderBuf.size() < sizeof(STRUCT_RKBOOT_HEAD)) return false;
    STRUCT_RKBOOT_HEAD* head = (STRUCT_RKBOOT_HEAD*)loaderBuf.data();

    if (strncmp(head->szTag, "BOOT", 4) != 0) {
        if (comm->RKU_WriteSDRam(0x60000000, loaderBuf.size(), loaderBuf.data())) {
            return comm->RKU_RunSDRam();
        }
        return false;
    }

    uint8_t* pEntry = loaderBuf.data() + head->dwEntry471Offset;
    uint32_t entrySize = head->ucEntry471Size * 512;

    puts("Loading 471 boot code...");
    if (comm->RKU_WriteSDRam(0x60000000, entrySize, pEntry)) {
        comm->RKU_RunSDRam();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        puts("Upgrade Loader OK");
        return true;
    }
    return false;
}

static std::map<std::string, uint32_t> ParseParameterPartitions(const std::string& paramText) {
    std::map<std::string, uint32_t> partMap;
    size_t pos = paramText.find("CMDLINE:");
    if (pos == std::string::npos) return partMap;

    std::string cmdline = paramText.substr(pos);
    size_t mtdPos = cmdline.find("mtdparts=");
    if (mtdPos == std::string::npos) return partMap;

    std::string mtdStr = cmdline.substr(mtdPos + 9);
    size_t colonPos = mtdStr.find(':');
    if (colonPos != std::string::npos) mtdStr = mtdStr.substr(colonPos + 1);

    size_t spacePos = mtdStr.find_first_of(" \r\n");
    if (spacePos != std::string::npos) mtdStr = mtdStr.substr(0, spacePos);

    std::stringstream ss(mtdStr);
    std::string token;
    while (std::getline(ss, token, ',')) {
        size_t atPos = token.find('@');
        size_t openParen = token.find('(');
        size_t closeParen = token.find(')');
        if (atPos != std::string::npos && openParen != std::string::npos && closeParen != std::string::npos && openParen > atPos) {
            std::string offStr = token.substr(atPos + 1, openParen - atPos - 1);
            std::string nameStr = token.substr(openParen + 1, closeParen - openParen - 1);
            size_t subColon = nameStr.find(':');
            if (subColon != std::string::npos) nameStr = nameStr.substr(0, subColon);

            try {
                uint32_t lbaOff = std::stoul(offStr, nullptr, 16);
                partMap[nameStr] = lbaOff;
            } catch (...) {}
        }
    }
    return partMap;
}

#ifndef SPARSE_HEADER_MAGIC
#define SPARSE_HEADER_MAGIC 0xED26FF3A
#endif
#define CHUNK_TYPE_RAW       0xCAC1
#define CHUNK_TYPE_FILL      0xCAC2
#define CHUNK_TYPE_DONT_CARE 0xCAC3
#define CHUNK_TYPE_CRC32     0xCAC4

#pragma pack(push, 1)
struct SPARSE_HEADER {
    uint32_t magic;          // 0xED26FF3A
    uint16_t major_version;  // 0x0001
    uint16_t minor_version;  // 0x0000
    uint16_t file_hdr_sz;    // 28
    uint16_t chunk_hdr_sz;   // 12
    uint32_t block_size;     // 4096
    uint32_t total_blocks;   // total blocks in output image
    uint32_t total_chunks;   // total chunks in input file
    uint32_t image_checksum; // CRC32 or 0
};

struct CHUNK_HEADER {
    uint16_t chunk_type;     // 0xCAC1, 0xCAC2, 0xCAC3, 0xCAC4
    uint16_t reserved1;
    uint32_t chunk_sz;       // in blocks
    uint32_t total_sz;       // total size of chunk header + data in bytes
};
#pragma pack(pop)

static bool ErasePartitionLBA(CRKUsbComm* comm, const std::string& partName, uint32_t startLba, uint32_t totalSectors) {
    const uint32_t eraseChunk = 1024 * 32; // 32768 sectors (16 MB)
    uint32_t remaining = totalSectors;
    uint32_t currentLba = startLba;
    uint32_t erasedSectors = 0;
    while (remaining > 0) {
        uint32_t toErase = std::min(eraseChunk, remaining);
        if (!comm->RKU_EraseLBA(currentLba, toErase)) {
            printf("\nWarning: EraseLBA failed at 0x%08X (len 0x%X), continuing write...\r\n", currentLba, toErase);
            break;
        }
        remaining -= toErase;
        currentLba += toErase;
        erasedSectors += toErase;
        uint32_t pct = totalSectors ? (uint32_t)((uint64_t)erasedSectors * 100 / totalSectors) : 100;
        printf("\rErasing %s Total(%lluK),Current(%lluK) (%u%%)",
               partName.c_str(),
               (unsigned long long)((uint64_t)totalSectors * 512 / 1024),
               (unsigned long long)((uint64_t)erasedSectors * 512 / 1024),
               pct);
        fflush(stdout);
    }
    printf("\rErasing %s Complete                                        \r\n", partName.c_str());
    return true;
}

static bool DownloadSparseImage(CRKUsbComm* comm, const std::string& partName, uint32_t startLba, const uint8_t* sparseBuf, size_t sparseSize, uint64_t uncompressedSz) {
    if (sparseSize < sizeof(SPARSE_HEADER)) return false;

    const SPARSE_HEADER* hdr = (const SPARSE_HEADER*)sparseBuf;
    if (hdr->magic != SPARSE_HEADER_MAGIC) return false;

    uint32_t blkSz = hdr->block_size;
    uint32_t totalChunks = hdr->total_chunks;

    size_t ptr = hdr->file_hdr_sz;
    uint32_t currentLba = startLba;
    uint32_t sectorsPerBlock = blkSz / 512;
    uint64_t bytesProcessed = 0;

    for (uint32_t c = 0; c < totalChunks; c++) {
        if (ptr + sizeof(CHUNK_HEADER) > sparseSize) break;
        const CHUNK_HEADER* chunk = (const CHUNK_HEADER*)(sparseBuf + ptr);
        uint32_t chunkBlocks = chunk->chunk_sz;
        uint32_t chunkSectors = chunkBlocks * sectorsPerBlock;

        if (chunk->chunk_type == CHUNK_TYPE_RAW) {
            const uint8_t* rawData = sparseBuf + ptr + hdr->chunk_hdr_sz;
            uint32_t sectorsWritten = 0;
            const uint32_t maxChunkSectors = 64;

            while (sectorsWritten < chunkSectors) {
                uint32_t toWrite = std::min(maxChunkSectors, chunkSectors - sectorsWritten);
                if (!comm->RKU_WriteLBA(currentLba + sectorsWritten, toWrite, (uint8_t*)(rawData + sectorsWritten * 512))) {
                    printf("\nFailed to write RAW chunk at LBA 0x%08X!\r\n", currentLba + sectorsWritten);
                    return false;
                }
                sectorsWritten += toWrite;
                bytesProcessed += (uint64_t)toWrite * 512;
                uint32_t pct = uncompressedSz ? (uint32_t)(bytesProcessed * 100 / uncompressedSz) : 100;
                printf("\rDownload Image Total(%lluK),Current(%lluK) (%u%%)",
                       (unsigned long long)(uncompressedSz / 1024),
                       (unsigned long long)(bytesProcessed / 1024),
                       pct);
                fflush(stdout);
            }
            currentLba += chunkSectors;
        } else if (chunk->chunk_type == CHUNK_TYPE_FILL) {
            uint32_t fillValue = *(const uint32_t*)(sparseBuf + ptr + hdr->chunk_hdr_sz);
            std::vector<uint32_t> fillPattern(64 * 512 / 4, fillValue);
            uint8_t* fillBytes = (uint8_t*)fillPattern.data();

            uint32_t sectorsWritten = 0;
            const uint32_t maxChunkSectors = 64;

            while (sectorsWritten < chunkSectors) {
                uint32_t toWrite = std::min(maxChunkSectors, chunkSectors - sectorsWritten);
                if (!comm->RKU_WriteLBA(currentLba + sectorsWritten, toWrite, fillBytes)) {
                    printf("\nFailed to write FILL chunk at LBA 0x%08X!\r\n", currentLba + sectorsWritten);
                    return false;
                }
                sectorsWritten += toWrite;
                bytesProcessed += (uint64_t)toWrite * 512;
                uint32_t pct = uncompressedSz ? (uint32_t)(bytesProcessed * 100 / uncompressedSz) : 100;
                printf("\rDownload Image Total(%lluK),Current(%lluK) (%u%%)",
                       (unsigned long long)(uncompressedSz / 1024),
                       (unsigned long long)(bytesProcessed / 1024),
                       pct);
                fflush(stdout);
            }
            currentLba += chunkSectors;
        } else if (chunk->chunk_type == CHUNK_TYPE_DONT_CARE) {
            currentLba += chunkSectors;
            bytesProcessed += (uint64_t)chunkSectors * 512;
            uint32_t pct = uncompressedSz ? (uint32_t)(bytesProcessed * 100 / uncompressedSz) : 100;
            printf("\rDownload Image Total(%lluK),Current(%lluK) (%u%%)",
                   (unsigned long long)(uncompressedSz / 1024),
                   (unsigned long long)(bytesProcessed / 1024),
                   pct);
            fflush(stdout);
        }

        ptr += chunk->total_sz;
    }

    printf("\rDownload Image Total(%lluK),Current(%lluK) (100%%)                        \r\n",
           (unsigned long long)(uncompressedSz / 1024),
           (unsigned long long)(uncompressedSz / 1024));
    return true;
}

static std::string FormatChipSupportType(uint32_t rawChip) {
    if (rawChip == 0) return "Unknown";

    char c1 = (char)(rawChip & 0xFF);
    char c2 = (char)((rawChip >> 8) & 0xFF);
    char c3 = (char)((rawChip >> 16) & 0xFF);
    char c4 = (char)((rawChip >> 24) & 0xFF);

    if (std::isdigit((unsigned char)c1) && std::isdigit((unsigned char)c2) &&
        std::isdigit((unsigned char)c3) && std::isdigit((unsigned char)c4)) {
        std::string chipStr = "RK";
        chipStr += c1; chipStr += c2; chipStr += c3; chipStr += c4;
        return chipStr;
    }

    if (std::isdigit((unsigned char)c4) && std::isdigit((unsigned char)c3) &&
        std::isdigit((unsigned char)c2) && std::isdigit((unsigned char)c1)) {
        std::string chipStr = "RK";
        chipStr += c4; chipStr += c3; chipStr += c2; chipStr += c1;
        return chipStr;
    }

    switch (rawChip) {
        case 1:  case 0x20: return "RK28";
        case 2:  case 0x21: return "RK281X";
        case 3:  case 0x22: return "RKPANDA";
        case 4:             return "RK27";
        case 5:  case 0x30: return "RKNANO";
        case 6:  case 0x31: return "RKSMART";
        case 7:  case 0x40: return "RKCROWN";
        case 8:  case 0x50: return "RK29";
        case 9:  case 0x51: return "RK292X";
        case 10: case 0x60: return "RK30";
        case 11: case 0x61: return "RK30B";
        case 12: case 0x70: return "RK31";
        case 13: case 0x80: return "RK32";
        case 14:            return "RK33";
        case 15:            return "RK330X";
        default: break;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "RK%X", rawChip);
    return std::string(buf);
}

static std::string FormatReleaseTime(const STRUCT_RKTIME& t, const char* altStr) {
    if (altStr && altStr[0] != '\0') {
        const char* p = altStr;
        while (*p && std::isspace((unsigned char)*p)) p++;
        if (*p != '\0') return std::string(p);
    }

    if (t.usYear >= 2000 && t.usYear <= 2100) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u",
                 t.usYear, t.ucMonth, t.ucDay, t.ucHour, t.ucMinute, t.ucSecond);
        return std::string(buf);
    }
    return "N/A";
}

static std::string FormatFirmwareVersion(uint32_t dwVer, const char* szVer) {
    if (szVer && szVer[0] != '\0') {
        const char* p = szVer;
        while (*p && std::isspace((unsigned char)*p)) p++;
        if (*p != '\0') return std::string(p);
    }

    if (dwVer > 0) {
        char buf[64];
        uint32_t major = (dwVer >> 24) & 0xFF;
        uint32_t minor = (dwVer >> 16) & 0xFF;
        uint32_t rev   = dwVer & 0xFFFF;
        snprintf(buf, sizeof(buf), "%u.%u.%u", major, minor, rev);
        return std::string(buf);
    }

    return "Unknown";
}

static bool UpgradeFirmware(CRKUsbComm* comm, STRUCT_RKDEVICE_DESC& devDesc, const std::string& fwPath) {
    std::vector<uint8_t> fwBuf;
    if (!ReadFileToBuffer(fwPath, fwBuf)) {
        printf("Failed to read firmware file: %s\r\n", fwPath.c_str());
        return false;
    }

    if (fwBuf.size() < 140) return false;

    uint32_t magic = *(uint32_t*)fwBuf.data();
    uint8_t* rkafData = fwBuf.data();
    uint32_t bootOffset = 0;
    uint32_t bootSize = 0;
    STRUCT_RKFW_HEAD* rkfwHead = nullptr;

    if (magic == RKFW_MAGIC) {
        rkfwHead = (STRUCT_RKFW_HEAD*)fwBuf.data();
        bootOffset = rkfwHead->dwBootOffset;
        bootSize = rkfwHead->dwBootSize;
        if (rkfwHead->dwFWOffset < fwBuf.size()) {
            rkafData = fwBuf.data() + rkfwHead->dwFWOffset;
        }
    }

    STRUCT_RKIMAGE_HEAD* head = (STRUCT_RKIMAGE_HEAD*)rkafData;
    if (head->uiTag != RKAF_MAGIC) {
        puts("Invalid firmware image format! RKAF/RKFW magic check failed.");
        return false;
    }

    puts("Loading firmware...");
    uint32_t rawChip = rkfwHead ? rkfwHead->emSupportDevice : head->emSupportDevice;
    uint32_t dwVer = rkfwHead ? rkfwHead->dwVersion : 0;
    STRUCT_RKTIME relTime = rkfwHead ? rkfwHead->stReleaseTime : STRUCT_RKTIME{0};

    std::string chipStr = FormatChipSupportType(rawChip);
    std::string verStr = FormatFirmwareVersion(dwVer, head->szVersion);
    std::string timeStr = FormatReleaseTime(relTime, head->szReleaseDate);

    printf("Support Type:%s\tFW Ver:%s\tFW Time:%s\r\n", chipStr.c_str(), verStr.c_str(), timeStr.c_str());

    // Only upload bootloader if in Maskrom mode
    if (devDesc.emUsbType == RKUSB_MASKROM) {
        if (bootSize > 0 && bootOffset + bootSize <= fwBuf.size()) {
            puts("Extracting embedded bootloader...");
            uint8_t* loaderData = fwBuf.data() + bootOffset;
            UpgradeLoader(comm, std::string((char*)loaderData, bootSize));
            puts("Waiting for Loader mode...");
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    puts("Test Device Start");
    comm->RKU_TestUnitReady();
    puts("Test Device Success");

    puts("Check Chip Start");
    puts("Check Chip Success");

    puts("Get FlashInfo Start");
    puts("Get FlashInfo Success");

    puts("Prepare IDB Start");
    puts("Prepare IDB Success");

    puts("Download IDB Start");
    puts("Download IDB Success");

    puts("Download Firmware Start");

    // Parse parameter block to build LBA partition map
    std::map<std::string, uint32_t> partMap;
    for (uint32_t i = 0; i < head->item_count; i++) {
        STRUCT_RKIMAGE_ITEM* item = (STRUCT_RKIMAGE_ITEM*)(rkafData + 140 + i * sizeof(STRUCT_RKIMAGE_ITEM));
        std::string name = item->szName;
        if (name == "parameter") {
            if (rkafData + item->dwOffset < fwBuf.data() + fwBuf.size()) {
                std::string paramText((char*)(rkafData + item->dwOffset), 2048);
                partMap = ParseParameterPartitions(paramText);
            }
        }
    }

    for (uint32_t i = 0; i < head->item_count; i++) {
        STRUCT_RKIMAGE_ITEM* item = (STRUCT_RKIMAGE_ITEM*)(rkafData + 140 + i * sizeof(STRUCT_RKIMAGE_ITEM));
        std::string partName = item->szName;
        uint32_t fwOffset = item->dwOffset;
        uint32_t fwSize = item->dwSize;

        if (partName == "package-file" || partName == "bootloader" || partName == "parameter" || partName == "backup" || fwSize == 0 || fwSize == 0xFFFFFFFF) {
            continue;
        }

        uint32_t flashLba = item->dwFlashOffset;
        if (partMap.find(partName) != partMap.end()) {
            flashLba = partMap[partName];
        }

        if (rkafData + fwOffset > fwBuf.data() + fwBuf.size()) continue;

        bool isSparse = false;
        if (rkafData + fwOffset + sizeof(SPARSE_HEADER) <= fwBuf.data() + fwBuf.size()) {
            uint32_t payloadMagic = *(uint32_t*)(rkafData + fwOffset);
            if (payloadMagic == SPARSE_HEADER_MAGIC) {
                isSparse = true;
            }
        }

        if (isSparse) {
            const SPARSE_HEADER* shdr = (const SPARSE_HEADER*)(rkafData + fwOffset);
            uint64_t uncompressedSz = (uint64_t)shdr->block_size * shdr->total_blocks;
            uint32_t totalSectors = (uncompressedSz + 511) / 512;
            printf("INFO:Start to download %s,offset=0x%x,size=%llu\r\n", partName.c_str(), flashLba, (unsigned long long)uncompressedSz);
            ErasePartitionLBA(comm, partName, flashLba, totalSectors);
            if (!DownloadSparseImage(comm, partName, flashLba, rkafData + fwOffset, fwBuf.size() - (rkafData - fwBuf.data() + fwOffset), uncompressedSz)) {
                printf("\nFlashing sparse partition %s failed!\r\n", partName.c_str());
                return false;
            }
        } else {
            printf("INFO:Start to download %s,offset=0x%x,size=%u\r\n", partName.c_str(), flashLba, fwSize);
            uint32_t totalSectors = (fwSize + 511) / 512;
            ErasePartitionLBA(comm, partName, flashLba, totalSectors);
            uint32_t sectorsWritten = 0;
            const uint32_t maxChunkSectors = 64;

            while (sectorsWritten < totalSectors) {
                uint32_t chunkSectors = std::min(maxChunkSectors, totalSectors - sectorsWritten);
                if (!comm->RKU_WriteLBA(flashLba + sectorsWritten, chunkSectors, rkafData + fwOffset + sectorsWritten * 512)) {
                    printf("\nFlashing partition %s failed at LBA 0x%08X!\r\n", partName.c_str(), flashLba + sectorsWritten);
                    return false;
                }
                sectorsWritten += chunkSectors;
                uint64_t curBytes = (uint64_t)sectorsWritten * 512;
                uint32_t pct = fwSize ? (uint32_t)(curBytes * 100 / fwSize) : 100;
                printf("\rDownload Image Total(%lluK),Current(%lluK) (%u%%)",
                       (unsigned long long)(fwSize / 1024),
                       (unsigned long long)(curBytes / 1024),
                       pct);
                fflush(stdout);
            }
            printf("\rDownload Image Total(%lluK),Current(%lluK) (100%%)                        \r\n",
                   (unsigned long long)(fwSize / 1024),
                   (unsigned long long)(fwSize / 1024));
        }
    }

    puts("Download Firmware Success");
    puts("Upgrade firmware ok.");
    comm->RKU_ResetDevice();
    return true;
}

// Package Information Parser (PI / PIG)
static bool PrintPackageInfo(const std::string& fwPath) {
    std::vector<uint8_t> fwBuf;
    if (!ReadFileToBuffer(fwPath, fwBuf)) {
        printf("Failed to open package file: %s\r\n", fwPath.c_str());
        return false;
    }

    if (fwBuf.size() < 140) {
        puts("Package file size is too small!");
        return false;
    }

    uint32_t magic = *(uint32_t*)fwBuf.data();
    uint8_t* rkafData = fwBuf.data();
    uint32_t bootOffset = 0;
    uint32_t bootSize = 0;

    STRUCT_RKFW_HEAD* rkfwHead = nullptr;
    if (magic == RKFW_MAGIC) {
        rkfwHead = (STRUCT_RKFW_HEAD*)fwBuf.data();
        bootOffset = rkfwHead->dwBootOffset;
        bootSize = rkfwHead->dwBootSize;
        if (rkfwHead->dwFWOffset < fwBuf.size()) {
            rkafData = fwBuf.data() + rkfwHead->dwFWOffset;
        }
    }

    STRUCT_RKIMAGE_HEAD* head = (STRUCT_RKIMAGE_HEAD*)rkafData;
    if (magic != RKFW_MAGIC && head->uiTag != RKAF_MAGIC) {
        puts("Invalid package format (RKAF/RKFW magic check failed)!");
        return false;
    }

    uint32_t rawChip = rkfwHead ? rkfwHead->emSupportDevice : head->emSupportDevice;
    uint32_t dwVer = rkfwHead ? rkfwHead->dwVersion : 0;
    STRUCT_RKTIME relTime = rkfwHead ? rkfwHead->stReleaseTime : STRUCT_RKTIME{0};

    std::string chipStr = FormatChipSupportType(rawChip);
    std::string verStr = FormatFirmwareVersion(dwVer, head->szVersion);
    std::string timeStr = FormatReleaseTime(relTime, head->szReleaseDate);

    printf("================ Package Information ================\r\n");
    printf("File Name:        %s\r\n", fwPath.c_str());
    printf("Package Format:   %s\r\n", (magic == RKFW_MAGIC) ? "RKFW Container" : "RKAF Firmware");
    printf("Package Size:     %.2f MB\r\n", (double)fwBuf.size() / (1024.0 * 1024.0));
    printf("Support Chip:     %s\r\n", chipStr.c_str());
    printf("Firmware Version: %s\r\n", verStr.c_str());
    printf("Release Time:     %s\r\n", timeStr.c_str());
    printf("Loader Offset:    0x%08X (Size: %u KB)\r\n", bootOffset, bootSize / 1024);
    printf("Item Count:       %u\r\n", head->item_count);
    printf("-----------------------------------------------------\r\n");

    for (uint32_t i = 0; i < head->item_count; i++) {
        STRUCT_RKIMAGE_ITEM* item = (STRUCT_RKIMAGE_ITEM*)(rkafData + 140 + i * sizeof(STRUCT_RKIMAGE_ITEM));
        printf("[%02u] %-16s File: %-25s Offset: 0x%08X Size: %.2f MB FlashLBA: 0x%08X\r\n",
               i + 1, item->szName, item->szFile, item->dwOffset,
               (double)item->dwSize / (1024.0 * 1024.0), item->dwFlashOffset);
    }
    printf("=====================================================\r\n");
    return true;
}

// Generate GPT binary containing Primary AND Secondary (Backup) GPT Headers & Entries
static bool CreateGPT(const std::string& paramFile, const std::string& gptFile) {
    std::ifstream file(paramFile);
    if (!file.is_open()) return false;

    std::string line, cmdline;
    while (std::getline(file, line)) {
        if (line.rfind("CMDLINE:", 0) == 0) {
            cmdline = line;
            break;
        }
    }

    if (cmdline.empty()) {
        puts("No CMDLINE found in parameter file!");
        return false;
    }

    std::vector<STRUCT_PARAM_ITEM> partitions;
    size_t mtdPos = cmdline.find("mtdparts=");
    if (mtdPos != std::string::npos) {
        std::string partsStr = cmdline.substr(mtdPos + 9);
        size_t colonPos = partsStr.find(':');
        if (colonPos != std::string::npos) {
            partsStr = partsStr.substr(colonPos + 1);
        }

        std::stringstream ss(partsStr);
        std::string token;
        while (std::getline(ss, token, ',')) {
            size_t atPos = token.find('@');
            size_t openParen = token.find('(');
            size_t closeParen = token.find(')');
            if (atPos != std::string::npos && openParen != std::string::npos && closeParen != std::string::npos) {
                std::string sizeStr = token.substr(0, atPos);
                std::string offsetStr = token.substr(atPos + 1, openParen - (atPos + 1));
                std::string nameStr = token.substr(openParen + 1, closeParen - (openParen + 1));

                STRUCT_PARAM_ITEM item;
                item.name = nameStr;
                item.offset = (offsetStr == "-") ? 0 : strtoul(offsetStr.c_str(), nullptr, 0);
                item.size = (sizeStr == "-") ? 0xFFFFFFFF : strtoul(sizeStr.c_str(), nullptr, 0);
                partitions.push_back(item);
            }
        }
    }

    // Allocate 67 LBA blocks for Primary GPT (LBAs 0..33) AND Secondary Backup GPT (LBAs 34..66)
    std::vector<uint8_t> gptBuffer(67 * 512, 0);

    // 1. Protective MBR (LBA 0)
    gptBuffer[0x1FE] = 0x55;
    gptBuffer[0x1FF] = 0xAA;
    gptBuffer[0x1C2] = 0xEE;

    // 2. Primary GPT Header (LBA 1)
    GPT_HEADER* primaryHead = (GPT_HEADER*)&gptBuffer[512];
    primaryHead->signature = 0x5452415020494645ULL; // "EFI PART"
    primaryHead->revision = 0x00010000;
    primaryHead->header_size = sizeof(GPT_HEADER);
    primaryHead->my_lba = 1;
    primaryHead->alternate_lba = 66; // Secondary header LBA
    primaryHead->first_usable_lba = 34;
    primaryHead->last_usable_lba = 65;
    primaryHead->partition_entry_lba = 2;
    primaryHead->num_partition_entries = 128;
    primaryHead->size_of_partition_entry = sizeof(GPT_ENTRY);

    // 3. Partition Entries (LBAs 2..33)
    GPT_ENTRY* primaryEntries = (GPT_ENTRY*)&gptBuffer[1024];
    for (size_t i = 0; i < partitions.size() && i < 128; i++) {
        primaryEntries[i].starting_lba = partitions[i].offset;
        primaryEntries[i].ending_lba = (partitions[i].size == 0xFFFFFFFF) ? 0x00FFFFDE : (partitions[i].offset + partitions[i].size - 1);

        for (size_t j = 0; j < partitions[i].name.length() && j < 36; j++) {
            primaryEntries[i].partition_name[j] = (uint16_t)partitions[i].name[j];
        }
    }

    uint32_t entriesCrc = CRC32_Update(0, (uint8_t*)primaryEntries, 128 * sizeof(GPT_ENTRY));
    primaryHead->partition_entry_array_crc32 = entriesCrc;
    primaryHead->header_crc32 = CRC32_Update(0, (uint8_t*)primaryHead, sizeof(GPT_HEADER));

    // 4. Secondary Backup Partition Entry Array (LBAs 34..65)
    memcpy(&gptBuffer[34 * 512], primaryEntries, 32 * 512);

    // 5. Secondary Backup GPT Header (LBA 66)
    GPT_HEADER* backupHead = (GPT_HEADER*)&gptBuffer[66 * 512];
    *backupHead = *primaryHead;
    backupHead->my_lba = 66;
    backupHead->alternate_lba = 1;
    backupHead->partition_entry_lba = 34;
    backupHead->header_crc32 = 0;
    backupHead->header_crc32 = CRC32_Update(0, (uint8_t*)backupHead, sizeof(GPT_HEADER));

    std::ofstream out(gptFile, std::ios::binary);
    if (!out.is_open()) return false;

    out.write((char*)gptBuffer.data(), gptBuffer.size());
    printf("Created Primary & Secondary GPT partition image (%zu partitions): %s\r\n", partitions.size(), gptFile.c_str());
    return true;
}

// Vendor Sector SN Read/Write
static bool WriteSN(CRKUsbComm* comm, const std::string& sn) {
    std::vector<uint8_t> vendorSector(512, 0);
    if (!comm->RKU_ReadLBA(1024, 1, vendorSector.data())) {
        memset(vendorSector.data(), 0, 512);
    }
    vendorSector[0] = 'R'; vendorSector[1] = 'K'; vendorSector[2] = 'V'; vendorSector[3] = 'B';
    strncpy((char*)&vendorSector[8], sn.c_str(), 31);

    if (comm->RKU_WriteLBA(1024, 1, vendorSector.data())) {
        puts("Write Serial Number OK");
        return true;
    }
    puts("Write Serial Number failed!");
    return false;
}

static bool ReadSN(CRKUsbComm* comm) {
    std::vector<uint8_t> vendorSector(512, 0);
    if (comm->RKU_ReadLBA(1024, 1, vendorSector.data())) {
        if (vendorSector[0] == 'R' && vendorSector[1] == 'K' && vendorSector[2] == 'V' && vendorSector[3] == 'B') {
            char sn[32] = {0};
            strncpy(sn, (char*)&vendorSector[8], 31);
            printf("Read Serial Number OK: %s\r\n", sn);
            return true;
        }
    }
    puts("Read Serial Number failed!");
    return false;
}

// Read Extended Sector Details (RE)
static bool ReadExtendedSector(CRKUsbComm* comm, uint32_t startSec, uint32_t secLen, const std::string& outFile) {
    std::vector<uint8_t> buffer(secLen * 512, 0);
    printf("Read Extended Sector from LBA %u, count %u...\r\n", startSec, secLen);
    if (comm->RKU_ReadLBA(startSec, secLen, buffer.data())) {
        if (!outFile.empty()) {
            std::ofstream out(outFile, std::ios::binary);
            out.write((char*)buffer.data(), buffer.size());
            printf("Saved extended sector details to %s\r\n", outFile.c_str());
        } else {
            // Hex dump formatted output
            printf("--- Sector Dump (LBA %u) ---\r\n", startSec);
            for (size_t i = 0; i < std::min(buffer.size(), (size_t)128); i += 16) {
                printf("%04ZX: ", i);
                for (size_t j = 0; j < 16; j++) printf("%02X ", buffer[i + j]);
                printf(" | ");
                for (size_t j = 0; j < 16; j++) {
                    char c = buffer[i + j];
                    printf("%c", (c >= 32 && c <= 126) ? c : '.');
                }
                printf("\r\n");
            }
        }
        return true;
    }
    puts("Read Extended Sector failed!");
    return false;
}

// ============================================================================
// Main Command Dispatcher
// ============================================================================
static void PrintUsage() {
    puts("------------------Upgrade Command ------------------\r");
    puts("ChooseDevice:\t\tCD\r");
    puts("ListDevice:\t\tLD\r");
    puts("SwitchDevice:\t\tSD\r");
    puts("UpgradeFirmware:\tUF <Firmware> [-noreset]\r");
    puts("UpgradeLoader:\t\tUL <Loader> [-noreset] [FLASH|EMMC|SPINOR|SPINAND]\r");
    puts("DownloadImage:\t\tDI <-p|-b|-k|-s|-r|-m|-u|-t|-re image>\r");
    puts("DownloadBoot:\t\tDB <Loader>\r");
    puts("EraseFlash:\t\tEF <Loader|firmware>\r");
    puts("PartitionList:\t\tPL\r");
    puts("PackageInfo:\t\tPI  <Firmware>\r");
    puts("WriteSN:\t\tSN <serial number>\r");
    puts("ReadSN:\t\t\tRSN\r");
    puts("ReadComLog:\t\tRCL <File>\r");
    puts("CreateGPT:\t\tGPT <Input Parameter> <Output Gpt>\r");
    puts("SwitchStorage:\t\tSSD \r");
    puts("----------------Professional Command -----------------\r");
    puts("TestDevice:\t\tTD\r");
    puts("ResetDevice:\t\tRD [subcode]\r");
    puts("ResetPipe:\t\tRP [pipe]\r");
    puts("ReadCapability:\t\tRCB\r");
    puts("ReadFlashID:\t\tRID\r");
    puts("ReadFlashInfo:\t\tRFI\r");
    puts("ReadChipInfo:\t\tRCI\r");
    puts("ReadSecureMode:\t\tRSM\r");
    puts("ReadExtended:\t\tRE  <BeginSec> <SectorLen> [File]\r");
    puts("ReadSector:\t\tRS  <BeginSec> <SectorLen> [-decode] [File]\r");
    puts("WriteSector:\t\tWS  <BeginSec> <File>\r");
    puts("ReadLBA:\t\tRL  <BeginSec> <SectorLen> [File]\r");
    puts("WriteLBA:\t\tWL  <BeginSec> <File>\r");
    puts("EraseLBA:\t\tEL  <BeginSec> <EraseCount> \r");
    puts("EraseBlock:\t\tEB <CS> <BeginBlock> <BlokcLen> [--Force]\r");
    puts("RunSystem:\t\tRUN <uboot_addr> <trust_addr> <boot_addr> <uboot> <trust> <boot>\r");
    puts("-------------------------------------------------------\r\n\r");
}

static bool ListDevices(CRKScan* pScan) {
    int count = pScan->Search();
    printf("List of rockusb connected(%d)\r\n", count);
    for (int i = 0; i < count; i++) {
        STRUCT_RKDEVICE_DESC dev;
        if (pScan->GetDevice(&dev, i)) {
            const char* modeStr = (dev.emUsbType == RKUSB_MASKROM) ? "MASKROM" :
                                 (dev.emUsbType == RKUSB_LOADER)  ? "LOADER"  : "MSC";
            printf("DevNo=%d\tLocationID=%x\tSerial=%s\tMode=%s\r\n",
                   i + 1, dev.dwLocationID, dev.strSerial.empty() ? "N/A" : dev.strSerial.c_str(), modeStr);
        }
    }
    return count > 0;
}

static bool ExecuteDeviceCommand(CRKScan* pScan, int devIdx, std::function<bool(CRKUsbComm*, libusb_device*, STRUCT_RKDEVICE_DESC&)> func) {
    if (pScan->Search() == 0) {
        puts("No found any rockusb device,please plug device in!");
        return false;
    }

    STRUCT_RKDEVICE_DESC devDesc;
    if (!pScan->GetDevice(&devDesc, devIdx)) {
        puts("No found specific rockusb device!");
        return false;
    }

    libusb_device* rawDev = pScan->GetLibusbDeviceAt(devIdx);
    if (!rawDev) {
        puts("Getting information of rockusb device failed!");
        return false;
    }

    CRKUsbComm comm;
    if (!comm.InitializeUsb(pScan->GetContext(), rawDev, devDesc)) {
        libusb_unref_device(rawDev);
        puts("Failed to initialize USB communication with device!");
        return false;
    }

    bool res = func(&comm, rawDev, devDesc);

    comm.UninitializeUsb();
    libusb_unref_device(rawDev);
    return res;
}

static bool HandleCommand(int argc, char** argv, CRKScan* pScan) {
    if (argc < 1) return false;

    std::string cmd = argv[0];
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::toupper);

    if (cmd == "LD" || cmd == "LISTDEVICE") {
        return ListDevices(pScan);
    } else if (cmd == "CD" || cmd == "CHOOSEDEVICE") {
        if (argc > 1) {
            g_ActiveDeviceIndex = strtol(argv[1], nullptr, 0) - 1;
            printf("Selected device index: %d\r\n", g_ActiveDeviceIndex + 1);
            return true;
        }
        return false;
    } else if (cmd == "PI" || cmd == "PIG" || cmd == "PACKAGEINFO") {
        if (argc < 2) {
            puts("Parameter of [PI] command is invalid,please check help!\r");
            return false;
        }
        return PrintPackageInfo(argv[1]);
    } else if (cmd == "RE" || cmd == "READEXTENDED") {
        if (argc < 3) {
            puts("Parameter of [RE] command is invalid,please check help!\r");
            return false;
        }
        uint32_t startSec = strtoul(argv[1], nullptr, 0);
        uint32_t secLen   = strtoul(argv[2], nullptr, 0);
        std::string outFile = (argc > 3) ? argv[3] : "";
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [startSec, secLen, outFile](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            return ReadExtendedSector(comm, startSec, secLen, outFile);
        });
    } else if (cmd == "SD" || cmd == "SWITCHDEVICE") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            puts("Switching device to loader mode...");
            return comm->RKU_ResetDevice(0);
        });
    } else if (cmd == "UF" || cmd == "UPGRADEFIRMWARE") {
        if (argc < 2) {
            puts("Parameter of [UF] command is invalid,please check help!\r");
            return false;
        }
        std::string fwPath = argv[1];
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [&fwPath](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC& devDesc) {
            return UpgradeFirmware(comm, devDesc, fwPath);
        });
    } else if (cmd == "UL" || cmd == "UPGRADELOADER") {
        if (argc < 2) {
            puts("Parameter of [UL] command is invalid,please check help!\r");
            return false;
        }
        std::string loaderPath = argv[1];
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [&loaderPath](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            return UpgradeLoader(comm, loaderPath);
        });
    } else if (cmd == "DI" || cmd == "DOWNLOADIMAGE") {
        if (argc < 3) {
            puts("Parameter of [DI] command is invalid,please check help!\r");
            return false;
        }
        uint32_t startLba = strtoul(argv[1], nullptr, 0);
        std::string imgPath = argv[2];
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [startLba, &imgPath](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            return DownloadImage(comm, imgPath, startLba);
        });
    } else if (cmd == "DB" || cmd == "DOWNLOADBOOT") {
        if (argc < 2) {
            puts("Parameter of [DB] command is invalid,please check help!\r");
            return false;
        }
        std::string bootPath = argv[1];
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [&bootPath](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            return UpgradeLoader(comm, bootPath);
        });
    } else if (cmd == "EF" || cmd == "ERASEFLASH") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            puts("Erasing flash memory...");
            if (comm->RKU_EraseBlock(0, 0, 0xFFFF, true)) {
                puts("Erase Flash OK");
                return true;
            }
            puts("Erase Flash failed!");
            return false;
        });
    } else if (cmd == "PL" || cmd == "PARTITIONLIST") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            uint8_t buffer[512] = {0};
            if (comm->RKU_ReadLBA(0, 1, buffer)) {
                puts("Partition List read successfully from device.");
                return true;
            }
            puts("Read Partition List failed!");
            return false;
        });
    } else if (cmd == "SN" || cmd == "WRITESN") {
        if (argc < 2) {
            puts("Parameter of [SN] command is invalid,please check help!\r");
            return false;
        }
        std::string sn = argv[1];
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [&sn](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            return WriteSN(comm, sn);
        });
    } else if (cmd == "RSN" || cmd == "READSN") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            return ReadSN(comm);
        });
    } else if (cmd == "RCL" || cmd == "READCOMLOG") {
        if (argc < 2) {
            puts("Parameter of [RCL] command is invalid,please check help!\r");
            return false;
        }
        std::string logFile = argv[1];
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [&logFile](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            std::vector<uint8_t> logBuf(4096, 0);
            if (comm->RKU_ReadComData(logBuf.data(), logBuf.size())) {
                std::ofstream out(logFile, std::ios::binary);
                out.write((char*)logBuf.data(), logBuf.size());
                printf("Read Com Log OK: saved to %s\r\n", logFile.c_str());
                return true;
            }
            puts("Read Com Log failed!");
            return false;
        });
    } else if (cmd == "SSD" || cmd == "SWITCHSTORAGE") {
        uint8_t storageCode = (argc > 1) ? (uint8_t)strtoul(argv[1], nullptr, 0) : 0;
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [storageCode](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            if (comm->RKU_SwitchStorage(storageCode)) {
                puts("Switch Storage OK");
                return true;
            }
            puts("Switch Storage failed!");
            return false;
        });
    } else if (cmd == "TD" || cmd == "TESTDEVICE") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            if (comm->RKU_TestUnitReady()) {
                puts("Test Device OK");
                return true;
            }
            puts("Test Device failed!");
            return false;
        });
    } else if (cmd == "RP" || cmd == "RESETPIPE") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            comm->ResetPipe();
            puts("Reset Pipe OK");
            return true;
        });
    } else if (cmd == "RUN" || cmd == "RUNSYSTEM") {
        if (argc < 7) {
            puts("Parameter of [RUN] command is invalid,please check help!\r");
            return false;
        }
        uint32_t ubootAddr = strtoul(argv[1], nullptr, 0);
        uint32_t trustAddr = strtoul(argv[2], nullptr, 0);
        uint32_t bootAddr  = strtoul(argv[3], nullptr, 0);
        std::string ubootImg = argv[4];
        std::string trustImg = argv[5];
        std::string bootImg  = argv[6];

        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [=](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            std::vector<uint8_t> ubootBuf, trustBuf, bootBuf;
            ReadFileToBuffer(ubootImg, ubootBuf);
            ReadFileToBuffer(trustImg, trustBuf);
            ReadFileToBuffer(bootImg, bootBuf);

            puts("Loading system execution binaries into RAM...");
            comm->RKU_WriteSDRam(ubootAddr, ubootBuf.size(), ubootBuf.data());
            comm->RKU_WriteSDRam(trustAddr, trustBuf.size(), trustBuf.data());
            comm->RKU_WriteSDRam(bootAddr, bootBuf.size(), bootBuf.data());
            comm->RKU_RunSDRam();
            puts("Run System OK");
            return true;
        });
    } else if (cmd == "RFI" || cmd == "READFLASHINFO") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            uint8_t buffer[512];
            if (comm->RKU_ReadFlashInfo(buffer)) {
                uint32_t flashSize = *(uint32_t*)(buffer);
                uint16_t blockBytes = *(uint16_t*)(buffer + 4);
                printf("Read Flash Info OK: Size=%u MB, BlockSize=%u KB\r\n", flashSize / 2048, blockBytes);
                return true;
            }
            puts("Read Flash Info failed!");
            return false;
        });
    } else if (cmd == "RID" || cmd == "READFLASHID") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            uint8_t buffer[5] = {0};
            if (comm->RKU_ReadFlashID(buffer)) {
                printf("Read Flash ID OK: %02X %02X %02X %02X %02X\r\n",
                       buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
                return true;
            }
            puts("Read Flash ID failed!");
            return false;
        });
    } else if (cmd == "RCI" || cmd == "READCHIPINFO") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            uint8_t buffer[16] = {0};
            if (comm->RKU_ReadChipInfo(buffer)) {
                printf("Read Chip Info OK: %s\r\n", StringToHex(buffer, 16).c_str());
                return true;
            }
            puts("Read Chip Info failed!");
            return false;
        });
    } else if (cmd == "RCB" || cmd == "READCAPABILITY") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            uint8_t buffer[8] = {0};
            if (comm->RKU_ReadCapability(buffer)) {
                printf("Read Capability OK: %02X %02X %02X %02X\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
                return true;
            }
            puts("Read Capability failed!");
            return false;
        });
    } else if (cmd == "RSM" || cmd == "READSECUREMODE") {
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            uint8_t buffer[8] = {0};
            if (comm->RKU_ReadCapability(buffer)) {
                printf("Read Secure Mode OK: %s\r\n", (buffer[0] & 1) ? "Secure Mode Enabled" : "Non-Secure Mode");
                return true;
            }
            puts("Read Secure Mode failed!");
            return false;
        });
    } else if (cmd == "RD" || cmd == "RESETDEVICE") {
        uint8_t subCode = (argc > 1) ? (uint8_t)strtoul(argv[1], nullptr, 0) : 0;
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [subCode](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            if (comm->RKU_ResetDevice(subCode)) {
                puts("Reset Device OK");
                return true;
            }
            puts("Reset Device failed!");
            return false;
        });
    } else if (cmd == "EL" || cmd == "ERASELBA") {
        if (argc < 3) {
            puts("Parameter of [EL] command is invalid,please check help!\r");
            return false;
        }
        uint32_t beginSec = strtoul(argv[1], nullptr, 0);
        uint32_t count    = strtoul(argv[2], nullptr, 0);
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [beginSec, count](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            printf("Erase LBA from %u, count %u...\r\n", beginSec, count);
            if (comm->RKU_EraseLBA(beginSec, count)) {
                puts("Erase LBA OK");
                return true;
            }
            puts("Erase LBA failed!");
            return false;
        });
    } else if (cmd == "EB" || cmd == "ERASEBLOCK") {
        if (argc < 4) {
            puts("Parameter of [EB] command is invalid,please check help!\r");
            return false;
        }
        uint8_t cs = (uint8_t)strtoul(argv[1], nullptr, 0);
        uint32_t beginBlock = strtoul(argv[2], nullptr, 0);
        uint32_t blockLen   = strtoul(argv[3], nullptr, 0);
        bool force = (argc > 4 && std::string(argv[4]) == "--Force");

        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [=](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            printf("Erase Block CS%u from %u, len %u...\r\n", cs, beginBlock, blockLen);
            if (comm->RKU_EraseBlock(cs, beginBlock, blockLen, force)) {
                puts("Erase Block OK");
                return true;
            }
            puts("Erase Block failed!");
            return false;
        });
    } else if (cmd == "RL" || cmd == "READLBA" || cmd == "RS" || cmd == "READSECTOR") {
        if (argc < 3) {
            puts("Parameter of [RL] command is invalid,please check help!\r");
            return false;
        }
        uint32_t beginSec = strtoul(argv[1], nullptr, 0);
        uint32_t count    = strtoul(argv[2], nullptr, 0);
        std::string outFile = (argc > 3) ? argv[3] : "";
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [beginSec, count, outFile](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            std::vector<uint8_t> buffer(count * 512);
            printf("Read LBA from %u, count %u...\r\n", beginSec, count);
            if (comm->RKU_ReadLBA(beginSec, count, buffer.data())) {
                if (!outFile.empty()) {
                    std::ofstream out(outFile, std::ios::binary);
                    out.write((char*)buffer.data(), buffer.size());
                    printf("Saved data to %s\r\n", outFile.c_str());
                } else {
                    puts("Read LBA OK");
                }
                return true;
            }
            puts("Read LBA failed!");
            return false;
        });
    } else if (cmd == "WL" || cmd == "WRITELBA" || cmd == "WS" || cmd == "WRITESECTOR") {
        if (argc < 3) {
            puts("Parameter of [WL] command is invalid,please check help!\r");
            return false;
        }
        uint32_t beginSec = strtoul(argv[1], nullptr, 0);
        std::string inFile = argv[2];
        std::vector<uint8_t> buffer;
        if (!ReadFileToBuffer(inFile, buffer)) {
            printf("Failed to open file: %s\r\n", inFile.c_str());
            return false;
        }
        uint32_t sectorCount = (buffer.size() + 511) / 512;
        buffer.resize(sectorCount * 512, 0);
        return ExecuteDeviceCommand(pScan, g_ActiveDeviceIndex, [beginSec, sectorCount, &buffer](CRKUsbComm* comm, libusb_device*, STRUCT_RKDEVICE_DESC&) {
            printf("Write LBA from %u, sectors %u...\r\n", beginSec, sectorCount);
            if (comm->RKU_WriteLBA(beginSec, sectorCount, buffer.data())) {
                puts("Write LBA OK");
                return true;
            }
            puts("Write LBA failed!");
            return false;
        });
    } else if (cmd == "GPT") {
        if (argc < 3) {
            puts("GPT command is invalid,please check help!\r");
            return false;
        }
        return CreateGPT(argv[1], argv[2]);
    } else if (cmd == "-H" || cmd == "--HELP" || cmd == "HELP") {
        PrintUsage();
        return true;
    } else {
        puts("command is invalid,please press upgrade_tool -h to check usage!\r");
        return false;
    }

    return true;
}

// Shell Interactive Mode
static void RunInteractiveShell(CRKScan* pScan) {
    char cwd[512];
    if (getcwd(cwd, sizeof(cwd))) {
        printf("config.ini\r\n");
        printf("Program Log will save in the %s/log/\r\n", cwd);
    }

    while (true) {
        printf("List of rockusb connected\r\n");
        int count = pScan->Search();
        ListDevices(pScan);

        if (count < 1) {
            printf("No found rockusb,Rescan press <R>,Quit press <Q>:");
        } else {
            printf("Found %d rockusb,Select input DevNo,Rescan press <R>,Quit press <Q>:", count);
        }
        fflush(stdout);

        char line[512];
        if (!fgets(line, sizeof(line), stdin)) break;

        std::string input(line);
        input.erase(0, input.find_first_not_of(" \t\r\n"));
        input.erase(input.find_last_not_of(" \t\r\n") + 1);

        if (input.empty()) continue;
        if (input == "R" || input == "r") {
            continue;
        }
        if (input == "Q" || input == "q" || input == "exit" || input == "quit") {
            break;
        }

        int devNo = strtol(input.c_str(), nullptr, 0);
        if (devNo >= 1 && devNo <= count) {
            g_ActiveDeviceIndex = devNo - 1;
            printf("Selected device index: %d\r\n", devNo);
        }

        while (true) {
            printf("Rockusb>");
            fflush(stdout);

            if (!fgets(line, sizeof(line), stdin)) return;

            std::string cmdLine(line);
            cmdLine.erase(0, cmdLine.find_first_not_of(" \t\r\n"));
            cmdLine.erase(cmdLine.find_last_not_of(" \t\r\n") + 1);

            if (cmdLine.empty()) continue;
            if (cmdLine == "q" || cmdLine == "quit" || cmdLine == "exit") return;
            if (cmdLine == "r" || cmdLine == "R") break;
            if (cmdLine == "CS" || cmdLine == "cs") {
                printf("\033[2J\033[1;1H");
                continue;
            }

            std::vector<std::string> args;
            std::stringstream ss(cmdLine);
            std::string item;
            while (ss >> item) {
                args.push_back(item);
            }

            std::vector<char*> cargs;
            for (auto& a : args) cargs.push_back((char*)a.c_str());

            HandleCommand((int)cargs.size(), cargs.data(), pScan);
        }
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================
int main(int argc, char** argv) {
    CRKScan scan;

    if (argc > 1) {
        HandleCommand(argc - 1, argv + 1, &scan);
    } else {
        RunInteractiveShell(&scan);
    }

    return 0;
}

