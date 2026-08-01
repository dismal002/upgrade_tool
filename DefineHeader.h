#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <errno.h>

#include <string>
#include <vector>
#include <map>
#include <set>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <algorithm>
#include <iomanip>
#include <thread>
#include <chrono>
#include <mutex>
#include <functional>

// ============================================================================
// Libusb Interface Declarations (Compatible with system libusb-1.0.so)
// ============================================================================
typedef struct libusb_context libusb_context;
typedef struct libusb_device libusb_device;
typedef struct libusb_device_handle libusb_device_handle;

struct libusb_device_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
};

struct libusb_endpoint_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
    uint8_t  bRefresh;
    uint8_t  bSynchAddress;
    const uint8_t *extra;
    int extra_length;
};

struct libusb_interface_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
    const struct libusb_endpoint_descriptor *endpoint;
    const uint8_t *extra;
    int extra_length;
};

struct libusb_interface {
    const struct libusb_interface_descriptor *altsetting;
    int num_altsetting;
};

struct libusb_config_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  MaxPower;
    const struct libusb_interface *interface;
    const uint8_t *extra;
    int extra_length;
};

#define LIBUSB_ENDPOINT_IN 0x80
#define LIBUSB_ENDPOINT_OUT 0x00
#define LIBUSB_ENDPOINT_DIR_MASK 0x80
#define LIBUSB_ERROR_PIPE -9

extern "C" {
int libusb_init(libusb_context **ctx);
void libusb_exit(libusb_context *ctx);
ssize_t libusb_get_device_list(libusb_context *ctx, libusb_device ***list);
void libusb_free_device_list(libusb_device **list, int unref_devices);
uint8_t libusb_get_bus_number(libusb_device *dev);
uint8_t libusb_get_port_number(libusb_device *dev);
int libusb_get_device_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc);
int libusb_get_string_descriptor_ascii(libusb_device_handle *dev, uint8_t desc_index, unsigned char *data, int length);
int libusb_get_active_config_descriptor(libusb_device *dev, struct libusb_config_descriptor **config);
void libusb_free_config_descriptor(struct libusb_config_descriptor *config);
libusb_device *libusb_ref_device(libusb_device *dev);
void libusb_unref_device(libusb_device *dev);
int libusb_open(libusb_device *dev, libusb_device_handle **dev_handle);
void libusb_close(libusb_device_handle *dev_handle);
int libusb_claim_interface(libusb_device_handle *dev_handle, int interface_number);
int libusb_release_interface(libusb_device_handle *dev_handle, int interface_number);
int libusb_detach_kernel_driver(libusb_device_handle *dev_handle, int interface_number);
int libusb_clear_halt(libusb_device_handle *dev_handle, unsigned char endpoint);
int libusb_bulk_transfer(libusb_device_handle *dev_handle, unsigned char endpoint,
                         unsigned char *data, int length, int *actual_length, unsigned int timeout);
}

// ============================================================================
// Constants & USB Protocol Definitions
// ============================================================================
#define ROCKCHIP_VID 0x2207

#define CBW_SIGNATURE 0x43425355 // "USBC"
#define CSW_SIGNATURE 0x53425355 // "USBS"

#define CBW_FLAG_IN  0x80
#define CBW_FLAG_OUT 0x00

#define SPARSE_HEADER_MAGIC 0xED26FF3A
#define CHUNK_TYPE_RAW       0xCAC1
#define CHUNK_TYPE_FILL      0xCAC2
#define CHUNK_TYPE_DONT_CARE 0xCAC3
#define CHUNK_TYPE_CRC32     0xCAC4

#define RKAF_MAGIC 0x46414B52 // "RKAF"
#define RKBOOT_MAGIC 0x544F4F42 // "BOOT"

enum USB_OPERATION_CODE {
    TEST_UNIT_READY   = 0x00,
    READ_FLASH_ID     = 0x01,
    READ_FLASH_INFO   = 0x02,
    READ_FLASH_ID_ALT = 0x04,
    ERASE_NORMAL      = 0x0A,
    ERASE_FORCE       = 0x0B,
    READ_LBA          = 0x14,
    WRITE_LBA         = 0x15,
    READ_SECTOR       = 0x17,
    WRITE_SECTOR      = 0x18,
    READ_FLASH_INFO_2 = 0x1A,
    READ_CHIP_INFO    = 0x1B,
    WRITE_SDRAM       = 0x1C,
    EXECUTE_SDRAM     = 0x1D,
    READ_COM_DATA     = 0x1E,
    ERASE_LBA         = 0x25,
    LIST_STORAGE      = 0x2A,
    SWITCH_STORAGE    = 0x2B,
    READ_CAPABILITY   = 0xAA,
    MSC_TO_ROCKUSB    = 0xFE,
    DEVICE_RESET      = 0xFF
};

enum ENUM_RKDEVICE_TYPE {
    RK_UNKNOWN  = 0,
    RK28        = 1,
    RK281X      = 2,
    RKPANDA     = 3,
    RK27        = 4,
    RKNANO      = 5,
    RKSMART     = 6,
    RKCROWN     = 7,
    RK29        = 8,
    RK292X      = 9,
    RK30        = 10,
    RK30B       = 11,
    RK31        = 12,
    RK32        = 13,
    RK33        = 14,
    RK330X      = 15
};

enum ENUM_RKUSB_TYPE {
    RKUSB_NONE    = 0,
    RKUSB_MASKROM = 1,
    RKUSB_LOADER  = 2,
    RKUSB_MSC     = 3
};

#pragma pack(push, 1)
struct CBWCB {
    uint8_t  ucOperCode;
    uint8_t  ucReserved1;
    uint32_t dwAddress;
    uint8_t  ucReserved2;
    uint16_t usLength;
    uint8_t  ucReserved3[7];
};

struct CBW {
    uint32_t dwCBWSignature;
    uint32_t dwCBWTag;
    uint32_t dwCBWTransferLength;
    uint8_t  ucCBWFlags;
    uint8_t  ucCBWLUN;
    uint8_t  ucCBWCBLength;
    CBWCB    cbwcb;
};

struct CSW {
    uint32_t dwCSWSignature;
    uint32_t dwCSWTag;
    uint32_t dwCSWDataResidue;
    uint8_t  bCSWStatus;
};

struct sparse_header {
    uint32_t magic;
    uint16_t major_version;
    uint16_t minor_version;
    uint16_t file_hdr_sz;
    uint16_t chunk_hdr_sz;
    uint32_t blk_sz;
    uint32_t total_blks;
    uint32_t total_chunks;
    uint32_t image_checksum;
};

struct chunk_header {
    uint16_t chunk_type;
    uint16_t reserved1;
    uint32_t chunk_sz;
    uint32_t total_sz;
};

struct STRUCT_RKTIME {
    uint16_t usYear;
    uint8_t  ucMonth;
    uint8_t  ucDay;
    uint8_t  ucHour;
    uint8_t  ucMinute;
    uint8_t  ucSecond;
};

struct STRUCT_RKIMAGE_HEAD {
    uint32_t uiTag;             // 0x00: "RKAF"
    uint32_t dwFWSize;          // 0x04: Firmware size
    char     szVersion[32];     // 0x08: Version string
    char     szReleaseDate[32]; // 0x28: Release date
    uint32_t emSupportDevice;   // 0x48: Supported chip
    uint32_t dwItemSize;        // 0x4C: Item entry size (112)
    uint8_t  reserved[56];      // 0x50..0x87
    uint32_t item_count;        // 0x88 (136): Number of partition items
};

struct STRUCT_RKIMAGE_ITEM {
    char szName[32];
    char szFile[64];
    uint32_t dwOffset;
    uint32_t dwSize;
    uint32_t dwFlashOffset;
    uint32_t dwFlashSize;
};

struct STRUCT_RKBOOT_HEAD {
    char szTag[4];
    uint16_t usSize;
    uint32_t dwVersion;
    uint32_t dwMergeVersion;
    STRUCT_RKTIME stReleaseTime;
    uint32_t emSupportDevice;
    uint8_t  ucEntry471Count;
    uint32_t dwEntry471Offset;
    uint8_t  ucEntry471Size;
    uint8_t  ucEntry472Count;
    uint32_t dwEntry472Offset;
    uint8_t  ucEntry472Size;
    uint8_t  ucLoaderCount;
    uint32_t dwLoaderOffset;
    uint8_t  ucLoaderSize;
};

#define RKFW_MAGIC 0x57464B52 // "RKFW"

struct STRUCT_RKFW_HEAD {
    uint32_t uiTag;           // "RKFW" (0x57464B52)
    uint16_t usSize;          // Header size (102 / 0x66)
    uint32_t dwVersion;       // Version
    uint32_t dwMergeVersion;  // Merge version
    STRUCT_RKTIME stReleaseTime;
    uint32_t emSupportDevice;
    uint32_t dwBootOffset;    // Loader offset
    uint32_t dwBootSize;      // Loader size
    uint32_t dwFWOffset;      // Embedded RKAF firmware offset
    uint64_t dwFWSize;        // Embedded RKAF firmware size
};

struct GPT_HEADER {
    uint64_t signature;
    uint32_t revision;
    uint32_t header_size;
    uint32_t header_crc32;
    uint32_t reserved;
    uint64_t my_lba;
    uint64_t alternate_lba;
    uint64_t first_usable_lba;
    uint64_t last_usable_lba;
    uint8_t  disk_guid[16];
    uint64_t partition_entry_lba;
    uint32_t num_partition_entries;
    uint32_t size_of_partition_entry;
    uint32_t partition_entry_array_crc32;
};

struct GPT_ENTRY {
    uint8_t  partition_type_guid[16];
    uint8_t  unique_partition_guid[16];
    uint64_t starting_lba;
    uint64_t ending_lba;
    uint64_t attributes;
    uint16_t partition_name[36];
};
#pragma pack(pop)

struct STRUCT_RKDEVICE_DESC {
    uint16_t usVid;
    uint16_t usPid;
    uint32_t dwLocationID;
    ENUM_RKUSB_TYPE emUsbType;
    ENUM_RKDEVICE_TYPE emDevType;
    std::string strSerial;
    std::string strDevPath;
};

struct STRUCT_PARAM_ITEM {
    std::string name;
    uint32_t offset;
    uint32_t size;
};

// ============================================================================
