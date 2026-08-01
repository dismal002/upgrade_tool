#include "RKComm.h"
#include <string.h>
#include <stdio.h>
    CRKUsbComm::CRKUsbComm() : m_ctx(nullptr), m_devHandle(nullptr), m_epOut(0x01), m_epIn(0x81), m_tagCounter(1) {}
    CRKUsbComm::~CRKUsbComm() { UninitializeUsb(); }

    uint32_t CRKUsbComm::MakeCBWTag() { return m_tagCounter++; }

    void CRKUsbComm::InitializeCBW(CBW* pCBW, USB_OPERATION_CODE code) {
        memset(pCBW, 0, sizeof(CBW));
        pCBW->dwCBWSignature = CBW_SIGNATURE;
        pCBW->dwCBWTag = MakeCBWTag();
        pCBW->cbwcb.ucOperCode = (uint8_t)code;

        switch (code) {
        case TEST_UNIT_READY:
        case READ_FLASH_ID:
        case READ_FLASH_INFO:
        case READ_CHIP_INFO:
        case READ_CAPABILITY:
            pCBW->ucCBWFlags = CBW_FLAG_IN;
            pCBW->ucCBWCBLength = 6;
            break;
        case READ_SECTOR:
        case READ_LBA:
        case READ_COM_DATA:
            pCBW->ucCBWFlags = CBW_FLAG_IN;
            pCBW->ucCBWCBLength = 10;
            break;
        case WRITE_SECTOR:
        case ERASE_NORMAL:
        case ERASE_FORCE:
        case WRITE_LBA:
        case WRITE_SDRAM:
        case EXECUTE_SDRAM:
        case ERASE_LBA:
            pCBW->ucCBWFlags = CBW_FLAG_OUT;
            pCBW->ucCBWCBLength = 10;
            break;
        case DEVICE_RESET:
        case SWITCH_STORAGE:
            pCBW->ucCBWFlags = CBW_FLAG_OUT;
            pCBW->ucCBWCBLength = 6;
            break;
        default:
            pCBW->ucCBWFlags = CBW_FLAG_IN;
            pCBW->ucCBWCBLength = 10;
            break;
        }
    }

    bool CRKUsbComm::InitializeUsb(libusb_context* ctx, libusb_device* dev, STRUCT_RKDEVICE_DESC devDesc) {
        m_ctx = ctx;
        m_devDesc = devDesc;

        int res = libusb_open(dev, &m_devHandle);
        if (res < 0 || !m_devHandle) {
            if (res == -3) {
                printf("Error: Open USB device failed due to insufficient permissions (LIBUSB_ERROR_ACCESS).\r\n");
                printf("Please run with 'sudo' or install udev rules for Rockchip device (VID 0x2207).\r\n");
            } else {
                printf("Error: Open USB device failed, err=%d\r\n", res);
            }
            return false;
        }

        int targetInterface = 0;
        libusb_config_descriptor* config = nullptr;
        if (libusb_get_active_config_descriptor(dev, &config) == 0 && config) {
            if (config->bNumInterfaces > 0 && config->interface[0].num_altsetting > 0) {
                const libusb_interface_descriptor* alt = &config->interface[0].altsetting[0];
                targetInterface = alt->bInterfaceNumber;
                for (int i = 0; i < alt->bNumEndpoints; i++) {
                    uint8_t ep = alt->endpoint[i].bEndpointAddress;
                    if ((ep & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN) {
                        m_epIn = ep;
                    } else {
                        m_epOut = ep;
                    }
                }
            }
            libusb_free_config_descriptor(config);
        }

        libusb_detach_kernel_driver(m_devHandle, targetInterface);
        res = libusb_claim_interface(m_devHandle, targetInterface);
        if (res < 0) {
            printf("Error: Claim USB interface %d failed, err=%d\r\n", targetInterface, res);
            libusb_close(m_devHandle);
            m_devHandle = nullptr;
            return false;
        }
        return true;
    }

    void CRKUsbComm::UninitializeUsb() {
        if (m_devHandle) {
            libusb_release_interface(m_devHandle, 0);
            libusb_close(m_devHandle);
            m_devHandle = nullptr;
        }
    }

    void CRKUsbComm::ResetPipe() {
        if (m_devHandle) {
            libusb_clear_halt(m_devHandle, m_epOut);
            libusb_clear_halt(m_devHandle, m_epIn);
        }
    }

    bool CRKUsbComm::ExecuteCBW(CBW* pCBW, uint8_t* pBuffer, uint32_t dwSize, uint32_t timeoutMs) {
        if (!m_devHandle) return false;

        int transferred = 0;
        pCBW->dwCBWTransferLength = dwSize;

        int res = libusb_bulk_transfer(m_devHandle, m_epOut, (uint8_t*)pCBW, sizeof(CBW), &transferred, timeoutMs);
        if (res != 0 || transferred != sizeof(CBW)) {
            ResetPipe();
            return false;
        }

        if (dwSize > 0 && pBuffer != nullptr) {
            uint8_t ep = (pCBW->ucCBWFlags & CBW_FLAG_IN) ? m_epIn : m_epOut;
            res = libusb_bulk_transfer(m_devHandle, ep, pBuffer, dwSize, &transferred, timeoutMs);
            if (res != 0) {
                ResetPipe();
                return false;
            }
        }

        CSW csw;
        memset(&csw, 0, sizeof(CSW));
        res = libusb_bulk_transfer(m_devHandle, m_epIn, (uint8_t*)&csw, sizeof(CSW), &transferred, timeoutMs);
        if (res != 0 || transferred != sizeof(CSW)) {
            ResetPipe();
            return false;
        }

        if (csw.dwCSWSignature != CSW_SIGNATURE || csw.dwCSWTag != pCBW->dwCBWTag || csw.bCSWStatus != 0) {
            ResetPipe();
            return false;
        }

        return true;
    }

    bool CRKUsbComm::RKU_TestUnitReady() {
        CBW cbw;
        InitializeCBW(&cbw, TEST_UNIT_READY);
        return ExecuteCBW(&cbw, nullptr, 0, 5000);
    }

    bool CRKUsbComm::RKU_ReadFlashID(uint8_t* pBuffer) {
        CBW cbw;
        InitializeCBW(&cbw, READ_FLASH_ID);
        return ExecuteCBW(&cbw, pBuffer, 5, 5000);
    }

    bool CRKUsbComm::RKU_ReadFlashInfo(uint8_t* pBuffer) {
        CBW cbw;
        InitializeCBW(&cbw, READ_FLASH_INFO);
        return ExecuteCBW(&cbw, pBuffer, 512, 5000);
    }

    bool CRKUsbComm::RKU_ReadChipInfo(uint8_t* pBuffer) {
        CBW cbw;
        InitializeCBW(&cbw, READ_CHIP_INFO);
        return ExecuteCBW(&cbw, pBuffer, 16, 5000);
    }

    bool CRKUsbComm::RKU_ReadCapability(uint8_t* pBuffer) {
        CBW cbw;
        InitializeCBW(&cbw, READ_CAPABILITY);
        return ExecuteCBW(&cbw, pBuffer, 8, 5000);
    }

    bool CRKUsbComm::RKU_ReadComData(uint8_t* pBuffer, uint32_t dwSize) {
        CBW cbw;
        InitializeCBW(&cbw, READ_COM_DATA);
        return ExecuteCBW(&cbw, pBuffer, dwSize, 10000);
    }

    bool CRKUsbComm::RKU_ReadLBA(uint32_t dwPos, uint32_t dwCount, uint8_t* pBuffer) {
        CBW cbw;
        InitializeCBW(&cbw, READ_LBA);
        cbw.cbwcb.dwAddress = __builtin_bswap32(dwPos);
        cbw.cbwcb.usLength  = __builtin_bswap16((uint16_t)dwCount);
        return ExecuteCBW(&cbw, pBuffer, dwCount * 512, 20000);
    }

    bool CRKUsbComm::RKU_WriteLBA(uint32_t dwPos, uint32_t dwCount, uint8_t* pBuffer) {
        CBW cbw;
        InitializeCBW(&cbw, WRITE_LBA);
        cbw.cbwcb.dwAddress = __builtin_bswap32(dwPos);
        cbw.cbwcb.usLength  = __builtin_bswap16((uint16_t)dwCount);
        return ExecuteCBW(&cbw, pBuffer, dwCount * 512, 30000);
    }

    bool CRKUsbComm::RKU_EraseLBA(uint32_t dwPos, uint32_t dwCount) {
        CBW cbw;
        InitializeCBW(&cbw, ERASE_LBA);
        cbw.cbwcb.dwAddress = __builtin_bswap32(dwPos);
        cbw.cbwcb.usLength  = __builtin_bswap16((uint16_t)dwCount);
        return ExecuteCBW(&cbw, nullptr, 0, 120000); // Extended timeout for LBA erase
    }

    bool CRKUsbComm::RKU_EraseBlock(uint8_t ucFlashCS, uint32_t dwPos, uint32_t dwCount, bool bForce) {
        CBW cbw;
        InitializeCBW(&cbw, bForce ? ERASE_FORCE : ERASE_NORMAL);
        cbw.ucCBWLUN = ucFlashCS;
        cbw.cbwcb.dwAddress = __builtin_bswap32(dwPos);
        cbw.cbwcb.usLength  = __builtin_bswap16((uint16_t)dwCount);
        return ExecuteCBW(&cbw, nullptr, 0, 180000); // 3 minutes timeout for Full Erase Flash
    }

    bool CRKUsbComm::RKU_ResetDevice(uint8_t subCode) {
        CBW cbw;
        InitializeCBW(&cbw, DEVICE_RESET);
        cbw.cbwcb.ucReserved1 = subCode;
        return ExecuteCBW(&cbw, nullptr, 0, 5000);
    }

    bool CRKUsbComm::RKU_SwitchStorage(uint8_t storageCode) {
        CBW cbw;
        InitializeCBW(&cbw, SWITCH_STORAGE);
        cbw.cbwcb.ucReserved1 = storageCode;
        return ExecuteCBW(&cbw, nullptr, 0, 5000);
    }

    bool CRKUsbComm::RKU_WriteSDRam(uint32_t dwAddr, uint32_t dwSize, uint8_t* pBuffer) {
        CBW cbw;
        InitializeCBW(&cbw, WRITE_SDRAM);
        cbw.cbwcb.dwAddress = __builtin_bswap32(dwAddr);
        cbw.cbwcb.usLength  = __builtin_bswap16((uint16_t)(dwSize / 512));
        return ExecuteCBW(&cbw, pBuffer, dwSize, 30000);
    }

    bool CRKUsbComm::RKU_RunSDRam() {
        CBW cbw;
        InitializeCBW(&cbw, EXECUTE_SDRAM);
        return ExecuteCBW(&cbw, nullptr, 0, 5000);
    }

