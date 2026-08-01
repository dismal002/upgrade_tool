#pragma once
#include "DefineHeader.h"
#include <stdint.h>

class CRKUsbComm {
private:
    libusb_context*       m_ctx;
    libusb_device_handle* m_devHandle;
    uint8_t               m_epOut;
    uint8_t               m_epIn;
    uint32_t              m_tagCounter;
    STRUCT_RKDEVICE_DESC  m_devDesc;

public:
    CRKUsbComm();
    ~CRKUsbComm();

    uint32_t MakeCBWTag();
    void InitializeCBW(CBW* pCBW, USB_OPERATION_CODE code);
    bool InitializeUsb(libusb_context* ctx, libusb_device* dev, STRUCT_RKDEVICE_DESC devDesc);
    void UninitializeUsb();
    void ResetPipe();
    bool ExecuteCBW(CBW* pCBW, uint8_t* pBuffer, uint32_t dwSize, uint32_t timeoutMs = 15000);

    bool RKU_TestUnitReady();
    bool RKU_ReadFlashID(uint8_t* pBuffer);
    bool RKU_ReadFlashInfo(uint8_t* pBuffer);
    bool RKU_ReadChipInfo(uint8_t* pBuffer);
    bool RKU_ReadCapability(uint8_t* pBuffer);
    bool RKU_ReadComData(uint8_t* pBuffer, uint32_t dwSize);
    bool RKU_ReadLBA(uint32_t dwPos, uint32_t dwCount, uint8_t* pBuffer);
    bool RKU_WriteLBA(uint32_t dwPos, uint32_t dwCount, uint8_t* pBuffer);
    bool RKU_EraseLBA(uint32_t dwPos, uint32_t dwCount);
    bool RKU_EraseBlock(uint8_t ucFlashCS, uint32_t dwPos, uint32_t dwCount, bool bForce = false);
    bool RKU_ResetDevice(uint8_t subCode = 0);
    bool RKU_SwitchStorage(uint8_t storageCode);
    bool RKU_WriteSDRam(uint32_t dwAddr, uint32_t dwSize, uint8_t* pBuffer);
    bool RKU_RunSDRam();
};
