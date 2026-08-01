#pragma once
#include "DefineHeader.h"
#include <vector>
#include <string>

class CRKScan {
private:
    libusb_context* m_ctx;
    std::vector<STRUCT_RKDEVICE_DESC> m_deviceList;

public:
    CRKScan();
    ~CRKScan();

    libusb_context* GetContext();
    int Search(const std::string& serialFilter = "");
    bool GetDevice(STRUCT_RKDEVICE_DESC* pDev, int pos = 0);
    libusb_device* GetLibusbDeviceAt(int index);
};
