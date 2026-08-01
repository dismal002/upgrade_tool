#include "RKScan.h"
#include <stdlib.h>
#include <stdio.h>
    CRKScan::CRKScan() { libusb_init(&m_ctx); }
    CRKScan::~CRKScan() { if (m_ctx) libusb_exit(m_ctx); }

    libusb_context* CRKScan::GetContext() { return m_ctx; }

    int CRKScan::Search(const std::string& serialFilter) {
        m_deviceList.clear();
        libusb_device** list = nullptr;
        ssize_t cnt = libusb_get_device_list(m_ctx, &list);
        if (cnt < 0) return 0;

        // Check environment variable filter SELECTED_DEVICE or LOCATION_ID
        std::string envSelected = serialFilter;
        if (envSelected.empty()) {
            const char* envDev = getenv("SELECTED_DEVICE");
            if (!envDev) envDev = getenv("LOCATION_ID");
            if (envDev) envSelected = envDev;
        }

        for (ssize_t i = 0; i < cnt; i++) {
            libusb_device_descriptor desc;
            if (libusb_get_device_descriptor(list[i], &desc) == 0) {
                if (desc.idVendor == ROCKCHIP_VID) {
                    STRUCT_RKDEVICE_DESC dev;
                    dev.usVid = desc.idVendor;
                    dev.usPid = desc.idProduct;
                    dev.dwLocationID = (libusb_get_bus_number(list[i]) << 8) | libusb_get_port_number(list[i]);
                    dev.emDevType = RK30;

                    // Read Serial String Descriptor if available
                    libusb_device_handle* hDev = nullptr;
                    if (libusb_open(list[i], &hDev) == 0 && hDev) {
                        unsigned char serialBuf[64] = {0};
                        if (desc.iSerialNumber > 0 && libusb_get_string_descriptor_ascii(hDev, desc.iSerialNumber, serialBuf, sizeof(serialBuf)) > 0) {
                            dev.strSerial = (char*)serialBuf;
                        }
                        libusb_close(hDev);
                    }

                    uint16_t subPid = desc.idProduct & 0xFF00;
                    if ((desc.idProduct & 0x00FF) == 0x0A || subPid == 0x3000 || subPid == 0x3100 || subPid == 0x3200 || subPid == 0x3300) {
                        dev.emUsbType = RKUSB_MASKROM;
                    } else if ((desc.idProduct & 0x00FF) == 0x0B || desc.idProduct == 0x0006 || desc.idProduct == 0x0011 || desc.idProduct == 0x0018) {
                        dev.emUsbType = RKUSB_LOADER;
                    } else {
                        dev.emUsbType = RKUSB_MSC;
                    }

                    // Apply filter if specified
                    if (!envSelected.empty()) {
                        char locHex[16];
                        snprintf(locHex, sizeof(locHex), "%x", dev.dwLocationID);
                        if (dev.strSerial.find(envSelected) == std::string::npos && std::string(locHex).find(envSelected) == std::string::npos) {
                            continue;
                        }
                    }

                    m_deviceList.push_back(dev);
                }
            }
        }
        libusb_free_device_list(list, 1);
        return (int)m_deviceList.size();
    }

    bool CRKScan::GetDevice(STRUCT_RKDEVICE_DESC* pDev, int pos) {
        if (pos >= 0 && pos < (int)m_deviceList.size()) {
            *pDev = m_deviceList[pos];
            return true;
        }
        return false;
    }

    libusb_device* CRKScan::GetLibusbDeviceAt(int index) {
        libusb_device** list = nullptr;
        ssize_t cnt = libusb_get_device_list(m_ctx, &list);
        if (cnt < 0) return nullptr;

        int matchIdx = 0;
        libusb_device* targetDev = nullptr;
        for (ssize_t i = 0; i < cnt; i++) {
            libusb_device_descriptor desc;
            if (libusb_get_device_descriptor(list[i], &desc) == 0) {
                if (desc.idVendor == ROCKCHIP_VID) {
                    if (matchIdx == index) {
                        targetDev = list[i];
                        libusb_ref_device(targetDev);
                        break;
                    }
                    matchIdx++;
                }
            }
        }
        libusb_free_device_list(list, 1);
        return targetDev;
    }

