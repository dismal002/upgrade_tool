#pragma once
#include "DefineHeader.h"
#include <string>
#include <fstream>
#include <stdarg.h>

class CRKLog {
private:
    bool m_bEnable;
    std::string m_strPath;
    std::ofstream m_logFile;

public:
    CRKLog(std::string path = "upgrade_tool.log", bool enable = true);
    ~CRKLog();
    void Record(const char* fmt, ...);
};

extern CRKLog g_Log;
extern int g_ActiveDeviceIndex;
