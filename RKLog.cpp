#include "RKLog.h"

CRKLog::CRKLog(std::string path, bool enable) : m_bEnable(enable), m_strPath(path) {
    if (m_bEnable) {
        m_logFile.open(m_strPath, std::ios::out | std::ios::app);
    }
}

CRKLog::~CRKLog() {
    if (m_logFile.is_open()) m_logFile.close();
}

void CRKLog::Record(const char* fmt, ...) {
    if (!m_bEnable) return;
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (m_logFile.is_open()) {
        m_logFile << buffer << std::endl;
    }
}

CRKLog g_Log("upgrade_tool.log", true);
int g_ActiveDeviceIndex = 0;
