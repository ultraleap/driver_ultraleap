#include "DriverLog.h"

vr::IVRDriverLog* DriverLog::pLogFile = nullptr;

void DriverLog::Init() {
    pLogFile = vr::VRDriverLog();
}
