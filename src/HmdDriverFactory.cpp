#include <openvr_driver.h>

#include "LeapDeviceProvider.h"

LeapDeviceProvider sDeviceProvider{};

OVR_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    if (std::string_view{pInterfaceName} == vr::IServerTrackedDeviceProvider_Version) {
        return &sDeviceProvider;
    }

    // Indicate that there are no other supported interfaces than IServerTrackedDeviceProvider
    if (pReturnCode) {
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    }

    return nullptr;
}
