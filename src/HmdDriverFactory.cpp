#include <openvr_driver.h>

#include "LeapDeviceProvider.h"

#if defined(_WIN32)
#define OVR_EXPORT extern "C" __declspec(dllexport)
#define OVR_IMPORT extern "C" __declspec(dllimport)
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define OVR_EXPORT extern "C" __attribute__((visibility("default")))
#define OVR_IMPORT extern "C"
#else
#error "Unsupported Platform"
#endif

LeapDeviceProvider sDeviceProvider{};

OVR_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    const auto interfaceName = std::string_view{pInterfaceName};

    if (interfaceName == vr::IServerTrackedDeviceProvider_Version) {
        return &sDeviceProvider;
    }

    // Indicate if there interface is not one which is supported
    if (pReturnCode) {
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    }

    return nullptr;
}
