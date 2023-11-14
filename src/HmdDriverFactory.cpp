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
    if (std::string_view{pInterfaceName} == vr::IServerTrackedDeviceProvider_Version) {
        return &sDeviceProvider;
    }

    // Indicate that there are no other supported interfaces than IServerTrackedDeviceProvider
    if (pReturnCode) {
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    }

    return nullptr;
}
