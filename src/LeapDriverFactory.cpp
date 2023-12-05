#include <openvr_driver.h>

#include "LeapDeviceProvider.h"

LeapDeviceProvider device_provider{};

OVR_EXPORT void* HmdDriverFactory(const char* interface_name, int* return_code) {
    if (std::string_view{interface_name} == vr::IServerTrackedDeviceProvider_Version) {
        return &device_provider;
    }

    // Indicate that there are no other supported interfaces than IServerTrackedDeviceProvider
    if (return_code) {
        *return_code = vr::VRInitError_Init_InterfaceNotFound;
    }

    return nullptr;
}
