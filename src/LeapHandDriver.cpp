#include "LeapHandDriver.h"

#include "OvrUtils.h"

LeapHandDriver::LeapHandDriver(const eLeapHandType hand) : id{vr::k_unTrackedDeviceIndexInvalid}, hand{hand} {
}

auto LeapHandDriver::Activate(const uint32_t unObjectId) -> vr::EVRInitError {
    id = unObjectId;

    auto properties = OvrProperties::FromDeviceId(id);
    properties.Set(vr::Prop_ControllerType_String, "UltraleapHand");
    properties.Set(vr::Prop_ControllerHandSelectionPriority_Int32, 0);
    properties.Set(vr::Prop_InputProfilePath_String, "{ultraleap}/input/hand_profile.json");
    properties.Set(vr::Prop_ManufacturerName_String, "Ultraleap");
    properties.Set(vr::Prop_DeviceProvidesBatteryStatus_Bool, false);

    // Setup properties that are different per hand
    if (hand == eLeapHandType_Left) {
        properties.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_LeftHand);
        properties.Set(vr::Prop_ModelNumber_String, "LeftHand");
    } else {
        properties.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
        properties.Set(vr::Prop_ModelNumber_String, "RightHand");
    }

    return vr::VRInitError_None;
}

auto LeapHandDriver::Deactivate() -> void {
    id = vr::k_unTrackedDeviceIndexInvalid;
}

auto LeapHandDriver::EnterStandby() -> void {
}

auto LeapHandDriver::GetComponent(const char* pchComponentNameAndVersion) -> void* {
    const auto componentNameAndVersion = std::string_view{pchComponentNameAndVersion};

    if (componentNameAndVersion == vr::ITrackedDeviceServerDriver_Version) {
        return dynamic_cast<vr::ITrackedDeviceServerDriver*>(this);
    }

    return nullptr;
}

auto LeapHandDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, const uint32_t unResponseBufferSize) -> void {
    if (id != vr::k_unTrackedDeviceIndexInvalid) {
        // TODO: Implement any required debugging here, for now just clear the buffer.
        if (unResponseBufferSize > 0) {
            std::memset(pchResponseBuffer, 0, unResponseBufferSize);
        }
    }
}

auto LeapHandDriver::GetPose() -> vr::DriverPose_t {
    return {};
}
