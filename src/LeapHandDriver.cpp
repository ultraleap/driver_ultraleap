#include "LeapHandDriver.h"

#include "OvrProperties.h"

LeapHandDriver::LeapHandDriver(eLeapHandType hand) : hand{hand} {
}

vr::EVRInitError LeapHandDriver::Activate(uint32_t unObjectId) {
    id = unObjectId;

    auto properties = OvrProperties::FromDeviceId(id);
    properties.Set(vr::Prop_ControllerType_String, "UltraleapHand");
    properties.Set(vr::Prop_ControllerHandSelectionPriority_Int32, 0);
    //properties.Set(vr::Prop_InputProfilePath_String, "TODO");
    properties.Set(vr::Prop_ManufacturerName_String, "Ultraleap");
    properties.Set(vr::Prop_DeviceProvidesBatteryStatus_Bool, false);

    // Setup properties that are different per hand
    if (hand == eLeapHandType_Left) {
        properties.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_LeftHand);
        properties.Set(vr::Prop_ModeLabel_String, "LeftHand");
        properties.Set(vr::Prop_NamedIconPathDeviceOff_String, "{ultraleap}/icons/left_hand_status_off.png");
        properties.Set(vr::Prop_NamedIconPathDeviceReady_String, "{ultraleap}/icons/left_hand_status_ready.png");
        properties.Set(vr::Prop_NamedIconPathDeviceStandby_String, "{ultraleap}/icons/left_hand_status_standby.png");
    } else {
        properties.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
        properties.Set(vr::Prop_ModeLabel_String, "RightHand");
        properties.Set(vr::Prop_NamedIconPathDeviceOff_String, "{ultraleap}/icons/right_hand_status_off.png");
        properties.Set(vr::Prop_NamedIconPathDeviceReady_String, "{ultraleap}/icons/right_hand_status_ready.png");
        properties.Set(vr::Prop_NamedIconPathDeviceStandby_String, "{ultraleap}/icons/right_hand_status_standby.png");
    }

    return vr::VRInitError_None;
}

void LeapHandDriver::Deactivate() {
    id = vr::k_unTrackedDeviceIndexInvalid;
}

void LeapHandDriver::EnterStandby() {
}

void* LeapHandDriver::GetComponent(const char* pchComponentNameAndVersion) {
    const auto componentNameAndVersion = std::string_view{pchComponentNameAndVersion};

    if (componentNameAndVersion == vr::ITrackedDeviceServerDriver_Version) {
        return dynamic_cast<vr::ITrackedDeviceServerDriver*>(this);
    }

    return nullptr;
}

void LeapHandDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {
    if (id != vr::k_unTrackedDeviceIndexInvalid) {
        // TODO: Implement any required debugging here, for now just clear the buffer.
        if (unResponseBufferSize > 0) {
            std::memset(pchResponseBuffer, 0, unResponseBufferSize);
        }
    }
}

vr::DriverPose_t LeapHandDriver::GetPose() {
    return vr::DriverPose_t();
}
