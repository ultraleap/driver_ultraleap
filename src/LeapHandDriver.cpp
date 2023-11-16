#include "LeapHandDriver.h"

#include "OvrUtils.h"

#include <algorithm>
#include <span>

LeapHandDriver::LeapHandDriver(const eLeapHandType hand)
    : id{vr::k_unTrackedDeviceIndexInvalid},
      handType{hand},
      pose{kDefaultPose},
      inputPinch{0},
      inputGrip{0} {
}

auto LeapHandDriver::Activate(const uint32_t unObjectId) -> vr::EVRInitError {
    id = unObjectId;

    const auto properties = OvrPropertiesWrapper::FromDeviceId(id);
    properties.Set(vr::Prop_ControllerType_String, "ultraleap_hand");
    properties.Set(vr::Prop_ControllerHandSelectionPriority_Int32, 0);
    properties.Set(vr::Prop_ManufacturerName_String, "Ultraleap");
    properties.Set(vr::Prop_RenderModelName_String, "{ultraleap}/rendermodels/ultraleap_hand");

    // Device capabilities
    properties.Set(vr::Prop_DeviceIsWireless_Bool, true);
    properties.Set(vr::Prop_DeviceCanPowerOff_Bool, false);
    properties.Set(vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
    properties.Set(vr::Prop_Identifiable_Bool, false);

    // Setup properties that are different per hand.
    if (handType == eLeapHandType_Left) {
        properties.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_LeftHand);
        properties.Set(vr::Prop_ModelNumber_String, "left_hand");
    } else {
        properties.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
        properties.Set(vr::Prop_ModelNumber_String, "right_hand");
    }

    // Setup input profiles.
    properties.Set(vr::Prop_InputProfilePath_String, "{ultraleap}/input/hand_profile.json");
    vr::VRDriverInput()->CreateScalarComponent(
        properties,
        "/input/pinch/value",
        &inputPinch,
        vr::VRScalarType_Absolute,
        vr::VRScalarUnits_NormalizedOneSided
    );
    vr::VRDriverInput()->CreateScalarComponent(
        properties,
        "/input/grip/value",
        &inputGrip,
        vr::VRScalarType_Absolute,
        vr::VRScalarUnits_NormalizedOneSided
    );

    // Immediatly set default values for the inputs.
    vr::VRDriverInput()->UpdateScalarComponent(inputPinch, 0, 0);
    vr::VRDriverInput()->UpdateScalarComponent(inputGrip, 0, 0);

    return vr::VRInitError_None;
}

auto LeapHandDriver::Deactivate() -> void {
    id = vr::k_unTrackedDeviceIndexInvalid;
}

auto LeapHandDriver::EnterStandby() -> void {
}

auto LeapHandDriver::GetComponent(const char* pchComponentNameAndVersion) -> void* {
    if (std::string_view{pchComponentNameAndVersion} == vr::ITrackedDeviceServerDriver_Version) {
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
    return pose;
}

auto LeapHandDriver::UpdateHandFromFrame(const LEAP_TRACKING_EVENT* frame) -> void {
    // Work out the offset from now
    const auto timeOffset = static_cast<double>(frame->info.timestamp - LeapGetNow()) * std::micro::num / std::micro::den;
    pose.poseTimeOffset = timeOffset;
    pose.deviceIsConnected = true;

    // Find a hand that matches the correct chirality/
    const auto hands = std::span(frame->pHands, frame->nHands);
    if (const auto handIter = std::ranges::find_if(hands, [&](auto h) { return h.type == handType; }); handIter != hands.end()) {
        const auto leapHand = *handIter;

        // Get the HMD position for the timestamp of the frame.
        pose.result = vr::TrackingResult_Running_OK;
        const auto hmdPose = HmdPose::Get(timeOffset);
        if (hmdPose.IsValid()) {
            pose.poseIsValid = true;
            pose.qDriverFromHeadRotation = HmdQuaternion_Identity;
            pose.vecDriverFromHeadTranslation[0] = 0;
            pose.vecDriverFromHeadTranslation[1] = 0;
            pose.vecDriverFromHeadTranslation[2] = 0;

            // Space transform from LeapC -> OpenVR Space;
            pose.qWorldFromDriverRotation = hmdPose.Orientation() * HmdQuaternion_FromEulerAngles(0.0, M_PI / 2.0f, M_PI);
            auto [offsetPosition] = hmdPose.Position() + vr::HmdVector3_t{0, 0, -0.08f} * hmdPose.Orientation();
            std::ranges::copy(offsetPosition, pose.vecWorldFromDriverTranslation);

            pose.vecPosition[0] = 0.001f * leapHand.arm.next_joint.x;
            pose.vecPosition[1] = 0.001f * leapHand.arm.next_joint.y;
            pose.vecPosition[2] = 0.001f * leapHand.arm.next_joint.z;

            pose.qRotation = {
                leapHand.arm.rotation.w,
                leapHand.arm.rotation.x,
                leapHand.arm.rotation.y,
                leapHand.arm.rotation.z,
            };
        } else {
            pose.poseIsValid = false;
        }

        // Update input components.
        // TODO: Do hystersis etc.
        vr::VRDriverInput()->UpdateScalarComponent(inputPinch, leapHand.pinch_strength, timeOffset);
        vr::VRDriverInput()->UpdateScalarComponent(inputGrip, leapHand.grab_strength, timeOffset);
    } else {
        pose.result = vr::TrackingResult_Running_OutOfRange;
        pose.poseIsValid = false;
        pose.deviceIsConnected = true;
    }

    // Update the poser for this virtual hand;
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id, pose, sizeof(pose));
}
