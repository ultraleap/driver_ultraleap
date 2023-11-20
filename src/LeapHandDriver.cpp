#include "LeapHandDriver.h"

#include "VrUtils.h"
#include "VrLogging.h"

#include <span>

LeapHandDriver::LeapHandDriver(const eLeapHandType hand)
    : id_{vr::k_unTrackedDeviceIndexInvalid},
      hand_type_{hand},
      pose_{kDefaultPose},
      input_pinch_(),
      input_grip_(),
      input_skeleton_(),
      input_thumb_finger_(),
      input_index_finger_(),
      input_middle_finger_(),
      input_ring_finger_(),
      input_pinky_finger_() {
}

auto LeapHandDriver::Activate(const uint32_t object_id) -> vr::EVRInitError {
    id_ = object_id;

    try {
        const auto p = VrDeviceProperties::FromDeviceId(id_);
        p.Set(vr::Prop_ControllerType_String, "ultraleap_hand");
        p.Set(vr::Prop_ControllerHandSelectionPriority_Int32, 0);
        p.Set(vr::Prop_ManufacturerName_String, "Ultraleap");
        p.Set(vr::Prop_RenderModelName_String, "{ultraleap}/rendermodels/ultraleap_hand");

        // Device capabilities.
        p.Set(vr::Prop_DeviceIsWireless_Bool, true);
        p.Set(vr::Prop_DeviceCanPowerOff_Bool, false);
        p.Set(vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
        p.Set(vr::Prop_Identifiable_Bool, false);

        // Setup properties that are different per hand.
        if (hand_type_ == eLeapHandType_Left) {
            p.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_LeftHand);
            p.Set(vr::Prop_ModelNumber_String, "left_hand");
        } else {
            p.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
            p.Set(vr::Prop_ModelNumber_String, "right_hand");
        }

        // Setup input profiles.
        p.Set(vr::Prop_InputProfilePath_String, "{ultraleap}/input/hand_profile.json");

        input_pinch_ = p.CreateAbsoluteScalarInput("/input/pinch/value", vr::VRScalarUnits_NormalizedOneSided);
        input_grip_ = p.CreateAbsoluteScalarInput("/input/grip/value", vr::VRScalarUnits_NormalizedOneSided);

        // Setup hand-skeleton to have full tracking with no supplied transforms.
        input_skeleton_ = p.CreateSkeletonInput(
            hand_type_ == eLeapHandType_Left ? "/input/skeleton/left" : "/input/skeleton/right",
            hand_type_ == eLeapHandType_Left ? "/skeleton/hand/left" : "/skeleton/hand/right",
            "/pose/raw",
            vr::VRSkeletalTracking_Full
        );
        input_thumb_finger_ = p.CreateAbsoluteScalarInput("/input/finger/thumb", vr::VRScalarUnits_NormalizedOneSided);
        input_index_finger_ = p.CreateAbsoluteScalarInput("/input/finger/index", vr::VRScalarUnits_NormalizedOneSided);
        input_middle_finger_ = p.CreateAbsoluteScalarInput("/input/finger/middle", vr::VRScalarUnits_NormalizedOneSided);
        input_ring_finger_ = p.CreateAbsoluteScalarInput("/input/finger/ring", vr::VRScalarUnits_NormalizedOneSided);
        input_pinky_finger_ = p.CreateAbsoluteScalarInput("/input/finger/pinky", vr::VRScalarUnits_NormalizedOneSided);
    } catch (const std::runtime_error& error) {
        LOG_INFO("Failed to initialize LeapHandDriver: {}", error.what());
        return vr::VRInitError_Driver_Failed;
    }

    // Indicate that this driver has initialized successfully.
    return vr::VRInitError_None;
}

auto LeapHandDriver::Deactivate() -> void {
    id_ = vr::k_unTrackedDeviceIndexInvalid;
}

auto LeapHandDriver::EnterStandby() -> void {
}

auto LeapHandDriver::GetComponent(const char* component_name_and_version) -> void* {
    if (std::string_view{component_name_and_version} == vr::ITrackedDeviceServerDriver_Version) {
        return dynamic_cast<vr::ITrackedDeviceServerDriver*>(this);
    }

    return nullptr;
}

auto LeapHandDriver::DebugRequest(const char* request, char* response_buffer, const uint32_t response_buffer_size) -> void {
    if (id_ != vr::k_unTrackedDeviceIndexInvalid) {
        // TODO: Implement any required debugging here, for now just clear the buffer.
        if (response_buffer_size > 0) {
            std::memset(response_buffer, 0, response_buffer_size);
        }
    }
}

auto LeapHandDriver::GetPose() -> vr::DriverPose_t {
    return pose_;
}

auto LeapHandDriver::UpdateFromLeapFrame(const LEAP_TRACKING_EVENT* frame) -> void {
    // Work out the offset from now
    const auto time_offset = static_cast<double>(frame->info.timestamp - LeapGetNow()) * std::micro::num / std::micro::den;
    pose_.poseTimeOffset = time_offset;
    pose_.deviceIsConnected = true;

    // Find a hand that matches the correct chirality/
    const auto hands = std::span(frame->pHands, frame->nHands);
    if (const auto hand_iter = std::ranges::find_if(hands, [&](auto h) { return h.type == hand_type_; }); hand_iter != hands.end()) {
        const auto hand = *hand_iter;

        // Get the HMD position for the timestamp of the frame.
        pose_.result = vr::TrackingResult_Running_OK;
        if (const auto hmd_pose = HmdPose::Get(time_offset); hmd_pose.IsValid()) {
            pose_.poseIsValid = true;
            pose_.qDriverFromHeadRotation = HmdQuaternion_Identity;
            pose_.vecDriverFromHeadTranslation[0] = 0;
            pose_.vecDriverFromHeadTranslation[1] = 0;
            pose_.vecDriverFromHeadTranslation[2] = 0;

            // First work out forward and offset for the tracker depending on the tracking mode.
            auto trackerOrientation = tracking_mode_ == eLeapTrackingMode_HMD ? vr::HmdVector3_t{0.0f, M_PI / 2.0f, M_PI}
                                                                              : vr::HmdVector3_t{0.0f, 0.0f, 0.0f};
            auto trackerPositionOffset = tracking_mode_ == eLeapTrackingMode_HMD ? vr::HmdVector3_t{0.0f, 0.0f, -0.08f}
                                                                                 : vr::HmdVector3_t{0.0f, -0.25f, 0.1f};

            // Space transform from LeapC -> OpenVR Space;
            pose_.qWorldFromDriverRotation = hmd_pose.Orientation() * HmdQuaternion_FromEulerAngles(trackerOrientation.v[0], trackerOrientation.v[1], trackerOrientation.v[2]);
            auto [offsetPosition] = hmd_pose.Position() + trackerPositionOffset * hmd_pose.Orientation();
            std::ranges::copy(offsetPosition, pose_.vecWorldFromDriverTranslation);

            pose_.vecPosition[0] = 0.001f * hand.arm.next_joint.x;
            pose_.vecPosition[1] = 0.001f * hand.arm.next_joint.y;
            pose_.vecPosition[2] = 0.001f * hand.arm.next_joint.z;

            pose_.qRotation = {
                hand.arm.rotation.w,
                hand.arm.rotation.x,
                hand.arm.rotation.y,
                hand.arm.rotation.z,
            };
        } else {
            pose_.poseIsValid = false;
        }

        // Update input components.
        // TODO: Do hystersis etc.
        input_pinch_.Update(hand.pinch_strength, time_offset);
        input_grip_.Update(hand.grab_strength, time_offset);


        // Update finger curl amounts
        // TODO: Calculate correctly.
        input_thumb_finger_.Update(static_cast<float>(hand.digits[0].is_extended), time_offset);
        input_index_finger_.Update(static_cast<float>(hand.digits[0].is_extended), time_offset);
        input_middle_finger_.Update(static_cast<float>(hand.digits[0].is_extended), time_offset);
        input_ring_finger_.Update(static_cast<float>(hand.digits[0].is_extended), time_offset);
        input_pinky_finger_.Update(static_cast<float>(hand.digits[0].is_extended), time_offset);
    } else {
        pose_.result = vr::TrackingResult_Running_OutOfRange;
        pose_.poseIsValid = false;
        pose_.deviceIsConnected = true;
    }

    // Update the poser for this virtual hand;
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, pose_, sizeof(pose_));
}
