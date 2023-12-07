#include "LeapHandDriver.h"

#include "VrHand.h"

#include <span>
#include <numbers>
#include <ratio>

#include "VrUtils.h"
#include "VrLogging.h"
#include "VrMaths.h"

#include <numeric>
#include <ranges>
#include <thread>

const vr::VRBoneTransform_t kInitialHand[31]{
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{0.006954, 0.006954, 0.006954, 1.000000f}}, {0.006954, 0.006954, 0.006954, 0.006954}},
    {{{0.030358, 0.030358, 0.030358, 1.000000f}}, {0.030358, 0.030358, 0.030358, 0.030358}},
    {{{-0.000000, -0.000000, -0.000000, 1.000000f}}, {-0.000000, -0.000000, -0.000000, -0.000000}},
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{-0.000000, -0.000000, -0.000000, 1.000000f}}, {-0.000000, -0.000000, -0.000000, -0.000000}},
    {{{0.022294, 0.022294, 0.022294, 1.000000f}}, {0.022294, 0.022294, 0.022294, 0.022294}},
    {{{-0.000000, -0.000000, -0.000000, 1.000000f}}, {-0.000000, -0.000000, -0.000000, -0.000000}},
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{-0.000000, -0.000000, -0.000000, 1.000000f}}, {-0.000000, -0.000000, -0.000000, -0.000000}},
    {{{0.007883, 0.007883, 0.007883, 1.000000f}}, {0.007883, 0.007883, 0.007883, 0.007883}},
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{-0.000000, -0.000000, -0.000000, 1.000000f}}, {-0.000000, -0.000000, -0.000000, -0.000000}},
    {{{-0.000000, -0.000000, -0.000000, 1.000000f}}, {-0.000000, -0.000000, -0.000000, -0.000000}},
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{-0.002032, -0.002032, -0.002032, 1.000000f}}, {-0.002032, -0.002032, -0.002032, -0.002032}},
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{-0.000000, -0.000000, -0.000000, 1.000000f}}, {-0.000000, -0.000000, -0.000000, -0.000000}},
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{-0.012044, -0.012044, -0.012044, 1.000000f}}, {-0.012044, -0.012044, -0.012044, -0.012044}},
    {{{-0.000000, -0.000000, -0.000000, 1.000000f}}, {-0.000000, -0.000000, -0.000000, -0.000000}},
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{-0.000000, -0.000000, -0.000000, 1.000000f}}, {-0.000000, -0.000000, -0.000000, -0.000000}},
    {{{0.000000, 0.000000, 0.000000, 1.000000f}}, {0.000000, 0.000000, 0.000000, 0.000000}},
    {{{-0.020693, -0.020693, -0.020693, 1.000000f}}, {-0.020693, -0.020693, -0.020693, -0.020693}},
    {{{-0.015647, -0.015647, -0.015647, 1.000000f}}, {-0.015647, -0.015647, -0.015647, -0.015647}},
    {{{-0.011907, -0.011907, -0.011907, 1.000000f}}, {-0.011907, -0.011907, -0.011907, -0.011907}},
    {{{-0.013752, -0.013752, -0.013752, 1.000000f}}, {-0.013752, -0.013752, -0.013752, -0.013752}},
    {{{-0.012825, -0.012825, -0.012825, 1.000000f}}, {-0.012825, -0.012825, -0.012825, -0.012825}},
};

LeapHandDriver::LeapHandDriver(const std::shared_ptr<LeapDriverSettings>& settings, const eLeapHandType hand)
    : id_{vr::k_unTrackedDeviceIndexInvalid},
      settings_{settings},
      hand_type_{hand},
      pose_{kDefaultPose} {
}

auto LeapHandDriver::Activate(const uint32_t object_id) -> vr::EVRInitError {
    id_ = object_id;

    try {
        const auto properties = VrDeviceProperties::FromDeviceId(id_);
        properties.Set(vr::Prop_ControllerType_String, "ultraleap_hand");
        properties.Set(vr::Prop_ControllerHandSelectionPriority_Int32, 0);
        properties.Set(vr::Prop_ManufacturerName_String, "Ultraleap");
        properties.Set(vr::Prop_RenderModelName_String, "{ultraleap}/rendermodels/ultraleap_hand");
        properties.Set(vr::Prop_InputProfilePath_String, "{ultraleap}/input/ultraleap_hand_profile.json");

        // Device capabilities.
        properties.Set(vr::Prop_DeviceIsWireless_Bool, true);
        properties.Set(vr::Prop_DeviceCanPowerOff_Bool, false);
        properties.Set(vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
        properties.Set(vr::Prop_Identifiable_Bool, false);

        // Setup properties that are different per hand.
        if (hand_type_ == eLeapHandType_Left) {
            properties.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_LeftHand);
            properties.Set(vr::Prop_ModelNumber_String, "left_hand");
        } else {
            properties.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
            properties.Set(vr::Prop_ModelNumber_String, "right_hand");
        }

        // System input paths.
        input_system_menu_ = properties.CreateBooleanInput("/input/system/click");
        input_proximity_ = properties.CreateBooleanInput("/proximity");

        // Hand specific infput paths.
        input_pinch_ = properties.CreateAbsoluteScalarInput("/input/pinch/value", vr::VRScalarUnits_NormalizedOneSided);
        input_grip_ = properties.CreateAbsoluteScalarInput("/input/grip/value", vr::VRScalarUnits_NormalizedOneSided);

        // Setup hand-skeleton to have full tracking with no supplied transforms.
        input_skeleton_ = properties.CreateSkeletonInput(
            hand_type_ == eLeapHandType_Left ? "/input/skeleton/left" : "/input/skeleton/right",
            hand_type_ == eLeapHandType_Left ? "/skeleton/hand/left" : "/skeleton/hand/right",
            "/pose/raw",
            vr::VRSkeletalTracking_Full
        );
        input_index_finger_ = properties.CreateAbsoluteScalarInput("/input/finger/index", vr::VRScalarUnits_NormalizedOneSided);
        input_middle_finger_ = properties.CreateAbsoluteScalarInput("/input/finger/middle", vr::VRScalarUnits_NormalizedOneSided);
        input_ring_finger_ = properties.CreateAbsoluteScalarInput("/input/finger/ring", vr::VRScalarUnits_NormalizedOneSided);
        input_pinky_finger_ = properties.CreateAbsoluteScalarInput("/input/finger/pinky", vr::VRScalarUnits_NormalizedOneSided);

        // Send Skeleton data straight away.
        SetInitialBoneTransforms();
    } catch (const std::runtime_error& error) {
        LOG_INFO("Failed to initialize LeapHandDriver: {}", error.what());
        return vr::VRInitError_Driver_Failed;
    }

    // Indicate that this driver has initialized successfully.
    active_ = true;
    return vr::VRInitError_None;
}

auto LeapHandDriver::Deactivate() -> void {
    active_ = false;
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
    // Check we've been activated before allowing updates from the tracking thread.
    if (!active_) {
        return;
    }

    // Work out the offset from now
    const auto time_offset = static_cast<double>(frame->info.timestamp - LeapGetNow()) * std::micro::num / std::micro::den;
    pose_.poseTimeOffset = time_offset;
    pose_.deviceIsConnected = true;

    // Find a hand that matches the correct chirality/
    const auto hands = std::span(frame->pHands, frame->nHands);
    if (const auto hand_iter = std::ranges::find_if(hands, [&](auto h) { return h.type == hand_type_; });
        hand_iter != std::end(hands)) {
        const auto leap_hand = *hand_iter;

        // Get the HMD position for the timestamp of the frame.
        pose_.result = vr::TrackingResult_Running_OK;
        if (const auto hmd_pose = HmdPose::Get(time_offset); hmd_pose.IsValid()) {
            pose_.poseIsValid = true;

            // Space transform from LeapC -> OpenVR Head Space.
            const auto tracker_head_offset = VrVec3{0, 0, -0.08f};
            const auto tracker_head_rotation = VrQuat::FromEulerAngles(-std::numbers::pi / 2.0, 0, std::numbers::pi);
            pose_.qDriverFromHeadRotation = VrQuat::Identity;
            VrVec3::Zero.CopyToArray(pose_.vecDriverFromHeadTranslation);

            // Space transform from LeapC -> OpenVR World Space.
            const auto tracker_world_orientation = hmd_pose.Orientation() * tracker_head_rotation;
            const auto tracker_world_position = hmd_pose.Position() + tracker_head_offset * hmd_pose.Orientation();
            pose_.qWorldFromDriverRotation = tracker_world_orientation;
            tracker_world_position.CopyToArray(pose_.vecWorldFromDriverTranslation);

            // Copy joint applying LeapC -> OpenVR scaling correction.
            (VrVec3{leap_hand.palm.position} * 0.001).CopyToArray(pose_.vecPosition);
            pose_.qRotation = VrQuat{leap_hand.palm.orientation};

            // Velocity
            // TODO: Check.
            const auto hand_velocity = VrVec3{leap_hand.palm.velocity} * 0.001 * tracker_head_rotation;
            hand_velocity.CopyToArray(pose_.vecVelocity);
        } else {
            pose_.poseIsValid = false;
        }

        // Parse this hand into a VrHand
        const auto hand = VrHand{leap_hand};

        // Update input components and skeleton
        input_proximity_.Update(true, time_offset);

        // Update the Skeleton and finger curl.
        input_skeleton_.Update(vr::VRSkeletalMotionRange_WithController, hand.GetBoneTransforms());
        input_skeleton_.Update(vr::VRSkeletalMotionRange_WithoutController, hand.GetBoneTransforms());
        input_index_finger_.Update(hand.GetIndexFingerCurl(), time_offset);
        input_middle_finger_.Update(hand.GetMiddleFingerCurl(), time_offset);
        input_ring_finger_.Update(hand.GetRingFingerCurl(), time_offset);
        input_pinky_finger_.Update(hand.GetPinkyFingerCurl(), time_offset);

        input_pinch_.Update(hand.GetPinchStrength(), time_offset);
        input_grip_.Update(hand.GetGrabStrength(), time_offset);

        // TODO: Make this a proper classifier.
        if (frame->nHands == 2) {
            const auto& hand1 = frame->pHands[0];
            const auto& hand2 = frame->pHands[1];

            const double index_tip_distance = VrVec3{hand1.index.distal.next_joint - hand2.index.distal.next_joint}.Length();
            const double thumb_tip_distance = VrVec3{hand1.thumb.distal.next_joint - hand2.thumb.distal.next_joint}.Length();

            if (index_tip_distance < 20 && thumb_tip_distance < 20) {
                input_system_menu_.Update(true);
            } else {
                input_system_menu_.Update(false);
            }
        } else {
            input_system_menu_.Update(false);
        }

    } else {
        pose_.result = vr::TrackingResult_Running_OutOfRange;
        pose_.poseIsValid = false;
        pose_.deviceIsConnected = true;

        // Mark all input components as zero'ed values.
        input_system_menu_.Update(false);
        input_proximity_.Update(false, time_offset);

        input_index_finger_.Update(0.0, time_offset);
        input_middle_finger_.Update(0.0, time_offset);
        input_ring_finger_.Update(0.0, time_offset);
        input_pinky_finger_.Update(0.0, time_offset);

        input_pinch_.Update(0.0, time_offset);
        input_grip_.Update(0.0, time_offset);
    }

    // Update the pose for this virtual hand;
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, pose_, sizeof(pose_));
}

auto LeapHandDriver::SetInitialBoneTransforms() -> void {
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithController, kInitialHand);
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithoutController, kInitialHand);
}
