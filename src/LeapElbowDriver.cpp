#include "LeapElbowDriver.h"

#include "VrHand.h"

#include <span>
#include <numbers>
#include <ratio>

#include "VrUtils.h"
#include "VrMaths.h"

LeapElbowDriver::LeapElbowDriver(const std::shared_ptr<LeapDriverSettings>& settings, const eLeapHandType hand)
    : LeapTrackedDriver{vr::k_unTrackedDeviceIndexInvalid, settings},
      hand_type_{hand},
      pose_{kDefaultPose} {
}

auto LeapElbowDriver::Activate(const uint32_t object_id) -> vr::EVRInitError {
    id_ = object_id;

    try {
        const auto properties = VrDeviceProperties::FromDeviceId(id_);
        properties.Set(vr::Prop_ControllerType_String, "ultraleap_elbow");
        properties.Set(vr::Prop_ModelNumber_String, "elbow_tracker");
        properties.Set(vr::Prop_ManufacturerName_String, "Ultraleap");
        properties.Set(vr::Prop_RenderModelName_String, "{ultraleap}/rendermodels/ultraleap_elbow");
        properties.Set(vr::Prop_InputProfilePath_String, "{ultraleap}/input/ultraleap_elbow_profile.json");

        // Device capabilities.
        properties.Set(vr::Prop_DeviceIsWireless_Bool, true);
        properties.Set(vr::Prop_DeviceCanPowerOff_Bool, false);
        properties.Set(vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
        properties.Set(vr::Prop_Identifiable_Bool, false);

        // System input paths.
        input_proximity_ = properties.CreateBooleanInput("/proximity");
    } catch (const std::runtime_error& error) {
        LOG_INFO("Failed to initialize LeapElbowDriver: {}", error.what());
        return vr::VRInitError_Driver_Failed;
    }

    // Indicate that this driver has initialized successfully.
    active_ = true;
    return vr::VRInitError_None;
}

auto LeapElbowDriver::Deactivate() -> void {
    active_ = false;
    id_ = vr::k_unTrackedDeviceIndexInvalid;
}

auto LeapElbowDriver::EnterStandby() -> void {
}

auto LeapElbowDriver::GetComponent(const char* component_name_and_version) -> void* {
    if (std::string_view{component_name_and_version} == vr::ITrackedDeviceServerDriver_Version) {
        return dynamic_cast<ITrackedDeviceServerDriver*>(this);
    }

    return nullptr;
}

auto LeapElbowDriver::GetPose() -> vr::DriverPose_t {
    return pose_;
}

auto LeapElbowDriver::UpdateFromLeapFrame(const LEAP_TRACKING_EVENT* frame) -> void {
    // Check we've been activated before allowing updates from the tracking thread.
    if (!active_) {
        return;
    }

    // Work out the offset from now based on the frame timestamp.
    const auto time_offset = static_cast<double>(frame->info.timestamp - LeapGetNow()) * std::micro::num / std::micro::den;
    pose_.poseTimeOffset = time_offset;
    pose_.deviceIsConnected = true;

    // Find a hand that matches the correct chirality.
    const auto hands = std::span(frame->pHands, frame->nHands);
    if (const auto hand_iter = std::ranges::find_if(hands, [&](auto h) { return h.type == hand_type_; });
        hand_iter != std::end(hands)) {
        const auto leap_hand = *hand_iter;

        // Get the HMD position for the timestamp of the frame.
        pose_.result = vr::TrackingResult_Running_OK;
        if (const auto hmd_pose = HmdPose::Get(time_offset); hmd_pose.IsValid()) {
            pose_.poseIsValid = true;

            // Space transform from LeapC -> OpenVR Head Space.
            const auto tracker_head_offset = settings_->HmdTrackerOffset();
            const auto tracker_head_rotation = VrQuat::FromEulerAngles(-std::numbers::pi / 2.0, 0, std::numbers::pi);
            pose_.qDriverFromHeadRotation = VrQuat::Identity;
            VrVec3::Zero.CopyToArray(pose_.vecDriverFromHeadTranslation);

            // Space transform from LeapC -> OpenVR World Space.
            const auto tracker_world_orientation = hmd_pose.Orientation() * tracker_head_rotation;
            const auto tracker_world_position = hmd_pose.Position() + tracker_head_offset * hmd_pose.Orientation();
            pose_.qWorldFromDriverRotation = tracker_world_orientation;
            tracker_world_position.CopyToArray(pose_.vecWorldFromDriverTranslation);

            // Copy joint applying LeapC -> OpenVR scaling correction.
            (VrVec3{leap_hand.arm.prev_joint} * 0.001).CopyToArray(pose_.vecPosition);
            pose_.qRotation = VrQuat{leap_hand.arm.rotation};

            // Velocity
            // TODO: Implement this independantly from the palm.
            const auto hand_velocity = VrVec3{leap_hand.palm.velocity} * 0.001 * tracker_head_rotation;
            hand_velocity.CopyToArray(pose_.vecVelocity);
        } else {
            pose_.poseIsValid = false;
        }

        // Update proximity.
        input_proximity_.Update(true, time_offset);
    } else {
        pose_.result = vr::TrackingResult_Running_OutOfRange;
        pose_.poseIsValid = false;
        pose_.deviceIsConnected = true;

        input_proximity_.Update(false, time_offset);
    }

    // Update the pose for this virtual hand;
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, pose_, sizeof(pose_));
}

auto LeapElbowDriver::ProcessDebugRequestInputs(const DebugRequestPayload& request_payload, nlohmann::json& response) const
    -> void {
    // TODO: Handle any input related debug requests here.
}
