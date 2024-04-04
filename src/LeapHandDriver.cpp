#include "LeapHandDriver.h"

#include "VrHand.h"

#include <span>
#include <numbers>
#include <ratio>
#include <format>

#include "VrMaths.h"

constexpr vr::VRBoneTransform_t kInitialHand[31]{
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
    : LeapTrackedDriver{vr::k_unTrackedDeviceIndexInvalid, settings},
      hand_type_{hand},
      pose_{kDefaultPose},
      extended{settings_->ExtendedHandProfile()} {
}

auto LeapHandDriver::Activate(const uint32_t object_id) -> vr::EVRInitError {
    id_ = object_id;

    try {
        const auto properties = VrDeviceProperties::FromDeviceId(id_);

        properties.Set(vr::Prop_ManufacturerName_String, "Ultraleap");
        properties.Set(vr::Prop_ControllerHandSelectionPriority_Int32, 0);

        // Device capabilities.
        properties.Set(vr::Prop_DeviceIsWireless_Bool, true);
        properties.Set(vr::Prop_DeviceCanPowerOff_Bool, false);
        properties.Set(vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
        properties.Set(vr::Prop_Identifiable_Bool, false);

        // Setup the controller in normal or extended mode.
        if (!extended) {
            properties.Set(vr::Prop_ControllerType_String, "ultraleap_hand");
            properties.Set(vr::Prop_InputProfilePath_String, "{ultraleap}/input/ultraleap_hand_profile.json");
        } else {
            properties.Set(vr::Prop_ControllerType_String, "ultraleap_hand_extended");
            properties.Set(vr::Prop_InputProfilePath_String, "{ultraleap}/input/ultraleap_hand_extended_profile.json");
        }

        // Setup properties that are different per hand.
        if (hand_type_ == eLeapHandType_Left) {
            properties.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_LeftHand);
            properties.Set(vr::Prop_ModelNumber_String, "left_hand");
            properties.Set(vr::Prop_RenderModelName_String, "{ultraleap}/rendermodels/ultraleap_hand_left");
        } else {
            properties.Set(vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
            properties.Set(vr::Prop_ModelNumber_String, "right_hand");
            properties.Set(vr::Prop_RenderModelName_String, "{ultraleap}/rendermodels/ultraleap_hand_right");
        }

        // System input paths.
        input_system_menu_ = properties.CreateBooleanInput("/input/system/click");
        path_inputs_map_.insert({InputPath{InputSource::SYSTEM, InputComponent::CLICK}, &input_system_menu_});

        input_proximity_ = properties.CreateBooleanInput("/proximity");
        path_inputs_map_.insert({{InputSource::PROXIMITY, InputComponent::NONE}, &input_proximity_});

        // Hand specific input paths.
        input_pinch_ = properties.CreateAbsoluteScalarInput("/input/pinch/value", vr::VRScalarUnits_NormalizedOneSided);
        path_inputs_map_.insert({{InputSource::PINCH, InputComponent::VALUE}, &input_pinch_});

        input_grip_ = properties.CreateAbsoluteScalarInput("/input/grip/value", vr::VRScalarUnits_NormalizedOneSided);
        path_inputs_map_.insert({{InputSource::GRIP, InputComponent::VALUE}, &input_grip_});

        // Finger curl amounts.
        input_thumb_finger_ = properties.CreateAbsoluteScalarInput("/input/finger/thumb/value", vr::VRScalarUnits_NormalizedOneSided);
        input_index_finger_ = properties.CreateAbsoluteScalarInput("/input/finger/index/value", vr::VRScalarUnits_NormalizedOneSided);
        input_middle_finger_ = properties.CreateAbsoluteScalarInput("/input/finger/middle/value", vr::VRScalarUnits_NormalizedOneSided);
        input_ring_finger_ = properties.CreateAbsoluteScalarInput("/input/finger/ring/value", vr::VRScalarUnits_NormalizedOneSided);
        input_pinky_finger_ = properties.CreateAbsoluteScalarInput("/input/finger/pinky/value", vr::VRScalarUnits_NormalizedOneSided);
        path_inputs_map_.insert({{InputSource::THUMB_FINGER, InputComponent::VALUE}, &input_thumb_finger_});
        path_inputs_map_.insert({{InputSource::INDEX_FINGER, InputComponent::VALUE}, &input_index_finger_});
        path_inputs_map_.insert({{InputSource::MIDDLE_FINGER, InputComponent::VALUE}, &input_middle_finger_});
        path_inputs_map_.insert({{InputSource::RING_FINGER, InputComponent::VALUE}, &input_ring_finger_});
        path_inputs_map_.insert({{InputSource::PINKY_FINGER, InputComponent::VALUE}, &input_pinky_finger_});

        // Setup hand-skeleton to have full tracking with no supplied transforms.
        input_skeleton_ = properties.CreateSkeletonInput(
            hand_type_ == eLeapHandType_Left ? "/input/skeleton/left" : "/input/skeleton/right",
            hand_type_ == eLeapHandType_Left ? "/skeleton/hand/left" : "/skeleton/hand/right",
            "/pose/raw",
            vr::VRSkeletalTracking_Full
        );

        // Setup additional input paths if operating in extended mode.
        if (extended) {
            input_button_a_click_ = properties.CreateBooleanInput("/input/a/click");
            input_button_a_touch_ = properties.CreateBooleanInput("/input/a/touch");
            path_inputs_map_.insert({{InputSource::BUTTON_A, InputComponent::CLICK}, &input_button_a_click_});
            path_inputs_map_.insert({{InputSource::BUTTON_A, InputComponent::TOUCH}, &input_button_a_touch_});

            input_button_b_click_ = properties.CreateBooleanInput("/input/b/click");
            input_button_b_touch_ = properties.CreateBooleanInput("/input/b/touch");
            path_inputs_map_.insert({{InputSource::BUTTON_B, InputComponent::CLICK}, &input_button_b_click_});
            path_inputs_map_.insert({{InputSource::BUTTON_B, InputComponent::TOUCH}, &input_button_b_touch_});

            input_trigger_click_ = properties.CreateBooleanInput("/input/trigger/click");
            input_trigger_touch_ = properties.CreateBooleanInput("/input/trigger/touch");
            input_trigger_value_ = properties.CreateAbsoluteScalarInput("/input/trigger/value", vr::VRScalarUnits_NormalizedOneSided);
            path_inputs_map_.insert({{InputSource::TRIGGER, InputComponent::CLICK}, &input_trigger_click_});
            path_inputs_map_.insert({{InputSource::TRIGGER, InputComponent::TOUCH}, &input_trigger_touch_});
            path_inputs_map_.insert({{InputSource::TRIGGER, InputComponent::VALUE}, &input_trigger_value_});

            input_trackpad_force_ = properties.CreateAbsoluteScalarInput("/input/trackpad/force", vr::VRScalarUnits_NormalizedOneSided);
            input_trackpad_touch_ = properties.CreateBooleanInput("/input/trackpad/touch");
            input_trackpad_x_ = properties.CreateAbsoluteScalarInput("/input/trackpad/x", vr::VRScalarUnits_NormalizedTwoSided);
            input_trackpad_y_ = properties.CreateAbsoluteScalarInput("/input/trackpad/y", vr::VRScalarUnits_NormalizedTwoSided);
            path_inputs_map_.insert({{InputSource::TRACKPAD, InputComponent::FORCE}, &input_trackpad_force_});
            path_inputs_map_.insert({{InputSource::TRACKPAD, InputComponent::TOUCH}, &input_trackpad_touch_});
            path_inputs_map_.insert({{InputSource::TRACKPAD, InputComponent::X}, &input_trackpad_x_});
            path_inputs_map_.insert({{InputSource::TRACKPAD, InputComponent::Y}, &input_trackpad_y_});

            input_thumbstick_click_ = properties.CreateBooleanInput("/input/thumbstick/click");
            input_thumbstick_touch_ = properties.CreateBooleanInput("/input/thumbstick/touch");
            input_thumbstick_x_ = properties.CreateAbsoluteScalarInput("/input/thumbstick/x", vr::VRScalarUnits_NormalizedTwoSided);
            input_thumbstick_y_ = properties.CreateAbsoluteScalarInput("/input/thumbstick/y", vr::VRScalarUnits_NormalizedTwoSided);
            path_inputs_map_.insert({{InputSource::THUMBSTICK, InputComponent::CLICK}, &input_thumbstick_click_});
            path_inputs_map_.insert({{InputSource::THUMBSTICK, InputComponent::TOUCH}, &input_thumbstick_touch_});
            path_inputs_map_.insert({{InputSource::THUMBSTICK, InputComponent::X}, &input_thumbstick_x_});
            path_inputs_map_.insert({{InputSource::THUMBSTICK, InputComponent::Y}, &input_thumbstick_y_});
        }

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
        return dynamic_cast<ITrackedDeviceServerDriver*>(this);
    }

    return nullptr;
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
            (VrVec3{leap_hand.palm.position} * 0.001).CopyToArray(pose_.vecPosition);
            pose_.qRotation = VrQuat{leap_hand.palm.orientation};

            // Velocity
            // TODO: Check.
            const auto hand_velocity = VrVec3{leap_hand.palm.velocity} * 0.001 * tracker_head_rotation;
            hand_velocity.CopyToArray(pose_.vecVelocity);
        } else {
            pose_.poseIsValid = false;
        }

        // Update the skeletal and finger curl data as we alway drive direct hand data
        const auto hand = VrHand{leap_hand};
        input_skeleton_.Update(vr::VRSkeletalMotionRange_WithController, hand.GetBoneTransforms());
        input_skeleton_.Update(vr::VRSkeletalMotionRange_WithoutController, hand.GetBoneTransforms());
        input_thumb_finger_.Update(hand.GetThumbCurl(), time_offset);
        input_index_finger_.Update(hand.GetIndexFingerCurl(), time_offset);
        input_middle_finger_.Update(hand.GetMiddleFingerCurl(), time_offset);
        input_ring_finger_.Update(hand.GetRingFingerCurl(), time_offset);
        input_pinky_finger_.Update(hand.GetPinkyFingerCurl(), time_offset);

        // If external input only isn't set then also update the interaction based components of the profile
        if (!settings_->ExternalInputOnly()) {
            input_proximity_.Update(true, time_offset);
            input_pinch_.Update(hand.GetPinchStrength(), time_offset);
            input_grip_.Update(hand.GetGrabStrength(), time_offset);
            input_system_menu_.Update(VrHand::GetSystemMenuTriggered(hands), time_offset);
        }
    } else {
        pose_.result = vr::TrackingResult_Running_OutOfRange;
        pose_.poseIsValid = false;
        pose_.deviceIsConnected = true;

        // Mark all input components as zero'ed values if we are currently driving inputs.
        if (!settings_->ExternalInputOnly()) {
            input_system_menu_.Update(false, time_offset);
            input_proximity_.Update(false, time_offset);
            input_pinch_.Update(0.0, time_offset);
            input_grip_.Update(0.0, time_offset);
        }

        // Still update the finger curl values as we are always driving them alongside the skeletal data.
        input_index_finger_.Update(0.0, time_offset);
        input_middle_finger_.Update(0.0, time_offset);
        input_ring_finger_.Update(0.0, time_offset);
        input_pinky_finger_.Update(0.0, time_offset);
    }

    // Update the pose for this virtual hand;
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, pose_, sizeof(pose_));
}

auto LeapHandDriver::SetInitialBoneTransforms() -> void {
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithController, kInitialHand);
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithoutController, kInitialHand);
}

auto LeapHandDriver::ProcessDebugRequestInputs(const DebugRequestPayload& request_payload, nlohmann::json& response) const -> void {
    // Loop through all the received InputEvents and fire off updates to the corresponding inputs.
    for (const auto& [key, input_entry] : request_payload.inputs_) {
        // Skip input components which don't exist.
        if (!path_inputs_map_.contains(key)) {
            LOG_INFO("No mapping to an InputComponent exists for the key: {}", DebugRequestPayload::InputPathToString(key));
            continue;
        }
        const auto& prop_variant = path_inputs_map_.at(key);

        // Special case for joysticks as x and y are sent together. Beyond that ensure that the types we've parsed
        // from the debug payload correctly match our input types.
        if (std::holds_alternative<vr::HmdVector2_t>(input_entry.value_)) {
            // TODO: this will allow for joysticks in the future and can be ignored for now.
        } else if (std::holds_alternative<float>(input_entry.value_) && std::holds_alternative<VrScalarInputComponent*>(prop_variant)) {
            const auto& val = std::get<float>(input_entry.value_);
            auto* prop = std::get<VrScalarInputComponent*>(prop_variant);
            prop->Update(val, input_entry.time_offset_);
        } else if (std::holds_alternative<bool>(input_entry.value_) && std::holds_alternative<VrBooleanInputComponent*>(prop_variant)) {
            const auto& val = std::get<bool>(input_entry.value_);
            auto* prop = std::get<VrBooleanInputComponent*>(prop_variant);
            prop->Update(val, input_entry.time_offset_);
        } else {
            LOG_INFO(
                "Failed to process input for path: '{}'; Either the payload value didn't match the input type or path_inputs_map_ "
                "is missing an entry for the given key",
                input_entry.full_path_
            );
            response[response_warnings_key_] += std::format(
                "Failed to process input for path: '{}'; Either the payload value didn't match the input type or path_inputs_map_ "
                "is missing an entry for the given key",
                input_entry.full_path_
            );
        }
    }
}
