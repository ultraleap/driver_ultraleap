#include "LeapHandDriver.h"

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

constexpr double kOvrBoneLengths[31]{
    0.0,                  // Root,
    0.0,                  // Wrist,
    0.040406133979558945, // Thumb0,
    0.03251682594418526,  // Thumb1,
    0.030463997274637222, // Thumb2,
    0.0,                  // Thumb3,
    0.0737975463271141,   // IndexFinger0,
    0.043286554515361786, // IndexFinger1,
    0.028275206685066223, // IndexFinger2,
    0.022821536287665367, // IndexFinger3,
    0.0,                  // IndexFinger4,
    0.0708855390548706,   // MiddleFinger0,
    0.04310855641961098,  // MiddleFinger1,
    0.03326592966914177,  // MiddleFinger2,
    0.02589227259159088,  // MiddleFinger3,
    0.0,                  // MiddleFinger4,
    0.06597498059272766,  // RingFinger0,
    0.040331222116947174, // RingFinger1,
    0.02848881110548973,  // RingFinger2,
    0.02243015542626381,  // RingFinger3,
    0.0,                  // RingFinger4,
    0.06285565346479416,  // PinkyFinger0,
    0.029874319210648537, // PinkyFinger1,
    0.01797856017947197,  // PinkyFinger2,
    0.018017906695604324, // PinkyFinger3,
    0.0,                  // PinkyFinger4,
    0.0,                  // AuxThumb,
    0.0,                  // AuxIndexFinger,
    0.0,                  // AuxMiddleFinger,
    0.0,                  // AuxRingFinger,
    0.0,                  // AuxPinkyFinger,
};

LeapHandDriver::LeapHandDriver(const eLeapHandType hand)
    : id_{vr::k_unTrackedDeviceIndexInvalid},
      hand_type_{hand},
      pose_{kDefaultPose},
      hmd_tracker_offset_{GetHmdTrackerOffset()},
      desktop_tracker_offset_{GetDesktopTrackerOffset()} {
}

auto LeapHandDriver::Activate(const uint32_t object_id) -> vr::EVRInitError {
    id_ = object_id;

    try {
        const auto p = VrDeviceProperties::FromDeviceId(id_);

        p.Set(vr::Prop_ControllerType_String, "ultraleap_hand");
        p.Set(vr::Prop_ControllerHandSelectionPriority_Int32, 0);
        p.Set(vr::Prop_ManufacturerName_String, "Ultraleap");
        //p.Set(vr::Prop_RenderModelName_String, "{ultraleap}/rendermodels/ultraleap_hand");
        p.Set(vr::Prop_InputProfilePath_String, "{ultraleap}/input/ultraleap_hand_profile.json");

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

        // System input paths.
        input_system_ = p.CreateBooleanInput("/input/system/click");
        input_proximity_ = p.CreateBooleanInput("/proximity");

        // Hand specific infput paths.
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

auto LeapHandDriver::GetHmdTrackerOffset() -> vr::HmdVector3_t {
    const auto x = VrSettings::Get<float>("hmd_offset_x");
    const auto y = VrSettings::Get<float>("hmd_offset_y");
    const auto z = VrSettings::Get<float>("hmd_offset_z");
    return {x, y, z};
}

auto LeapHandDriver::GetDesktopTrackerOffset() -> vr::HmdVector3_t {
    const auto x = VrSettings::Get<float>("desktop_offset_x");
    const auto y = VrSettings::Get<float>("desktop_offset_y");
    const auto z = VrSettings::Get<float>("desktop_offset_z");
    return {x, y, z};
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
        hand_iter != hands.end()) {
        const auto hand = *hand_iter;

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
            (VrVec3{hand.palm.position} * 0.001).CopyToArray(pose_.vecPosition);
            pose_.qRotation = VrQuat{hand.palm.orientation};

            // Velocity
            // TODO: Check.
            const auto hand_velocity = VrVec3{hand.palm.velocity} * 0.001 * tracker_head_rotation;
            hand_velocity.CopyToArray(pose_.vecVelocity);

            // Transform the skeleton joints.
            UpdateBoneTransforms(hand, time_offset);
        } else {
            pose_.poseIsValid = false;
        }

        // Update input components.
        //input_proximity_.Update(true, time_offset);
        input_pinch_.Update(hand.pinch_strength, time_offset);
        input_grip_.Update(hand.grab_strength, time_offset);
    } else {
        pose_.result = vr::TrackingResult_Running_OutOfRange;
        pose_.poseIsValid = false;
        pose_.deviceIsConnected = true;

        input_proximity_.Update(false, time_offset);
        // TODO: Do we need to zero other input components here as well?
    }

    // TODO: Make this a proper classifier.
    if (frame->nHands == 2) {
        const auto& hand1 = frame->pHands[0];
        const auto& hand2 = frame->pHands[1];

        const float index_tip_distance = VrVec3{hand1.index.distal.next_joint - hand2.index.distal.next_joint}.Length();
        const float thumb_tip_distance = VrVec3{hand1.thumb.distal.next_joint - hand2.thumb.distal.next_joint}.Length();

        if (index_tip_distance < 20 && thumb_tip_distance < 20) {
            input_system_.Update(true);
        } else {
            input_system_.Update(false);
        }
    } else {
        input_system_.Update(false);
    }

    // Update the pose for this virtual hand;
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, pose_, sizeof(pose_));
}

auto LeapHandDriver::SetInitialBoneTransforms() -> void {
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithController, kInitialHand);
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithoutController, kInitialHand);
}

auto LeapHandDriver::UpdateBoneTransforms(const LEAP_HAND& hand, const double time_offset) -> void {
    const auto GetBoneRotation = [&](const VrHandSkeletonBone bone) -> VrQuat {
        if (bone == Root) {
            return VrQuat{hand.palm.orientation};
        }
        if (bone == Wrist) {
            return VrQuat{hand.palm.orientation}; // SteamVR expects the wrist to be hand static not arm static.
        }
        if (bone < IndexFinger0) {
            // Ignore the zero length metacarpal and the tip has the same rotation as the preceeding bone.
            return VrQuat{hand.thumb.bones[std::min(bone - Thumb0 + 1, 3)].rotation};
        }
        if (bone < AuxThumb) {
            // Tip has the same rotation as preceeding bone.
            const auto digit_index = (bone - IndexFinger0) / 5 + 1;
            const auto bone_index = (bone - IndexFinger0) % 5;
            return VrQuat{hand.digits[digit_index].bones[std::min(bone_index, 3)].rotation};
        }

        // TODO: AUX Bones
        return VrQuat::Identity;
    };

    const auto GetBonePosition = [&](const VrHandSkeletonBone bone) -> VrVec3 {
        if (bone == Root) {
            return VrVec3{hand.palm.position} * 0.001f;
        }
        if (bone == Wrist) {
            return VrVec3{hand.arm.next_joint} * 0.001f;
        }
        if (bone < IndexFinger0) {
            // Special case checking for the finger tip.
            if (bone == Thumb3) {
                return VrVec3{hand.thumb.distal.next_joint} * 0.001f;
            }
            // Ignore the zero length metacarpal.
            return VrVec3{hand.thumb.bones[bone - Thumb0 + 1].prev_joint} * 0.001f;
        }
        if (bone < AuxThumb) {
            // Again account for the tip specially.
            const auto digit_index = (bone - IndexFinger0) / 5 + 1;
            const auto bone_index = (bone - IndexFinger0) % 5;
            if (bone_index == 4) {
                return VrVec3{hand.digits[digit_index].bones[bone_index - 1].next_joint} * 0.001f;
            }
            return VrVec3{hand.digits[digit_index].bones[bone_index].prev_joint} * 0.001f;
        }

        // TODO: AUX Bones
        return VrVec3::Zero;
    };

    const auto ComputeDigitBoneTransforms =
        [&](VrVec3 parent_position, VrQuat parent_rotation, const size_t index_start, const size_t index_end) {
            for (auto index = index_start; index <= index_end; ++index) {

                const auto handed_orientation = VrQuat::FromEulerAngles(
                    -std::numbers::pi,
                    hand_type_ == eLeapHandType_Left ? std::numbers::pi / 2.0 : -std::numbers::pi / 2.0,
                    hand_type_ == eLeapHandType_Left ? 0 : std::numbers::pi
                );
                auto bone_position = GetBonePosition(static_cast<VrHandSkeletonBone>(index));
                auto bone_rotation = GetBoneRotation(static_cast<VrHandSkeletonBone>(index)) * handed_orientation;

                auto local_bone_position = (bone_position - parent_position) * parent_rotation;
                auto local_bone_rotation = parent_rotation.Inverse() * bone_rotation;

                // auto bone_length = local_bone_position.Length();
                // auto &bone_length_min_max = bone_lengths_[index - 1];
                // if (bone_length_min_max.first < 0.01) {
                //     bone_length_min_max.first = bone_length;
                //     bone_length_min_max.second = bone_length;
                // } else {
                //     bone_length_min_max.first = std::min(bone_length_min_max.first, bone_length);
                //     bone_length_min_max.second = std::max(bone_length_min_max.second, bone_length);
                // }
                // auto average_bone_length = std::lerp(bone_length_min_max.first, bone_length_min_max.second, 0.8f);
                //
                // local_bone_position /= bone_length;
                // local_bone_position *= average_bone_length;

                // Metacarpals need to handle the additional 90 degree rotation that OpenVR expects.
                if (index == index_start) {
                    const auto wrist_correction = VrQuat::FromEulerAngles(
                        0,
                        std::numbers::pi,
                        hand_type_ == eLeapHandType_Left ? -std::numbers::pi / 2.0 : std::numbers::pi / 2.0
                    );
                    local_bone_position = local_bone_position * wrist_correction;
                    local_bone_rotation = wrist_correction.Inverse() * local_bone_rotation;
                }

                bones_transforms_[index] = {local_bone_position,local_bone_rotation};

                parent_position = bone_position;
                parent_rotation = bone_rotation;
            }
        };

    const auto ComputeFingerCurl = [&](const LEAP_DIGIT& finger) -> float {
        const auto bone_directions = std::ranges::views::transform(finger.bones, [](const LEAP_BONE& bone) -> glm::dvec3 {
            return (bone.next_joint - bone.prev_joint).Normalised();
        });
        const auto total_finger_angles = glm::acos(glm::dot(bone_directions[0], bone_directions[1])) // Proximal
                                       + glm::acos(glm::dot(bone_directions[1], bone_directions[2])) // Intermediate
                                       + glm::acos(glm::dot(bone_directions[2], bone_directions[3]));
        return static_cast<float>(glm::clamp(total_finger_angles / (std::numbers::pi * 2.0), 0.0, 1.0));
    };

    const auto root_position = GetBonePosition(Root);
    const auto root_rotation = GetBoneRotation(Root);
    const auto wrist_position = GetBonePosition(Wrist);
    const auto wrist_rotation = GetBoneRotation(Wrist);

    bones_transforms_[Root] = vr::VRBoneTransform_t{VrVec3::Zero, VrQuat::Identity};
    if (hand_type_ == eLeapHandType_Left) {
        bones_transforms_[Wrist] = vr::VRBoneTransform_t{
            (wrist_position - root_position) * root_rotation,
            VrQuat::FromEulerAngles(0.0, std::numbers::pi, -std::numbers::pi / 2.0f) * (root_rotation.Inverse() * wrist_rotation),
        };
    } else {
        bones_transforms_[Wrist] = vr::VRBoneTransform_t{
            (wrist_position - root_position) * root_rotation,
            VrQuat::FromEulerAngles(0.0, std::numbers::pi, std::numbers::pi / 2.0f) * (root_rotation.Inverse() * wrist_rotation),
        };
    }

    // Compute the bone digits.
    ComputeDigitBoneTransforms(wrist_position, wrist_rotation, Thumb0, Thumb3);
    ComputeDigitBoneTransforms(wrist_position, wrist_rotation, IndexFinger0, IndexFinger4);
    ComputeDigitBoneTransforms(wrist_position, wrist_rotation, MiddleFinger0, MiddleFinger4);
    ComputeDigitBoneTransforms(wrist_position, wrist_rotation, RingFinger0, RingFinger4);
    ComputeDigitBoneTransforms(wrist_position, wrist_rotation, PinkyFinger0, PinkyFinger4);

    // Compute the scalar finger curls.
    input_index_finger_.Update(ComputeFingerCurl(hand.index), time_offset);
    input_middle_finger_.Update(ComputeFingerCurl(hand.middle), time_offset);
    input_ring_finger_.Update(ComputeFingerCurl(hand.ring), time_offset);
    input_pinky_finger_.Update(ComputeFingerCurl(hand.pinky), time_offset);

    // Calculate the AUX bones.
    // Documented at: https://github.com/ValveSoftware/openvr/wiki/Hand-Skeleton#auxiliary-bones
    bones_transforms_[AuxThumb] = {
        (GetBonePosition(Thumb2) - root_position) * root_rotation,
        root_rotation.Inverse() * GetBoneRotation(Thumb2)
    };
    bones_transforms_[AuxIndexFinger] = {
        (GetBonePosition(IndexFinger3) - root_position) * root_rotation,
        root_rotation.Inverse() * GetBoneRotation(IndexFinger3)
    };
    bones_transforms_[AuxMiddleFinger] = {
        (GetBonePosition(MiddleFinger3) - root_position) * root_rotation,
        root_rotation.Inverse() * GetBoneRotation(MiddleFinger3)
    };
    bones_transforms_[AuxRingFinger] = {
        (GetBonePosition(RingFinger3) - root_position) * root_rotation,
        root_rotation.Inverse() * GetBoneRotation(RingFinger3)
    };
    bones_transforms_[AuxPinkyFinger] = {
        (GetBonePosition(PinkyFinger3) - root_position) * root_rotation,
        root_rotation.Inverse() * GetBoneRotation(PinkyFinger3)
    };

    // // Debug all the transforms as euler angles
    // static bool logged = false;
    // if (!logged) {
    //     for (auto i = 0; i < 31; ++i) {
    //         const auto& p = bones_transforms_[i].position;
    //         const auto& o = bones_transforms_[i].orientation;
    //         LOG_INFO("{{ {{ {{ {1:.6f}, {1:.6f}, {1:.6f}, 1.000000f }} }}, {{ {1:.6f}, {1:.6f}, {1:.6f}, {1:.6f} }} }},", p.v[0],
    //         p.v[1], p.v[2], o.w, o.x, o.y, o.z);
    //     }
    //     logged = true;
    // }

    // Update the skeletons.
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithController, bones_transforms_);
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithoutController, bones_transforms_);
}