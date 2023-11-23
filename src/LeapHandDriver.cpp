#include "LeapHandDriver.h"

#include <span>
#include <numbers>
#include <ratio>

#include "VrUtils.h"
#include "VrLogging.h"
#include "VrMaths.h"

#include <thread>

const vr::VRBoneTransform_t kOpenHandGesture[31]{
    {{{0.000000f, 0.000000f, 0.000000f, 1.000000f}}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}},
    {{{-0.034038f, 0.026503f, 0.174722f, 1.000000f}}, {-0.055147f, -0.078608f, -0.920279f, 0.379296f}},
    {{{-0.012083f, 0.028070f, 0.025050f, 1.000000f}}, {0.464112f, 0.567418f, 0.272106f, 0.623374f}},
    {{{0.040406f, 0.000000f, -0.000000f, 1.000000f}}, {0.994838f, 0.082939f, 0.019454f, 0.055130f}},
    {{{0.032517f, 0.000000f, 0.000000f, 1.000000f}}, {0.974793f, -0.003213f, 0.021867f, -0.222015f}},
    {{{0.030464f, -0.000000f, -0.000000f, 1.000000f}}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}},
    {{{0.000632f, 0.026866f, 0.015002f, 1.000000f}}, {0.644251f, 0.421979f, -0.478202f, 0.422133f}},
    {{{0.074204f, -0.005002f, 0.000234f, 1.000000f}}, {0.995332f, 0.007007f, -0.039124f, 0.087949f}},
    {{{0.043930f, -0.000000f, -0.000000f, 1.000000f}}, {0.997891f, 0.045808f, 0.002142f, -0.045943f}},
    {{{0.028695f, 0.000000f, 0.000000f, 1.000000f}}, {0.999649f, 0.001850f, -0.022782f, -0.013409f}},
    {{{0.022821f, 0.000000f, -0.000000f, 1.000000f}}, {1.000000f, -0.000000f, 0.000000f, -0.000000f}},
    {{{0.002177f, 0.007120f, 0.016319f, 1.000000f}}, {0.546723f, 0.541276f, -0.442520f, 0.460749f}},
    {{{0.070953f, 0.000779f, 0.000997f, 1.000000f}}, {0.980294f, -0.167261f, -0.078959f, 0.069368f}},
    {{{0.043108f, 0.000000f, 0.000000f, 1.000000f}}, {0.997947f, 0.018493f, 0.013192f, 0.059886f}},
    {{{0.033266f, 0.000000f, 0.000000f, 1.000000f}}, {0.997394f, -0.003328f, -0.028225f, -0.066315f}},
    {{{0.025892f, -0.000000f, 0.000000f, 1.000000f}}, {0.999195f, -0.000000f, 0.000000f, 0.040126f}},
    {{{0.000513f, -0.006545f, 0.016348f, 1.000000f}}, {0.516692f, 0.550143f, -0.495548f, 0.429888f}},
    {{{0.065876f, 0.001786f, 0.000693f, 1.000000f}}, {0.990420f, -0.058696f, -0.101820f, 0.072495f}},
    {{{0.040697f, 0.000000f, 0.000000f, 1.000000f}}, {0.999545f, -0.002240f, 0.000004f, 0.030081f}},
    {{{0.028747f, -0.000000f, -0.000000f, 1.000000f}}, {0.999102f, -0.000721f, -0.012693f, 0.040420f}},
    {{{0.022430f, -0.000000f, 0.000000f, 1.000000f}}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},
    {{{-0.002478f, -0.018981f, 0.015214f, 1.000000f}}, {0.526918f, 0.523940f, -0.584025f, 0.326740f}},
    {{{0.062878f, 0.002844f, 0.000332f, 1.000000f}}, {0.986609f, -0.059615f, -0.135163f, 0.069132f}},
    {{{0.030220f, 0.000000f, 0.000000f, 1.000000f}}, {0.994317f, 0.001896f, -0.000132f, 0.106446f}},
    {{{0.018187f, 0.000000f, 0.000000f, 1.000000f}}, {0.995931f, -0.002010f, -0.052079f, -0.073526f}},
    {{{0.018018f, 0.000000f, -0.000000f, 1.000000f}}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},
    {{{-0.006059f, 0.056285f, 0.060064f, 1.000000f}}, {0.737238f, 0.202745f, 0.594267f, 0.249441f}},
    {{{-0.040416f, -0.043018f, 0.019345f, 1.000000f}}, {-0.290331f, 0.623527f, -0.663809f, -0.293734f}},
    {{{-0.039354f, -0.075674f, 0.047048f, 1.000000f}}, {-0.187047f, 0.678062f, -0.659285f, -0.265683f}},
    {{{-0.038340f, -0.090987f, 0.082579f, 1.000000f}}, {-0.183037f, 0.736793f, -0.634757f, -0.143936f}},
    {{{-0.031806f, -0.087214f, 0.121015f, 1.000000f}}, {-0.003659f, 0.758407f, -0.639342f, -0.126678f}}
};

LeapHandDriver::LeapHandDriver(const eLeapHandType hand)
    : id_{vr::k_unTrackedDeviceIndexInvalid},
      hand_type_{hand},
      pose_{kDefaultPose} {
}

auto LeapHandDriver::Activate(const uint32_t object_id) -> vr::EVRInitError {
    id_ = object_id;

    try {
        const auto p = VrDeviceProperties::FromDeviceId(id_);

        // p.Set(vr::Prop_ControllerType_String, "ultraleap_hand");
        p.Set(vr::Prop_ControllerHandSelectionPriority_Int32, 0);
        p.Set(vr::Prop_ManufacturerName_String, "Ultraleap");
        // p.Set(vr::Prop_RenderModelName_String, "{ultraleap}/rendermodels/ultraleap_hand");
        // p.Set(vr::Prop_InputProfilePath_String, "{ultraleap}/input/ultraleap_hand_profile.json");

        // Emulate knuckles for now.
        p.Set(vr::Prop_InputProfilePath_String, "{indexcontroller}/input/index_controller_profile.json");
        p.Set(vr::Prop_ControllerType_String, "knuckles");

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

            // Transform the skeleton joints.
            UpdateBoneTransforms(hand);
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
        input_index_finger_.Update(static_cast<float>(hand.digits[1].is_extended), time_offset);
        input_middle_finger_.Update(static_cast<float>(hand.digits[2].is_extended), time_offset);
        input_ring_finger_.Update(static_cast<float>(hand.digits[3].is_extended), time_offset);
        input_pinky_finger_.Update(static_cast<float>(hand.digits[4].is_extended), time_offset);
    } else {
        pose_.result = vr::TrackingResult_Running_OutOfRange;
        pose_.poseIsValid = false;
        pose_.deviceIsConnected = true;
    }

    // Update the pose for this virtual hand;
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, pose_, sizeof(pose_));
}

auto LeapHandDriver::SetInitialBoneTransforms() -> void {
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithController, kOpenHandGesture);
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithoutController, kOpenHandGesture);
}

auto LeapHandDriver::UpdateBoneTransforms(const LEAP_HAND& hand) -> void {
    const auto GetBoneRotation = [&](const VrHandSkeletonBone bone) -> VrQuat {
        if (bone == Root) {
            return VrQuat{hand.palm.orientation};
        }
        if (bone == Wrist) {
            return VrQuat{hand.arm.rotation};
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
                auto bone_position = GetBonePosition(static_cast<VrHandSkeletonBone>(index));
                auto bone_rotation = GetBoneRotation(static_cast<VrHandSkeletonBone>(index));

                bones_transforms_[index] = vr::VRBoneTransform_t{
                    (bone_position - parent_position) * parent_rotation, // Local bone position.
                    (parent_rotation.Inverse() * bone_rotation),         // Local bone rotation
                };

                parent_position = bone_position;
                parent_rotation = bone_rotation;
            }
        };

    const auto root_position = GetBonePosition(Root);
    const auto root_rotation = GetBoneRotation(Root);
    const auto wrist_position = GetBonePosition(Wrist);
    const auto wrist_rotation = GetBoneRotation(Wrist);
    bones_transforms_[Root] = vr::VRBoneTransform_t{VrVec3::Zero, VrQuat::Identity};
    bones_transforms_[Wrist] = vr::VRBoneTransform_t{
        (wrist_position - root_position) * root_rotation,
        root_rotation.Inverse() * wrist_rotation
    };

    // Computer the bone digits.
    ComputeDigitBoneTransforms(wrist_position, wrist_rotation, Thumb0, Thumb3);
    ComputeDigitBoneTransforms(wrist_position, wrist_rotation, IndexFinger0, IndexFinger4);
    ComputeDigitBoneTransforms(wrist_position, wrist_rotation, MiddleFinger0, MiddleFinger4);
    ComputeDigitBoneTransforms(wrist_position, wrist_rotation, RingFinger0, RingFinger4);
    ComputeDigitBoneTransforms(wrist_position, wrist_rotation, PinkyFinger0, PinkyFinger4);

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

    // Debug all the transforms as euler angles
    std::array<std::tuple<int, int, int>, 31> bone_angles_{};
    for (auto i = 0; i < 31; ++i) {
        auto [pitch, yaw, roll] = VrQuat{bones_transforms_[i].orientation}.ToEulerAngles();
        constexpr auto radiansToDegrees = 180.0 / std::numbers::pi;
        bone_angles_[i] = {
            static_cast<int>(pitch * radiansToDegrees),
            static_cast<int>(yaw * radiansToDegrees),
            static_cast<int>(roll * radiansToDegrees)
        };
    }

    // Update the skeletons.
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithController, bones_transforms_);
    input_skeleton_.Update(vr::VRSkeletalMotionRange_WithoutController, bones_transforms_);
}