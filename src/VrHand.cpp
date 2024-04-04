#include "VrHand.h"

#include "VrUtils.h"

#include <numbers>
#include <ranges>

VrHand::VrHand(const LEAP_HAND& leap_hand)
    : pinch_strength_{0},
      grab_strength_{0},
      index_finger_curl_{0},
      middle_finger_curl_{0},
      ring_finger_curl_{0},
      pinky_finger_curl_{0} {

    const auto hand_type = leap_hand.type == eLeapHandType_Left ? VrHandType::Left : VrHandType::Right;

    const auto GetBoneRotation = [&](const VrHandSkeletonBone bone) -> VrQuat {
        if (bone == Root) {
            return VrQuat{leap_hand.palm.orientation};
        }
        if (bone == Wrist) {
            return VrQuat{leap_hand.palm.orientation}; // SteamVR expects the wrist to be hand static not arm static.
        }
        if (bone < IndexFinger0) {
            // Ignore the zero length metacarpal and the tip has the same rotation as the preceeding bone.
            return VrQuat{leap_hand.thumb.bones[std::min(bone - Thumb0 + 1, 3)].rotation};
        }
        if (bone < AuxThumb) {
            // Tip has the same rotation as preceeding bone.
            const auto digit_index = (bone - IndexFinger0) / 5 + 1;
            const auto bone_index = (bone - IndexFinger0) % 5;
            return VrQuat{leap_hand.digits[digit_index].bones[std::min(bone_index, 3)].rotation};
        }

        // TODO: AUX Bones
        return VrQuat::Identity;
    };

    const auto GetBonePosition = [&](const VrHandSkeletonBone bone) -> VrVec3 {
        if (bone == Root) {
            return VrVec3{leap_hand.palm.position} * 0.001f;
        }
        if (bone == Wrist) {
            return VrVec3{leap_hand.arm.next_joint} * 0.001f;
        }
        if (bone < IndexFinger0) {
            // Special case checking for the finger tip.
            if (bone == Thumb3) {
                return VrVec3{leap_hand.thumb.distal.next_joint} * 0.001f;
            }
            // Ignore the zero length metacarpal.
            return VrVec3{leap_hand.thumb.bones[bone - Thumb0 + 1].prev_joint} * 0.001f;
        }
        if (bone < AuxThumb) {
            // Again account for the tip specially.
            const auto digit_index = (bone - IndexFinger0) / 5 + 1;
            const auto bone_index = (bone - IndexFinger0) % 5;
            if (bone_index == 4) {
                return VrVec3{leap_hand.digits[digit_index].bones[bone_index - 1].next_joint} * 0.001f;
            }
            return VrVec3{leap_hand.digits[digit_index].bones[bone_index].prev_joint} * 0.001f;
        }

        // TODO: AUX Bones
        return VrVec3::Zero;
    };

    const auto ComputeDigitBoneTransforms =
        [&](VrVec3 parent_position, VrQuat parent_rotation, const size_t index_start, const size_t index_end) {
            for (auto index = index_start; index <= index_end; ++index) {

                const auto handed_orientation = VrQuat::FromEulerAngles(
                    -std::numbers::pi,
                    hand_type == VrHandType::Left ? std::numbers::pi / 2.0 : -std::numbers::pi / 2.0,
                    hand_type == VrHandType::Left ? 0 : std::numbers::pi
                );
                auto bone_position = GetBonePosition(static_cast<VrHandSkeletonBone>(index));
                auto bone_rotation = GetBoneRotation(static_cast<VrHandSkeletonBone>(index)) * handed_orientation;

                auto local_bone_position = (bone_position - parent_position) * parent_rotation;
                auto local_bone_rotation = parent_rotation.Inverse() * bone_rotation;

                // Metacarpals need to handle the additional 90 degree rotation that OpenVR expects.
                if (index == index_start) {
                    const auto wrist_correction = VrQuat::FromEulerAngles(
                        0,
                        std::numbers::pi,
                        hand_type == VrHandType::Left ? -std::numbers::pi / 2.0 : std::numbers::pi / 2.0
                    );
                    local_bone_position = local_bone_position * wrist_correction;
                    local_bone_rotation = wrist_correction.Inverse() * local_bone_rotation;
                }

                bones_transforms_[index] = {local_bone_position, local_bone_rotation};

                parent_position = bone_position;
                parent_rotation = bone_rotation;
            }
        };

    const auto ComputeThumbCurl = [&](const LEAP_DIGIT& finger) -> float {
        const auto bone_directions = std::ranges::views::transform(finger.bones, [](const LEAP_BONE& bone) -> glm::dvec3 {
            return (bone.next_joint - bone.prev_joint).Normalised();
        });
        const auto angle1 = glm::acos(glm::dot(bone_directions[1], bone_directions[2])); // Intermediate
        const auto angle2 = glm::acos(glm::dot(bone_directions[2], bone_directions[3])); // Distal
        return static_cast<float>(glm::clamp(glm::max(angle1, angle2) / (std::numbers::pi / 5.0), 0.0, 1.0));
    };

    const auto ComputeFingerCurl = [&](const LEAP_DIGIT& finger) -> float {
        const auto bone_directions = std::ranges::views::transform(finger.bones, [](const LEAP_BONE& bone) -> glm::dvec3 {
            return (bone.next_joint - bone.prev_joint).Normalised();
        });
        const auto total_finger_angles = glm::acos(glm::dot(bone_directions[0], bone_directions[1]))  // Proximal
                                       + glm::acos(glm::dot(bone_directions[1], bone_directions[2]))  // Intermediate
                                       + glm::acos(glm::dot(bone_directions[2], bone_directions[3])); // Distal
        return static_cast<float>(glm::clamp(total_finger_angles / std::numbers::pi, 0.0, 1.0));
    };

    const auto root_position = GetBonePosition(Root);
    const auto root_rotation = GetBoneRotation(Root);
    const auto wrist_position = GetBonePosition(Wrist);
    const auto wrist_rotation = GetBoneRotation(Wrist);

    bones_transforms_[Root] = vr::VRBoneTransform_t{VrVec3::Zero, VrQuat::Identity};
    if (hand_type == VrHandType::Left) {
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

    // Calculate the AUX bones.
    // TODO: This needs further checking.
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

    grab_strength_ = leap_hand.grab_strength;
    pinch_strength_ = leap_hand.pinch_strength;

    // Compute the scalar finger curls.
    thumb_finger_curl_ = ComputeThumbCurl(leap_hand.thumb);
    index_finger_curl_ = ComputeFingerCurl(leap_hand.index);
    middle_finger_curl_ = ComputeFingerCurl(leap_hand.middle);
    ring_finger_curl_ = ComputeFingerCurl(leap_hand.ring);
    pinky_finger_curl_ = ComputeFingerCurl(leap_hand.pinky);
}

auto VrHand::GetSystemMenuTriggered(std::span<const LEAP_HAND> hands) -> bool {
    // We need both hands to be tracked to trigger this pose.
    if (hands.size() != 2) {
        return false;
    }

    // Below defines the pose of making a triangle by bringing both hands index fingers and thumbs together.
    const auto& hand1 = hands[0];
    const auto& hand2 = hands[1];
    const double index_tip_distance = VrVec3{hand1.index.distal.next_joint - hand2.index.distal.next_joint}.Length();
    const double thumb_tip_distance = VrVec3{hand1.thumb.distal.next_joint - hand2.thumb.distal.next_joint}.Length();
    return index_tip_distance < 20 && thumb_tip_distance < 20;
}
