#pragma once

#include <array>
#include <span>

#include <openvr_driver.h>
#include <LeapC.h>

enum class VrHandType {
    Left = 0,
    Right = 1,
};

class VrHand {
  public:
    explicit VrHand(const LEAP_HAND& leap_hand);

    [[nodiscard]] auto GetBoneTransforms() const -> std::span<const vr::VRBoneTransform_t> { return std::span{bones_transforms_}; }
    [[nodiscard]] auto GetPinchStrength() const -> float { return pinch_strength_; }
    [[nodiscard]] auto GetGrabStrength() const -> float { return grab_strength_; }
    [[nodiscard]] auto GetIndexFingerCurl() const -> float { return index_finger_curl_; }
    [[nodiscard]] auto GetMiddleFingerCurl() const -> float { return middle_finger_curl_; }
    [[nodiscard]] auto GetRingFingerCurl() const -> float { return ring_finger_curl_; }
    [[nodiscard]] auto GetPinkyFingerCurl() const -> float { return pinky_finger_curl_; }

private:
    std::array<vr::VRBoneTransform_t, 31> bones_transforms_{};

    float pinch_strength_;
    float grab_strength_;

    float index_finger_curl_;
    float middle_finger_curl_;
    float ring_finger_curl_;
    float pinky_finger_curl_ ;
};
