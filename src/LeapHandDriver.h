#pragma once

#include "VrUtils.h"

#include <openvr_driver.h>
#include <LeapC.h>

class LeapHandDriver final : public vr::ITrackedDeviceServerDriver {
  public:
    explicit LeapHandDriver(eLeapHandType hand);
    virtual ~LeapHandDriver() = default;

    // ITrackedDeviceServerDriver
    auto Activate(uint32_t object_id) -> vr::EVRInitError override;
    auto Deactivate() -> void override;
    auto EnterStandby() -> void override;
    auto GetComponent(const char* component_name_and_version) -> void* override;
    auto DebugRequest(const char* request, char* response_buffer, uint32_t response_buffer_size) -> void override;
    auto GetPose() -> vr::DriverPose_t override;

    [[nodiscard]] auto Id() const -> uint32_t { return id_; }

    auto UpdateFromLeapFrame(const LEAP_TRACKING_EVENT* frame) -> void;

  private:
    auto SetInitialBoneTransforms() -> void;
    auto UpdateBoneTransforms(const LEAP_HAND& hand) -> void;

    uint32_t id_;
    eLeapHandType hand_type_;

    vr::DriverPose_t pose_;
    std::array<vr::VRBoneTransform_t, 31> bones_transforms_;

    VrScalarInputComponent input_pinch_;
    VrScalarInputComponent input_grip_;

    VrSkeletonInputComponent input_skeleton_;
    VrScalarInputComponent input_thumb_finger_;
    VrScalarInputComponent input_index_finger_;
    VrScalarInputComponent input_middle_finger_;
    VrScalarInputComponent input_ring_finger_;
    VrScalarInputComponent input_pinky_finger_;
};
