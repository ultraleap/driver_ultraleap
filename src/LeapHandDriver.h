#pragma once

#include "LeapDriverSettings.h"

#include <atomic>
#include <unordered_map>
#include <variant>

#include "VrUtils.h"
#include "RequestDeserializer.h"

#include <openvr_driver.h>
#include <LeapC.h>

class LeapHandDriver final : public vr::ITrackedDeviceServerDriver {
  public:
    using InputComponent = std::variant<VrScalarInputComponent*, VrBooleanInputComponent*>;
    LeapHandDriver(const std::shared_ptr<LeapDriverSettings>& settings, eLeapHandType hand);
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

    uint32_t id_;
    std::atomic<bool> active_ = false;

    std::shared_ptr<LeapDriverSettings> settings_;
    eLeapHandType hand_type_;

    vr::DriverPose_t pose_;
    std::array<vr::VRBoneTransform_t, 31> bones_transforms_{};
    std::array<std::pair<double, double>, 31> bone_lengths_{};

    VrBooleanInputComponent input_system_menu_;
    VrBooleanInputComponent input_proximity_;

    VrScalarInputComponent input_pinch_;
    VrScalarInputComponent input_grip_;

    VrSkeletonInputComponent input_skeleton_;
    VrScalarInputComponent input_index_finger_;
    VrScalarInputComponent input_middle_finger_;
    VrScalarInputComponent input_ring_finger_;
    VrScalarInputComponent input_pinky_finger_;

    std::unordered_map<InputPaths, InputComponent> path_inputs_map_;
};
