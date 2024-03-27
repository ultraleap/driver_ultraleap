#pragma once

#include <map>

#include "LeapTrackedDriver.h"
#include "VrUtils.h"

#include <LeapC.h>

class LeapHandDriver final : public LeapTrackedDriver {
  public:
    using VrInputComponent = std::variant<VrScalarInputComponent*, VrBooleanInputComponent*>;
    LeapHandDriver(const std::shared_ptr<LeapDriverSettings>& settings, eLeapHandType hand);
    ~LeapHandDriver() override = default;

    // ITrackedDeviceServerDriver
    auto Activate(uint32_t object_id) -> vr::EVRInitError override;
    auto Deactivate() -> void override;
    auto EnterStandby() -> void override;
    auto GetComponent(const char* component_name_and_version) -> void* override;
    auto GetPose() -> vr::DriverPose_t override;

    auto UpdateFromLeapFrame(const LEAP_TRACKING_EVENT* frame) -> void;

  private:
    auto SetInitialBoneTransforms() -> void;
    auto ProcessDebugRequestInputs(const DebugRequestPayload& request_payload, nlohmann::json& response) const -> void override;

    eLeapHandType hand_type_;

    vr::DriverPose_t pose_;
    std::array<vr::VRBoneTransform_t, 31> bones_transforms_{};
    std::array<std::pair<double, double>, 31> bone_lengths_{};

    VrBooleanInputComponent input_system_menu_;
    VrBooleanInputComponent input_proximity_;

    VrScalarInputComponent input_pinch_;
    VrScalarInputComponent input_grip_;

    VrSkeletonInputComponent input_skeleton_;

    VrScalarInputComponent input_thumb_finger_;
    VrScalarInputComponent input_index_finger_;
    VrScalarInputComponent input_middle_finger_;
    VrScalarInputComponent input_ring_finger_;
    VrScalarInputComponent input_pinky_finger_;

    // Used to store mappings of our Vr*InputComponents to the received paths from the Debug Request Parser
    std::map<InputPath, VrInputComponent> path_inputs_map_;
};