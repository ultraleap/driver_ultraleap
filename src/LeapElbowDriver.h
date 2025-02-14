#pragma once

#include "LeapTrackedDriver.h"
#include "VrUtils.h"

#include <LeapC.h>

class LeapElbowDriver final : public LeapTrackedDriver {
  public:
    LeapElbowDriver(const std::shared_ptr<LeapDriverSettings>& settings, eLeapHandType hand);
    ~LeapElbowDriver() override = default;

    // ITrackedDeviceServerDriver
    auto Activate(uint32_t object_id) -> vr::EVRInitError override;
    auto Deactivate() -> void override;
    auto EnterStandby() -> void override;
    auto GetComponent(const char* component_name_and_version) -> void* override;
    auto GetPose() -> vr::DriverPose_t override;

    auto UpdateFromLeapFrame(const LEAP_TRACKING_EVENT* frame) -> void;

  private:
    auto ProcessDebugRequestInputs(const DebugRequestPayload& request_payload, nlohmann::json& response) const -> void override;

    eLeapHandType hand_type_;
    vr::DriverPose_t pose_;
    VrBooleanInputComponent input_proximity_;
};
