#pragma once

#include <memory>

#include "LeapTrackedDriver.h"
#include "LeapDevice.h"
#include "VrUtils.h"

class LeapDeviceDriver final : public LeapTrackedDriver {
  public:
    explicit LeapDeviceDriver(const std::shared_ptr<LeapDevice>& leap_device, const std::shared_ptr<LeapDriverSettings>& settings);
    ~LeapDeviceDriver() override = default;

    // ITrackedDeviceServerDriver
    auto Activate(uint32_t object_id) -> vr::EVRInitError override;
    auto Deactivate() -> void override;
    auto EnterStandby() -> void override;
    auto GetComponent(const char* component_name_and_version) -> void* override;
    auto GetPose() -> vr::DriverPose_t override;

    [[nodiscard]] auto Device() const -> const LeapDevice* { return leap_device_.get(); }

    auto SetLeapDevice(const std::shared_ptr<LeapDevice>& new_device) { leap_device_ = new_device; }

  private:
    auto SetDeviceModelProperties(const VrDeviceProperties& properties) const -> void;
    auto ProcessDebugRequestInputs(const DebugRequestPayload& request_payload, nlohmann::json& response) const -> void override;

    std::shared_ptr<LeapDevice> leap_device_;
};
