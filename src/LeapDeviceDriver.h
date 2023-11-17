#pragma once

#include <memory>

#include <openvr_driver.h>

#include "LeapDevice.h"
#include "VrUtils.h"

class LeapDeviceDriver final : public vr::ITrackedDeviceServerDriver {
  public:
    explicit LeapDeviceDriver(const std::shared_ptr<LeapDevice>& leap_device);
    virtual ~LeapDeviceDriver() = default;

    // ITrackedDeviceServerDriver
    auto Activate(uint32_t object_id) -> vr::EVRInitError override;
    auto Deactivate() -> void override;
    auto EnterStandby() -> void override;
    auto GetComponent(const char* component_name_and_version) -> void* override;
    auto DebugRequest(const char* request, char* response_buffer, uint32_t response_buffer_size) -> void override;
    auto GetPose() -> vr::DriverPose_t override;

    [[nodiscard]] auto Id() const -> uint32_t { return id_; }
    [[nodiscard]] auto Device() const -> const LeapDevice* { return leap_device_.get(); }

    auto SetLeapDevice(const std::shared_ptr<LeapDevice>& new_device) { leap_device_ = new_device; }

  private:
    auto SetDeviceModelProperties(const VrDeviceProperties& properties) const -> void;

    uint32_t id_;
    std::shared_ptr<LeapDevice> leap_device_;
};
