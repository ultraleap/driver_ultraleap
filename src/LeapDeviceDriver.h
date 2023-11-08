#pragma once

#include "LeapDevice.h"

#include <memory>

#include <openvr_driver.h>
#include <LeapC.h>

#include "OvrUtils.h"

class LeapDeviceDriver final : public vr::ITrackedDeviceServerDriver {
  public:
    explicit LeapDeviceDriver(const std::shared_ptr<LeapDevice>& leapDevice);
    virtual ~LeapDeviceDriver() = default;

    // ITrackedDeviceServerDriver
    auto Activate(uint32_t unObjectId) -> vr::EVRInitError override;
    auto Deactivate() -> void override;
    auto EnterStandby() -> void override;
    auto GetComponent(const char* pchComponentNameAndVersion) -> void* override;
    auto DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) -> void override;
    auto GetPose() -> vr::DriverPose_t override;

    [[nodiscard]] auto Id() const -> uint32_t { return id; }
    [[nodiscard]] auto Device() const -> const LeapDevice* { return leapDevice.get(); }

    auto SetLeapDevice(const std::shared_ptr<LeapDevice>& newDevice) { leapDevice = newDevice; }

  private:
    auto SetDeviceModelProperties(const OvrProperties& properties) const -> void;

    uint32_t                    id;
    std::shared_ptr<LeapDevice> leapDevice;
};
