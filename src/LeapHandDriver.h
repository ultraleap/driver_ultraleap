#pragma once

#include <openvr_driver.h>
#include <LeapC.h>

class LeapHandDriver final : public vr::ITrackedDeviceServerDriver {
  public:
    explicit LeapHandDriver(eLeapHandType hand);
    virtual ~LeapHandDriver() = default;

    // ITrackedDeviceServerDriver
    auto Activate(uint32_t unObjectId) -> vr::EVRInitError override;
    auto Deactivate() -> void override;
    auto EnterStandby() -> void override;
    auto GetComponent(const char* pchComponentNameAndVersion) -> void* override;
    auto DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) -> void override;
    auto GetPose() -> vr::DriverPose_t override;

    [[nodiscard]] auto Id() const -> uint32_t { return id; }

  private:
    uint32_t id;
    eLeapHandType hand;
};
