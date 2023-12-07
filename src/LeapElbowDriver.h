#pragma once

#include "LeapDriverSettings.h"

#include <atomic>

#include "VrUtils.h"

#include <openvr_driver.h>
#include <LeapC.h>

class LeapElbowDriver final : public vr::ITrackedDeviceServerDriver {
  public:
    LeapElbowDriver(const std::shared_ptr<LeapDriverSettings>& settings, eLeapHandType hand);
    virtual ~LeapElbowDriver() = default;

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
    uint32_t id_;
    std::atomic<bool> active_ = false;

    std::shared_ptr<LeapDriverSettings> settings_;
    eLeapHandType hand_type_;
    vr::DriverPose_t pose_;
    VrBooleanInputComponent input_proximity_;
};
