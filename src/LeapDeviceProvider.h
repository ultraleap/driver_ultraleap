#pragma once

#include <memory>
#include <thread>
#include <map>

#include <openvr_driver.h>
#include <LeapC.h>

#include "LeapDeviceDriver.h"
#include "LeapHandDriver.h"

class LeapDeviceProvider final : public vr::IServerTrackedDeviceProvider {
  public:
    LeapDeviceProvider() = default;
    virtual ~LeapDeviceProvider() = default;

    // IServerTrackedDeviceProvider
    auto Init(vr::IVRDriverContext* pDriverContext) -> vr::EVRInitError override;
    auto Cleanup() -> void override;
    auto GetInterfaceVersions() -> const char* const* override;
    auto RunFrame() -> void override;
    auto ShouldBlockStandbyMode() -> bool override;
    auto EnterStandby() -> void override;
    auto LeaveStandby() -> void override;

  private:
    // All these methods will be called by the tracking Serivce
    auto ServiceMessageLoop() -> void;
    auto SetThreadName(const std::string_view& name) -> void;
    auto DeviceDetected(uint32_t deviceId, const LEAP_DEVICE_EVENT* event) -> void;
    auto DeviceLost(uint32_t deviceId, const LEAP_DEVICE_EVENT* event) -> void;
    auto DeviceStatusChanged(uint32_t deviceId, const LEAP_DEVICE_STATUS_CHANGE_EVENT* event) -> void;
    auto TrackingFrame(uint32_t deviceId, const LEAP_TRACKING_EVENT* event) const -> void;
    auto TrackingModeChanged(uint32_t deviceId, const LEAP_TRACKING_MODE_EVENT* event) const -> void;

    auto CreateDeviceDriver(const std::shared_ptr<LeapDevice>& leapDevice) -> void;
    auto ReconnectDeviceDriver(const std::shared_ptr<LeapDevice>& leapDevice) const -> void;
    auto DisconnectDeviceDriver(uint32_t deviceId) -> void;

    auto CreateHandControllers() -> void;
    auto DisconnectHandControllers() const -> void;

    auto DeviceDriverFromLeapId(uint32_t deviceId) const -> const std::shared_ptr<LeapDeviceDriver>&;

    LEAP_CONNECTION leapConnection = nullptr;
    std::atomic<bool> isRunning = false;
    std::atomic<bool> isConnected = false;

    std::map<uint32_t, std::shared_ptr<LeapDevice>> leapDeviceById;
    std::map<std::string, std::shared_ptr<LeapDeviceDriver>> deviceDriverBySerial;

    std::unique_ptr<LeapHandDriver> leftHand;
    std::unique_ptr<LeapHandDriver> rightHand;

    std::thread serviceThread;
};