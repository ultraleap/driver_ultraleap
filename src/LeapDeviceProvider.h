#pragma once

#include <memory>
#include <thread>
#include <map>
#include <optional>

#include <openvr_driver.h>
#include <LeapC.h>

#include "LeapDeviceDriver.h"
#include "LeapDriverSettings.h"
#include "LeapHandDriver.h"

class LeapDeviceProvider final : public vr::IServerTrackedDeviceProvider {
  public:
    LeapDeviceProvider() = default;
    virtual ~LeapDeviceProvider() = default;

    // IServerTrackedDeviceProvider
    auto Init(vr::IVRDriverContext* driver_context) -> vr::EVRInitError override;
    auto Cleanup() -> void override;
    auto GetInterfaceVersions() -> const char* const* override;
    auto RunFrame() -> void override;
    auto ShouldBlockStandbyMode() -> bool override;
    auto EnterStandby() -> void override;
    auto LeaveStandby() -> void override;

  private:
    // All these methods will be called by the tracking Service
    auto ServiceMessageLoop() -> void;
    auto DeviceDetected(uint32_t device_id, const LEAP_DEVICE_EVENT* event) -> void;
    auto DeviceLost(uint32_t device_id, const LEAP_DEVICE_EVENT* event) -> void;
    auto DeviceStatusChanged(uint32_t device_id, const LEAP_DEVICE_STATUS_CHANGE_EVENT* event) -> void;
    auto TrackingFrame([[maybe_unused]] uint32_t device_id, const LEAP_TRACKING_EVENT* event) const -> void;
    auto TrackingModeChanged(uint32_t device_id, const LEAP_TRACKING_MODE_EVENT* event) const -> void;

    // VRServerDriverHost events
    auto OtherSectionSettingsChanged() const -> void;

    auto CreateDeviceDriver(const std::shared_ptr<LeapDevice>& leap_device) -> void;
    auto ReconnectDeviceDriver(const std::shared_ptr<LeapDevice>& leap_device) const -> void;
    auto DisconnectDeviceDriver(uint32_t device_id) -> void;

    auto CreateHandControllers() -> void;
    auto DisconnectHandControllers() const -> void;

    auto DeviceDriverFromLeapId(uint32_t device_id) const -> const std::shared_ptr<LeapDeviceDriver>&;

    // VrSettings related functions
    auto UpdateServiceAndDriverTrackingMode(eLeapTrackingMode mode, std::optional<LeapDevice*> device) const -> void;

    LEAP_CONNECTION leap_connection_ = nullptr;
    std::atomic<bool> is_running_ = false;
    std::atomic<bool> is_connected_ = false;
    std::thread service_thread_;

    std::map<uint32_t, std::shared_ptr<LeapDevice>> leap_device_by_id_;
    std::map<std::string, std::shared_ptr<LeapDeviceDriver>> leap_device_driver_by_serial_;

    std::unique_ptr<LeapHandDriver> left_hand_;
    std::unique_ptr<LeapHandDriver> right_hand_;
    std::shared_ptr<LeapDriverSettings> settings_;
};