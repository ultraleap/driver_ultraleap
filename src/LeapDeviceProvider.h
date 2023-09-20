#pragma once

#include <memory>
#include <thread>
#include <map>

#include <openvr_driver.h>
#include <LeapC.h>

#include "LeapDeviceDriver.h"
#include "LeapHandDriver.h"

class LeapDeviceProvider : public vr::IServerTrackedDeviceProvider {
  public:
    vr::EVRInitError   Init(vr::IVRDriverContext* pDriverContext) override;
    void               Cleanup() override;
    const char* const* GetInterfaceVersions() override;
    void               RunFrame() override;
    bool               ShouldBlockStandbyMode() override;
    void               EnterStandby() override;
    void               LeaveStandby() override;

  private:
    void ServiceMessageLoop();
    void SetThreadName(const std::string_view& name);
    void DeviceDetected(uint32_t deviceId, const LEAP_DEVICE_EVENT* event);
    void DeviceLost(uint32_t deviceId, const LEAP_DEVICE_EVENT* event);
    void TrackingFrame(uint32_t deviceId, const LEAP_TRACKING_EVENT* event);
    void TrackingModeChanged(uint32_t deviceId, const LEAP_TRACKING_MODE_EVENT* event);

    static void NotifyDeviceDisconnected(const std::shared_ptr<LeapDeviceDriver>& device);
    static void NotifyDeviceConnected(const std::shared_ptr<LeapDeviceDriver>& device);

    LEAP_CONNECTION   leapConnection;
    std::atomic<bool> isRunning   = false;
    std::atomic<bool> isConnected = false;

    std::map<uint32_t, std::shared_ptr<LeapDeviceDriver>> devices;
    std::unique_ptr<LeapHandDriver> leftHand;
    std::unique_ptr<LeapHandDriver> rightHand;

    std::thread serviceThread;
};