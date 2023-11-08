#pragma once

#include <memory>

#include <openvr_driver.h>
#include <LeapC.h>

#include "OvrUtils.h"

class LeapDeviceDriver : public vr::ITrackedDeviceServerDriver {
  public:
    LeapDeviceDriver(LEAP_DEVICE leapDevice, LEAP_DEVICE_INFO leapDeviceInfo, std::string leapSerial);

    vr::EVRInitError    Activate(uint32_t unObjectId) override;
    void                Deactivate() override;
    void                EnterStandby() override;
    [[nodiscard]] void* GetComponent(const char* pchComponentNameAndVersion) override;
    void                DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
    [[nodiscard]] vr::DriverPose_t   GetPose() override;
    void                             UpdateDevice(LEAP_DEVICE newDevice, LEAP_DEVICE_INFO newDeviceInfo);
    [[nodiscard]] uint32_t           GetId() const { return id; }
    [[nodiscard]] const std::string& GetSerialNumber() const { return leapSerial; }
    [[nodiscard]] bool               IsDeviceConnected() const { return leapDevice != nullptr; }
    void                             Disconnect();

  private:
    void SetDeviceModelProperties(const OvrProperties& properties) const;

    uint32_t         id = vr::k_unTrackedDeviceIndexInvalid;
    LEAP_DEVICE      leapDevice;
    LEAP_DEVICE_INFO leapDeviceInfo;
    std::string      leapSerial = "Unknown";
};
