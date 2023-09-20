#pragma once

#include <openvr_driver.h>
#include <LeapC.h>

class LeapHandDriver : public vr::ITrackedDeviceServerDriver {
  public:
    explicit LeapHandDriver(eLeapHandType hand);

    vr::EVRInitError Activate(uint32_t unObjectId) override;
    void             Deactivate() override;
    void             EnterStandby() override;
    void*            GetComponent(const char* pchComponentNameAndVersion) override;
    void             DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
    vr::DriverPose_t GetPose() override;

  private:
    uint32_t id = vr::k_unTrackedDeviceIndexInvalid;

    eLeapHandType hand;
};
