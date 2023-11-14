#pragma once

#include <string>

#include <LeapC.h>

class LeapDevice {
  public:
    LeapDevice(LEAP_CONNECTION leapConnection, LEAP_DEVICE_REF leapDeviceReference);
    ~LeapDevice();

    [[nodiscard]] auto SerialNumber() const -> const std::string& { return leapSerial; }
    [[nodiscard]] auto ProductId() const -> eLeapDevicePID { return leapDeviceInfo.pid; }
    [[nodiscard]] auto Handle() const -> LEAP_DEVICE { return leapDevice; }

    // Disable copying as this is uniquely owns a LEAP_DEVICE.
    LeapDevice(const LeapDevice& other) = delete;
    auto operator=(const LeapDevice& other) -> LeapDevice& = delete;

  private:
    LEAP_DEVICE leapDevice;
    LEAP_DEVICE_INFO leapDeviceInfo;
    std::string leapSerial;
};
