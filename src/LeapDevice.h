#pragma once

#include <string>

#include <LeapC.h>
#include <memory>

class LeapDevice {
  public:
    explicit LeapDevice(LEAP_DEVICE_REF leapDeviceReference);

    ~LeapDevice();

    LeapDevice(const LeapDevice& other);

    auto operator=(const LeapDevice& other) -> LeapDevice&;

    [[nodiscard]] auto SerialNumber() const -> const std::string& { return leapSerial; }
    [[nodiscard]] auto ProductId() const -> eLeapDevicePID { return leapDeviceInfo.pid; }
    [[nodiscard]] auto Handle() const -> LEAP_DEVICE { return leapDevice; }

  private:
    LEAP_DEVICE      leapDevice;
    LEAP_DEVICE_INFO leapDeviceInfo{};
    std::string      leapSerial;
};
