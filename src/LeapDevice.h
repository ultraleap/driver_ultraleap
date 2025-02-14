#pragma once

#include <string>

#include <LeapC.h>

class LeapDevice {
  public:
    LeapDevice(LEAP_CONNECTION leap_connection, LEAP_DEVICE_REF leap_device_reference);
    ~LeapDevice();

    [[nodiscard]] auto SerialNumber() const -> const std::string& { return leap_serial_; }
    [[nodiscard]] auto ProductId() const -> eLeapDevicePID { return leap_device_info_.pid; }
    [[nodiscard]] auto Handle() const -> LEAP_DEVICE { return leap_device_; }

    // Disable copying as this is uniquely owns a LEAP_DEVICE.
    LeapDevice(const LeapDevice& other) = delete;
    auto operator=(const LeapDevice& other) -> LeapDevice& = delete;

  private:
    LEAP_DEVICE leap_device_;
    LEAP_DEVICE_INFO leap_device_info_;
    std::string leap_serial_;
};
