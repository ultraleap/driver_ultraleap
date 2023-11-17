#include "LeapDevice.h"

#include "VrLogging.h"
#include "VrUtils.h"

LeapDevice::LeapDevice(LEAP_CONNECTION leap_connection, const LEAP_DEVICE_REF leap_device_reference)
    : leap_device_{nullptr},
      leap_device_info_{sizeof(leap_device_info_)},
      leap_serial_{"Unknown"} {

    // Open the device handle.
    if (auto result = LeapOpenDevice(leap_device_reference, &leap_device_); LEAP_FAILED(result)) {
        LOG_INFO("Failed to open Ultraleap device: {}", result);
    }

    // Retrieve the serial number and the device information.
    // This uses a two-call idiom function to first retrieve the required capacity, then the actual data.
    if (auto result = LeapGetDeviceInfo(leap_device_, &leap_device_info_); LEAP_SUCCEEDED(result)) {
        leap_serial_.resize(leap_device_info_.serial_length - 1);
        leap_device_info_.serial = leap_serial_.data();
        leap_device_info_.serial_length = leap_serial_.length() + 1;

        if (result = LeapGetDeviceInfo(leap_device_, &leap_device_info_); LEAP_FAILED(result)) {
            LOG_INFO("Failed to read Ultraleap tracking device serial: {}", result);
        }
    } else {
        LOG_INFO("Failed to read Ultraleap device serial length: {}", result);
    }

    // Subscribe to events.
    if (auto result = LeapSubscribeEvents(leap_connection, leap_device_); LEAP_FAILED(result)) {
        LOG_INFO("Failed to subscribe to device events: {}", result);
    }

    // Set the tracking mode to HMD.
    if (auto result = LeapGetTrackingModeEx(leap_connection, leap_device_); LEAP_FAILED(result)) {
        LOG_INFO("Failed to subscribe to device events: {}", result);
    }
}

LeapDevice::~LeapDevice() {
    LeapCloseDevice(leap_device_);
}
