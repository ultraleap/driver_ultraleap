#include "LeapDevice.h"

#include "OvrUtils.h"

LeapDevice::LeapDevice(LEAP_CONNECTION leapConnection, const LEAP_DEVICE_REF leapDeviceReference)
    : leapDevice{nullptr},
      leapDeviceInfo{sizeof(leapDeviceInfo)},
      leapSerial{"Unknown"} {

    // Open the device handle.
    if (auto result = LeapOpenDevice(leapDeviceReference, &leapDevice);LEAP_FAILED(result)) {
        OVR_LOG("Failed to open Ultraleap device: {}", result);
    }

    // Retrieve the serial number and the device information.
    // This uses a two-call idiom function to first retrieve the required capacity, then the actual data.
    if (auto result = LeapGetDeviceInfo(leapDevice, &leapDeviceInfo); LEAP_SUCCEEDED(result)) {
        leapSerial.resize(leapDeviceInfo.serial_length - 1);
        leapDeviceInfo.serial = leapSerial.data();
        leapDeviceInfo.serial_length = leapSerial.length() + 1;

        if (result = LeapGetDeviceInfo(leapDevice, &leapDeviceInfo); LEAP_FAILED(result)) {
            OVR_LOG("Failed to read Ultraleap tracking device serial: {}", result);
        }
    } else {
        OVR_LOG("Failed to read Ultraleap device serial length: {}", result);
    }

    // Subscribe to events.
    if (auto result = LeapSubscribeEvents(leapConnection, leapDevice); LEAP_FAILED(result)) {
        OVR_LOG("Failed to subscribe to device events: {}", result);
    }

    // Set the tracking mode to HMD.
    if (auto result = LeapGetTrackingModeEx(leapConnection, leapDevice); LEAP_FAILED(result)) {
        OVR_LOG("Failed to subscribe to device events: {}", result);
    }
}

LeapDevice::~LeapDevice() {
    LeapCloseDevice(leapDevice);
}
