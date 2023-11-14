#include "LeapDevice.h"

#include "OvrUtils.h"

LeapDevice::LeapDevice(const LEAP_DEVICE_REF leapDeviceReference)
    : leapDevice{nullptr},
      leapDeviceInfo{sizeof(leapDeviceInfo)},
      leapSerial{"Unknown"} {

    // Open the device handle.
    auto result = LeapOpenDevice(leapDeviceReference, &leapDevice);
    if (LEAP_FAILED(result)) {
        OVR_LOG("Failed to open Ultraleap device: {}", result);
    }

    // Retrieve the serial number and the device information.
    leapDeviceInfo.serial = leapSerial.data();
    leapDeviceInfo.serial_length = leapSerial.length() + 1;
    if (result = LeapGetDeviceInfo(leapDevice, &leapDeviceInfo); LEAP_SUCCEEDED(result)) {
        leapSerial.resize(leapDeviceInfo.serial_length - 1);
        leapDeviceInfo.serial = leapSerial.data();
        result = LeapGetDeviceInfo(leapDevice, &leapDeviceInfo);
        if (LEAP_FAILED(result)) {
            OVR_LOG("Failed to read Ultraleap tracking device serial: {}", result);
        }
    } else {
        OVR_LOG("Failed to read Ultraleap device serial length: {}", result);
    }
}

LeapDevice::~LeapDevice() {
    LeapCloseDevice(leapDevice);
}
