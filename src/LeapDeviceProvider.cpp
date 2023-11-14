#include "LeapDeviceProvider.h"

#include <algorithm>

#if defined(_WIN32)
#include <ShlObj_core.h>
#else
#include <pthread.h>
#endif

#include "OvrUtils.h"
#include "VrMath.h"

auto LeapDeviceProvider::Init(vr::IVRDriverContext* pDriverContext) -> vr::EVRInitError {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext)

    // Initialise logging subsystem.
    OvrLogging::Init();

    // Initialize the LeapC Connection
    if (auto result = LeapCreateConnection(nullptr, &leapConnection); LEAP_FAILED(result)) {
        OVR_LOG("Failed to create connection to Ultraleap tracking service: {}", result);
        return vr::VRInitError_Driver_Failed;
    }

    if (auto result = LeapOpenConnection(leapConnection); LEAP_FAILED(result)) {
        LeapDestroyConnection(leapConnection);
        OVR_LOG("Failed to open connection to Ultraleap tracking service: {}", result);
        return vr::VRInitError_Driver_Failed;
    }

    // Start the service thread.
    isRunning = true;
    serviceThread = std::thread{&LeapDeviceProvider::ServiceMessageLoop, this};

    // Indicate that we have initialized successfully.
    return vr::VRInitError_None;
}

auto LeapDeviceProvider::Cleanup() -> void {
    // Request the thread termination and close the connection to ensure that LeapPollConnection doesn't hang.
    {
        isRunning = false;

        // Close all remaining device handles before closing the connection.
        deviceSerialById.clear();
        deviceDriverBySerial.clear();
        LeapCloseConnection(leapConnection);
    }

    if (serviceThread.joinable()) {
        serviceThread.join();
    }

    LeapDestroyConnection(leapConnection);
}

auto LeapDeviceProvider::GetInterfaceVersions() -> const char* const* {
    return vr::k_InterfaceVersions;
}

auto LeapDeviceProvider::RunFrame() -> void {
    // Update the base-station pose for each device.
    //    for (auto [leapId, device] : devices) {
    //        const auto devicePose = device->GetPose();
    //        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device->GetId(), devicePose, sizeof(devicePose));
    //    }
}

auto LeapDeviceProvider::ShouldBlockStandbyMode() -> bool {
    return false;
}

auto LeapDeviceProvider::EnterStandby() -> void {
    LeapSetPause(leapConnection, true);
}

auto LeapDeviceProvider::LeaveStandby() -> void {
    LeapSetPause(leapConnection, false);
}

auto LeapDeviceProvider::ServiceMessageLoop() -> void {
    SetThreadName("UltraleapTrackingThread");

    for (LEAP_CONNECTION_MESSAGE msg; isRunning;) {
        eLeapRS result = LeapPollConnection(leapConnection, 1000, &msg);
        if (LEAP_FAILED(result)) {
            if (result != eLeapRS_Timeout) {
                OVR_LOG("Failed to poll tracking connection: {}", result);
            }
            continue;
        }

        switch (msg.type) {
        case eLeapEventType_Connection: isConnected = true; break;
        case eLeapEventType_ConnectionLost: isConnected = false; break;
        case eLeapEventType_Device: DeviceDetected(msg.device_id, msg.device_event); break;
        case eLeapEventType_DeviceLost: DeviceLost(msg.device_id, msg.device_event); break;
        case eLeapEventType_Tracking: TrackingFrame(msg.device_id, msg.tracking_event); break;
        case eLeapEventType_TrackingMode: TrackingModeChanged(msg.device_id, msg.tracking_mode_event); break;
        default: continue;
        }
    }
}

auto LeapDeviceProvider::SetThreadName(const std::string_view& name) -> void {
    // Set the thread name (This has to utilize a platform specific method).
#if defined(_WIN32)
    auto narrowName = std::string{name};
    auto length = MultiByteToWideChar(CP_ACP, 0, narrowName.c_str(), static_cast<int>(narrowName.length()), nullptr, 0);
    auto wideName = std::wstring(length, 0);
    if (MultiByteToWideChar(CP_ACP, 0, narrowName.c_str(), static_cast<int>(narrowName.length()), wideName.data(), length) == 0) {
        OVR_LOG("Failed to set thread name to \"{}\"", name);
    }
    SetThreadDescription(serviceThread.native_handle(), wideName.c_str());
#elif defined(__APPLE__)
    // Can only set the current thread name on MacOS
    if (std::this_thread::get_id() != serviceThread.get_id()) {
        UL_LOG_ERROR("Thread name can only be set for the current thread on MacOS");
    }
    pthread_setname_np(std::string{name}.c_str());
#elif defined(__GNUC__) || defined(COMPILER_GCC)
    pthread_setname_np(serviceThread.native_handle(), std::string{name}.c_str());
#else
#error "PlatformContext::SetThreadName() not implemented for current platform"
#endif
}

auto LeapDeviceProvider::TrackingFrame(const uint32_t deviceId, const LEAP_TRACKING_EVENT* event) -> void {
    auto leftHandPose = kDeviceConnectedPose;
    auto rightHandPose = kDeviceConnectedPose;

    // Process all the hands that are seen in the frame.
    for (auto i = 0; i < event->nHands; ++i) {
        const auto& hand = event->pHands[i];
        auto& pose = hand.type == eLeapHandType_Left ? leftHandPose : rightHandPose;

        // Space transform from LeapC -> OpenVR Space;
        pose.qWorldFromDriverRotation = HmdQuaternion_FromEulerAngles(0.0, M_PI / 2.0f, M_PI);
        pose.qDriverFromHeadRotation = HmdQuaternion_Identity;

        pose.vecPosition[0] = hand.palm.position.x;
        pose.vecPosition[1] = hand.palm.position.y;
        pose.vecPosition[2] = hand.palm.position.z;
        pose.qRotation = {
            hand.palm.orientation.w,
            hand.palm.orientation.x,
            hand.palm.orientation.y,
            hand.palm.orientation.z,
        };
        pose.result = vr::TrackingResult_Running_OK;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;

        (hand.type == eLeapHandType_Left ? leftHandPose : rightHandPose) = pose;
    }

    if (leftHand != nullptr) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(leftHand->Id(), leftHandPose, sizeof(leftHandPose));
    }

    if (rightHand != nullptr) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(rightHand->Id(), rightHandPose, sizeof(rightHandPose));
    }
}

auto LeapDeviceProvider::TrackingModeChanged(const uint32_t deviceId, const LEAP_TRACKING_MODE_EVENT* event) -> void {
    if (event->current_tracking_mode != eLeapTrackingMode_HMD) {
        OVR_LOG("Device is not currently in HMD mode, setting to HMD");
        LeapSetTrackingModeEx(leapConnection, DeviceDriverFromLeapId(deviceId)->Device()->Handle(), eLeapTrackingMode_HMD);
    }
}

auto LeapDeviceProvider::DeviceDetected(const uint32_t /*deviceId*/, const LEAP_DEVICE_EVENT* event) -> void {
    // WARNING!
    // In this context, deviceId will be 0 as this is a system message, for the ID of the affected device, use event->device.id.

    // Construct and open the device and track the serial number.
    const auto leapDevice = std::make_shared<LeapDevice>(event->device);
    deviceSerialById.insert({event->device.id, leapDevice->SerialNumber()});

    // Create a device if it's the first time we've seen this serial, otherwise just notify that it's now active again.
    if (!deviceDriverBySerial.contains(leapDevice->SerialNumber())) {
        CreateDeviceDriver(leapDevice);
    } else {
        UpdateDeviceDriverWithNewDevice(leapDevice);
    }

    // If this is the first device, construct the two hand-controllers.
    if (deviceSerialById.size() == 1 && leftHand == nullptr && rightHand == nullptr) {
        CreateHandControllers();
    }
}

auto LeapDeviceProvider::DeviceLost(const uint32_t /*deviceId*/, const LEAP_DEVICE_EVENT* event) -> void {
    // WARNING!
    // In this context, deviceId will be 0 as this is a system message, for the ID of the affected device, use event->device.id.
    DisconnectDeviceDriver(event->device.id);

    // If all devices are now disconnected, indicate that the hands are no-longer trackable.
    if (deviceSerialById.empty()) {
        DisconnectHandControllers();
    }
}

auto LeapDeviceProvider::CreateDeviceDriver(const std::shared_ptr<LeapDevice>& leapDevice) -> void {
    // Track the device by serial-number
    auto [leapDeviceDriver, inserted] = deviceDriverBySerial.emplace(
        leapDevice->SerialNumber(),
        std::make_shared<LeapDeviceDriver>(leapDevice)
    );

    // Register the device driver.
    if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
            leapDevice->SerialNumber().c_str(),
            vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference,
            leapDeviceDriver->second.get()
        )) {
        OVR_LOG("Failed to add new Ultraleap device: {}", leapDevice->SerialNumber());
    }
}

auto LeapDeviceProvider::UpdateDeviceDriverWithNewDevice(const std::shared_ptr<LeapDevice>& leapDevice) const -> void {
    // Update the driver for this device to reference the new underlying device.
    auto& leapDeviceDriver = deviceDriverBySerial.at(leapDevice->SerialNumber());
    leapDeviceDriver->SetLeapDevice(leapDevice);

    // Register that the device is now connected again.
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(leapDeviceDriver->Id(), kDeviceConnectedPose, sizeof(kDeviceConnectedPose));
}

auto LeapDeviceProvider::DisconnectDeviceDriver(const uint32_t deviceId) -> void {
    // Register the device as disconnected.
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
        DeviceDriverFromLeapId(deviceId)->Id(),
        kDeviceDisconnectedPose,
        sizeof(kDeviceDisconnectedPose)
    );

    // Remove the device.
    deviceSerialById.erase(deviceId);
}

auto LeapDeviceProvider::CreateHandControllers() -> void {
    leftHand = std::make_unique<LeapHandDriver>(eLeapHandType_Left);
    rightHand = std::make_unique<LeapHandDriver>(eLeapHandType_Right);

    if (!vr::VRServerDriverHost()
             ->TrackedDeviceAdded("LeftHand", vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, leftHand.get())) {
        OVR_LOG("Failed to add Ultraleap left hand controller");
    }

    if (!vr::VRServerDriverHost()
             ->TrackedDeviceAdded("RightHand", vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, rightHand.get())) {
        OVR_LOG("Failed to add Ultraleap right hand controller");
    }
}

auto LeapDeviceProvider::DisconnectHandControllers() const -> void {
    OVR_LOG("Disconnecting Devices");
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(leftHand->Id(), kDeviceDisconnectedPose, sizeof(kDeviceDisconnectedPose));
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(rightHand->Id(), kDeviceDisconnectedPose, sizeof(kDeviceDisconnectedPose));
}

auto LeapDeviceProvider::DeviceDriverFromLeapId(const uint32_t deviceId) const -> const std::shared_ptr<LeapDeviceDriver>& {
    const auto& serialNumber = deviceSerialById.at(deviceId);
    const auto& leapDeviceDriver = deviceDriverBySerial.at(serialNumber);
    return leapDeviceDriver;
}
