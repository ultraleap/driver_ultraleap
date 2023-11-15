#include "LeapDeviceProvider.h"

#include <algorithm>

#if defined(_WIN32)
#include <ShlObj_core.h>
#else
#include <pthread.h>
#endif

#include "OvrUtils.h"
#include "vrmath.h"

auto LeapDeviceProvider::Init(vr::IVRDriverContext* pDriverContext) -> vr::EVRInitError {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext)

    // Initialise logging subsystem.
    OvrLogging::Init();

    // Initialize the LeapC Connection
    constexpr LEAP_CONNECTION_CONFIG connectionConfig{
        sizeof(connectionConfig),
        eLeapConnectionConfig_MultiDeviceAware,
        nullptr,
        eLeapTrackingOrigin_DeviceCenter,
    };
    if (auto result = LeapCreateConnection(&connectionConfig, &leapConnection); LEAP_FAILED(result)) {
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
        leapDeviceById.clear();
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
        if (auto result = LeapPollConnection(leapConnection, 1000, &msg); LEAP_FAILED(result)) {
            if (result != eLeapRS_Timeout) {
                OVR_LOG("Failed to poll tracking connection: {}", result);
            }
            continue;
        }

        switch (msg.type) {
        default: continue;
        case eLeapEventType_Connection: isConnected = true; break;
        case eLeapEventType_ConnectionLost: isConnected = false; break;
        case eLeapEventType_Device: DeviceDetected(msg.device_id, msg.device_event); break;
        case eLeapEventType_DeviceLost: DeviceLost(msg.device_id, msg.device_event); break;
        case eLeapEventType_DeviceStatusChange: DeviceStatusChanged(msg.device_id, msg.device_status_change_event); break;
        case eLeapEventType_Tracking: TrackingFrame(msg.device_id, msg.tracking_event); break;
        case eLeapEventType_TrackingMode: TrackingModeChanged(msg.device_id, msg.tracking_mode_event); break;
        }
    }
}

auto LeapDeviceProvider::SetThreadName(const std::string_view& name) -> void {
    // Set the thread name (This has to utilize a platform specific method).
#if defined(_WIN32)
    const auto narrowName = std::string{name};
    const auto length = MultiByteToWideChar(CP_ACP, 0, narrowName.c_str(), static_cast<int>(narrowName.length()), nullptr, 0);
    auto wideName = std::wstring(length, 0);
    if (MultiByteToWideChar(CP_ACP, 0, narrowName.c_str(), static_cast<int>(narrowName.length()), wideName.data(), length) == 0
        || SetThreadDescription(serviceThread.native_handle(), wideName.c_str()) != S_OK) {
        OVR_LOG("Failed to set thread name to \"{}\"", name);
    }
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

auto LeapDeviceProvider::TrackingFrame(const uint32_t /*deviceId*/, const LEAP_TRACKING_EVENT* event) const -> void {
    auto leftHandPose = kDeviceConnectedPose;
    auto rightHandPose = kDeviceConnectedPose;

    // Process all the hands that are seen in the frame.
    for (auto i = 0; i < event->nHands; ++i) {
        const auto& hand = event->pHands[i];
        auto& pose = hand.type == eLeapHandType_Left ? leftHandPose : rightHandPose;

        // Space transform from LeapC -> OpenVR Space;
        pose.qWorldFromDriverRotation = HmdQuaternion_FromEulerAngles(0.0, M_PI / 2.0f, M_PI);
        pose.qDriverFromHeadRotation = HmdQuaternion_Identity;

        pose.vecPosition[0] = 0.001f * hand.palm.position.x;
        pose.vecPosition[1] = 0.001f * hand.palm.position.y;
        pose.vecPosition[2] = 0.001f * hand.palm.position.z;
        pose.qRotation = {
            hand.palm.orientation.w,
            hand.palm.orientation.x,
            hand.palm.orientation.y,
            hand.palm.orientation.z,
        };
        pose.result = vr::TrackingResult_Running_OK;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;

        // Calculate the time offset from this frame's timestamp.
        pose.poseTimeOffset = static_cast<float>(event->info.timestamp - LeapGetNow()) * std::micro::num / std::micro::den;

        (hand.type == eLeapHandType_Left ? leftHandPose : rightHandPose) = pose;
    }

    if (leftHand != nullptr) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(leftHand->Id(), leftHandPose, sizeof(leftHandPose));
    }

    if (rightHand != nullptr) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(rightHand->Id(), rightHandPose, sizeof(rightHandPose));
    }
}

auto LeapDeviceProvider::TrackingModeChanged(const uint32_t deviceId, const LEAP_TRACKING_MODE_EVENT* event) const -> void {
    if (event->current_tracking_mode != eLeapTrackingMode_HMD) {
        const auto& leapDevice = leapDeviceById.at(deviceId);
        OVR_LOG("Device {} is not currently in HMD mode, setting to HMD", leapDevice->SerialNumber());
        LeapSetTrackingModeEx(leapConnection, leapDevice->Handle(), eLeapTrackingMode_HMD);
    }
}

auto LeapDeviceProvider::DeviceDetected(const uint32_t /*deviceId*/, const LEAP_DEVICE_EVENT* event) -> void {
    // WARNING!
    // In this context, deviceId will be 0 as this is a system message, for the ID of the affected device, use event->device.id.

    // Construct and open the device and track the serial number.
    const auto leapDevice = std::make_shared<LeapDevice>(leapConnection, event->device);
    leapDeviceById.insert({event->device.id, leapDevice});

    // Create a device if it's the first time we've seen this serial, otherwise just notify that it's now active again.
    if (!deviceDriverBySerial.contains(leapDevice->SerialNumber())) {
        CreateDeviceDriver(leapDevice);
    } else {
        ReconnectDeviceDriver(leapDevice);
    }

    // If this is the first device, construct the two hand-controllers.
    if (leapDeviceById.size() == 1 && leftHand == nullptr && rightHand == nullptr) {
        CreateHandControllers();
    }
}

auto LeapDeviceProvider::DeviceLost(const uint32_t /*deviceId*/, const LEAP_DEVICE_EVENT* event) -> void {
    // WARNING!
    // In this context, deviceId will be 0 as this is a system message, for the ID of the affected device, use event->device.id.
    DisconnectDeviceDriver(event->device.id);

    // If all devices are now disconnected, indicate that the hands are no-longer trackable.
    if (leapDeviceById.empty()) {
        DisconnectHandControllers();
    }
}

auto LeapDeviceProvider::DeviceStatusChanged(uint32_t deviceId, const LEAP_DEVICE_STATUS_CHANGE_EVENT* event) -> void {
    // Check for any of the error statuses and set a device error in that instance.
    if (event->status >= eLeapDeviceStatus_UnknownFailure) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
            DeviceDriverFromLeapId(event->device.id)->Id(),
            kDeviceErrorPose,
            sizeof(kDeviceErrorPose));
    }
}

auto LeapDeviceProvider::CreateDeviceDriver(const std::shared_ptr<LeapDevice>& leapDevice) -> void {
    // Track the device by serial-number
    auto [leapDeviceDriver, _] = deviceDriverBySerial.emplace(
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

    OVR_LOG("Added {} device with Serial: {}", leapDevice->ProductId(), leapDevice->SerialNumber());
}

auto LeapDeviceProvider::ReconnectDeviceDriver(const std::shared_ptr<LeapDevice>& leapDevice) const -> void {
    // Update the driver for this device to reference the new underlying device.
    auto& leapDeviceDriver = deviceDriverBySerial.at(leapDevice->SerialNumber());
    leapDeviceDriver->SetLeapDevice(leapDevice);

    // Register that the device is now connected again.
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(leapDeviceDriver->Id(), kDeviceConnectedPose, sizeof(kDeviceConnectedPose));

    OVR_LOG("Reconnected device with serial: {}", leapDevice->SerialNumber());
}

auto LeapDeviceProvider::DisconnectDeviceDriver(const uint32_t deviceId) -> void {
    // Register the device as disconnected.
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
        DeviceDriverFromLeapId(deviceId)->Id(),
        kDeviceDisconnectedPose,
        sizeof(kDeviceDisconnectedPose)
    );

    // Remove the device by remember and log the serial number.
    const auto serialNumber = leapDeviceById.at(deviceId)->SerialNumber();
    leapDeviceById.erase(deviceId);
    OVR_LOG("Disconnected device with serial: {}", serialNumber);
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
    OVR_LOG("Added virtual hand devices");
}

auto LeapDeviceProvider::DisconnectHandControllers() const -> void {
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(leftHand->Id(), kDeviceDisconnectedPose, sizeof(kDeviceDisconnectedPose));
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(rightHand->Id(), kDeviceDisconnectedPose, sizeof(kDeviceDisconnectedPose));
    OVR_LOG("Disconnected virtual hand devices");
}

auto LeapDeviceProvider::DeviceDriverFromLeapId(const uint32_t deviceId) const -> const std::shared_ptr<LeapDeviceDriver>& {
    const auto& serialNumber = leapDeviceById.at(deviceId)->SerialNumber();
    const auto& leapDeviceDriver = deviceDriverBySerial.at(serialNumber);
    return leapDeviceDriver;
}
