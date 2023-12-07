#include "LeapDeviceProvider.h"

#include <algorithm>
#include <ranges>

#if defined(_WIN32)
#include <ShlObj_core.h>
#else
#include <pthread.h>
#endif

#include "OsUtils.h"
#include "VrUtils.h"
#include "VrLogging.h"

auto LeapDeviceProvider::Init(vr::IVRDriverContext* driver_context) -> vr::EVRInitError {
    VR_INIT_SERVER_DRIVER_CONTEXT(driver_context)

    // Load the driver settings
    settings_ = std::make_shared<LeapDriverSettings>();

    // Initialize the LeapC Connection
    constexpr LEAP_CONNECTION_CONFIG connectionConfig{
        sizeof(connectionConfig),
        eLeapConnectionConfig_MultiDeviceAware,
        nullptr,
        eLeapTrackingOrigin_DeviceCenter,
    };
    if (auto result = LeapCreateConnection(&connectionConfig, &leap_connection_); LEAP_FAILED(result)) {
        LOG_INFO("Failed to create connection to Ultraleap tracking service: {}", result);
        return vr::VRInitError_Driver_Failed;
    }

    if (auto result = LeapOpenConnection(leap_connection_); LEAP_FAILED(result)) {
        LeapDestroyConnection(leap_connection_);
        LOG_INFO("Failed to open connection to Ultraleap tracking service: {}", result);
        return vr::VRInitError_Driver_Failed;
    }

    // Start the service thread.
    is_running_ = true;
    service_thread_ = std::thread{&LeapDeviceProvider::ServiceMessageLoop, this};

    // Indicate that we have initialized successfully.
    return vr::VRInitError_None;
}

auto LeapDeviceProvider::Cleanup() -> void {
    // Request the thread termination and close the connection to ensure that LeapPollConnection doesn't hang.
    is_running_ = false;

    // Close all remaining device handles before closing the connection.
    leap_device_by_id_.clear();
    leap_device_driver_by_serial_.clear();
    LeapCloseConnection(leap_connection_);

    // Await the termination of the service thready by joining on it.
    if (service_thread_.joinable()) {
        service_thread_.join();
    }

    LeapDestroyConnection(leap_connection_);
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

    // Poll the runtime for events; for now we only care if the settings change
    vr::VREvent_t event{};
    while(vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
        switch (event.eventType) {
        default: continue;
        case vr::EVREventType::VREvent_OtherSectionSettingChanged: OtherSectionSettingsChanged(); break;
        }
    }
}

auto LeapDeviceProvider::OtherSectionSettingsChanged() const -> void {
    // Reload the settings
    settings_->LoadSettings();

    // Prompt a re-fetch of all the settings file variables.
    UpdateServiceAndDriverTrackingMode(settings_->TrackingMode(), std::nullopt);
}

auto LeapDeviceProvider::ShouldBlockStandbyMode() -> bool {
    return false;
}

auto LeapDeviceProvider::EnterStandby() -> void {
    LeapSetPause(leap_connection_, true);
}

auto LeapDeviceProvider::LeaveStandby() -> void {
    LeapSetPause(leap_connection_, false);
}

auto LeapDeviceProvider::ServiceMessageLoop() -> void {
    OsUtils::SetThreadName(service_thread_, "Ultraleap OpenVR Driver");

    for (LEAP_CONNECTION_MESSAGE msg; is_running_;) {
        if (auto result = LeapPollConnection(leap_connection_, 1000, &msg); LEAP_FAILED(result)) {
            if (result != eLeapRS_Timeout) {
                LOG_INFO("Failed to poll tracking connection: {}", result);
            }
            continue;
        }

        switch (msg.type) {
        default: continue;
        case eLeapEventType_Connection: is_connected_ = true; break;
        case eLeapEventType_ConnectionLost: is_connected_ = false; break;
        case eLeapEventType_Device: DeviceDetected(msg.device_id, msg.device_event); break;
        case eLeapEventType_DeviceLost: DeviceLost(msg.device_id, msg.device_event); break;
        case eLeapEventType_DeviceStatusChange: DeviceStatusChanged(msg.device_id, msg.device_status_change_event); break;
        case eLeapEventType_Tracking: TrackingFrame(msg.device_id, msg.tracking_event); break;
        case eLeapEventType_TrackingMode: TrackingModeChanged(msg.device_id, msg.tracking_mode_event); break;
        }
    }
}

auto LeapDeviceProvider::TrackingFrame([[maybe_unused]] const uint32_t device_id, const LEAP_TRACKING_EVENT* event) const -> void {
    // Pass the tracking frame to each virtual hand driver.
    for (const auto handDriver : {left_hand_.get(), right_hand_.get()}) {
        if (handDriver != nullptr) {
            handDriver->UpdateFromLeapFrame(event);
        }
    }

    // Pass the tracking frame to each virtual elbow driver.
    for (const auto elbowDriver : {left_elbow_.get(), right_elbow_.get()}) {
        if (elbowDriver != nullptr) {
            elbowDriver->UpdateFromLeapFrame(event);
        }
    }
}

auto LeapDeviceProvider::TrackingModeChanged(const uint32_t device_id, const LEAP_TRACKING_MODE_EVENT* event) const -> void {
    const auto& leapDevice = leap_device_by_id_.at(device_id);
    UpdateServiceAndDriverTrackingMode(settings_->TrackingMode(), leapDevice.get());
}

auto LeapDeviceProvider::DeviceDetected(const uint32_t /*device_id*/, const LEAP_DEVICE_EVENT* event) -> void {
    // WARNING!
    // In this context, deviceId will be 0 as this is a system message, for the ID of the affected device, use event->device.id.

    // Construct and open the device and track the serial number.
    const auto leapDevice = std::make_shared<LeapDevice>(leap_connection_, event->device);
    leap_device_by_id_.insert({event->device.id, leapDevice});

    // Create a device if it's the first time we've seen this serial, otherwise just notify that it's now active again.
    if (!leap_device_driver_by_serial_.contains(leapDevice->SerialNumber())) {
        CreateDeviceDriver(leapDevice);
    } else {
        ReconnectDeviceDriver(leapDevice);
    }

    // If this is the first device, construct the two hand-controllers.
    if (leap_device_by_id_.size() == 1 && left_hand_ == nullptr && right_hand_ == nullptr) {
        CreateHandControllers();
    }
}

auto LeapDeviceProvider::DeviceLost(const uint32_t /*device_id*/, const LEAP_DEVICE_EVENT* event) -> void {
    // WARNING!
    // In this context, deviceId will be 0 as this is a system message, for the ID of the affected device, use event->device.id.
    DisconnectDeviceDriver(event->device.id);

    // If all devices are now disconnected, indicate that the hands are no-longer trackable.
    if (leap_device_by_id_.empty()) {
        DisconnectHandControllers();
    }
}

auto LeapDeviceProvider::DeviceStatusChanged(const uint32_t /*device_id*/, const LEAP_DEVICE_STATUS_CHANGE_EVENT* event) -> void {
    // WARNING!
    // In this context, device_id will be 0 as this is a system message, for the ID of the affected device, use event->device.id.

    // Check for any of the error statuses and set a device error in that instance.
    if (event->status >= eLeapDeviceStatus_UnknownFailure) {
        vr::VRServerDriverHost()
            ->TrackedDevicePoseUpdated(DeviceDriverFromLeapId(event->device.id)->Id(), kDeviceErrorPose, sizeof(kDeviceErrorPose));
    }
}

auto LeapDeviceProvider::CreateDeviceDriver(const std::shared_ptr<LeapDevice>& leap_device) -> void {
    // Track the device by serial-number
    auto [leapDeviceDriver, _] = leap_device_driver_by_serial_.emplace(
        leap_device->SerialNumber(),
        std::make_shared<LeapDeviceDriver>(leap_device)
    );

    // Register the device driver.
    if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
            leap_device->SerialNumber().c_str(),
            vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference,
            leapDeviceDriver->second.get()
        )) {
        LOG_INFO("Failed to add new Ultraleap device: {}", leap_device->SerialNumber());
    }

    LOG_INFO("Added {} device with Serial: {}", leap_device->ProductId(), leap_device->SerialNumber());
}

auto LeapDeviceProvider::ReconnectDeviceDriver(const std::shared_ptr<LeapDevice>& leap_device) const -> void {
    // Update the driver for this device to reference the new underlying device.
    auto& leapDeviceDriver = leap_device_driver_by_serial_.at(leap_device->SerialNumber());
    leapDeviceDriver->SetLeapDevice(leap_device);

    // Register that the device is now connected again.
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(leapDeviceDriver->Id(), kDeviceConnectedPose, sizeof(kDeviceConnectedPose));

    LOG_INFO("Reconnected device with serial: {}", leap_device->SerialNumber());
}

auto LeapDeviceProvider::DisconnectDeviceDriver(const uint32_t device_id) -> void {
    // Register the device as disconnected.
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
        DeviceDriverFromLeapId(device_id)->Id(),
        kDeviceDisconnectedPose,
        sizeof(kDeviceDisconnectedPose)
    );

    // Remove the device by remember and log the serial number.
    const auto serialNumber = leap_device_by_id_.at(device_id)->SerialNumber();
    leap_device_by_id_.erase(device_id);
    LOG_INFO("Disconnected device with serial: {}", serialNumber);
}

auto LeapDeviceProvider::CreateHandControllers() -> void {
    left_hand_ = std::make_unique<LeapHandDriver>(settings_, eLeapHandType_Left);
    right_hand_ = std::make_unique<LeapHandDriver>(settings_, eLeapHandType_Right);

    if (!vr::VRServerDriverHost()
             ->TrackedDeviceAdded("LeftHand", vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, left_hand_.get())) {
        LOG_INFO("Failed to add Ultraleap left hand controller");
    }

    if (!vr::VRServerDriverHost()
             ->TrackedDeviceAdded("RightHand", vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, right_hand_.get())) {
        LOG_INFO("Failed to add Ultraleap right hand controller");
    }
    LOG_INFO("Added virtual hand devices");

    if (settings_->EnableElbowTrackers()) {
        left_elbow_ = std::make_unique<LeapElbowDriver>(settings_, eLeapHandType_Left);
        right_elbow_ = std::make_unique<LeapElbowDriver>(settings_, eLeapHandType_Right);

        if (!vr::VRServerDriverHost()
                 ->TrackedDeviceAdded("LeftElbow", vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker, left_elbow_.get())) {
            LOG_INFO("Failed to add Ultraleap left elbow controller");
                 }

        if (!vr::VRServerDriverHost()
                 ->TrackedDeviceAdded("RightElbow", vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker, right_elbow_.get())) {
            LOG_INFO("Failed to add Ultraleap right elbow controller");
                 }
        LOG_INFO("Added virtual elbow devices");
    }
}

auto LeapDeviceProvider::DisconnectHandControllers() const -> void {
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(left_hand_->Id(), kDeviceDisconnectedPose, sizeof(kDeviceDisconnectedPose));
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(right_hand_->Id(), kDeviceDisconnectedPose, sizeof(kDeviceDisconnectedPose));
    LOG_INFO("Disconnected virtual hand devices");

    if (left_elbow_ != nullptr && right_elbow_ != nullptr) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(left_elbow_->Id(), kDeviceDisconnectedPose, sizeof(kDeviceDisconnectedPose));
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(right_elbow_->Id(), kDeviceDisconnectedPose, sizeof(kDeviceDisconnectedPose));
        LOG_INFO("Disconnected virtual elbow devices");
    }
}

auto LeapDeviceProvider::DeviceDriverFromLeapId(const uint32_t device_id) const -> const std::shared_ptr<LeapDeviceDriver>& {
    const auto& serialNumber = leap_device_by_id_.at(device_id)->SerialNumber();
    const auto& leapDeviceDriver = leap_device_driver_by_serial_.at(serialNumber);
    return leapDeviceDriver;
}

auto LeapDeviceProvider::UpdateServiceAndDriverTrackingMode(const eLeapTrackingMode mode, const std::optional<LeapDevice*> device)
    const -> void {
    // If we've been given a specific device only change that one, otherwise change all.
    if (device.has_value()) {
        LeapSetTrackingModeEx(leap_connection_, device.value()->Handle(), mode);
    } else {
        for (const auto& val : std::views::values(leap_device_by_id_)) {
            LeapSetTrackingModeEx(leap_connection_, val->Handle(), mode);
        }
    }
}
