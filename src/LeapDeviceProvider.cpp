#include "LeapDeviceProvider.h"

#if defined(_WIN32)
#include <ShlObj_core.h>
#else
#include <pthread.h>
#endif

#include "DriverLog.h"

vr::EVRInitError LeapDeviceProvider::Init(vr::IVRDriverContext* pDriverContext) {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    // Initialise logging subsystem.
    DriverLog::Init();

    // Initialize the LeapC Connection
    {
        eLeapRS result = LeapCreateConnection(nullptr, &leapConnection);
        if (LEAP_FAILED(result)) {
            OVR_LOG("Failed to create connection to Ultraleap tracking service: {}", result);
        }

        result = LeapOpenConnection(leapConnection);
        if (LEAP_FAILED(result)) {
            LeapDestroyConnection(leapConnection);
            OVR_LOG("Failed to open connection to Ultraleap tracking service: {}", result);
        }

        // Start the service thread.
        isRunning     = true;
        serviceThread = std::thread{&LeapDeviceProvider::ServiceMessageLoop, this};
    }

    // Indicate that we have initialized successfully.
    return vr::VRInitError_None;
}

void LeapDeviceProvider::Cleanup() {
    // Request the thread termination and close the connection to ensure that LeapPollConnection doesn't hang.
    {
        isRunning = false;

        // Close all remaining device handles before closing the connection.
        devices.clear();
        LeapCloseConnection(leapConnection);
    }

    if (serviceThread.joinable()) {
        serviceThread.join();
    }

    LeapDestroyConnection(leapConnection);
}

const char* const* LeapDeviceProvider::GetInterfaceVersions() {
    return vr::k_InterfaceVersions;
}

void LeapDeviceProvider::RunFrame() {
    // Update the base-station pose for each device.
//    for (auto [leapId, device] : devices) {
//        const auto devicePose = device->GetPose();
//        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device->GetId(), devicePose, sizeof(devicePose));
//    }
}

bool LeapDeviceProvider::ShouldBlockStandbyMode() {
    return false;
}

void LeapDeviceProvider::EnterStandby() {
    LeapSetPause(leapConnection, true);
}

void LeapDeviceProvider::LeaveStandby() {
    LeapSetPause(leapConnection, false);
}

void LeapDeviceProvider::ServiceMessageLoop() {
    SetThreadName("UltraleapTrackingThread");
    LEAP_CONNECTION_MESSAGE msg;

    for (LEAP_CONNECTION_MESSAGE msg; isRunning; ) {
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

void LeapDeviceProvider::SetThreadName(const std::string_view& name) {
    // Set the thread name (This has to utilize a platform specific method).
#if defined(_WIN32)
    auto narrowName = std::string{name};
    auto length     = MultiByteToWideChar(CP_ACP, 0, narrowName.c_str(), static_cast<int>(narrowName.length()), nullptr, 0);
    auto wideName   = std::wstring(length, 0);
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

void LeapDeviceProvider::TrackingFrame(const uint32_t deviceId, const LEAP_TRACKING_EVENT* event) {
    //
    //    // Add the frame to the relevant device for latency tracking and for hands if they are present.
    //    try {
    //        std::shared_ptr<LeapDevice> device;
    //        // Lock to get a device reference.
    //        {
    //            std::shared_lock lock{mutex};
    //            device = devices.at(deviceId);
    //        }
    //
    //        // Construct a frame with the correct OpenXR timestamps, latency and device pointer.
    //        Frame frame{
    //            clockRebaser.RebaseToXrTime(event->info.timestamp),
    //            clockRebaser.RebaseToXrTime(LeapGetNow()) - frame.timestamp,
    //            device,
    //        };
    //
    //        // Only copy the frame data if there are hands in it.
    //        if (event->nHands > 0) {
    //            // Create a copy of the tracking frame
    //            const size_t handsSize = sizeof(LEAP_HAND) * event->nHands;
    //            const size_t frameSize = sizeof(*event) + (handsSize);
    //
    //            // Copy the frame data and the hands array.
    //            frame.data  = std::shared_ptr<LEAP_TRACKING_EVENT>{reinterpret_cast<LEAP_TRACKING_EVENT*>(new
    //            uint8_t[frameSize])}, *frame.data = *event; frame.data->pHands = reinterpret_cast<LEAP_HAND*>(frame.data.get() +
    //            1); std::copy(event->pHands, event->pHands + event->nHands, frame.data->pHands);
    //
    //        }
    //
    //        // Add the frame.
    //        device->AddFrame(std::move(frame));
    //    } catch (const std::out_of_range& err) {
    //        // Received a frame for a device we are not actively tracking: Ignore.
    //    }
}

void LeapDeviceProvider::TrackingModeChanged(const uint32_t deviceId, const LEAP_TRACKING_MODE_EVENT* event) {
    if (event->current_tracking_mode != eLeapTrackingMode_HMD) {
        OVR_LOG("Device is not currently in HMD mode, setting to HMD");
        LeapSetTrackingMode(leapConnection, eLeapTrackingMode_HMD);
    }
}

void LeapDeviceProvider::DeviceDetected(const uint32_t deviceId, const LEAP_DEVICE_EVENT* event) {
    // WARNING!
    // In this context, deviceId will be 0 as this is a system message, for the ID of the affected device, use event->device.id.

    // Open the device handle.
    LEAP_DEVICE leapDevice;
    auto        result = LeapOpenDevice(event->device, &leapDevice);
    if (LEAP_FAILED(result)) {
        OVR_LOG("Failed to open Ultraleap device: {}", result);
    }

    // Retrieve the serial number.
    std::string      leapSerial = "Unknown";
    LEAP_DEVICE_INFO leapDeviceInfo{sizeof(leapDeviceInfo)};

    result = LeapGetDeviceInfo(leapDevice, &leapDeviceInfo);
    if (LEAP_SUCCEEDED(result)) {
        leapSerial.resize(leapDeviceInfo.serial_length - 1);
        leapDeviceInfo.serial = leapSerial.data();
        result                = LeapGetDeviceInfo(leapDevice, &leapDeviceInfo);
        if (LEAP_FAILED(result)) {
            OVR_LOG("Failed to read Ultraleap tracking device serial: {}", result);
        }
    } else {
        OVR_LOG("Failed to read Ultraleap device serial length: {}", result);
    }

    // If this is the first device, construct the two hand-controllers.
    if (devices.empty()) {
//        leftHand = std::make_unique<LeapHandDriver>(eLeapHandType_Left);
//        rightHand = std::make_unique<LeapHandDriver>(eLeapHandType_Right);
//
//        if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
//                "LeftHand",
//                vr::ETrackedDeviceClass::TrackedDeviceClass_Controller,
//                leftHand.get())) {
//            OVR_LOG("Failed to add Ultraleap left hand controller");
//        }
//
//        if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
//                "RightHand",
//                vr::ETrackedDeviceClass::TrackedDeviceClass_Controller,
//                rightHand.get())) {
//            OVR_LOG("Failed to add Ultraleap right hand controller");
//        }
    } else {
        // Check if this device has been seen before and if so, track the new id we've seen it on.
        for (auto [_, device] : devices) {
            if (leapSerial == device->GetSerialNumber()) {
                NotifyDeviceConnected(device);
                devices.insert({event->device.id, device});
                return;
            }
        }
    }

    // Otherwise, construct and add a new device.
    auto device = std::make_shared<LeapDeviceDriver>(leapDevice, leapDeviceInfo, leapSerial);
    if (vr::VRServerDriverHost()->TrackedDeviceAdded(
            leapSerial.c_str(),
            vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference,
            device.get())) {
        NotifyDeviceConnected(device);
        devices.insert({event->device.id, std::move(device)});
    } else {
        OVR_LOG("Failed to add new Ultraleap device: {}", leapSerial);
    }
}

void LeapDeviceProvider::DeviceLost(const uint32_t deviceId, const LEAP_DEVICE_EVENT* event) {
    // WARNING!
    // In this context, deviceId will be 0 as this is a system message, for the ID of the affected device, use event->device.id.
    NotifyDeviceDisconnected(devices.at(event->device.id));
}

void LeapDeviceProvider::NotifyDeviceConnected(const std::shared_ptr<LeapDeviceDriver>& device) {
    // Mark this device as running correctly and connected, but with no valid pose.
    vr::DriverPose_t pose{0};
    pose.result            = vr::TrackingResult_Running_OK;
    pose.poseIsValid       = false;
    pose.deviceIsConnected = true;
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device->GetId(), pose, sizeof(pose));
}

void LeapDeviceProvider::NotifyDeviceDisconnected(const std::shared_ptr<LeapDeviceDriver>& device) {
    // Indicate that this device is no longer connected with a pose that has `deviceIsConnected` set to false.
    vr::DriverPose_t pose{0};
    pose.result            = vr::TrackingResult_Running_OK;
    pose.poseIsValid       = false;
    pose.deviceIsConnected = false;
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device->GetId(), pose, sizeof(pose));
}
