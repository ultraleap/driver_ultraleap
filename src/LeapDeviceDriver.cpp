#include "LeapDeviceDriver.h"

#include "VrUtils.h"
#include "VrLogging.h"

LeapDeviceDriver::LeapDeviceDriver(const std::shared_ptr<LeapDevice>& leap_device)
    : id_(vr::k_unTrackedDeviceIndexInvalid),
      leap_device_{leap_device} {
}

auto LeapDeviceDriver::Activate(const uint32_t object_id) -> vr::EVRInitError {
    // Store our id for layer use.
    id_ = object_id;

    // Set up our property container and some utility functions.
    try {
        const auto properties = VrDeviceProperties::FromDeviceId(id_);

        // Setup manufacture/device information.
        properties.Set(vr::Prop_ManufacturerName_String, "Ultraleap");

        // Set up the base-station properties as a tracking reference point.
        properties.Set(vr::Prop_CanWirelessIdentify_Bool, false);
        properties.Set(vr::Prop_IsOnDesktop_Bool, false);
        properties.Set(vr::Prop_NeverTracked_Bool, false);
        properties.Set(vr::Prop_WillDriftInYaw_Bool, false);

        // Don't locate the device in 3D space.
        properties.Set(vr::Prop_NeverTracked_Bool, true);

        // Setup details of the FoV and range depending on device type.
        SetDeviceModelProperties(properties);

        // Mark this device as running correctly and connected, but with no valid pose.
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, kDeviceConnectedPose, sizeof(kDeviceConnectedPose));
    } catch (const std::runtime_error& error) {
        LOG_INFO("Failed to initialize LeapDeviceDriver: {}", error.what());
        return vr::VRInitError_Driver_Failed;
    }

    // Indicate that this driver has initialized successfully.
    active_ = true;
    return vr::VRInitError_None;
}

auto LeapDeviceDriver::Deactivate() -> void {
    active_ = false;
    id_ = vr::k_unTrackedDeviceIndexInvalid;
}

auto LeapDeviceDriver::EnterStandby() -> void {
}

auto LeapDeviceDriver::GetComponent(const char* component_name_and_version) -> void* {
    if (std::string_view{component_name_and_version} == vr::ITrackedDeviceServerDriver_Version) {
        return dynamic_cast<vr::ITrackedDeviceServerDriver*>(this);
    }

    return nullptr;
}

auto LeapDeviceDriver::DebugRequest(const char* request, char* response_buffer, const uint32_t response_buffer_size) -> void {
    if (id_ != vr::k_unTrackedDeviceIndexInvalid) {
        // TODO: Implement any required debugging here, for now just clear the buffer.
        if (response_buffer_size > 0) {
            std::memset(response_buffer, 0, response_buffer_size);
        }
    }
}

auto LeapDeviceDriver::GetPose() -> vr::DriverPose_t {
    // Initialise our tracker structure.
    vr::DriverPose_t tracker_pose{0};
    tracker_pose.qWorldFromDriverRotation.w = 1.f;
    tracker_pose.qDriverFromHeadRotation.w = 1.0f;

    // Get the HMD pose as the tracker is mounted there in most scenarios.
    const auto hmd_pose = HmdPose::Get();

    // Configurable tracker position.
    const auto tracker_tilt = VrQuat::FromEulerAngles(0, 0, 0);
    const auto tracker_offset = VrVec3{0.0f, 0.0f, -0.08f};

    // Apply the offsets and update the position.
    const auto tracker_orientation = hmd_pose.Orientation() * tracker_tilt;
    const auto tracker_position = hmd_pose.Position() + tracker_offset * hmd_pose.Orientation();
    tracker_pose.qRotation = tracker_orientation;
    tracker_position.CopyToArray(tracker_pose.vecPosition);

    // Set the tracking status.
    tracker_pose.poseIsValid = hmd_pose.IsValid();
    tracker_pose.deviceIsConnected = true; // TODO: Update this if the device is lost? (Surely we just remove it?)
    tracker_pose.result = vr::TrackingResult_Running_OK;

    // Return the pose of the tracking device.
    return tracker_pose;
}

auto LeapDeviceDriver::SetDeviceModelProperties(const VrDeviceProperties& properties) const -> void {
    LOG_INFO("Setting device as {}", leap_device_->ProductId());
    switch (leap_device_->ProductId()) {
    case eLeapDevicePID_Peripheral: {
        properties.Set(vr::Prop_ModelNumber_String, "lmc");
        properties.Set(vr::Prop_FieldOfViewLeftDegrees_Float, 140.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewRightDegrees_Float, 140.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewTopDegrees_Float, 120.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewBottomDegrees_Float, 120.0f / 2.0f);
        properties.Set(vr::Prop_TrackingRangeMinimumMeters_Float, 0.1f);
        properties.Set(vr::Prop_TrackingRangeMaximumMeters_Float, 0.8f);
        break;
    }
    case eLeapDevicePID_LMC2: {
        properties.Set(vr::Prop_ModelNumber_String, "lmc2");
        properties.Set(vr::Prop_FieldOfViewLeftDegrees_Float, 160.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewRightDegrees_Float, 160.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewTopDegrees_Float, 160.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewBottomDegrees_Float, 160.0f / 2.0f);
        properties.Set(vr::Prop_TrackingRangeMinimumMeters_Float, 0.1f);
        properties.Set(vr::Prop_TrackingRangeMaximumMeters_Float, 1.1f);
        break;
    }
    case eLeapDevicePID_Rigel:
    case eLeapDevicePID_SIR170: {
        properties.Set(vr::Prop_ModelNumber_String, "sir170");
        properties.Set(vr::Prop_FieldOfViewLeftDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewRightDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewTopDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewBottomDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_TrackingRangeMinimumMeters_Float, 0.1f);
        properties.Set(vr::Prop_TrackingRangeMaximumMeters_Float, 1.0f);
        break;
    }
    case eLeapDevicePID_3Di: {
        properties.Set(vr::Prop_ModelNumber_String, "3di");
        properties.Set(vr::Prop_FieldOfViewLeftDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewRightDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewTopDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewBottomDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_TrackingRangeMinimumMeters_Float, 0.1f);
        properties.Set(vr::Prop_TrackingRangeMaximumMeters_Float, 1.0f);
        break;
    }
    default: {
        properties.Set(vr::Prop_ModelNumber_String, "unknown");
        break;
    }
    }
}