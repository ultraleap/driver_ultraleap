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
    const auto p = VrDeviceProperties::FromDeviceId(id_);
    try {

        // Setup manufacture/device information.
        p.Set(vr::Prop_ManufacturerName_String, "Ultraleap");

        // Set up the base-station properties as a tracking reference point.
        p.Set(vr::Prop_CanWirelessIdentify_Bool, false);
        p.Set(vr::Prop_IsOnDesktop_Bool, false);
        p.Set(vr::Prop_NeverTracked_Bool, false);
        p.Set(vr::Prop_WillDriftInYaw_Bool, false);

        // Don't locate the device in 3D space.
        p.Set(vr::Prop_NeverTracked_Bool, true);

        // Setup details of the FoV and range depending on device type.
        SetDeviceModelProperties(p);

        // Mark this device as running correctly and connected, but with no valid pose.
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, kDeviceConnectedPose, sizeof(kDeviceConnectedPose));
    } catch (const std::runtime_error& error) {
        LOG_INFO("Failed to initialize LeapDeviceDriver: {}", error.what());
        return vr::VRInitError_Driver_Failed;
    }

    // Indicate that this driver has initialized successfully.
    return vr::VRInitError_None;
}

auto LeapDeviceDriver::Deactivate() -> void {
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
    vr::TrackedDevicePose_t hmd_pose{};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, &hmd_pose, 1);
    const auto hmd_position = HmdVector3_From34Matrix(hmd_pose.mDeviceToAbsoluteTracking);
    const auto hmd_orientation = HmdQuaternion_FromMatrix(hmd_pose.mDeviceToAbsoluteTracking);

    // Configurable tracker position.
    constexpr auto tracker_tilt = vr::HmdQuaternion_t{1.0f, 0.0f, 0.0f, 0.0f};
    constexpr auto tracker_offset = vr::HmdVector3_t{0.0f, 0.0f, -0.08f};

    // Apply the offsets and update the position.
    const auto tracker_orientation = hmd_orientation * tracker_tilt;
    const auto [v] = hmd_position + tracker_offset * hmd_orientation;
    tracker_pose.qRotation = tracker_orientation;
    tracker_pose.vecPosition[0] = v[0];
    tracker_pose.vecPosition[1] = v[1];
    tracker_pose.vecPosition[2] = v[2];

    // Set the tracking status.
    tracker_pose.poseIsValid = hmd_pose.bPoseIsValid;
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