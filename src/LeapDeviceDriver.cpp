#include "LeapDeviceDriver.h"

#include "OvrUtils.h"
#include "vrmath.h"

LeapDeviceDriver::LeapDeviceDriver(const std::shared_ptr<LeapDevice>& leapDevice)
    : id(vr::k_unTrackedDeviceIndexInvalid),
      leapDevice{leapDevice} {
}

vr::EVRInitError LeapDeviceDriver::Activate(const uint32_t unObjectId) {
    // Store our id for layer use.
    id = unObjectId;

    // Set up our property container and some utility functions.
    auto properties = OvrProperties::FromDeviceId(id);

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
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id, kDeviceConnectedPose, sizeof(kDeviceConnectedPose));

    // Return initialization success
    return vr::VRInitError_None;
}

void LeapDeviceDriver::Deactivate() {
    id = vr::k_unTrackedDeviceIndexInvalid;
}

void LeapDeviceDriver::EnterStandby() {
}

void* LeapDeviceDriver::GetComponent(const char* pchComponentNameAndVersion) {
    const auto componentNameAndVersion = std::string_view{pchComponentNameAndVersion};

    if (componentNameAndVersion == vr::ITrackedDeviceServerDriver_Version) {
        return dynamic_cast<vr::ITrackedDeviceServerDriver*>(this);
    }

    return nullptr;
}

void LeapDeviceDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {
    if (id != vr::k_unTrackedDeviceIndexInvalid) {
        // TODO: Implement any required debugging here, for now just clear the buffer.
        if (unResponseBufferSize > 0) {
            std::memset(pchResponseBuffer, 0, unResponseBufferSize);
        }
    }
}

vr::DriverPose_t LeapDeviceDriver::GetPose() {
    // Initialise our tracker structure.
    vr::DriverPose_t trackerPose{0};
    trackerPose.qWorldFromDriverRotation.w = 1.f;
    trackerPose.qDriverFromHeadRotation.w = 1.0f;

    // Get the HMD pose as the tracker is mounted there in most scenarios.
    vr::TrackedDevicePose_t hmdPose{};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, &hmdPose, 1);
    const auto hmdPosition = HmdVector3_From34Matrix(hmdPose.mDeviceToAbsoluteTracking);
    const auto hmdOrientation = HmdQuaternion_FromMatrix(hmdPose.mDeviceToAbsoluteTracking);

    // Configurable tracker position.
    const auto trackerTilt = vr::HmdQuaternion_t{1.0f, 0.0f, 0.0f, 0.0f};
    const auto trackerOffset = vr::HmdVector3_t{0.0f, 0.0f, -0.08f};

    // Apply the offsets and update the position.
    const auto trackerOrientation = hmdOrientation * trackerTilt;
    const auto trackerPosition = hmdPosition + (trackerOffset * hmdOrientation);
    trackerPose.qRotation = trackerOrientation;
    trackerPose.vecPosition[0] = trackerPosition.v[0];
    trackerPose.vecPosition[1] = trackerPosition.v[1];
    trackerPose.vecPosition[2] = trackerPosition.v[2];

    // Set the tracking status.
    trackerPose.poseIsValid = hmdPose.bPoseIsValid;
    trackerPose.deviceIsConnected = true; // TODO: Update this if the device is lost? (Surely we just remove it?)
    trackerPose.result = vr::TrackingResult_Running_OK;

    // Return the pose of the tracking device.
    return trackerPose;
}

void LeapDeviceDriver::SetDeviceModelProperties(const OvrProperties& properties) const {
    switch (leapDevice->ProductId()) {
    case eLeapDevicePID_Peripheral: {
        OVR_LOG("Setting device as LMC");
        properties.Set(vr::Prop_ModelNumber_String, "LMC");
        properties.Set(vr::Prop_FieldOfViewLeftDegrees_Float, 140.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewRightDegrees_Float, 140.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewTopDegrees_Float, 120.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewBottomDegrees_Float, 120.0f / 2.0f);
        properties.Set(vr::Prop_TrackingRangeMinimumMeters_Float, 0.1f);
        properties.Set(vr::Prop_TrackingRangeMaximumMeters_Float, 0.8f);
        break;
    }
    case eLeapDevicePID_Rigel:
    case eLeapDevicePID_SIR170: {
        OVR_LOG("Setting device as SIR-170");
        properties.Set(vr::Prop_ModelNumber_String, "SIR170");
        properties.Set(vr::Prop_FieldOfViewLeftDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewRightDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewTopDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewBottomDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_TrackingRangeMinimumMeters_Float, 0.1f);
        properties.Set(vr::Prop_TrackingRangeMaximumMeters_Float, 1.0f);
    }
    case eLeapDevicePID_3Di: {
        OVR_LOG("Setting device as 3Di");
        properties.Set(vr::Prop_ModelNumber_String, "3Di");
        properties.Set(vr::Prop_FieldOfViewLeftDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewRightDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewTopDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewBottomDegrees_Float, 170.0f / 2.0f);
        properties.Set(vr::Prop_TrackingRangeMinimumMeters_Float, 0.1f);
        properties.Set(vr::Prop_TrackingRangeMaximumMeters_Float, 1.0f);
        break;
    }
    case eLeapDevicePID_LMC2: {
        OVR_LOG("Setting device as LMC2");
        properties.Set(vr::Prop_ModelNumber_String, "LMC2");
        properties.Set(vr::Prop_FieldOfViewLeftDegrees_Float, 160.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewRightDegrees_Float, 160.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewTopDegrees_Float, 160.0f / 2.0f);
        properties.Set(vr::Prop_FieldOfViewBottomDegrees_Float, 160.0f / 2.0f);
        properties.Set(vr::Prop_TrackingRangeMinimumMeters_Float, 0.1f);
        properties.Set(vr::Prop_TrackingRangeMaximumMeters_Float, 1.1f);
        break;
    }
    default: {
        OVR_LOG("Setting device as Unknown");
    }
    }
}