#include "LeapDriverSettings.h"

#include "VrUtils.h"

LeapDriverSettings::LeapDriverSettings() : tracking_mode_{} {
    LoadSettings();
}

auto LeapDriverSettings::LoadSettings() -> void {
    const auto tracking_mode = VrSettings::Get<std::string>("tracking_mode");
    if (tracking_mode == "hmd") {
        tracking_mode_ = eLeapTrackingMode_HMD;
    } else if (tracking_mode == "desktop") {
        tracking_mode_ = eLeapTrackingMode_Desktop;
    } else {
        // Default to HMD if this is set incorrectly.
        LOG_INFO("Unrecogonised setting for \"tracking_mode\": \"{}\" (should be \"hmd\" or \"desktop\")", tracking_mode);
        tracking_mode_ = eLeapTrackingMode_HMD;
    }

    hmd_tracker_offset_ = VrVec3{
        VrSettings::Get<float>("hmd_offset_x"),
        VrSettings::Get<float>("hmd_offset_y"),
        VrSettings::Get<float>("hmd_offset_z"),
    };

    desktop_tracker_offset_ = VrVec3{
        VrSettings::Get<float>("desktop_offset_x"),
        VrSettings::Get<float>("desktop_offset_y"),
        VrSettings::Get<float>("desktop_offset_z"),
    };
}