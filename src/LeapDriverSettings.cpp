#include "LeapDriverSettings.h"

#include "VrUtils.h"
#include "VrLogging.h"

LeapDriverSettings::LeapDriverSettings() {
    LoadSettings();
}

auto LeapDriverSettings::LoadSettings() -> void {
    if (const auto tracking_mode = VrSettings::Get<std::string>("tracking_mode"); tracking_mode == "hmd") {
        tracking_mode_ = eLeapTrackingMode_HMD;
    } else if (tracking_mode == "desktop") {
        tracking_mode_ = eLeapTrackingMode_Desktop;
    } else {
        // Default to HMD if this is set incorrectly.
        LOG_INFO(R"(Unrecogonised setting for "tracking_mode": "{}" (should be "hmd" or "desktop"))", tracking_mode);
        tracking_mode_ = eLeapTrackingMode_HMD;
    }

    hmd_tracker_offset_ = VrVec3{
        VrSettings::Get<float>("hmd_tracker_offset_x"),
        VrSettings::Get<float>("hmd_tracker_offset_y"),
        VrSettings::Get<float>("hmd_tracker_offset_z"),
    };

    desktop_tracker_offset_ = VrVec3{
        VrSettings::Get<float>("desktop_tracker_offset_x"),
        VrSettings::Get<float>("desktop_tracker_offset_y"),
        VrSettings::Get<float>("desktop_tracker_offset_z"),
    };

    enable_elbow_trackers_ = VrSettings::Get<bool>("enable_elbow_trackers");
    external_input_only_ = VrSettings::Get<bool>("external_input_only");
    extended_hand_profile_ = VrSettings::Get<bool>("extended_hand_profile");
}
