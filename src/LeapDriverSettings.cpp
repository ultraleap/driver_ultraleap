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


auto LeapDriverSettings::UpdateTrackingMode(const eLeapTrackingMode value) -> void {
    if (tracking_mode_ != value) {
        tracking_mode_ = value;

        switch (value) {
        case eLeapTrackingMode_Desktop: VrSettings::Set("tracking_mode", "desktop"); break;
        case eLeapTrackingMode_HMD:
        default: VrSettings::Set("tracking_mode", "hmd");
        }
    }

}

auto LeapDriverSettings::UpdateHmdTrackerOffset(const VrVec3& value) -> void {
    if (!IsEqual(hmd_tracker_offset_, value)) {
        hmd_tracker_offset_ = value;
        VrSettings::Set("hmd_tracker_offset_x", static_cast<float>(value.x));
        VrSettings::Set("hmd_tracker_offset_y", static_cast<float>(value.y));
        VrSettings::Set("hmd_tracker_offset_z", static_cast<float>(value.z));
    }
}

auto LeapDriverSettings::UpdateDesktopTrackerOffset(const VrVec3& value) -> void {
    if (!IsEqual(desktop_tracker_offset_, value)) {
        desktop_tracker_offset_ = value;
        VrSettings::Set("desktop_tracker_offset_x", static_cast<float>(value.x));
        VrSettings::Set("desktop_tracker_offset_y", static_cast<float>(value.y));
        VrSettings::Set("desktop_tracker_offset_z", static_cast<float>(value.z));
    }
}

auto LeapDriverSettings::UpdateEnableElbowTrackers(const bool value) -> void {
    if (enable_elbow_trackers_ != value) {
        enable_elbow_trackers_ = value;
        VrSettings::Set("enable_elbow_trackers", value);
        if (!is_restarting) {
            is_restarting = true;
            vr::VRServerDriverHost()->RequestRestart(restart_text, nullptr, nullptr, nullptr);
        }
    }
}

auto LeapDriverSettings::UpdateExternalInputOnly(const bool value) -> void {
    if (external_input_only_ != value) {
        external_input_only_ = value;
        VrSettings::Set("external_input_only", value);
    }
}

auto LeapDriverSettings::UpdateExtendedHandProfile(const bool value) -> void {
    if (extended_hand_profile_ != value) {
        extended_hand_profile_ = value;
        VrSettings::Set("extended_hand_profile", value);
        if (!is_restarting) {
            is_restarting = true;
            vr::VRServerDriverHost()->RequestRestart(restart_text, nullptr, nullptr, nullptr);
        }
    }

}

auto LeapDriverSettings::IsEqual(const VrVec3& lhs, const VrVec3& rhs) -> bool {
    // Any changes below a micrometer we can safely ignore as that precision is generally too high for the application here
    const auto eps = 1e-6;

    return
    std::abs(lhs.x - rhs.x) < eps &&
    std::abs(lhs.y - rhs.y) < eps &&
    std::abs(lhs.z - rhs.z) < eps;
}
