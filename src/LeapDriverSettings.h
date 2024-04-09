#pragma once
#include "VrMaths.h"

#include <atomic>

#include <LeapC.h>
#include "VrUtils.h"

class LeapDriverSettings {
  public:
    LeapDriverSettings();
    auto LoadSettings() -> void;

    [[nodiscard]] auto TrackingMode() const -> eLeapTrackingMode { return tracking_mode_; }
    [[nodiscard]] auto HmdTrackerOffset() const -> VrVec3 { return hmd_tracker_offset_; }
    [[nodiscard]] auto DesktopTrackerOffset() const -> VrVec3 { return desktop_tracker_offset_; }
    [[nodiscard]] auto EnableElbowTrackers() const -> bool { return enable_elbow_trackers_; }
    [[nodiscard]] auto ExternalInputOnly() const -> bool { return external_input_only_ ; }
    [[nodiscard]] auto ExtendedHandProfile() const -> bool { return extended_hand_profile_ ; }

    auto UpdateTrackingMode(eLeapTrackingMode value) -> void;
    auto UpdateHmdTrackerOffset(const VrVec3& value) -> void;
    auto UpdateDesktopTrackerOffset(const VrVec3& value) -> void;
    auto UpdateEnableElbowTrackers(bool value) -> void;
    auto UpdateExternalInputOnly(bool value) -> void;
    auto UpdateExtendedHandProfile(bool value) -> void;

  private:
    static auto IsEqual(const VrVec3& lhs, const VrVec3& rhs) -> bool;

    bool is_restarting = false;
    const char* restart_text = "Pending changes to the settings require a restart to apply.";
    std::atomic<eLeapTrackingMode> tracking_mode_;
    std::atomic<VrVec3> hmd_tracker_offset_{};
    std::atomic<VrVec3> desktop_tracker_offset_{};
    std::atomic<bool> enable_elbow_trackers_{};
    std::atomic<bool> external_input_only_{};
    std::atomic<bool> extended_hand_profile_{};
};
