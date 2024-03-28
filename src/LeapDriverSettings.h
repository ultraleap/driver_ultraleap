#pragma once
#include "VrMaths.h"

#include <atomic>

#include <LeapC.h>

class LeapDriverSettings {
  public:
    LeapDriverSettings();
    auto LoadSettings() -> void;

    [[nodiscard]] auto TrackingMode() const -> eLeapTrackingMode { return tracking_mode_; }
    [[nodiscard]] auto HmdTrackerOffset() const -> VrVec3 { return hmd_tracker_offset_; }
    [[nodiscard]] auto DesktopTrackerOffset() const -> VrVec3 { return desktop_tracker_offset_; }
    [[nodiscard]] auto EnableElbowTrackers() const -> bool { return enable_elbow_trackers_; }
    [[nodiscard]] auto ExternalInputOnly() const -> bool { return external_input_only ; }
    [[nodiscard]] auto ExtendedHandProfile() const -> bool { return extended_hand_profile ; }

    auto UpdateTrackingMode(const eLeapTrackingMode value) -> void { tracking_mode_ = value; }
    auto UpdateHmdTrackerOffset(const VrVec3& value) -> void { hmd_tracker_offset_ = value; }
    auto UpdateDesktopTrackerOffset(const VrVec3& value) -> void { desktop_tracker_offset_ = value; }
    auto UpdateEnableElbowTrackers(const bool value) -> void { enable_elbow_trackers_ = value; }
    auto UpdateExternalInputOnly(const bool value) -> void { external_input_only  = value; }
    auto UpdateExtendedHandProfile(const bool value) -> void { extended_hand_profile  = value; }

  private:
    std::atomic<eLeapTrackingMode> tracking_mode_;
    std::atomic<VrVec3> hmd_tracker_offset_{};
    std::atomic<VrVec3> desktop_tracker_offset_{};
    std::atomic<bool> enable_elbow_trackers_{};
    std::atomic<bool> external_input_only{};
    std::atomic<bool> extended_hand_profile{};
};
