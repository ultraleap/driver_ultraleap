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
    [[nodiscard]] auto InputFromDriver() const -> bool { return input_from_driver_; }

    auto UpdateTrackingMode(const eLeapTrackingMode value) -> void { tracking_mode_ = value; }
    auto UpdateHmdTrackerOffset(VrVec3 value) -> void { hmd_tracker_offset_ = value; }
    auto UpdateDesktopTrackerOffset(VrVec3 value) -> void { desktop_tracker_offset_ = value; }
    auto UpdateEnableElbowTrackers(bool value) -> void { enable_elbow_trackers_ = value; }
    auto UpdateInputFromDriver(bool value) -> void { input_from_driver_ = value; }

  private:
    std::atomic<eLeapTrackingMode> tracking_mode_;
    std::atomic<VrVec3> hmd_tracker_offset_{};
    std::atomic<VrVec3> desktop_tracker_offset_{};
    std::atomic<bool> enable_elbow_trackers_{};
    std::atomic<bool> input_from_driver_{};
};
