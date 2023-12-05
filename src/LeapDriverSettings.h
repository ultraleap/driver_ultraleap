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

  private:
    std::atomic<eLeapTrackingMode> tracking_mode_;
    std::atomic<VrVec3> hmd_tracker_offset_{};
    std::atomic<VrVec3> desktop_tracker_offset_{};
};
