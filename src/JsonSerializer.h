#pragma once

#include "openvr_driver.h"

#include <optional>
#include <variant>
#include <string>
#include <unordered_map>

enum class InputPaths {
    SYSTEM_MENU,
    PROXIMITY,
    PINCH,
    GRIP,
    INDEX_FINGER,
    MIDDLE_FINGER,
    RING_FINGER,
    PINKY_FINGER,
    UNKNOWN
};

class DebugRequestJson{
public:
    using InputValue = std::variant<bool, float, vr::HmdVector2_t>;
    using SettingsValue = std::variant<bool, float>;

    class InputEntry {
    public:
        InputEntry(std::string pathStr, InputValue val) : path_{std::move(pathStr)}, value_{val} {};
        const std::string path_;
        const InputValue value_;
    };

    class SettingsEntry {
        SettingsEntry(std::string key, SettingsValue val) : key_{std::move(key)}, value_{val} {};
        const std::string key_;
        const SettingsValue value_;
    };

    static auto Parse(const char* string) -> std::optional<DebugRequestJson>;

    std::unordered_map<InputPaths, InputEntry> inputs_;
    std::vector<SettingsEntry> settings_;
private:
    DebugRequestJson() = default;
    DebugRequestJson(std::unordered_map<InputPaths, InputEntry> inputValues, std::vector<SettingsEntry> settingValues)
        : inputs_{std::move(inputValues)},
          settings_{std::move(settingValues)} {};
};