#pragma once

#include "VrMaths.h"
#include "openvr_driver.h"

#include <optional>
#include <utility>
#include <variant>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>

enum class InputPaths {
    SYSTEM_MENU,
    PROXIMITY,
    PINCH,
    GRIP,
    INDEX_FINGER,
    MIDDLE_FINGER,
    RING_FINGER,
    PINKY_FINGER,
    TIME_OFFSET,
    UNKNOWN
};

using InputValue = std::variant<bool, float, vr::HmdVector2_t>;
using SettingsValue = std::variant<bool, float, VrVec3, std::string>;

class DebugRequestPayload{
public:
    class InputEntry {
    public:
        InputEntry(std::string_view pathStr, std::string_view key, InputValue val) : full_path_{pathStr}, key_{key}, value_{val} {};
        const std::string full_path_;
        const std::string key_;
        const InputValue value_;
    };

    class SettingsEntry {
    public:
        SettingsEntry(std::string_view keyStr, SettingsValue val) : key_{keyStr}, value_{std::move(val)} {};
        const std::string key_;
        const SettingsValue value_;
    };

    static auto Parse(const char* json_string) -> std::optional<DebugRequestPayload>;

    std::unordered_map<InputPaths, InputEntry> inputs_;
    std::vector<SettingsEntry> settings_;
private:
    DebugRequestPayload() = default;
    DebugRequestPayload(std::unordered_map<InputPaths, InputEntry> input_values, std::vector<SettingsEntry> setting_values)
        : inputs_{std::move(input_values)},
          settings_{std::move(setting_values)} {};

    static auto ParseInputs(const nlohmann::json& request) -> std::unordered_map<InputPaths, InputEntry>;
    static auto ParseSettings(const nlohmann::json& request) -> std::vector<SettingsEntry>;
    static auto GetInputPath(std::string_view path) -> InputPaths;

    inline static const std::string inputs_json_key_ = "inputs";
    inline static const std::string settings_json_key_ = "settings";
};