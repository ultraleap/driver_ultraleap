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
    UNKNOWN
};

using InputValue = std::variant<bool, float, vr::HmdVector2_t>;
using SettingsValue = std::variant<bool, float, VrVec3, std::string>;

class DebugRequestPayload{
public:
    class InputEntry {
    public:
        InputEntry(std::string_view path_str, InputValue val, float time_offset = 0.0f)
            : full_path_{path_str},
              value_{val},
              time_offset_{time_offset} {};
        const std::string full_path_;
        const InputValue value_;
        const float time_offset_;
    };

    class SettingsEntry {
    public:
        SettingsEntry(std::string_view key_str, SettingsValue val) : key_{key_str}, value_{std::move(val)} {};
        const std::string key_;
        const SettingsValue value_;
    };

    static auto Parse(const char* json_string) -> std::optional<DebugRequestPayload>;
    static auto StringToInputPath(std::string_view path) -> InputPaths;
    static auto InputPathToString(InputPaths path) -> std::string;

    std::unordered_map<InputPaths, InputEntry> inputs_;
    std::vector<SettingsEntry> settings_;
private:
    DebugRequestPayload() = default;
    DebugRequestPayload(std::unordered_map<InputPaths, InputEntry> input_values, std::vector<SettingsEntry> setting_values)
        : inputs_{std::move(input_values)},
          settings_{std::move(setting_values)} {};

    static auto ParseInputs(const nlohmann::json& request) -> std::unordered_map<InputPaths, InputEntry>;
    static auto ParseInputPath(std::string_view path_string, const nlohmann::basic_json<>& value, float time_offset) -> std::optional<std::pair<InputPaths, InputEntry>>;
    static auto ParseSettings(const nlohmann::json& request) -> std::vector<SettingsEntry>;

    inline static const std::string inputs_json_key_ = "inputs";
    inline static const std::string inputs_time_offset_json_key_ = "time_offset";
    inline static const std::string inputs_paths_json_key_ = "paths";
    inline static const std::string settings_json_key_ = "settings";
};