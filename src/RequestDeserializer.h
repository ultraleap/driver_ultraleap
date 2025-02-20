#pragma once

#include "VrMaths.h"
#include "openvr_driver.h"

#include <optional>
#include <utility>
#include <variant>
#include <string>
#include <map>

#include <nlohmann/json.hpp>

enum class InputSource {
    SYSTEM,
    PROXIMITY,

    INDEX_PINCH,
    GRIP,

    THUMB_FINGER,
    INDEX_FINGER,
    MIDDLE_FINGER,
    RING_FINGER,
    PINKY_FINGER,

    BUTTON_A,
    BUTTON_B,
    TRIGGER,
    TRACKPAD,
    THUMBSTICK,

    UNKNOWN,
};

enum class InputComponent {
    TOUCH,
    CLICK,
    FORCE,
    VALUE,
    X,
    Y,
    NONE,
};

const std::map<std::string, InputSource> kInputSourceMapping = {
    {"/input/system", InputSource::SYSTEM},
    {"/proximity", InputSource::PROXIMITY},

    {"/input/index_pinch", InputSource::INDEX_PINCH},
    {"/input/grip", InputSource::GRIP},

    {"/input/a", InputSource::BUTTON_A},
    {"/input/b", InputSource::BUTTON_B},
    {"/input/trigger", InputSource::TRIGGER},
    {"/input/trackpad", InputSource::TRACKPAD},
    {"/input/thumbstick", InputSource::THUMBSTICK},

    {"/input/finger/thumb", InputSource::THUMB_FINGER},
    {"/input/finger/index", InputSource::INDEX_FINGER},
    {"/input/finger/middle", InputSource::MIDDLE_FINGER},
    {"/input/finger/ring", InputSource::RING_FINGER},
    {"/input/finger/pinky", InputSource::PINKY_FINGER},
};

const std::map<std::string, InputComponent> kInputComponentMapping = {
    {"/touch", InputComponent::TOUCH},
    {"/click", InputComponent::CLICK},
    {"/force", InputComponent::FORCE},
    {"/value", InputComponent::VALUE},
    {"/x", InputComponent::X},
    {"/y", InputComponent::Y},
};


using InputPath = std::pair<InputSource, InputComponent>;

using InputValue = std::variant<bool, float>;
using SettingsValue = std::variant<bool, float, VrVec3, std::string>;

class DebugRequestPayload{
public:
    class InputEntry {
    public:
        InputEntry(const std::string_view path_str, const InputValue val, const float time_offset = 0.0f)
            : full_path_{path_str},
              value_{val},
              time_offset_{time_offset} {}
        const std::string full_path_;
        const InputValue value_;
        const float time_offset_;
    };

    class SettingsEntry {
    public:
        SettingsEntry(const std::string_view key_str, SettingsValue val) : key_{key_str}, value_{std::move(val)} {}
        const std::string key_;
        const SettingsValue value_;
    };

    static auto Parse(const char* json_string) -> std::optional<DebugRequestPayload>;
    static auto StringToInputPath(std::string_view path) -> InputPath;
    static auto InputPathToString(InputPath path) -> std::string;

    std::map<InputPath, InputEntry> inputs_;
    std::vector<SettingsEntry> settings_;
private:
    DebugRequestPayload() = default;
    DebugRequestPayload(std::map<InputPath, InputEntry> input_values, std::vector<SettingsEntry> setting_values)
        : inputs_{std::move(input_values)},
          settings_{std::move(setting_values)} {}

    static auto ParseInputs(const nlohmann::json& request) -> std::map<InputPath, InputEntry>;
    static auto ParseInputPath(std::string_view path_string, const nlohmann::basic_json<>& value, float time_offset) -> std::optional<std::pair<InputPath, InputEntry>>;
    static auto ParseSettings(const nlohmann::json& request) -> std::vector<SettingsEntry>;

    inline static const std::string inputs_json_key_ = "inputs";
    inline static const std::string inputs_time_offset_json_key_ = "time_offset";
    inline static const std::string inputs_paths_json_key_ = "paths";
    inline static const std::string settings_json_key_ = "settings";
};