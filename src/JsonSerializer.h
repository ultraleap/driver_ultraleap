#pragma once

#include "openvr_driver.h"

#include <optional>
#include <variant>
#include <string>
#include <unordered_map>
#include <any>

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
        InputEntry(std::string pathStr, InputValue val) : path{std::move(pathStr)}, value{val} {};
        const std::string path;
        const InputValue value;
    };

    class SettingsEntry {
        SettingsEntry(std::string key, SettingsValue val) : key{std::move(key)}, value{val} {};
        const std::string key;
        const SettingsValue value;
    };

    static std::optional<DebugRequestJson> Parse(const char* string);

    std::unordered_map<InputPaths, InputEntry> inputs;
    std::vector<SettingsEntry> settings;

    // TODO: if theres a better way to ASSIGN a type to a variable PLEASE tell me. I spent far too long
    // Fannying about with decltype and declval trying to get this idea in my brain to work.
    inline const static std::unordered_map<InputPaths, std::any> InputPathTypeLookup {
        {InputPaths::SYSTEM_MENU, true},
        {InputPaths::PROXIMITY, true},
        {InputPaths::PINCH, 0.0f},
        {InputPaths::GRIP, 0.0f},
        {InputPaths::INDEX_FINGER, 0.0f},
        {InputPaths::MIDDLE_FINGER, 0.0f},
        {InputPaths::RING_FINGER, 0.0f},
        {InputPaths::PINKY_FINGER, 0.0f},
    };

    DebugRequestJson() = default;
private:

    DebugRequestJson(std::unordered_map<InputPaths, InputEntry> inputValues, std::vector<SettingsEntry> settingValues)
        : inputs{std::move(inputValues)},
          settings{std::move(settingValues)} {};



};