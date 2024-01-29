#include "JsonSerializer.h"

#include <nlohmann/json.hpp>


auto DebugRequestJson::Parse(const char* string) -> std::optional<DebugRequestJson> {
    std::unordered_map<InputPaths, InputEntry> inputs;
    std::vector<SettingsEntry> settings;
//     // First do inputs translating their path into InputPaths enum options
//
//     // Create an Create an InputEntry of the options for that and add it to the unordered map along with the above key
    // std::string path = "woooow";
    // InputPaths pathEnum;
    // float valueFromJSON = 1.0f;
    // switch(path) {
    // case "woooow":
    //     pathEnum = InputPaths::GRIP;
    //     break;
    // default:
    //     pathEnum = InputPaths::UNKNOWN;
    // }
    // inputs.insert({pathEnum, InputEntry{path, valueFromJSON}});


//     // Also parse the settings if any were sent
//
//     // Create an instance of DebugRequestJson using the two above values and return its instance.
    return std::nullopt;
}