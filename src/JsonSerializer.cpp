#include "JsonSerializer.h"

#include <nlohmann/json.hpp>


std::optional<DebugRequestJson> DebugRequestJson::Parse(const char* string) {
    std::unordered_map<InputPaths, InputEntry> inputs;
    std::vector<SettingsEntry> settings;
//     //Parse the string and read the json
//     /*
//  *
// *    // Attempt to read the JSON configuration file
//     nlohmann::json configData;
//     try {
//         configData = nlohmann::json::parse(std::ifstream{filePath});
//     } catch (std::exception &e) {
//         UL_LOG_ERROR("Failed to read JSON configuration: {}", e.what());
//         UL_LOG_WARNING("JSON configuration will be skipped");
//         return;
//     }
//  */
//
//     // First do inputs translating their path into InputPaths enum options
//
//     // Create an Create an InputEntry of the options for that and add it to the unordered map along with the above key
//
        // EG of the above
    std::string path = "woooow";
    InputPaths pathEnum;
    float valueFromJSON = 1.0f;
    switch(path) {
    case "woooow":
        pathEnum = InputPaths::GRIP;
        break;
    default:
        pathEnum = InputPaths::UNKNOWN;
    }
    inputs.insert({pathEnum, InputEntry{path, valueFromJSON}});


//     // Also parse the settings if any were sent
//
//     // Create an instance of DebugRequestJson using the two above values and return its instance.
}