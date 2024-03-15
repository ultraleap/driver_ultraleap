#include "RequestDeserializer.h"

#include "VrLogging.h"

auto DebugRequestPayload::Parse(const char* jsonString) -> std::optional<DebugRequestPayload> {
    // First attempt to destructure the json string into a json object for interrogation.
    nlohmann::json request;
    try {
        request = nlohmann::json::parse(jsonString);
    } catch (...) {
        LOG_INFO("Error parsing JSON input");
        return std::nullopt;
    }

    // After we have our JSON object take the data from it that we're interested in.
    auto inputs = ParseInputs(request);
    auto settings = ParseSettings(request);

    // Then return our created DebugRequestPayload
    return DebugRequestPayload{std::move(inputs), std::move(settings)};
}

auto DebugRequestPayload::ParseInputs(const nlohmann::json& request) -> std::unordered_map<InputPaths, InputEntry> {
    std::unordered_map<InputPaths, InputEntry> parsed_inputs;

    // Ensure that there is an entry in this json for inputs and it has values.
    if (request.contains(inputs_json_key_)) {
        // Check for a time offset within here as all other InputEntry's will use this.
        const auto& inputs_object = request[inputs_json_key_];
        auto time_offset = 0.0f;
        if (inputs_object.contains(inputs_time_offset_json_key_) && inputs_object[inputs_time_offset_json_key_].is_number_float()) {
            time_offset = inputs_object[inputs_time_offset_json_key_].get<float>();
        }

        // Check for the paths array and process the input paths accordingly.
        if (inputs_object.contains(inputs_paths_json_key_) && inputs_object[inputs_paths_json_key_].is_array()) {
            for(const auto& path_iter : inputs_object[inputs_paths_json_key_].items()) {
                auto path_object = path_iter.value();
                auto key = path_object.items().begin().key();
                auto value = path_object.items().begin().value();
                const auto parsed_optional = ParseInputPath(key, value, time_offset);

                if (parsed_optional.has_value()) {
                    parsed_inputs.insert(parsed_optional.value());
                }
            }
        }
    }

    // Finally returned the inputs we've parsed.
    return parsed_inputs;
}

auto DebugRequestPayload::ParseInputPath(std::string_view path_string, const nlohmann::basic_json<>& value, float time_offset)
    -> std::optional<std::pair<InputPaths, InputEntry>> {
    // Lookup to make sure the path is a valid one we support
    const auto input_path = StringToInputPath(path_string);
    if (input_path == InputPaths::UNKNOWN) {
        LOG_INFO("Path: {}, isn't a supported path. skipping...", path_string);
        return std::nullopt;
    }

    // Finally do validation against the input path vs the value given
    switch(input_path) {
    // Float based input paths
    case InputPaths::PINCH:
    case InputPaths::GRIP:
    case InputPaths::INDEX_FINGER:
    case InputPaths::MIDDLE_FINGER:
    case InputPaths::RING_FINGER:
    case InputPaths::PINKY_FINGER:
        if (!value.is_number_float()) { break; }
        return std::pair{input_path, InputEntry{path_string, InputValue{value.get<float>()}, time_offset}};
    // Boolean based input paths
    case InputPaths::SYSTEM_MENU:
    case InputPaths::PROXIMITY:
        if (!value.is_boolean()) { break; }
        return std::pair{input_path, InputEntry{path_string, InputValue{value.get<bool>()}, time_offset}};
    }

    // Failthrough for unsupported path
    LOG_INFO("Incorrect value type given for path: '{}'", path_string);
    return std::nullopt;
}

auto DebugRequestPayload::ParseSettings(const nlohmann::json& request) -> std::vector<SettingsEntry> {
    std::vector<SettingsEntry> settings;
    if (request.contains(settings_json_key_) && !request[settings_json_key_].empty()) {
        for(const auto& item : request[settings_json_key_].items()) {
            std::optional<SettingsValue> setting_value;
            const auto& value = item.value();

            if (value.is_boolean()) {
                setting_value = value.get<bool>();
            } else if (value.is_number_float()) {
                setting_value = value.get<float>();
            } else if (value.is_array() && value.size() == 3) {
                const auto& x = value.at(0);
                const auto& y = value.at(1);
                const auto& z = value.at(2);

                if (x.is_number_float() && y.is_number_float() && z.is_number_float()) {
                    setting_value = VrVec3{x.get<float>(), y.get<float>(), z.get<float>()};
                }
            } else if (value.is_string()) {
                setting_value = value.get<std::string>();
            }

            if (setting_value.has_value()) {
                settings.emplace_back(item.key(), setting_value.value());
            } else {
                LOG_INFO("Settings Key's '{}' value is invalid. Must be a float or bool an array of floats with a size of 3 ([x,y,x])", item.key());
            }
        }
    }
    return settings;
}

auto DebugRequestPayload::StringToInputPath(std::string_view path) -> InputPaths {
    if (path.starts_with("/input/system/click")) {
        return InputPaths::SYSTEM_MENU;
    }
    if (path.starts_with("/input/pinch")) {
        return InputPaths::PINCH;
    }
    if (path.starts_with("/input/grip")) {
        return InputPaths::GRIP;
    }
    if (path.starts_with("/input/finger/index")) {
        return InputPaths::INDEX_FINGER;
    }
    if (path.starts_with("/input/finger/middle")) {
        return InputPaths::MIDDLE_FINGER;
    }
    if (path.starts_with("/input/finger/ring")) {
        return InputPaths::RING_FINGER;
    }
    if (path.starts_with("/input/finger/pinky")) {
        return InputPaths::PINKY_FINGER;
    }
    if (path.starts_with("/proximity")) {
        return InputPaths::PROXIMITY;
    }

    return InputPaths::UNKNOWN;
}

auto DebugRequestPayload::InputPathToString(InputPaths path) -> std::string {
    switch(path) {
    case InputPaths::SYSTEM_MENU: return "/input/system/click";
    case InputPaths::PROXIMITY: return "/proximity";
    case InputPaths::PINCH: return "/input/pinch";
    case InputPaths::GRIP: return "/input/grip";
    case InputPaths::INDEX_FINGER: return "/input/finger/index";
    case InputPaths::MIDDLE_FINGER: return "/input/finger/middle";
    case InputPaths::RING_FINGER: return "/input/finger/ring";
    case InputPaths::PINKY_FINGER: return "/input/finger/pinky";
    case InputPaths::UNKNOWN:
    default:
        return "UNKNOWN_PATH";
    }
}