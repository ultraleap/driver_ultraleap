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

    // First ensure that there is an entry in this json for inputs and it has values.
    if (request.contains(inputs_json_key_) && !request[inputs_json_key_].empty()) {
        for (const auto& input : request[inputs_json_key_].items()) {
            const std::string& input_path_string = input.key();
            auto values = input.value();
            const InputPaths input_path_key = GetInputPath(input_path_string);

            if (input_path_key == InputPaths::UNKNOWN) {
                LOG_INFO("Path isn't matched to a corresponding key. Skipping...");
                continue;
            }

            // If theres X and Y values; process them first and ignore them later as we want them processed together.
            if (values.contains("x") && values.contains("y")) {
                const auto& x = values["x"];
                const auto& y = values["y"];

                if (x.is_number_float() && y.is_number_float()) {
                    vr::HmdVector2_t point = {x.get<float>(), y.get<float>()};
                    parsed_inputs.insert({input_path_key, InputEntry{input_path_string, "joystick", point}});
                } else {
                    LOG_INFO("The X and Y values for the path '{}' arent the correct type. Expected float.", input_path_string);
                }
            }

            // Otherwise iterate through each entry and make InputEntry's for each;
            // Shared paths / keys dont matter here. If we have multiple for a given path thats fine.
            for (const auto& item : values.items()) {
                // Since we've already processed these keys above, skip them.
                if (item.key() == "x" || item.key() == "y") {
                    continue;
                }

                std::optional<InputValue> input_value;
                const auto& value = item.value();
                if (value.is_boolean()) {
                    input_value = value.get<bool>();
                } else if (value.is_number_float()) {
                    input_value = value.get<float>();
                }

                if (input_value.has_value()) {
                    parsed_inputs.insert({input_path_key, InputEntry{input_path_string, item.key(), input_value.value()}});
                }
            }
        }
    }

    // Finally returned the inputs we've parsed.
    return parsed_inputs;
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

auto DebugRequestPayload::GetInputPath(std::string_view path) -> InputPaths {
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
    if (path.starts_with("/time_offset")) {
        return InputPaths::TIME_OFFSET;
    }
    if (path.starts_with("/proximity")) {
        return InputPaths::PROXIMITY;
    }

    return InputPaths::UNKNOWN;
}