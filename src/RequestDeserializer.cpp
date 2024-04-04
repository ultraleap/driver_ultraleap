#include "RequestDeserializer.h"

#include <sstream>

#include "VrLogging.h"

auto DebugRequestPayload::Parse(const char* json_string) -> std::optional<DebugRequestPayload> {
    // First attempt to destructure the json string into a json object for interrogation.
    nlohmann::json request;
    try {
        request = nlohmann::json::parse(json_string);
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

auto DebugRequestPayload::ParseInputs(const nlohmann::json& request) -> std::map<InputPath, InputEntry> {
    std::map<InputPath, InputEntry> parsed_inputs;

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

                if (const auto parsed_optional = ParseInputPath(key, value, time_offset); parsed_optional.has_value()) {
                    parsed_inputs.insert(parsed_optional.value());
                }
            }
        }
    }

    // Finally returned the inputs we've parsed.
    return parsed_inputs;
}

auto DebugRequestPayload::ParseInputPath(std::string_view path_string, const nlohmann::basic_json<>& value, const float time_offset)
    -> std::optional<std::pair<InputPath, InputEntry>> {
    // Lookup to make sure the path is a valid one we support
    const auto input_path = StringToInputPath(path_string);
    const auto [input_source, input_component] = input_path;

    if (input_source == InputSource::UNKNOWN) {
        LOG_INFO("Path: {}, isn't a supported path. skipping...", path_string);
        return std::nullopt;
    }

    if (input_source == InputSource::PROXIMITY) {
        if (value.is_boolean()) {
            return std::pair{input_path, InputEntry{path_string, InputValue{value.get<bool>()}, time_offset}};
        }
        return std::nullopt;
    }

    // Use the appropriate type look for the component path that's been requested.
    switch (input_component) {
    case InputComponent::TOUCH:
    case InputComponent::CLICK: {
        if (value.is_boolean()) {
            return std::pair{input_path, InputEntry{path_string, InputValue{value.get<bool>()}, time_offset}};
        }
        break;
    }
    case InputComponent::FORCE:
    case InputComponent::VALUE:
    case InputComponent::X:
    case InputComponent::Y:
        if (value.is_number_float()) {
            return std::pair{input_path, InputEntry{path_string, InputValue{value.get<float>()}, time_offset}};
        }
        break;
    case InputComponent::NONE:
        return std::nullopt;
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

            if (const auto& value = item.value(); value.is_boolean()) {
                setting_value = value.get<bool>();
            } else if (value.is_number_float()) {
                setting_value = value.get<float>();
            } else if (value.is_array() && value.size() == 3) {
                // ReSharper disable CppTooWideScopeInitStatement
                const auto& x = value.at(0);
                const auto& y = value.at(1);
                const auto& z = value.at(2);
                // ReSharper restore CppTooWideScopeInitStatement

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

auto DebugRequestPayload::StringToInputPath(const std::string_view path) -> InputPath {
    auto input_source = InputSource::UNKNOWN;
    auto input_component = InputComponent::NONE;

    for (auto& [path_prefix, source] : kInputSourceMapping) {
        if (path.starts_with(path_prefix)) {
            input_source = source;
            break;
        }
    }

    for (auto& [path_suffix, component] : kInputComponentMapping) {
        if (path.ends_with(path_suffix)) {
            input_component = component;
            break;
        }
    }

    return {input_source, input_component};
}

auto DebugRequestPayload::InputPathToString(InputPath path) -> std::string {
    auto ss = std::stringstream{};
    auto& [input_source, input_component] = path;

    for (auto& [path_prefix, source] : kInputSourceMapping) {
        if (input_source == source) {
            ss << path_prefix;
            break;
        }
    }

    for (auto& [path_suffix, component] : kInputComponentMapping) {
        if (input_component == component) {
            ss << path_suffix;
            break;
        }
    }

    if (auto string = ss.str(); !string.empty()) {
        return string;
    }

    return "UNKNOWN_PATH";
}