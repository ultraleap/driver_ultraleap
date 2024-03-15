#include "LeapTrackedDriver.h"


auto LeapTrackedDriver::DebugRequest(const char* request, char* response_buffer, uint32_t response_buffer_size) -> void {
    auto SendResponse = [&](const nlohmann::json& response) {
        static const nlohmann::json buffer_insufficient_response = {
            {response_result_key_, "error"},
            {response_errors_key_, nlohmann::json::array({"Buffer Size Insufficient"})}
        };
        static const auto buffer_insufficient_string = buffer_insufficient_response.dump();
        const auto response_string = response.dump();

        // Attempt to send the correct response, otherwise attempt to fit the buffer insufficient error in it instead.
        if (response_string.size() <= response_buffer_size) {
            strcpy_s(response_buffer, response_buffer_size, response_string.c_str());
        } else if (buffer_insufficient_string.size() < response_buffer_size) {
            strcpy_s(response_buffer, response_buffer_size, buffer_insufficient_string.c_str());
        }
    };

    if (id_ == vr::k_unTrackedDeviceIndexInvalid) {
        LOG_INFO("Id of hand driver is invalid, implying driver hasnt been initialized yet; skipping....");
        return;
    }

    if (response_buffer_size <= 0) {
        LOG_INFO("Response Buffer for a debug payload was zero; Ignoring request...");
        return;
    }

    // After safety checks we can now parse our received payload and process.
    const auto debug_request_payload = DebugRequestPayload::Parse(request);
    nlohmann::json response;
    response[response_result_key_] = "success";
    response[response_warnings_key_] = nlohmann::json::array();
    response[response_errors_key_] = nlohmann::json::array();

    if (!debug_request_payload.has_value()) {
        LOG_INFO("Failed to parse debug json, skipping...");
        response[response_result_key_] = "error";
        response[response_errors_key_] += "Failed to parse debug json.";
        SendResponse(response);
        return;
    }

    ProcessDebugRequestInputs(debug_request_payload.value(), response);
    ProcessDebugRequestSettings(debug_request_payload.value(), response);

    // If we had any errors change our result to a failure
    if (!response[response_errors_key_].empty()) {
        response[response_result_key_] = "error";
    }

    // Copy our response into the provided buffer.
    SendResponse(response);
}
auto LeapTrackedDriver::ProcessDebugRequestSettings(const DebugRequestPayload& request_payload, nlohmann::json& response) const
    -> void {
    for (const auto& setting : request_payload.settings_) {
        if (setting.key_ == "tracking_mode") {
            // This one is a bit special as we're mapping the string to an internal enum
            if (std::holds_alternative<std::string>(setting.value_)) {
                const auto& value_string = std::get<std::string>(setting.value_);
                if (value_string == "hmd") {
                    settings_->UpdateTrackingMode(eLeapTrackingMode_HMD);
                    continue;
                }
                if (value_string == "desktop") {
                    settings_->UpdateTrackingMode(eLeapTrackingMode_Desktop);
                    continue;
                }
            }
            LOG_INFO("Incorrect type or value passed in for key: 'tracking_mode'. Expected the strings 'hmd' or 'desktop'");
            response[response_warnings_key_] += "Incorrect type or value passed in for key: 'tracking_mode'. Expected the strings "
                                                "'hmd' or 'desktop'";
        } else if (setting.key_ == "hmd_tracker_offset") {
            UpdateSetting<VrVec3>(
                [&](const VrVec3& val) { settings_->UpdateHmdTrackerOffset(val); },
                response,
                setting.key_,
                setting.value_
            );
        } else if (setting.key_ == "desktop_tracker_offset") {
            UpdateSetting<VrVec3>(
                [&](const VrVec3& val) { settings_->UpdateDesktopTrackerOffset(val); },
                response,
                setting.key_,
                setting.value_
            );
        } else if (setting.key_ == "enable_elbow_trackers") {
            UpdateSetting<bool>(
                [&](const bool& val) { settings_->UpdateEnableElbowTrackers(val); },
                response,
                setting.key_,
                setting.value_
            );
        } else if (setting.key_ == "external_input_only") {
            UpdateSetting<bool>(
                [&](const bool& val) { settings_->UpdateExternalInputOnly(val); },
                response,
                setting.key_,
                setting.value_
            );
        } else {
            LOG_INFO("Failed to find setting with key: '{}', skipping...", setting.key_);
            response[response_warnings_key_] += std::format("Failed to find setting with key: '{}'", setting.key_);
        }
    }
}