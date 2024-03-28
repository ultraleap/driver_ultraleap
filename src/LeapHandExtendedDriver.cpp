//
// Created by rober on 25/03/2024.
//

#include "LeapHandExtendedDriver.h"

vr::EVRInitError LeapHandExtendedDriver::Activate(uint32_t object_id) {
    // Use the base implementation for most of the setup.
    auto result = LeapHandDriver::Activate(object_id);

    try {
        const auto properties = VrDeviceProperties::FromDeviceId(id_);

        // Override to set as a extended hand-profile instead
        properties.Set(vr::Prop_ControllerType_String, "ultraleap_hand_extended");
        properties.Set(vr::Prop_InputProfilePath_String, "{ultraleap}/input/ultraleap_hand_extended_profile.json");


        // Button A
        input_button_a_touch = properties.CreateBooleanInput("/input/a/touch");
        path_inputs_map_.insert({InputPaths::BUTTON_A_TOUCH, &input_button_a_touch});
        input_button_a_click = properties.CreateBooleanInput("/input/a/click");
        path_inputs_map_.insert({InputPaths::BUTTON_A_CLICK, &input_button_a_click});

        // Button B
        input_button_b_touch = properties.CreateBooleanInput("/input/b/touch");
        path_inputs_map_.insert({InputPaths::BUTTON_B_TOUCH, &input_button_b_touch});
        input_button_b_click = properties.CreateBooleanInput("/input/b/click");
        path_inputs_map_.insert({InputPaths::BUTTON_B_CLICK, &input_button_b_click});

        // Trigger
        input_button_b_touch = properties.CreateBooleanInput("/input/b/touch");
        path_inputs_map_.insert({InputPaths::BUTTON_B_TOUCH, &input_button_b_touch});
        input_button_b_click = properties.CreateBooleanInput("/input/b/click");
        path_inputs_map_.insert({InputPaths::BUTTON_B_CLICK, &input_button_b_click});


    return result;
}

auto LeapHandExtendedDriver::ProcessDebugRequestInputs(const DebugRequestPayload& request_payload,
    nlohmann::json& response) const -> void {
}