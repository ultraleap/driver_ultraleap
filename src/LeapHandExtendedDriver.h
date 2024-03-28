#pragma once
#include "LeapHandDriver.h"

class LeapHandExtendedDriver : LeapHandDriver {
public:
    auto Activate(uint32_t object_id) -> vr::EVRInitError override;

private:
    auto ProcessDebugRequestInputs(const DebugRequestPayload& request_payload, nlohmann::json& response) const -> void override;




    VrBooleanInputComponent input_button_a_touch;
    VrBooleanInputComponent input_button_a_click;
    VrBooleanInputComponent input_button_b_touch;
    VrBooleanInputComponent input_button_b_click;

    VrScalarInputComponent input_pinch_;
    VrScalarInputComponent input_grip_;

};

