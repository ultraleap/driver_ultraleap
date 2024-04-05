#include <iostream>
#include <chrono>
#include <thread>

#include <openvr.h>

#include "nlohmann/json.hpp"

vr::IVRSystem* gOvrSystem = nullptr;
vr::IVRDebug* gOvrDebug = nullptr;

struct TrackedDeviceInfo {
    vr::TrackedDeviceIndex_t device_index;
    vr::TrackedDeviceClass device_class;
    std::string manifacturer;
    std::string model;
};

auto GetStringProperty(const vr::TrackedDeviceIndex_t id, const vr::ETrackedDeviceProperty prop) -> std::string {
    if (const auto size = gOvrSystem->GetStringTrackedDeviceProperty(id, prop, nullptr, 0); size > 0) {
        std::array<char, vr::k_unMaxPropertyStringSize> value{};
        gOvrSystem->GetStringTrackedDeviceProperty(id, prop, value.data(), value.size());
        return std::string{value.data()};
    }
    return std::string{};
}

auto DebugRequestToDriver(const vr::TrackedDeviceIndex_t id, const std::string& payload) {
    std::array<char, vr::k_unMaxDriverDebugResponseSize> buffer{};
    std::cout << "Sending Payload...." << std::endl;
    std::cout << "Request: " << payload << std::endl;
    gOvrDebug->DriverDebugRequest(id, payload.c_str(), buffer.data(), buffer.size());
    std::cout << "Response: " << nlohmann::json::parse(buffer) << std::endl;
}

auto SendSimpleDebugPayload(const vr::TrackedDeviceIndex_t id, const bool secondValues) {
    nlohmann::json full_payload;

    if (secondValues) {

        full_payload = nlohmann::json::parse(R"(
        {
          "inputs": {
            "time_offset": -0.123,
            "paths": {
              "/input/index_pinch/value": 0.75,
              "/input/grip/value": 0.75
            }
          },
          "settings": {
            "tracking_mode": "hmd",
            "desktop_tracker_offset": [1.0, 2.0, 3.0],
            "enable_elbow_trackers": true,
            "external_input_only": true
          }
        }
    )");
    } else {
        full_payload = nlohmann::json::parse(R"(
        {
          "inputs": {
            "time_offset": -0.321,
            "paths": {
              "/input/index_pinch/value": 0.75,
              "/input/grip/value": 0.75
            }
          },
          "settings": {
            "tracking_mode": "hmd",
            "desktop_tracker_offset": [1.0, 2.0, 3.0],
            "enable_elbow_trackers": true,
            "external_input_only": true
          }
        }
    )");
    }

    const std::string payload = full_payload.dump();
    DebugRequestToDriver(id, payload);
}

auto SendExtendedDebugPayload(const vr::TrackedDeviceIndex_t id) {
    const nlohmann::json full_payload = nlohmann::json::parse(R"(
        {
          "inputs": {
            "time_offset": -0.321,
            "paths": {
              "/input/index_pinch/value": 0.5,
              "/input/grip/value": 0.5,
              "/input/a/click": true,
              "/input/b/click": true,
              "/input/trigger/click": true,
              "/input/trigger/value": 0.5,
              "/input/trackpad/touch": true,
              "/input/trackpad/x": 0.5,
              "/input/trackpad/y": 0.5,
              "/input/trackpad/force": 0.25,
              "/input/thumbstick/touch": true,
              "/input/thumbstick/x": -0.5,
              "/input/thumbstick/y": -0.5
            }
          },
          "settings": {
            "tracking_mode": "hmd",
            "desktop_tracker_offset": [1.0, 2.0, 3.0],
            "enable_elbow_trackers": true,
            "external_input_only": true
          }
        }
    )");

    const std::string payload = full_payload.dump();
    DebugRequestToDriver(id, payload);
}

auto SendBrokenPayload(vr::TrackedDeviceIndex_t id) {
    const std::string payload = R"({"inputs"=[{"Greg": 0.25f}]})";
    DebugRequestToDriver(id, payload);
}

auto EnableDriverInput(vr::TrackedDeviceIndex_t id) {
    nlohmann::json payload;
    const nlohmann::json settings = {{"external_input_only", false}};
    payload["settings"] = settings;

    const std::string payloadString = payload.dump();
    DebugRequestToDriver(id, payloadString);
}

int main() {
    vr::EVRInitError init_error = vr::VRInitError_None;
    const char* pStartupInfo = "";

    gOvrSystem = vr::VR_Init(&init_error, vr::VRApplication_Background, pStartupInfo);
    if (init_error != vr::VRInitError_None) {
        std::cout << "Failed to init OVR. Make sure SteamVR is running!\n";
        system("pause");
        return -1;
    }

    std::vector<TrackedDeviceInfo> tracked_devices;

    // Loop through the tracked devices to find out what we have access too
    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
        if (const vr::TrackedDeviceClass device_class = gOvrSystem->GetTrackedDeviceClass(i);
            device_class != vr::TrackedDeviceClass_Invalid) {
            std::string manufacturer = GetStringProperty(i, vr::Prop_ManufacturerName_String);
            std::string model = GetStringProperty(i, vr::Prop_ModelNumber_String);
            tracked_devices.emplace_back(TrackedDeviceInfo{i, device_class, std::move(manufacturer), std::move(model)});
        }
    }

    // Once we know the tracked device we want to operate on we can ping that handle with the payload
    gOvrDebug = vr::VRDebug();
    if (gOvrDebug == nullptr) {
        std::cout << "Failed to Fetch Debug Interface.\n";
        system("pause");
        return -1;
    }



    std::cout << "Attached Devices" << std::endl;
    std::cout << "ID | Device & Manufacturer" << std::endl;
    for (const auto& [device_index, device_class, manufacturer, model] : tracked_devices) {
        std::cout << std::format("{}: {} ({})", device_index, manufacturer, model) << std::endl;
    }
    std::cout << std::endl;

    const vr::TrackedDeviceIndex_t selected_device = 2;
    // std::cin >> selected_device;

    if (tracked_devices[selected_device].model == "left_hand_ext") {
        SendExtendedDebugPayload(selected_device);
    } else {
        // Take the selected device and send the payload
        for (int i = 0; i < 4; ++i) {
            SendSimpleDebugPayload(selected_device, i % 2 == 0);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        // Send a broken payload to make sure error response is formatted correctly
        SendBrokenPayload(selected_device);
    }



    // Re-enable the driver input after the requests
    EnableDriverInput(selected_device);

    std::cout << "Shutting Down..." << std::endl;
    vr::VR_Shutdown();
    return 0;
}
