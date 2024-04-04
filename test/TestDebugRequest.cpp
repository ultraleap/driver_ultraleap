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
        std::string outstr;
        outstr.resize(size);
        gOvrSystem->GetStringTrackedDeviceProperty(id, prop, outstr.data(), size);
        return outstr;
    }
    return std::string{};
}

auto DebugRequestToDriver(const vr::TrackedDeviceIndex_t id, const std::string& payload) {
    std::string response;
    size_t size = 2048;
    response.resize(size);
    std::cout << "Sending Payload...." << std::endl;
    std::cout << "Request: " << payload << std::endl;
    gOvrDebug->DriverDebugRequest(id, payload.c_str(), response.data(), size);
    std::cout << "Response: " << nlohmann::json::parse(response) << std::endl;
}

auto SendDebugPayload(const vr::TrackedDeviceIndex_t id, const bool secondValues) {
    nlohmann::json full_payload;

    if (secondValues) {

        full_payload = nlohmann::json::parse(R"(
        {
          "inputs": {
            "time_offset": -0.123,
            "paths": [
              {"/input/pinch": 0.75},
              {"/input/grip": 0.75},
              {"/input/dpad": [0.5, 0.5, 5.0]}
            ]
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
            "paths": [
              {"/input/pinch": 0.25},
              {"/input/grip": 0.25},
              {"/input/dpad": [0.5, 0.5, 5.0]}
            ]
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

    std::string payload = full_payload.dump();
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
        if (vr::TrackedDeviceClass device_class = gOvrSystem->GetTrackedDeviceClass(i);
            device_class != vr::TrackedDeviceClass_Invalid) {
            std::string manifacturer = GetStringProperty(i, vr::Prop_ManufacturerName_String);
            std::string model = GetStringProperty(i, vr::Prop_ModelNumber_String);
            tracked_devices.emplace_back(TrackedDeviceInfo{i, device_class, std::move(manifacturer), std::move(model)});
        }
    }

    // Once we know the tracked device we want to operate on we can ping that handle with the payload
    gOvrDebug = vr::VRDebug();
    if (gOvrDebug == nullptr) {
        std::cout << "Failed to Fetch Debug Interface.\n";
        system("pause");
        return -1;
    }

    std::cout << "Please Select device desired send payload:\n";
    std::cout << "  ID -------- Manifacturer -------- Model\n";
    for (const auto& [device_index, device_class, manifacturer, model] : tracked_devices) {
        std::cout << "  " << device_index << " -------- " << manifacturer << " -------- " << model << "\n";
    }
    std::cout << std::endl;

    vr::TrackedDeviceIndex_t selected_device = 2;
    // std::cin >> selected_device;

    // Take the selected device and send the payload
    for (int i = 0; i < 4; ++i) {
        SendDebugPayload(selected_device, i % 2 == 0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    // Send a broken payload to make sure error response is formatted correctly
    SendBrokenPayload(selected_device);

    // Re-enable the driver input after the requests
    EnableDriverInput(selected_device);

    std::cout << "Shutting Down..." << std::endl;
    vr::VR_Shutdown();
    return 0;
}
