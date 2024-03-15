#pragma once

#include <atomic>
#include <variant>
#include <typeinfo>

#include "RequestDeserializer.h"
#include "LeapDriverSettings.h"
#include "VrLogging.h"

#include <openvr_driver.h>

class LeapTrackedDriver : public vr::ITrackedDeviceServerDriver {
public:
    LeapTrackedDriver(const uint32_t id, const std::shared_ptr<LeapDriverSettings>& settings) : id_{id}, settings_{settings} {};
    virtual ~LeapTrackedDriver() = default;

    // ITrackedDeviceServerDriver
    auto Activate(uint32_t object_id) -> vr::EVRInitError override = 0;
    auto Deactivate() -> void override = 0;
    auto EnterStandby() -> void override = 0;
    auto GetComponent(const char* component_name_and_version) -> void* override = 0;
    auto DebugRequest(const char* request, char* response_buffer, uint32_t response_buffer_size) -> void final;
    auto GetPose() -> vr::DriverPose_t override = 0;

    [[nodiscard]] auto Id() const -> uint32_t { return id_; }

protected:
    virtual auto ProcessDebugRequestInputs(const DebugRequestPayload& request_payload, nlohmann::json& response) const -> void = 0;

    uint32_t id_ = vr::k_unTrackedDeviceIndexInvalid;
    std::atomic<bool> active_ = false;
    std::shared_ptr<LeapDriverSettings> settings_;
    inline static const std::string response_result_key_ = "result";
    inline static const std::string response_warnings_key_ = "warnings";
    inline static const std::string response_errors_key_ = "errors";

private:
    auto ProcessDebugRequestSettings(const DebugRequestPayload& request_payload, nlohmann::json& response) const -> void;

  template <typename T>
  static auto UpdateSetting(std::function<void(T)> update_func, nlohmann::json& response, std::string_view key, SettingsValue value)
      -> void {
      if (std::holds_alternative<T>(value)) {
          update_func(std::get<T>(value));
      } else {
          LOG_INFO("Incorrect type passed in for key: '{}'. Expected: '{}'", key, typeid(T).name());
          response[response_warnings_key_] += std::format(
              "Incorrect type passed in for key: '{}'. Expected: '{}'",
              key,
              typeid(T).name()
          );
      }
  };
};