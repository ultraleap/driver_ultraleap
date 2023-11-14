#pragma once

#include <array>
#include <format>
#include <string>

#include <openvr_driver.h>

#include "LeapC.h"

#define OVR_LOG(...) OvrLogging::Log(__VA_ARGS__)

class OvrLogging {
  public:
    static auto Init() -> void { pLogFile = vr::VRDriverLog(); }
    template <typename... T> static auto Log(std::format_string<T...> fmt, T&&... args) -> void {
        pLogFile->Log(std::format(fmt, std::forward<T>(args)...).c_str());
    }

  private:
    inline static vr::IVRDriverLog* pLogFile;
};

template <> struct [[maybe_unused]] std::formatter<eLeapDevicePID> {
    [[maybe_unused]] static constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

    [[maybe_unused]] static auto format(const eLeapDevicePID& input, std::format_context& ctx) {
        return std::format_to(ctx.out(), "{}", [](const eLeapDevicePID& devicePid) constexpr {
            switch (devicePid) {
            case eLeapDevicePID_Peripheral: return "Leap Motion Controller";
            case eLeapDevicePID_Rigel:
            case eLeapDevicePID_SIR170: return "Stereo IR 170";
            case eLeapDevicePID_3Di: return "3Di";
            case eLeapDevicePID_LMC2: return "Leap Motion Controller 2";
            default: return "Tracking Device";
            }
        }(input));
    }
};

template <> struct [[maybe_unused]] std::formatter<eLeapRS> {
    [[maybe_unused]] static constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

    [[maybe_unused]] static auto format(const eLeapRS& input, std::format_context& ctx) {
        return std::format_to(ctx.out(), "{}", [](const eLeapRS& r) constexpr {
            switch (r) {
            case eLeapRS_Success: return "eLeapRS_Success";
            case eLeapRS_UnknownError: return "eLeapRS_UnknownError";
            case eLeapRS_InvalidArgument: return "eLeapRS_InvalidArgument";
            case eLeapRS_InsufficientResources: return "eLeapRS_InsufficientResources";
            case eLeapRS_InsufficientBuffer: return "eLeapRS_InsufficientBuffer";
            case eLeapRS_Timeout: return "eLeapRS_Timeout";
            case eLeapRS_NotConnected: return "eLeapRS_NotConnected";
            case eLeapRS_HandshakeIncomplete: return "eLeapRS_HandshakeIncomplete";
            case eLeapRS_BufferSizeOverflow: return "eLeapRS_BufferSizeOverflow";
            case eLeapRS_ProtocolError: return "eLeapRS_ProtocolError";
            case eLeapRS_InvalidClientID: return "eLeapRS_InvalidClientID";
            case eLeapRS_UnexpectedClosed: return "eLeapRS_UnexpectedClosed";
            case eLeapRS_UnknownImageFrameRequest: return "eLeapRS_UnknownImageFrameRequest";
            case eLeapRS_UnknownTrackingFrameID: return "eLeapRS_UnknownTrackingFrameID";
            case eLeapRS_RoutineIsNotSeer: return "eLeapRS_RoutineIsNotSeer";
            case eLeapRS_TimestampTooEarly: return "eLeapRS_TimestampTooEarly";
            case eLeapRS_ConcurrentPoll: return "eLeapRS_ConcurrentPoll";
            case eLeapRS_NotAvailable: return "eLeapRS_NotAvailable";
            case eLeapRS_NotStreaming: return "eLeapRS_NotStreaming";
            case eLeapRS_CannotOpenDevice: return "eLeapRS_CannotOpenDevice";
            default: return "eLeapRS_Invalid(Unknown)";
            }
        }(input));
    };
};

template <> struct [[maybe_unused]] std::formatter<eLeapEventType> {
    [[maybe_unused]] static constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

    [[maybe_unused]] static auto format(const eLeapEventType& input, std::format_context& ctx) {
        return std::format_to(ctx.out(), "{}", [](const eLeapEventType& e) constexpr {
            switch (e) {
            case eLeapEventType_None: return "eLeapEventType_None";
            case eLeapEventType_Connection: return "eLeapEventType_Connection";
            case eLeapEventType_ConnectionLost: return "eLeapEventType_ConnectionLost";
            case eLeapEventType_Device: return "eLeapEventType_Device";
            case eLeapEventType_DeviceFailure: return "eLeapEventType_DeviceFailure";
            case eLeapEventType_Policy: return "eLeapEventType_Policy";
            case eLeapEventType_Tracking: return "eLeapEventType_Tracking";
            case eLeapEventType_ImageRequestError: return "eLeapEventType_ImageRequestError";
            case eLeapEventType_ImageComplete: return "eLeapEventType_ImageComplete";
            case eLeapEventType_LogEvent: return "eLeapEventType_LogEvent";
            case eLeapEventType_DeviceLost: return "eLeapEventType_DeviceLost";
            case eLeapEventType_ConfigResponse: return "eLeapEventType_ConfigResponse";
            case eLeapEventType_ConfigChange: return "eLeapEventType_ConfigChange";
            case eLeapEventType_DeviceStatusChange: return "eLeapEventType_DeviceStatusChange";
            case eLeapEventType_DroppedFrame: return "eLeapEventType_DroppedFrame";
            case eLeapEventType_Image: return "eLeapEventType_Image";
            case eLeapEventType_PointMappingChange: return "eLeapEventType_PointMappingChange";
            case eLeapEventType_LogEvents: return "eLeapEventType_LogEvents";
            case eLeapEventType_HeadPose: return "eLeapEventType_HeadPose";
            default: return "eLeapEventType_Invalid(Unknown)";
            }
        }(input));
    };
};

template <> struct [[maybe_unused]] std::formatter<LEAP_VERSION> {
    [[maybe_unused]] static constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

    [[maybe_unused]] static auto format(const LEAP_VERSION& input, std::format_context& ctx) {
        return std::format_to(ctx.out(), "{}.{}.{}", input.major, input.minor, input.patch);
    }
};

class OvrPropertiesWrapper {
  public:
    [[maybe_unused]] static auto FromDeviceId(const vr::TrackedDeviceIndex_t deviceId) -> OvrPropertiesWrapper {
        return OvrPropertiesWrapper{vr::VRProperties()->TrackedDeviceToPropertyContainer(deviceId)};
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const char* value) const -> void {
        vr::VRProperties()->SetStringProperty(handle, property, value);
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const std::string& value) const -> void {
        vr::VRProperties()->SetStringProperty(handle, property, value.c_str());
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const std::string_view& value) const -> void {
        vr::VRProperties()->SetStringProperty(handle, property, std::string{value}.c_str());
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const bool value) const -> void {
        vr::VRProperties()->SetBoolProperty(handle, property, value);
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const float value) const -> void {
        vr::VRProperties()->SetFloatProperty(handle, property, value);
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const double value) const -> void {
        vr::VRProperties()->SetDoubleProperty(handle, property, value);
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const int32_t value) const -> void {
        vr::VRProperties()->SetInt32Property(handle, property, value);
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const uint64_t value) const -> void {
        vr::VRProperties()->SetUint64Property(handle, property, value);
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const vr::HmdVector2_t value) const -> void {
        vr::VRProperties()->SetVec2Property(handle, property, value);
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const vr::HmdVector3_t value) const -> void {
        vr::VRProperties()->SetVec3Property(handle, property, value);
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const vr::HmdVector4_t value) const -> void {
        vr::VRProperties()->SetVec4Property(handle, property, value);
    }

    [[maybe_unused]] auto SetError(const vr::ETrackedDeviceProperty property, const vr::ETrackedPropertyError error) const -> void {
        vr::VRProperties()->SetPropertyError(handle, property, error);
    }

    [[maybe_unused]] auto Erase(const vr::ETrackedDeviceProperty property) const -> void {
        vr::VRProperties()->EraseProperty(handle, property);
    }

  private:
    explicit OvrPropertiesWrapper(const vr::PropertyContainerHandle_t handle) : handle{handle} {};

    vr::PropertyContainerHandle_t handle;
};

class OvrSettings {
  public:
    [[maybe_unused]] [[nodiscard]] static auto GetString(const std::string_view key) -> std::string {
        std::array<char, 4096> value{};
        vr::VRSettings()->GetString(kUltraleapSection, std::string{key}.c_str(), value.data(), value.size());
        return std::string{value.data()};
    }

    [[maybe_unused]] [[nodiscard]] static auto GetFloat(const std::string_view key) -> float {
        return vr::VRSettings()->GetFloat(kUltraleapSection, std::string{key}.c_str());
    }

    [[maybe_unused]] [[nodiscard]] static auto GetInt32(const std::string_view key) -> int32_t {
        return vr::VRSettings()->GetInt32(kUltraleapSection, std::string{key}.c_str());
    }

    [[maybe_unused]] [[nodiscard]] static auto GetBool(const std::string_view key) -> bool {
        return vr::VRSettings()->GetBool(kUltraleapSection, std::string{key}.c_str());
    }

    [[maybe_unused]] static auto Set(const std::string_view key, const std::string_view value) -> void {
        vr::VRSettings()->SetString(kUltraleapSection, std::string{key}.c_str(), std::string{value}.c_str());
    }

    [[maybe_unused]] static auto Set(const std::string_view key, const float value) -> void {
        vr::VRSettings()->SetFloat(kUltraleapSection, std::string{key}.c_str(), value);
    }

    [[maybe_unused]] static auto Set(const std::string_view key, const int32_t value) -> void {
        vr::VRSettings()->SetInt32(kUltraleapSection, std::string{key}.c_str(), value);
    }

    [[maybe_unused]] static auto Set(const std::string_view key, const bool value) -> void {
        vr::VRSettings()->SetBool(kUltraleapSection, std::string{key}.c_str(), value);
    }

  private:
    static constexpr auto kUltraleapSection = "driver_ultraleap";
};

constexpr vr::DriverPose_t kDeviceConnectedPose{
    .result = vr::TrackingResult_Running_OK,
    .poseIsValid = false,
    .deviceIsConnected = true,
};

constexpr vr::DriverPose_t kDeviceDisconnectedPose{
    .result = vr::TrackingResult_Running_OK,
    .poseIsValid = false,
    .deviceIsConnected = false,
};
