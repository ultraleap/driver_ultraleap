#pragma once

#include <format>
#include <string>

#include <openvr_driver.h>
#include "LeapC.h"

#define OVR_LOG(...) DriverLog::Log(__VA_ARGS__)

class DriverLog {
  public:
    static void Init();
    template<typename... T> static void Log(std::format_string<T...> fmt, T&&... args) {
        pLogFile->Log(std::format(fmt, std::forward<T>(args)...).c_str());
    }
  private:
    static vr::IVRDriverLog* pLogFile;
};

template <> struct std::formatter<eLeapDevicePID> {
    constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

    auto format(const eLeapDevicePID& input, std::format_context& ctx) {
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

template <> struct std::formatter<eLeapRS> {
    constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

    auto format(const eLeapRS& input, std::format_context& ctx) {
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

template <> struct std::formatter<eLeapEventType> {
    constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

    auto format(const eLeapEventType& input, std::format_context& ctx) {
        return std::format_to(ctx.out(), "{}", [](const eLeapEventType &e) constexpr {
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

template <> struct std::formatter<LEAP_VERSION> {
    constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

    auto format(const LEAP_VERSION& input, std::format_context& ctx) {
        return std::format_to(ctx.out(), "{}.{}.{}", input.major, input.minor, input.patch);
    }
};