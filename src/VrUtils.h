#pragma once

#include <array>
#include <format>
#include <span>
#include <string>

#include <openvr_driver.h>

#include "VrMaths.h"
#include "VrLogging.h"

// Platform specific export macros.
#if defined(_WIN32)
#define OVR_EXPORT extern "C" __declspec(dllexport)
#define OVR_IMPORT extern "C" __declspec(dllimport)
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define OVR_EXPORT extern "C" __attribute__((visibility("default")))
#define OVR_IMPORT extern "C"
#else
#error "Unsupported Platform"
#endif

typedef int32_t BoneIndex_t;
const BoneIndex_t INVALID_BONEINDEX = -1;
enum class VrHandSkeletonBone : BoneIndex_t
{
    Root = 0,
    Wrist,
    Thumb0,
    Thumb1,
    Thumb2,
    Thumb3,
    IndexFinger0,
    IndexFinger1,
    IndexFinger2,
    IndexFinger3,
    IndexFinger4,
    MiddleFinger0,
    MiddleFinger1,
    MiddleFinger2,
    MiddleFinger3,
    MiddleFinger4,
    RingFinger0,
    RingFinger1,
    RingFinger2,
    RingFinger3,
    RingFinger4,
    PinkyFinger0,
    PinkyFinger1,
    PinkyFinger2,
    PinkyFinger3,
    PinkyFinger4,
    Aux_Thumb,
    Aux_IndexFinger,
    Aux_MiddleFinger,
    Aux_RingFinger,
    Aux_PinkyFinger,
    Count
};


class VrBooleanInputComponent {
  public:
    VrBooleanInputComponent() = default;
    VrBooleanInputComponent(const vr::VRInputComponentHandle_t handle, const char* name) : handle{handle}, name{name} {}

    auto Update(const bool new_value, const double time_offset = 0) -> void {
        value = new_value;
        if (vr::VRDriverInput()->UpdateBooleanComponent(handle, new_value, time_offset) != vr::VRInputError_None) {
            throw std::runtime_error(std::format("Failed update boolean input component \"{}\"", name));
        }
    }

  private:
    vr::VRInputComponentHandle_t handle = vr::k_ulInvalidInputComponentHandle;
    std::string name = "/input/invalid";
    bool value = false;
};

class VrScalarInputComponent {
  public:
    VrScalarInputComponent() = default;
    VrScalarInputComponent(
        const vr::VRInputComponentHandle_t handle,
        const char* name,
        const vr::EVRScalarType type,
        const vr::EVRScalarUnits units
    )
        : handle{handle},
          name{name},
          type{type},
          units{units} {}

    [[nodiscard]] auto Handle() const -> vr::VRInputComponentHandle_t { return handle; }

    auto Update(const float new_value, const double time_offset = 0) -> void {
        value = new_value;
        if (vr::VRDriverInput()->UpdateScalarComponent(handle, new_value, time_offset) != vr::VRInputError_None) {
            throw std::runtime_error(std::format("Failed update scalar input component \"{}\"", name));
        }
    }

  private:
    vr::VRInputComponentHandle_t handle = vr::k_ulInvalidInputComponentHandle;
    std::string name = "/input/invalid";
    vr::EVRScalarType type = vr::VRScalarType_Absolute;
    vr::EVRScalarUnits units = vr::VRScalarUnits_NormalizedOneSided;
    float value = 0;
};

class VrSkeletonInputComponent {
  public:
    VrSkeletonInputComponent() = default;
    VrSkeletonInputComponent(const vr::VRInputComponentHandle_t handle, const char* name) : handle{handle}, name{name} {}

    auto Update(const vr::EVRSkeletalMotionRange motion_range, const std::span<const vr::VRBoneTransform_t> bone_transforms) -> void {
        if (vr::VRDriverInput()->UpdateSkeletonComponent(handle, motion_range, bone_transforms.data(), bone_transforms.size())
            != vr::VRInputError_None) {
            throw std::runtime_error(std::format("Failed update skeleton input component \"{}\"", name));
        }
    }

  private:
    vr::VRInputComponentHandle_t handle = vr::k_ulInvalidInputComponentHandle;
    std::string name = "/input/invalid";
};

class VrDeviceProperties {
  public:
    [[maybe_unused]] static auto FromDeviceId(const vr::TrackedDeviceIndex_t deviceId) -> VrDeviceProperties {
        const auto handle = vr::VRProperties()->TrackedDeviceToPropertyContainer(deviceId);
        if (handle == vr::k_ulInvalidPropertyContainer) {
            throw std::runtime_error(std::format("Invalid property container from device id {}", deviceId));
        }
        return VrDeviceProperties{handle};
    }

    [[nodiscard]] auto Handle() const -> vr::PropertyContainerHandle_t { return handle; }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const char* value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetStringProperty(handle, property, value));
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const std::string& value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetStringProperty(handle, property, value.c_str()));
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const std::string_view& value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetStringProperty(handle, property, std::string{value}.c_str()));
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const bool value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetBoolProperty(handle, property, value));
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const float value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetFloatProperty(handle, property, value));
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const double value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetDoubleProperty(handle, property, value));
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const int32_t value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetInt32Property(handle, property, value));
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const uint64_t value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetUint64Property(handle, property, value));
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const vr::HmdVector2_t value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetVec2Property(handle, property, value));
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const vr::HmdVector3_t value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetVec3Property(handle, property, value));
    }

    [[maybe_unused]] auto Set(const vr::ETrackedDeviceProperty property, const vr::HmdVector4_t value) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetVec4Property(handle, property, value));
    }

    [[maybe_unused]] auto SetError(const vr::ETrackedDeviceProperty property, const vr::ETrackedPropertyError error) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->SetPropertyError(handle, property, error));
    }

    [[maybe_unused]] auto Erase(const vr::ETrackedDeviceProperty property) const -> void {
        ThrowOnSetError(property, vr::VRProperties()->EraseProperty(handle, property));
    }

    [[maybe_unused]] auto CreateBooleanInput(const char* name) const -> VrBooleanInputComponent {
        vr::VRInputComponentHandle_t component_handle;
        if (vr::VRDriverInput()->CreateBooleanComponent(handle, name, &component_handle) != vr::VRInputError_None) {
            throw std::runtime_error(std::format("Failed to create scalar input component \"{}\"", name));
        }
        return VrBooleanInputComponent{component_handle, name};
    }

    [[maybe_unused]] auto CreateAbsoluteScalarInput(const char* name, const vr::EVRScalarUnits units) const
        -> VrScalarInputComponent {
        vr::VRInputComponentHandle_t component_handle;
        if (vr::VRDriverInput()->CreateScalarComponent(handle, name, &component_handle, vr::VRScalarType_Absolute, units)
            != vr::VRInputError_None) {
            throw std::runtime_error(std::format("Failed to create scalar input component \"{}\"", name));
        }
        return VrScalarInputComponent{component_handle, name, vr::VRScalarType_Absolute, units};
    }

    [[maybe_unused]] auto CreateRelativeScalarInput(const char* name, const vr::EVRScalarUnits units) const
        -> VrScalarInputComponent {
        vr::VRInputComponentHandle_t component_handle;
        if (vr::VRDriverInput()->CreateScalarComponent(handle, name, &component_handle, vr::VRScalarType_Relative, units)
            != vr::VRInputError_None) {
            throw std::runtime_error(std::format("Failed to create scalar input component \"{}\"", name));
        }
        return VrScalarInputComponent{component_handle, name, vr::VRScalarType_Relative, units};
    }

    // TODO: Wrap and re-enable?
    // [[maybe_unused]] auto CreateHapticOutput(const char* name) const -> vr::VRInputComponentHandle_t {
    //     vr::VRInputComponentHandle_t component_handle;
    //     if (vr::VRDriverInput()->CreateHapticComponent(handle, name, &component_handle) != vr::VRInputError_None) {
    //         throw std::runtime_error(std::format("Failed to create haptic output component \"{}\"", name));
    //     }
    //     return component_handle;
    // }

    [[maybe_unused]] auto CreateSkeletonInput(
        const char* name,
        const char* skeleton_path,
        const char* base_pose_path,
        const vr::EVRSkeletalTrackingLevel tracking_level
    ) const -> VrSkeletonInputComponent {
        vr::VRInputComponentHandle_t component_handle;
        if (vr::VRDriverInput()
                ->CreateSkeletonComponent(handle, name, skeleton_path, base_pose_path, tracking_level, nullptr, 0, &component_handle)
            != vr::VRInputError_None) {
            throw std::runtime_error(std::format("Failed to create skeleton input component \"{}\"", name));
        }
        return VrSkeletonInputComponent{component_handle, name};
    }

  private:
    explicit VrDeviceProperties(const vr::PropertyContainerHandle_t handle) : handle{handle} {}

    static auto ThrowOnSetError(const vr::ETrackedDeviceProperty property, vr::ETrackedPropertyError error) -> void {
        if (error != vr::TrackedProp_Success) {
            throw std::runtime_error(std::format("Failed to set property id {} with error {}", static_cast<int>(property), error));
        }
    }

    vr::PropertyContainerHandle_t handle;
};

class VrSettings {
  public:
    [[maybe_unused]] [[nodiscard]] static auto GetString(const std::string_view key) -> std::string {
        std::array<char, 4096> value{};
        vr::VRSettings()->GetString(kUltraleapSection, std::string{key}.c_str(), value.data(), value.size());
        return std::string{value.data()};
    }

    template <typename T> static auto Get(std::string_view key) -> T = delete;

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

template <> [[nodiscard]] inline auto VrSettings::Get<std::string>(const std::string_view key) -> std::string {
    std::array<char, 4096> value{};
    vr::VRSettings()->GetString(kUltraleapSection, std::string{key}.c_str(), value.data(), value.size());
    return std::string{value.data()};
}

template <> [[nodiscard]] inline auto VrSettings::Get<float>(const std::string_view key) -> float {
    return vr::VRSettings()->GetFloat(kUltraleapSection, std::string{key}.c_str());
}

template <> [[nodiscard]] inline auto VrSettings::Get<int32_t>(const std::string_view key) -> int32_t {
    return vr::VRSettings()->GetInt32(kUltraleapSection, std::string{key}.c_str());
}

template <> [[nodiscard]] inline auto VrSettings::Get<bool>(const std::string_view key) -> bool {
    return vr::VRSettings()->GetBool(kUltraleapSection, std::string{key}.c_str());
}

class HmdPose {
  public:
    [[nodiscard]] static auto Get(const double time_offset = 0) -> HmdPose {
        HmdPose pose{};
        vr::VRServerDriverHost()->GetRawTrackedDevicePoses(static_cast<float>(time_offset), &pose.hmd_pose, 1);
        if (pose.hmd_pose.bPoseIsValid && pose.hmd_pose.eTrackingResult == vr::TrackingResult_Running_OK) {
            pose.hmd_position = VrVec3::FromMatrix(pose.hmd_pose.mDeviceToAbsoluteTracking);
            pose.hmd_orientation = VrQuat::FromMatrix(pose.hmd_pose.mDeviceToAbsoluteTracking);
        }
        return pose;
    }

    [[nodiscard]] auto IsValid() const -> bool { return hmd_pose.bPoseIsValid; }
    [[nodiscard]] auto Position() const -> const VrVec3& { return hmd_position; }
    [[nodiscard]] auto Orientation() const -> const VrQuat& { return hmd_orientation; }
    [[nodiscard]] auto Velocity() const -> const VrVec3& { return hmd_pose.vVelocity; }
    [[nodiscard]] auto AngularVelocity() const -> const VrVec3& { return hmd_pose.vAngularVelocity; }

  private:
    HmdPose() = default;

    vr::TrackedDevicePose_t hmd_pose{};
    VrVec3 hmd_position{};
    VrQuat hmd_orientation{1.0f, 0, 0, 0};
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

constexpr vr::DriverPose_t kDeviceErrorPose{
    .result = vr::TrackingResult_Uninitialized,
    .poseIsValid = false,
    .deviceIsConnected = false,
};

constexpr vr::DriverPose_t kDefaultPose{
    // Timestamp
    .poseTimeOffset = 0,

    // Reference space transforms.
    .qWorldFromDriverRotation = {1.0f, 0, 0, 0},
    .vecWorldFromDriverTranslation = {0, 0, 0},
    .qDriverFromHeadRotation = {1.0f, 0, 0, 0},
    .vecDriverFromHeadTranslation = {0, 0, 0},

    // Spatial Position.
    .vecPosition = {0, 0, 0},
    .vecVelocity = {0, 0, 0},
    .qRotation = {1.0f, 0, 0, 0},
    .vecAngularVelocity = {0, 0, 0},
    .vecAngularAcceleration = {0, 0, 0},

    // Status & Timestamps.
    .result = vr::TrackingResult_Uninitialized,
    .poseIsValid = true,
    .willDriftInYaw = false,
    .shouldApplyHeadModel = false,
    .deviceIsConnected = false,
};