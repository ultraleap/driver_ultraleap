#pragma once

#include <openvr_driver.h>

class OvrProperties {
  public:
    [[maybe_unused]] static OvrProperties FromDeviceId(vr::TrackedDeviceIndex_t deviceId) {
        return OvrProperties{vr::VRProperties()->TrackedDeviceToPropertyContainer(deviceId)};
    }

    [[maybe_unused]] void Set(vr::ETrackedDeviceProperty property, std::string_view value) const {
        vr::VRProperties()->SetStringProperty(handle, property, std::string{value}.c_str());
    }

    [[maybe_unused]] void Set(vr::ETrackedDeviceProperty property, bool value) const {
        vr::VRProperties()->SetBoolProperty(handle, property, value);
    }

    [[maybe_unused]] void Set(vr::ETrackedDeviceProperty property, float value) const {
        vr::VRProperties()->SetFloatProperty(handle, property, value);
    }

    [[maybe_unused]] void Set(vr::ETrackedDeviceProperty property, double value) const {
        vr::VRProperties()->SetDoubleProperty(handle, property, value);
    }

    [[maybe_unused]] void Set(vr::ETrackedDeviceProperty property, int32_t value) const {
        vr::VRProperties()->SetInt32Property(handle, property, value);
    }

    [[maybe_unused]] void Set(vr::ETrackedDeviceProperty property, uint64_t value) const {
        vr::VRProperties()->SetUint64Property(handle, property, value);
    }

    [[maybe_unused]] void Set(vr::ETrackedDeviceProperty property, vr::HmdVector2_t value) const {
        vr::VRProperties()->SetVec2Property(handle, property, value);
    }

    [[maybe_unused]] void Set(vr::ETrackedDeviceProperty property, vr::HmdVector3_t value) const {
        vr::VRProperties()->SetVec3Property(handle, property, value);
    }

    [[maybe_unused]] void Set(vr::ETrackedDeviceProperty property, vr::HmdVector4_t value) const {
        vr::VRProperties()->SetVec4Property(handle, property, value);
    }

    [[maybe_unused]] void SetError(vr::ETrackedDeviceProperty property, vr::ETrackedPropertyError error) const {
        vr::VRProperties()->SetPropertyError(handle, property, error);
    }

    [[maybe_unused]] void Erase(vr::ETrackedDeviceProperty property) const {
        vr::VRProperties()->EraseProperty(handle, property);
    }

  private:
    explicit OvrProperties(vr::PropertyContainerHandle_t handle) : handle{handle} {};

    vr::PropertyContainerHandle_t handle;
};

