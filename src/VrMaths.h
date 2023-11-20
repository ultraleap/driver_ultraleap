
#pragma once

// This contains many deliberate implicit constructors and conversion operators so disable warnings about that.
// ReSharper disable CppNonExplicitConvertingConstructor
// ReSharper disable CppNonExplicitConversionOperator

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

// Concepts to allow interop with different OpenVR and LeapC types.
template <typename T>
concept Quaternion = requires(T q) {
    { q.w } -> std::assignable_from<double&>;
    { q.x } -> std::assignable_from<double&>;
    { q.y } -> std::assignable_from<double&>;
    { q.z } -> std::assignable_from<double&>;
};

template <typename T>
concept Vector3 = requires(T v) {
    { v.v } -> std::convertible_to<float(&)[3]>;
};

// Forward declarations.
class VrVec3;
class VrQuat;

class VrQuat : public glm::dquat {
  public:
    VrQuat() = default;
    VrQuat(const Quaternion auto& q) : glm::dquat{q.w, q.x, q.y, q.z} {} // NOLINT(*-explicit-constructor)
    explicit VrQuat(const glm::dquat& q) : glm::dquat{q} {}
    VrQuat(value_type w, value_type x, value_type y, value_type z);

    // Implicit conversion operators.
    operator vr::HmdQuaternion_t() const { return vr::HmdQuaternion_t{w, x, y, z}; } // NOLINT(*-explicit-constructor)
    operator vr::HmdQuaternionf_t() const {                                          // NOLINT(*-explicit-constructor)
        return vr::HmdQuaternionf_t{static_cast<float>(w), static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    }
    operator glm::dquat() const { return *this; }

    [[nodiscard]] auto operator-() const -> VrQuat { return VrQuat{-static_cast<const glm::dquat>(*this)}; }
    [[nodiscard]] auto operator+(const VrQuat& q) const -> VrQuat { return VrQuat{static_cast<const glm::dquat>(*this) + q}; }
    [[nodiscard]] auto operator-(const VrQuat& q) const -> VrQuat { return VrQuat{static_cast<const glm::dquat>(*this) - q}; }
    [[nodiscard]] auto operator*(const VrQuat& q) const -> VrQuat { return VrQuat{static_cast<const glm::dquat>(*this) * q}; }
    [[nodiscard]] auto operator+=(const VrQuat& q) -> VrQuat& { return *this = *this + q; }
    [[nodiscard]] auto operator-=(const VrQuat& q) -> VrQuat& { return *this = *this - q; }
    [[nodiscard]] auto operator*=(const VrQuat& q) -> VrQuat& { return *this = *this * q; }

    [[nodiscard]] auto Inverse() const -> VrQuat { return glm::inverse(*this); }
    [[nodiscard]] auto Normalized() const -> VrQuat { return glm::normalize(*this); }
    [[nodiscard]] auto Conjugate() const -> VrQuat { return glm::conjugate(*this); }
    [[nodiscard]] auto Length() const -> value_type { return glm::length(static_cast<glm::dquat>(*this)); }
    [[nodiscard]] auto Normalize() -> VrQuat& { return *this = Normalized(); }

    [[nodiscard]] static auto Dot(const VrQuat& a, const VrQuat& b) -> value_type {
        return glm::dot(static_cast<glm::dquat>(a), b);
    }
    [[nodiscard]] static auto Cross(const VrQuat& a, const VrQuat& b) -> VrQuat { return glm::cross(a, b); }
    [[nodiscard]] static auto Slerp(const VrQuat& a, const VrQuat& b, const std::floating_point auto& alpha) -> VrQuat& {
        return glm::slerp(a, b, static_cast<value_type>(alpha));
    }

    [[nodiscard]] static auto FromMatrix(const vr::HmdMatrix34_t& matrix) -> VrQuat {
        return VrQuat{glm::quat_cast(glm::transpose(glm::mat3x3{glm::make_mat4x3(matrix.m)}))};
    }
    [[nodiscard]] static auto FromMatrix(const vr::HmdMatrix33_t& matrix) -> VrQuat {
        return VrQuat{glm::quat_cast(glm::transpose(glm::mat3x3{glm::make_mat3x3(matrix.m)}))};
    }
    // [[nodiscard]] static auto FromMatrix(const vr::HmdMatrix44_t& matrix) -> VrQuat {
    //     return VrQuat{glm::quat_cast(glm::transpose(glm::mat3x3{glm::make_mat4x4(matrix.m)}))};
    // }

    [[nodiscard]] static auto FromEulerAngles(const double roll, const double pitch, const double yaw) -> VrQuat {
        return VrQuat{glm::dquat{glm::dvec3{roll, pitch, yaw}}};
    }
    [[nodiscard]] static auto FromAxisAngle(const VrVec3& axis, const std::floating_point auto& angle) -> VrQuat {
        return glm::rotate(axis, static_cast<value_type>(angle));
    }

    static const VrQuat Identity;
};

const VrQuat VrQuat::Identity = {1, 0, 0, 0};

class VrVec3 : public glm::dvec3 {
  public:
    // Constructors from different vector types (including deliberately implicit versions).
    VrVec3() = default;
    VrVec3(const Vector3 auto& v) : glm::dvec3{v.v[0], v.v[1], v.v[2]} {} // NOLINT(*-explicit-constructor)
    VrVec3(value_type x, value_type y, value_type z) : glm::dvec3{x, y, z} {}
    explicit VrVec3(const glm::dvec3& v) : glm::dvec3{v} {}

    // Implicit conversion operators: NOLINTNEXTLINE(*-explicit-constructor).
    operator vr::HmdVector3_t() const { return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)}; }
    operator vr::HmdVector3d_t() const { return {x, y, z}; } // NOLINT(*-explicit-constructor)
    operator glm::dvec3() const { return *this; }

    [[nodiscard]] auto operator-() const -> VrVec3 { return VrVec3{static_cast<const glm::dvec3>(*this)}; }
    [[nodiscard]] auto operator+(const VrVec3& v) const -> VrVec3 { return VrVec3{*this + static_cast<const glm::dvec3>(v)}; }
    [[nodiscard]] auto operator-(const VrVec3& v) const -> VrVec3 { return VrVec3{*this - static_cast<const glm::dvec3>(v)}; }
    [[nodiscard]] auto operator*(const VrVec3& v) const -> VrVec3 { return VrVec3{*this * static_cast<const glm::dvec3>(v)}; }
    [[nodiscard]] auto operator/(const VrVec3& v) const -> VrVec3 { return VrVec3{*this / static_cast<const glm::dvec3>(v)}; }
    [[nodiscard]] auto operator+=(const VrVec3& v) -> VrVec3& { return *this = *this + v; }
    [[nodiscard]] auto operator-=(const VrVec3& v) -> VrVec3& { return *this = *this - v; }
    [[nodiscard]] auto operator*=(const VrVec3& v) -> VrVec3& { return *this = *this * v; }
    [[nodiscard]] auto operator/=(const VrVec3& v) -> VrVec3& { return *this = *this / v; }

    auto operator*(const std::floating_point auto scale) const -> VrVec3 { return VrVec3{*this * glm::dvec3{scale}}; }
    auto operator/(const std::floating_point auto scale) const -> VrVec3 { return VrVec3{*this / glm::dvec3{scale}}; }
    auto operator*=(const std::floating_point auto scale) const -> VrVec3& { return *this * glm::dvec3{scale}; }
    auto operator/=(const std::floating_point auto scale) const -> VrVec3& { return *this / glm::dvec3{scale}; }

    [[nodiscard]] auto Dot(const Vector3 auto& other) const -> double { return glm::dot(*this, glm::make_vec3(other.v)); }
    [[nodiscard]] auto Dot(const VrVec3& other) const -> double { return glm::dot(*this, glm::make_vec3(other.data.data)); }

    auto CopyToArray(double destination[3]) const -> void { std::memcpy(destination, this->data.data, sizeof(destination)); }

    static auto FromMatrix(const vr::HmdMatrix34_t& matrix) -> VrVec3 {
        return VrVec3{matrix.m[0][3], matrix.m[1][3], matrix.m[2][3]};
    }
    static auto FromMatrix(const vr::HmdMatrix44_t& matrix) -> VrVec3 {
        return VrVec3{matrix.m[0][3], matrix.m[1][3], matrix.m[2][3]};
    }

    static const VrVec3 Zero;
    static const VrVec3 Up;
    static const VrVec3 Down;
    static const VrVec3 Left;
    static const VrVec3 Right;
    static const VrVec3 Forward;
    static const VrVec3 Backward;
};

const VrVec3 VrVec3::Zero = {0, 0, 0};
const VrVec3 VrVec3::Left = {-1, 0, 0};
const VrVec3 VrVec3::Right = {1, 0, 0};
const VrVec3 VrVec3::Up = {0, 1, 0};
const VrVec3 VrVec3::Down = {0, -1, 0};
const VrVec3 VrVec3::Forward = {0, 0, -1};
const VrVec3 VrVec3::Backward = {0, 0, 1};

// Global operators to allow operation on standard types and interop between VrVec3 and VrQuat.
static auto operator*(const VrVec3& lhs, const VrQuat& rhs) -> VrVec3 {
    return VrVec3{static_cast<glm::dvec3>(lhs) * rhs};
}

static auto operator+(const Quaternion auto& lhs, const VrQuat& rhs) -> VrQuat {
    return VrQuat{lhs} + rhs;
}
static auto operator-(const Quaternion auto& lhs, const VrQuat& rhs) -> VrQuat {
    return VrQuat{lhs} - rhs;
}
static auto operator*(const Quaternion auto& lhs, const VrQuat& rhs) -> VrQuat {
    return VrQuat{lhs} * rhs;
}

static auto operator*(const Vector3 auto& lhs, const VrQuat& rhs) -> VrVec3 {
    return VrVec3{lhs} * rhs;
}
static auto operator/(const Vector3 auto& lhs, const VrQuat& rhs) -> VrVec3 {
    return VrVec3{lhs} / rhs;
}

static auto operator+(const Vector3 auto& lhs, const VrVec3& rhs) -> VrVec3 {
    return VrVec3{lhs} + rhs;
}
static auto operator-(const Vector3 auto& lhs, const VrVec3& rhs) -> VrVec3 {
    return VrVec3{lhs} - rhs;
}
static auto operator*(const Vector3 auto& lhs, const VrVec3& rhs) -> VrVec3 {
    return VrVec3{lhs} * rhs;
}
static auto operator/(const Vector3 auto& lhs, const VrVec3& rhs) -> VrVec3 {
    return VrVec3{lhs} / rhs;
}
