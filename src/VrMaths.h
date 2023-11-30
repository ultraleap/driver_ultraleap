
#pragma once

// This contains many deliberate implicit constructors and conversion operators so disable warnings about that.
// ReSharper disable CppNonExplicitConvertingConstructor
// ReSharper disable CppNonExplicitConversionOperator

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

using namespace glm;

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

class VrQuat : public dquat {
  public:
    VrQuat() = default;
    VrQuat(const Quaternion auto& q) : dquat{q.w, q.x, q.y, q.z} {} // NOLINT(*-explicit-constructor)
    explicit VrQuat(const dquat& q) : dquat{q} {}
    VrQuat(const value_type w, const value_type x, const value_type y, const value_type z) : dquat{w, x, y, z} {}

    // Implicit conversion operators.
    operator vr::HmdQuaternion_t() const { return vr::HmdQuaternion_t{w, x, y, z}; } // NOLINT(*-explicit-constructor)
    operator vr::HmdQuaternionf_t() const {                                          // NOLINT(*-explicit-constructor)
        return vr::HmdQuaternionf_t{static_cast<float>(w), static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    }

    [[nodiscard]] auto operator-() const -> VrQuat { return VrQuat{-static_cast<const dquat>(*this)}; }
    [[nodiscard]] auto operator+(const VrQuat& q) const -> VrQuat { return VrQuat{static_cast<const dquat>(*this) + q}; }
    [[nodiscard]] auto operator-(const VrQuat& q) const -> VrQuat { return VrQuat{static_cast<const dquat>(*this) - q}; }
    [[nodiscard]] auto operator*(const VrQuat& q) const -> VrQuat { return VrQuat{static_cast<const dquat>(*this) * q}; }
    auto operator+=(const VrQuat& q) -> VrQuat& { return *this = *this + q; }
    auto operator-=(const VrQuat& q) -> VrQuat& { return *this = *this - q; }
    auto operator*=(const VrQuat& q) -> VrQuat& { return *this = *this * q; }

    [[nodiscard]] auto Inverse() const -> VrQuat { return inverse(*this); }
    [[nodiscard]] auto Normalized() const -> VrQuat { return normalize(*this); }
    [[nodiscard]] auto Conjugate() const -> VrQuat { return conjugate(*this); }
    [[nodiscard]] auto Length() const -> value_type { return glm::length(static_cast<dquat>(*this)); }
    [[nodiscard]] auto Normalize() -> VrQuat& { return *this = Normalized(); }

    [[nodiscard]] static auto Dot(const VrQuat& a, const VrQuat& b) -> value_type { return dot(static_cast<dquat>(a), b); }
    [[nodiscard]] static auto Cross(const VrQuat& a, const VrQuat& b) -> VrQuat { return cross(a, b); }
    [[nodiscard]] static auto Slerp(const VrQuat& a, const VrQuat& b, const std::floating_point auto& alpha) -> VrQuat& {
        return slerp(a, b, static_cast<value_type>(alpha));
    }

    [[nodiscard]] auto ToEulerAngles() const -> std::tuple<value_type, value_type, value_type> {
        value_type euler_angles[3];
        extractEulerAngleXYZ(mat4_cast(*this), euler_angles[0], euler_angles[1], euler_angles[2]);
        return {euler_angles[0], euler_angles[1], euler_angles[2]};
    }

    [[nodiscard]] static auto FromMatrix(const vr::HmdMatrix34_t& matrix) -> VrQuat {
        return VrQuat{quat_cast(transpose(mat3x3{make_mat3x4(matrix.m[0])}))};
    }
    [[nodiscard]] static auto FromMatrix(const vr::HmdMatrix33_t& matrix) -> VrQuat {
        return VrQuat{quat_cast(transpose(mat3x3{make_mat3x3(matrix.m[0])}))};
    }
    [[nodiscard]] static auto FromMatrix(const vr::HmdMatrix44_t& matrix) -> VrQuat {
        return VrQuat{quat_cast(transpose(make_mat4x4(matrix.m[0])))};
    }

    [[nodiscard]] static auto FromEulerAngles(const double pitch, const double yaw, const double roll) -> VrQuat {
        return VrQuat{dquat{dvec3{pitch, yaw, roll}}};
    }
    [[nodiscard]] static auto FromAxisAngle(const VrVec3& axis, const std::floating_point auto& angle) -> VrQuat {
        return rotate(axis, static_cast<value_type>(angle));
    }

    static const VrQuat Identity;
};

inline const VrQuat VrQuat::Identity = {1.0, 0.0, 0.0, 0.0};

class VrVec3 : public dvec3 {
  public:
    // Constructors from different vector types (including deliberately implicit versions).
    VrVec3() = default;
    VrVec3(const Vector3 auto& v) : dvec3{v.v[0], v.v[1], v.v[2]} {} // NOLINT(*-explicit-constructor)
    VrVec3(value_type x, value_type y, value_type z) : dvec3{x, y, z} {}
    explicit VrVec3(const dvec3& v) : dvec3{v} {}

    // NOLINTNEXTLINE(*-explicit-constructor).
    operator vr::HmdVector3_t() const { return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)}; }
    // NOLINTNEXTLINE(*-explicit-constructor)
    operator vr::HmdVector3d_t() const { return {x, y, z}; }
    // NOLINTNEXTLINE(*-explicit-constructor)
    operator vr::HmdVector4_t() const { return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), 1.0}; }

    [[nodiscard]] auto operator-() const -> VrVec3 { return VrVec3{static_cast<const dvec3>(*this)}; }
    [[nodiscard]] auto operator+(const VrVec3& v) const -> VrVec3 { return VrVec3{*this + static_cast<const dvec3>(v)}; }
    [[nodiscard]] auto operator-(const VrVec3& v) const -> VrVec3 { return VrVec3{*this - static_cast<const dvec3>(v)}; }
    [[nodiscard]] auto operator*(const VrVec3& v) const -> VrVec3 { return VrVec3{*this * static_cast<const dvec3>(v)}; }
    [[nodiscard]] auto operator/(const VrVec3& v) const -> VrVec3 { return VrVec3{*this / static_cast<const dvec3>(v)}; }
    auto operator+=(const VrVec3& v) -> VrVec3& { return *this = *this + v; }
    auto operator-=(const VrVec3& v) -> VrVec3& { return *this = *this - v; }
    auto operator*=(const VrVec3& v) -> VrVec3& { return *this = *this * v; }
    auto operator/=(const VrVec3& v) -> VrVec3& { return *this = *this / v; }

    auto operator*(const std::floating_point auto scale) const -> VrVec3 { return VrVec3{*this * dvec3{scale}}; }
    auto operator/(const std::floating_point auto scale) const -> VrVec3 { return VrVec3{*this / dvec3{scale}}; }
    auto operator*=(const std::floating_point auto scale) const -> VrVec3& { return *this * dvec3{scale}; }
    auto operator/=(const std::floating_point auto scale) const -> VrVec3& { return *this / dvec3{scale}; }

    [[nodiscard]] auto Dot(const Vector3 auto& other) const -> value_type { return dot(*this, make_vec3(other.v)); }
    [[nodiscard]] auto Dot(const VrVec3& other) const -> value_type { return dot(*this, make_vec3(other.data.data)); }
    [[nodiscard]] auto Cross(const Vector3 auto& other) const -> VrVec3 { return cross(*this, make_vec3(other.v)); }
    [[nodiscard]] auto Cross(const VrVec3& other) const -> VrVec3 { return VrVec3{cross(*this, make_vec3(other.data.data))}; }
    [[nodiscard]] auto Length() const -> value_type { return glm::length(static_cast<dvec3>(*this)); }

    auto CopyToArray(double destination[3]) const -> void { std::ranges::copy(this->data.data, destination); }

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

inline const VrVec3 VrVec3::Zero = {0.0, 0.0, 0.0};
inline const VrVec3 VrVec3::Left = {-1.0, 0.0, 0.0};
inline const VrVec3 VrVec3::Right = {1.0, 0.0, 0.0};
inline const VrVec3 VrVec3::Up = {0.0, 1.0, 0.0};
inline const VrVec3 VrVec3::Down = {0.0, -1.0, 0.0};
inline const VrVec3 VrVec3::Forward = {0.0, 0.0, -1.0};
inline const VrVec3 VrVec3::Backward = {0.0, 0.0, 1.0};

// Global operators to allow operation on standard types and interop between VrVec3 and VrQuat.
static auto operator*(const VrVec3& lhs, const VrQuat& rhs) -> VrVec3 {
    return VrVec3{static_cast<dvec3>(lhs) * rhs};
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
