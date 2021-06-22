#ifndef PROTO_RANDOM_H
#define PROTO_RANDOM_H

#include <cmath>
#include <utility>
#include <numbers>
#include <type_traits>

#include <proto/vec.h>
#include <proto/utils.h>

namespace proto {

/// Evaluates the probability density function used for cosine-weighted hemisphere sampling.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline T cosine_hemisphere_pdf(T cos) {
    return cos * std::numbers::inv_pi_v<T>;
}

/// Evaluates the probability density function used for cosine-power-weighted hemisphere sampling.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline T cosine_power_hemisphere_pdf(T power, T cos) {
    return std::pow(cos, power) * (power + T(1)) * T(0.5) * std::numbers::inv_pi_v<T>;
}

/// Evaluates the probability to uniformly sample a direction on a sphere.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline constexpr T uniform_sphere_pdf() {
    return T(0.25) * std::numbers::inv_pi_v<T>;
}

/// Samples a sphere uniformly.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline std::pair<Vec3<T>, T> sample_uniform_sphere(T u, T v) {
    auto cos = T(2) * v - T(1);
    auto sin = std::sqrt(T(1) - cos * cos);
    auto phi = T(2) * std::numbers::pi_v<T> * u;
    auto x = sin * std::cos(phi);
    auto y = sin * std::sin(phi);
    return std::pair { Vec3<T>(x, y, cos), uniform_sphere_pdf<T>() };
}

/// Samples a hemisphere proportionally to the cosine with the normal.
/// The hemisphere is oriented with the z-axis pointing upwards.
/// Returns the sampled direction, along with the value of the probability
/// density function for that direction.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline std::pair<Vec3<T>, T> sample_cosine_hemisphere(T u, T v) {
    auto cos = std::sqrt(T(1) - v);
    auto sin = std::sqrt(v);
    auto phi = T(2) * std::numbers::pi_v<T> * u;
    auto x = sin * std::cos(phi);
    auto y = sin * std::sin(phi);
    return std::pair { Vec3<T>(x, y, cos), cosine_hemisphere_pdf(cos) };
}

/// Samples a hemisphere proportionally to the cosine lobe spanned by the normal.
/// The hemisphere is oriented with the z-axis pointing upwards.
/// Returns the sampled direction, along with the value of the probability
/// density function for that direction.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline std::pair<Vec3<T>, T> sample_cosine_power_hemisphere(T power, T u, T v) {
    auto cos = std::pow(v, T(1) / (power + T(1)));
    auto sin = std::sqrt(T(1) - cos * cos);
    auto phi = T(2) * std::numbers::pi_v<T> * u;
    auto x = sin * std::cos(phi);
    auto y = sin * std::sin(phi);
    return std::pair { Vec3<T>(x, y, cos), cosine_power_hemisphere_pdf(power, cos) };
}

} // namespace proto

#endif
