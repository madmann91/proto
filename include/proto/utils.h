#ifndef PROTO_UTILS_H
#define PROTO_UTILS_H

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <string>
#include <fstream>

#if defined(__clang__)
#define PROTO_ENABLE_FP_CONTRACT \
    _Pragma("clang diagnostic push") \
    _Pragma("clang diagnostic ignored \"-Wunknown-pragmas\"") \
    _Pragma("STDC FP_CONTRACT ON") \
    _Pragma("clang diagnostic pop")
#else
#define PROTO_ENABLE_FP_CONTRACT
#endif

namespace proto {

inline float fast_mul_add(float a, float b, float c) {
#ifdef FP_FAST_FMAF
    return std::fmaf(a, b, c);
#else
    PROTO_ENABLE_FP_CONTRACT
    return a * b + c;
#endif
}

inline double fast_mul_add(double a, double b, double c) {
#ifdef FP_FAST_FMA
    return std::fma(a, b, c);
#else
    PROTO_ENABLE_FP_CONTRACT
    return a * b + c;
#endif
}

/// Computes the inverse of the given value, making sure not to divide by 0.
/// For very small values, this returns `1 / epsilon`.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
T safe_inverse(T x) {
    return std::fabs(x) <= std::numeric_limits<T>::epsilon()
        ? T(1) / std::numeric_limits<T>::epsilon() : T(1) / x;
}

/// Robust min function, guaranteed to return a non-NaN result if the right argument is not a NaN.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
T robust_min(T a, T b) { return a < b ? a : b; }

/// Robust max function, guaranteed to return a non-NaN result if the right argument is not a NaN.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
T robust_max(T a, T b) { return a > b ? a : b; }

/// Template to select an integer type based on the number of bits.
template <size_t Bits> struct SizedInteger {};
template <> struct SizedInteger<8 > { using Type = int8_t;  };
template <> struct SizedInteger<16> { using Type = int16_t; };
template <> struct SizedInteger<32> { using Type = int32_t; };
template <> struct SizedInteger<64> { using Type = int64_t; };

/// Reads a file in its entirety as a string.
inline std::string read_file(const std::string& file_name) {
    std::ifstream is(file_name, std::ifstream::binary);
    if (!is)
        throw std::runtime_error("Cannot read file \"" + file_name + "\"");
    return std::string(std::istreambuf_iterator<char>(is), std::istreambuf_iterator<char>());
}

} // namespace proto

#endif
