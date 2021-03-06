#ifndef PROTO_UTILS_H
#define PROTO_UTILS_H

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <string>
#include <cstring>
#include <climits>
#include <fstream>
#include <atomic>

#if defined(__GNUC__) || defined(__clang__)
#define proto_restrict      __restrict
#define proto_always_inline __attribute__((always_inline))
#elif defined(_MSC_VER)
#define proto_restrict      __restrict
#define proto_always_inline __forceinline
#else
#define proto_restrict
#define proto_always_inline inline
#endif

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

/// Template to select an integer type based on the number of bits.
template <size_t Bits> struct SizedInteger {};
template <> struct SizedInteger<8 > { using Type = int8_t;  };
template <> struct SizedInteger<16> { using Type = int16_t; };
template <> struct SizedInteger<32> { using Type = int32_t; };
template <> struct SizedInteger<64> { using Type = int64_t; };
template <size_t Bits> using SizedIntegerType = typename SizedInteger<Bits>::Type;

/// Helper function to use FMA instructions, on 32-bit floating-point values.
inline proto_always_inline float fast_mul_add(float a, float b, float c) {
#ifdef FP_FAST_FMAF
    return std::fmaf(a, b, c);
#else
    PROTO_ENABLE_FP_CONTRACT
    return a * b + c;
#endif
}

/// Helper function to use FMA instructions, on 64-bit floating-point values.
inline proto_always_inline double fast_mul_add(double a, double b, double c) {
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
inline proto_always_inline T safe_inverse(T x) {
    return std::fabs(x) <= std::numeric_limits<T>::epsilon()
        ? std::copysign(T(1) / std::numeric_limits<T>::epsilon(), x) : T(1) / x;
}

/// Robust min function, guaranteed to return a non-NaN result if the right argument is not a NaN.
template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
inline proto_always_inline T robust_min(T a, T b) { return a < b ? a : b; }

/// Robust max function, guaranteed to return a non-NaN result if the right argument is not a NaN.
template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
inline proto_always_inline T robust_max(T a, T b) { return a > b ? a : b; }

/// Atomically computes the maximum of two values. Returns the old value.
template <typename T>
inline proto_always_inline T atomic_max(std::atomic<T>& x, T y) {
    auto z = x.load();
    while (z < y && !x.compare_exchange_weak(z, y)) ;
    return z;
}

/// Atomically computes the minimum of two values. Returns the old value.
template <typename T>
inline proto_always_inline T atomic_min(std::atomic<T>& x, T y) {
    auto z = x.load();
    while (z < y && !x.compare_exchange_weak(z, y)) ;
    return z;
}

/// Reinterprets the bits of the given value as another type safely,
/// as long as the size of those types is the same.
template <typename To, typename From>
inline proto_always_inline To as(From from) {
    static_assert(sizeof(To) == sizeof(From));
    To to;
    std::memcpy(&to, &from, sizeof(from));
    return to;
}

/// Adds the given number of ULPs (Unit in the Last Place) to the floating-point argument.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline T add_ulp_magnitude(T x, unsigned ulps) {
    using U = std::make_unsigned_t<SizedIntegerType<sizeof(T) * CHAR_BIT>>;
    return std::isfinite(x) ? as<T>(as<U>(x) + ulps) : x;
}

/// Computes the (rounded-up) log in base-2 of an unsigned integer.
template <typename T, std::enable_if_t<std::is_unsigned_v<T>, int> = 0>
inline constexpr T log2(T i, T p = 0) {
    return (T(1) << p) >= i ? p : log2(i, p + 1);
}

/// Returns the smallest value greater than the first argument that is a multiple of the second.
template <typename T, std::enable_if_t<std::is_unsigned_v<T>, int> = 0>
inline constexpr T round_up(T i, T j) {
    return (i + j - 1) / j * j;
}

/// Returns the number of bits that are equal to zero,
/// starting from the most significant one.
template <typename T, std::enable_if_t<std::is_unsigned<T>::value, int> = 0>
inline proto_always_inline size_t count_leading_zeros(T value) {
    static constexpr size_t bit_count = sizeof(T) * CHAR_BIT;
#if defined(__GNUC__) || defined(__clang__)
    if constexpr (bit_count <= sizeof(unsigned int) * CHAR_BIT)       return __builtin_clz(value);
    if constexpr (bit_count <= sizeof(unsigned long) * CHAR_BIT)      return __builtin_clzl(value);
    if constexpr (bit_count <= sizeof(unsigned long long) * CHAR_BIT) return __builtin_clzll(value);
#elif defined(_MSC_VER)
    if constexpr (bit_count <= 16) return __lzcnt16(value);
    if constexpr (bit_count <= 32) return __lzcnt32(value);
    if constexpr (bit_count <= 64) return __lzcnt64(value);
#endif
    // Fallback, works with any bitwidth
    size_t a = 0;
    size_t b = bit_count;
    auto all = T(-1);
    for (size_t i = 0; i < log2(bit_count); i++) {
        auto m = (a + b) / 2;
        auto mask = all << m;
        if (value & mask) a = m + 1;
        else              b = m;
    }
    return bit_count - b;
}

/// Split an unsigned integer such that its bits are spaced by 2 zeros.
/// For instance, morton_split(0b00110010) = 0b000000001001000000001000.
template <typename T, std::enable_if_t<std::is_unsigned_v<T>, int> = 0>
inline proto_always_inline T morton_split(T x) {
    constexpr size_t log_bits = log2(sizeof(T) * CHAR_BIT);
    auto mask = std::numeric_limits<T>::max();
    for (size_t i = log_bits, n = 1 << log_bits; i > 0; --i, n >>= 1) {
        mask = (mask | (mask << n)) & ~(mask << (n / 2));
        x = (x | (x << n)) & mask;
    }
    return x;
}

/// Morton-encode three unsigned integers into one.
template <typename T, std::enable_if_t<std::is_unsigned_v<T>, int> = 0>
inline proto_always_inline T morton_encode(T x, T y, T z) {
    return morton_split(x) | (morton_split(y) << 1) | (morton_split(z) << 2);
}

/// Reads a file in its entirety as a string.
inline proto_always_inline std::string read_file(const std::string& file_name) {
    std::ifstream is(file_name, std::ifstream::binary);
    if (!is)
        throw std::runtime_error("Cannot read file \"" + file_name + "\"");
    return std::string(std::istreambuf_iterator<char>(is), std::istreambuf_iterator<char>());
}

/// Executes the given function over the given range `[Begin, End]`,
/// with each iteration generating a new call to the function.
template <size_t Begin, size_t End, typename F, std::enable_if_t<std::is_invocable_v<F, size_t>, int> = 0>
inline proto_always_inline void static_for(F&& f) {
    if constexpr (Begin < End) {
        f(Begin);
        static_for<Begin + 1, End>(f);
    }
}

/// Linearly interpolate the first two values using the third one as an interpolation factor.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline T lerp(T a, T b, T t) {
    return (1 - t) * a + t * b;
}

/// Linearly interpolate the first three values on a triangle, using the last two values as interpolation factors.
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline T lerp(T a, T b, T c, T u, T v) {
    return (1 - u - v) * a + u * b + v * c;
}

/// Constrain the given value to be in the interval `[min, max]`.
template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
inline proto_always_inline T clamp(T x, T min, T max) {
    return proto::robust_min(proto::robust_max(x, min), max);
}

} // namespace proto

#endif
