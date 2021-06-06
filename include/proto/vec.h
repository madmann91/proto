#ifndef PROTO_VEC_H
#define PROTO_VEC_H

#include <cstddef>
#include <algorithm>
#include <type_traits>
#include <numeric>
#include <tuple>
#include <cmath>
#include <ostream>

#include "proto/utils.h"

namespace proto {

/// An N-dimensional vector class.
template <typename T, size_t N>
struct Vec {
    static_assert(N > 0);

private:
    template <size_t I>
    struct Setter {
        template <typename... Args>
        static void set(Vec& vec, T t, Args&&... args) {
            static_assert(I < N);
            vec.values[I] = t;
            Setter<I + 1>::set(vec, std::forward<Args&&>(args)...);
        }
        template <size_t M, typename... Args>
        static void set(Vec& vec, const Vec<T, M>& other, Args&&... args) {
            static_assert(I + M < N);
            std::copy(other.values, other.values + M, vec.values + I);
            Setter<I + M>::set(vec, std::forward<Args&&>(args)...);
        }
        template <typename... Args>
        static void set(Vec&) {
            static_assert(I == N);
        }
    };

    template <typename... Args>
    proto_always_inline void set(Args&&... args) { Setter<0>::set(*this, std::forward<Args&&>(args)...); }

    template <typename U, size_t M>
    friend Vec<U, M> safe_inverse(const Vec<U, M>&);

public:
    T values[N];

    Vec() = default;
    explicit proto_always_inline Vec(T t) { std::fill(values, values + N, t); }

    template <size_t M, std::enable_if_t<(M > N), int> = 0>
    explicit proto_always_inline Vec(const Vec<T, M>& other) {
        std::copy(other.values, other.values + N, values);
    }

    template <typename... Args, std::enable_if_t<(std::tuple_size<std::tuple<Args...>>::value >= 2), int> = 0>
    explicit proto_always_inline Vec(Args&&... args) {
        set(std::forward<Args&&>(args)...);
    }

    template <typename F, std::enable_if_t<std::is_invocable_v<F, size_t>, int> = 0>
    explicit proto_always_inline Vec(F&& f) {
        proto::static_for<0, N>([&] (size_t i) { values[i] = f(i); });
    }

    proto_always_inline T& operator [] (size_t i) { return values[i]; }
    proto_always_inline const T& operator [] (size_t i) const { return values[i]; }

    proto_always_inline Vec operator - () const { return Vec([this] (size_t i) { return -values[i]; }); }

    proto_always_inline Vec& operator += (const Vec& other) { return *this = *this + other; }
    proto_always_inline Vec& operator -= (const Vec& other) { return *this = *this - other; }
    proto_always_inline Vec& operator *= (const Vec& other) { return *this = *this * other; }
    proto_always_inline Vec& operator /= (const Vec& other) { return *this = *this / other; }
    proto_always_inline Vec& operator /= (float other) { return *this = *this / other; }
    proto_always_inline Vec& operator *= (float other) { return *this = *this * other; }

    proto_always_inline Vec operator + (const Vec& other) const {
        return Vec([&] (size_t i) { return values[i] + other[i]; });
    }

    proto_always_inline Vec operator - (const Vec& other) const {
        return Vec([&] (size_t i) { return values[i] - other[i]; });
    }

    proto_always_inline Vec operator * (const Vec& other) const {
        return Vec([&] (size_t i) { return values[i] * other[i]; });
    }

    proto_always_inline Vec operator / (const Vec& other) const {
        return Vec([&] (size_t i) { return values[i] / other[i]; });
    }

    friend proto_always_inline Vec operator * (const Vec& a, T s) {
        return Vec([&] (size_t i) { return a[i] * s; });
    }

    friend proto_always_inline Vec operator * (T s, const Vec& b) {
        return b * s;
    }

    friend proto_always_inline Vec operator / (const Vec& a, T s) {
        auto inv_s = T(1) / s;
        return Vec([&] (size_t i) { return a[i] * inv_s; });
    }

    friend proto_always_inline Vec operator / (T s, const Vec& a) {
        return Vec([&] (size_t i) { return s / a[i]; });
    }
};

template <typename T> using Vec2 = Vec<T, 2>;
template <typename T> using Vec3 = Vec<T, 3>;
template <typename T> using Vec4 = Vec<T, 4>;
using Vec2f = Vec2<float>;
using Vec3f = Vec3<float>;
using Vec4f = Vec4<float>;
using Vec2d = Vec2<double>;
using Vec3d = Vec3<double>;
using Vec4d = Vec4<double>;

template <typename T, size_t N>
inline std::ostream& operator << (std::ostream& os, const Vec<T, N>& vec) {
    os << "[";
    for (size_t i = 0; i < N; ++i) {
        os << vec[i];
        if (i != N - 1) os << ", ";
    }
    return os << "]";
}

template <typename T>
inline proto_always_inline Vec3<T> cross(const Vec3<T>& a, const Vec3<T>& b) {
    return Vec3<T>(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]);
}

template <typename T, size_t N>
inline proto_always_inline Vec<T, N> min(const Vec<T, N>& a, const Vec<T, N>& b) {
    return Vec<T, N>([=] (size_t i) { return robust_min(a[i], b[i]); });
}

template <typename T, size_t N>
inline proto_always_inline Vec<T, N> max(const Vec<T, N>& a, const Vec<T, N>& b) {
    return Vec<T, N>([=] (size_t i) { return robust_max(a[i], b[i]); });
}

template <typename T, size_t N>
inline proto_always_inline T dot(const Vec<T, N>& a, const Vec<T, N>& b) {
    return std::inner_product(a.values, a.values + N, b.values, T(0));
}

template <typename T, size_t N>
inline proto_always_inline T length_squared(const Vec<T, N>& v) {
    return dot(v, v);
}

template <typename T, size_t N>
inline proto_always_inline T length(const Vec<T, N>& v) {
    return std::sqrt(length_squared(v));
}

template <typename T, size_t N>
inline proto_always_inline Vec<T, N> normalize(const Vec<T, N>& v) {
    auto inv = T(1) / length(v);
    return v * inv;
}

template <typename T, size_t N>
inline proto_always_inline Vec<T, N> safe_inverse(const Vec<T, N>& v) {
    return Vec<T, N>([&] (size_t i) { return proto::safe_inverse(v[i]); });
}

template <typename T, size_t N, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline Vec<T, N> lerp(const Vec<T, N>& a, const Vec<T, N>& b, T t) {
    return (1 - t) * a + t * b;
}

template <typename T, size_t N, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline Vec<T, N> lerp(const Vec<T, N>& a, const Vec<T, N>& b, const Vec<T, N>& c, T u, T v) {
    return (1 - u - v) * a + u * b + v * c;
}

template <typename T, size_t N, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
inline proto_always_inline Vec<T, N> clamp(const Vec<T, N>& x, const Vec<T, N>& min, const Vec<T, N>& max) {
    return proto::max(proto::min(x, max), min);
}

} // namespace proto

#endif
