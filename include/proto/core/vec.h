#ifndef PROTO_CORE_VEC_H
#define PROTO_CORE_VEC_H

#include <cstddef>
#include <algorithm>
#include <type_traits>
#include <numeric>
#include <tuple>
#include <cmath>
#include <ostream>

#include "proto/core/utils.h"

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
    void set(Args&&... args) { Setter<0>::set(*this, std::forward<Args&&>(args)...); }

public:
    T values[N];

    Vec() = default;
    explicit Vec(T t) { std::fill(values, values + N, t); }

    template <size_t M, std::enable_if_t<(M > N), int> = 0>
    explicit Vec(const Vec<T, M>& other) {
        std::copy(other.values, other.values + N, values);
    }

    template <typename... Args, std::enable_if_t<(std::tuple_size<std::tuple<Args...>>::value >= 2), int> = 0>
    explicit Vec(Args&&... args) {
        set(std::forward<Args&&>(args)...);
    }

    template <typename F, std::enable_if_t<std::is_invocable_v<F, size_t>, int> = 0>
    explicit Vec(F&& f) {
        for (size_t i = 0; i < N; ++i)
            values[i] = f(i);
    }

    Vec operator - () const { return Vec([this] (size_t i) { return -values[i]; }); }
    Vec safe_inverse() const { return Vec([this] (size_t i) { return safe_inverse(values[i]); }); }

    T& operator [] (size_t i) { return values[i]; }
    const T& operator [] (size_t i) const { return values[i]; }

    Vec& operator += (const Vec& other) { return *this = *this + other; }
    Vec& operator -= (const Vec& other) { return *this = *this - other; }
    Vec& operator *= (const Vec& other) { return *this = *this * other; }
    Vec& operator /= (const Vec& other) { return *this = *this / other; }
    Vec& operator *= (float other) { return *this = *this * other; }

    Vec operator + (const Vec& other) const {
        return Vec([=] (size_t i) { return values[i] + other[i]; });
    }

    Vec operator - (const Vec& other) const {
        return Vec([=] (size_t i) { return values[i] - other[i]; });
    }

    Vec operator * (const Vec& other) const {
        return Vec([=] (size_t i) { return values[i] * other[i]; });
    }

    Vec operator / (const Vec& other) const {
        return Vec([=] (size_t i) { return values[i] / other[i]; });
    }

    friend Vec operator * (const Vec& a, T s) {
        return Vec([=] (size_t i) { return a[i] * s; });
    }

    friend Vec operator * (T s, const Vec& b) {
        return b * s;
    }

    friend Vec operator / (const Vec& a, T s) {
        auto inv_s = T(1) / s;
        return Vec([=] (size_t i) { return a[i] * inv_s; });
    }

    friend Vec operator / (T s, const Vec& a) {
        return Vec([=] (size_t i) { return s / a[i]; });
    }

    friend T dot(const Vec& a, const Vec& b) {
        return std::inner_product(a.values, a.values + N, b.values, 0);
    }

    friend T length(const Vec& v) {
        return std::sqrt(dot(v, v));
    }

    friend Vec normalize(const Vec& v) {
        auto inv = T(1) / length(v);
        return v * inv;
    }

    static Vec min(const Vec& a, const Vec& b) {
        return Vec([=] (size_t i) { return std::min(a[i], b[i]); });
    }

    static Vec max(const Vec& a, const Vec& b) {
        return Vec([=] (size_t i) { return std::max(a[i], b[i]); });
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
std::ostream& operator << (std::ostream& os, const Vec<T, N>& vec) {
    os << "[";
    for (size_t i = 0; i < N; ++i) {
        os << vec[i];
        if (i != N - 1) os << ", ";
    }
    return os << "]";
}

template <typename T>
inline Vec3<T> cross(const Vec3<T>& a, const Vec3<T>& b) {
    return Vec3<T>([=] (size_t i) {
        size_t j = (i + 1) % 3;
        size_t k = (i + 2) % 3;
        return a[j] * b[k] - a[k] * b[j];
    });
}

} // namespace proto

#endif
