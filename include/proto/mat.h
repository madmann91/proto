#ifndef PROTO_MAT_H
#define PROTO_MAT_H

#include <algorithm>
#include <type_traits>
#include <cassert>
#include <cmath>
#include <ostream>
#include <tuple>

#include "proto/vec.h"

namespace proto {

/// Column-major matrix with N rows and M columns.
template <typename T, size_t N, size_t M>
struct Mat {
    static_assert(N > 0 && M > 0);

private:
    template <size_t I>
    struct Setter {
        template <typename... Args>
        static void set(Mat<T, N, M>& mat, T t, Args&&... args) {
            static_assert(I < N * M);
            mat.values[I] = t;
            Setter<I + 1>::set(mat, std::forward<Args&&>(args)...);
        }
        template <size_t P, typename... Args>
        static void set(Mat<T, N, M>& mat, const Vec<T, P>& vec, Args&&... args) {
            static_assert(I + P <= N * M);
            std::copy(vec.values, vec.values + P, mat.values + I);
            Setter<I + P>::set(mat, std::forward<Args&&>(args)...);
        }
        template <typename... Args>
        static void set(Mat<T, N, M>&) {
            static_assert(I == N * M);
        }
    };

    template <typename... Args>
    proto_always_inline void set(Args&&... args) { Setter<0>::set(*this, std::forward<Args&&>(args)...); }

public:
    T values[N * M];

    Mat() = default;

    template <typename... Args, std::enable_if_t<(std::tuple_size<std::tuple<Args...>>::value >= 2), int> = 0>
    explicit proto_always_inline Mat(Args... args) {
        set(args...);
    }

    template <typename F, std::enable_if_t<std::is_invocable_v<F, size_t, size_t>, int> = 0>
    explicit proto_always_inline Mat(F&& f) {
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < M; ++j)
                (*this)(i, j) = f(i, j);
        }
    }

    proto_always_inline T& operator () (size_t row, size_t col) { return values[col * N + row]; }
    proto_always_inline const T& operator () (size_t row, size_t col) const { return values[col * N + row]; }

    proto_always_inline Vec<T, N> col(size_t i) const {
        assert(i < M);
        return Vec<T, N>([&] (size_t j) { return (*this)(j, i); });
    }

    proto_always_inline Vec<T, M> row(size_t i) const {
        assert(i < N);
        return Vec<T, M>([&] (size_t j) { return (*this)(i, j); });
    }

    proto_always_inline Mat<T, M, N> transpose() const {
        return Mat<T, M, N>([&] (size_t i, size_t j) { return (*this)(j, i); });
    }

    proto_always_inline std::enable_if_t<M == N && M >= 1, T> minor(size_t row, size_t col) const {
        return Mat<T, M - 1, M - 1>([&] (size_t i, size_t j) {
            return (*this)(i >= row ? i + 1 : i, j >= col ? j + 1 : j);
        }).determinant();
    }

    proto_always_inline std::enable_if_t<M == N && M >= 1, T> cofactor(size_t row, size_t col) const {
        auto m = minor(row, col);
        return (row + col) % 2 == 0 ? m : -m;
    }

    proto_always_inline std::enable_if_t<M == N, T> determinant() const {
        if constexpr (M == 1)
            return (*this)(0, 0);
        else if constexpr (M == 2)
            return fast_mul_add((*this)(0, 0), (*this)(1, 1), -(*this)(1, 0) * (*this)(0, 1));
        else {
            // Laplace expansion
            T sum = 0;
            for (size_t j = 0; j < M; ++j)
                sum = fast_mul_add(cofactor(0, j), (*this)(0, j), sum);
            return sum;
        }
    }

    proto_always_inline std::enable_if_t<M == N, Mat<T, M, M>> inverse() const {
        auto det = determinant();
        if (std::fabs(det) < std::numeric_limits<T>::epsilon())
            return zero();
        auto inv_det = T(1) / det;
        return Mat<T, M, M>([&] (size_t i, size_t j) { return cofactor(j, i) * inv_det; });
    }

    template <size_t I, size_t J, size_t P, size_t Q>
    proto_always_inline std::enable_if_t<I + P <= N && J + Q <= M, Mat<T, P, Q>> submatrix() const {
        return Mat<T, P, Q>([&] (size_t i, size_t j) { return (*this)(i + I, j + J); });
    }

    template <size_t O>
    proto_always_inline Mat<T, N, O> operator * (const Mat<T, M, O>& other) const {
        return Mat<T, N, O>([&] (size_t i, size_t j) {
            return dot(this->row(i), other.col(j));
        });
    }

    proto_always_inline Vec<T, N> operator * (const Vec<T, N>& v) const {
        return Vec<T, N>([&] (size_t i) {
            return dot(this->row(i), v);
        });
    }

    proto_always_inline Mat operator + (const Mat& other) const {
        return Mat([&] (size_t i, size_t j) { return (*this)(i, j) + other(i, j); });
    }

    proto_always_inline Mat operator - (const Mat& other) const {
        return Mat([&] (size_t i, size_t j) { return (*this)(i, j) - other(i, j); });
    }

    proto_always_inline Mat operator * (T s) const {
        return Mat([&] (size_t i, size_t j) { return (*this)(i, j) * s; });
    }

    proto_always_inline Mat operator / (T s) const {
        return Mat([&] (size_t i, size_t j) { return (*this)(i, j) * (T(1) / s); });
    }

    static proto_always_inline Mat diagonal(T diag) {
        return Mat([&] (size_t i, size_t j) { return i == j ? diag : T(0); });
    }

    static proto_always_inline Mat zero() { return Mat([] (size_t, size_t) { return T(0); }); }
    static proto_always_inline Mat identity() { return diagonal(1); }
};

template <typename T> using Mat2x2 = Mat<T, 2, 2>;
template <typename T> using Mat2x3 = Mat<T, 2, 3>;
template <typename T> using Mat2x4 = Mat<T, 2, 3>;
template <typename T> using Mat3x2 = Mat<T, 3, 2>;
template <typename T> using Mat3x3 = Mat<T, 3, 3>;
template <typename T> using Mat3x4 = Mat<T, 3, 4>;
template <typename T> using Mat4x2 = Mat<T, 4, 2>;
template <typename T> using Mat4x3 = Mat<T, 4, 3>;
template <typename T> using Mat4x4 = Mat<T, 4, 4>;
using Mat2x2f = Mat2x2<float>;
using Mat2x3f = Mat2x3<float>;
using Mat2x4f = Mat2x4<float>;
using Mat3x2f = Mat3x2<float>;
using Mat3x3f = Mat3x3<float>;
using Mat3x4f = Mat3x4<float>;
using Mat4x2f = Mat4x2<float>;
using Mat4x3f = Mat4x3<float>;
using Mat4x4f = Mat4x4<float>;
using Mat2x2d = Mat2x2<double>;
using Mat2x3d = Mat2x3<double>;
using Mat2x4d = Mat2x4<double>;
using Mat3x2d = Mat3x2<double>;
using Mat3x3d = Mat3x3<double>;
using Mat3x4d = Mat3x4<double>;
using Mat4x2d = Mat4x2<double>;
using Mat4x3d = Mat4x3<double>;
using Mat4x4d = Mat4x4<double>;

template <typename T, size_t N, size_t M>
inline std::ostream& operator << (std::ostream& os, const Mat<T, N, M>& mat) {
    os << "[";
    for (size_t i = 0; i < N; ++i) {
        os << mat.row(i);
        if (i != N - 1) os << ",\n";
    }
    return os << "]";
}

template <typename T, size_t N, size_t M>
inline proto_always_inline Mat<T, N, M> abs(const Mat<T, N, M>& m) {
    return Mat<T, N, M>([&] (size_t i, size_t j) { return std::abs(m(i, j)); });
}

} // namespace proto

#endif
