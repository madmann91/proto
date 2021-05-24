#ifndef PROTO_CORE_MAT_H
#define PROTO_CORE_MAT_H

#include <algorithm>
#include <type_traits>
#include <cassert>
#include <cmath>
#include <ostream>
#include <tuple>

#include "proto/core/vec.h"

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
    void set(Args&&... args) { Setter<0>::set(*this, std::forward<Args&&>(args)...); }

public:
    T values[N * M];

    Mat() = default;

    template <typename... Args, std::enable_if_t<(std::tuple_size<std::tuple<Args...>>::value >= 2), int> = 0>
    explicit Mat(Args... args) {
        set(args...);
    }

    template <typename F, std::enable_if_t<std::is_invocable_v<F, size_t, size_t>, int> = 0>
    explicit Mat(F&& f) {
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < M; ++j)
                (*this)(i, j) = f(i, j);
        }
    }

    T& operator () (size_t row, size_t col) { return values[col * N + row]; }
    const T& operator () (size_t row, size_t col) const { return values[col * N + row]; }

    Vec<T, N> col(size_t i) const {
        assert(i < M);
        return Vec<T, N>([&] (size_t j) { return (*this)(j, i); });
    }

    Vec<T, M> row(size_t i) const {
        assert(i < N);
        return Vec<T, M>([&] (size_t j) { return (*this)(i, j); });
    }

    Mat<T, M, N> transpose() const {
        return Mat<T, M, N>([&] (size_t i, size_t j) { return (*this)(j, i); });
    }

    std::enable_if_t<M == N && M >= 1, T> minor(size_t row, size_t col) const {
        return Mat<T, M - 1, M - 1>([&] (size_t i, size_t j) {
            return (*this)(i >= row ? i + 1 : i, j >= col ? j + 1 : j);
        }).determinant();
    }

    std::enable_if_t<M == N && M >= 1, T> cofactor(size_t row, size_t col) const {
        auto m = minor(row, col);
        return (row + col) % 2 == 0 ? m : -m;
    }

    std::enable_if_t<M == N, T> determinant() const {
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

    std::enable_if_t<M == N, Mat<T, M, M>> inverse() const {
        auto det = determinant();
        if (std::fabs(det) < std::numeric_limits<T>::epsilon())
            return zero();
        auto inv_det = T(1) / det;
        return Mat<T, M, M>([&] (size_t i, size_t j) { return cofactor(j, i) * inv_det; });
    }

    template <size_t I, size_t J, size_t P, size_t Q>
    std::enable_if_t<I + P <= N && J + Q <= M, Mat<T, P, Q>> submatrix() const {
        return Mat<T, P, Q>([&] (size_t i, size_t j) { return (*this)(i + I, j + J); });
    }

    template <size_t O>
    Mat<T, N, O> operator * (const Mat<T, M, O>& other) const {
        return Mat<T, N, O>([&] (size_t i, size_t j) {
            return dot(this->row(i), other.col(j));
        });
    }

    Vec<T, N> operator * (const Vec<T, N>& v) const {
        return Vec<T, N>([&] (size_t i) {
            return dot(this->row(i), v);
        });
    }

    Mat operator + (const Mat& other) const {
        return Mat([&] (size_t i, size_t j) { return (*this)(i, j) + other(i, j); });
    }

    Mat operator - (const Mat& other) const {
        return Mat([&] (size_t i, size_t j) { return (*this)(i, j) - other(i, j); });
    }

    Mat operator * (T s) const {
        return Mat([&] (size_t i, size_t j) { return (*this)(i, j) * s; });
    }

    Mat operator / (T s) const {
        return Mat([&] (size_t i, size_t j) { return (*this)(i, j) * (T(1) / s); });
    }

    static Mat abs(const Mat& mat) {
        return Mat([&] (size_t i, size_t j) { return std::abs(mat(i, j)); });
    }

    static Mat diagonal(T diag) {
        return Mat([&] (size_t i, size_t j) { return i == j ? diag : T(0); });
    }

    static Mat zero() { return Mat([] (size_t, size_t) { return T(0); }); }
    static Mat identity() { return diagonal(1); }
};

template <typename T> using Mat2x2 = Mat<T, 2, 2>;
template <typename T> using Mat3x3 = Mat<T, 3, 3>;
template <typename T> using Mat4x4 = Mat<T, 4, 4>;
using Mat2x2f = Mat2x2<float>;
using Mat3x3f = Mat3x3<float>;
using Mat4x4f = Mat4x4<float>;
using Mat2x2d = Mat2x2<double>;
using Mat3x3d = Mat3x3<double>;
using Mat4x4d = Mat4x4<double>;

template <typename T, size_t N, size_t M>
std::ostream& operator << (std::ostream& os, const Mat<T, N, M>& mat) {
    os << "[";
    for (size_t i = 0; i < N; ++i) {
        os << mat.row(i);
        if (i != N - 1) os << ",\n";
    }
    return os << "]";
}

} // namespace proto

#endif
