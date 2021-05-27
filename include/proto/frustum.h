#ifndef PROTO_FRUSTUM_H
#define PROTO_FRUSTUM_H

#include <array>

#include "proto/plane_set.h"

namespace proto {

template <typename T> using Frustum = PlaneSet<T, 6>;
using Frustumf = Frustum<float>;
using Frustumd = Frustum<double>;

/// Extracts the 6 frustum planes (left, right, top, bottom, near, far)
/// from a projection or view-projection matrix.
/// See: http://www8.cs.umu.se/kurser/5DV051/HT12/lab/plane_extraction.pdf
template <typename T>
inline proto_always_inline Frustum<T> extract_frustum(const Mat4x4<T>& transform) {
    return Frustum<T> {
        std::array {
            Plane<T>(transform.row(3) + transform.row(0)), // Left
            Plane<T>(transform.row(3) - transform.row(0)), // Right
            Plane<T>(transform.row(3) + transform.row(1)), // Top
            Plane<T>(transform.row(3) - transform.row(1)), // Bottom
            Plane<T>(transform.row(3) + transform.row(2)), // Near
            Plane<T>(transform.row(3) - transform.row(2))  // Far
        }
    };
}

} // namespace proto

#endif
