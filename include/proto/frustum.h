#ifndef PROTO_FRUSTUM_H
#define PROTO_FRUSTUM_H

#include <array>

#include "proto/plane.h"
#include "proto/mat.h"

namespace proto {

/// Frustum (pyramid) with 6 planes (near, far, left, right, bottom, top).
template <typename T>
struct Frustum {
    std::array<Plane<T>, 6> planes;

    proto_always_inline typename Plane<T>::Intersection intersect(const BBox<T>& bbox) const {
        auto intr = Plane<T>::In;
        for (auto& plane : planes) {
            switch (plane.intersect(bbox)) {
                case Plane<T>::Out: return Plane<T>::Out;
                case Plane<T>::Straddling:
                    intr = Plane<T>::Straddling;
                    [[fallthrough]];
                case Plane<T>::In:
                    break;
            }
        }
        return intr;
    }

    proto_always_inline Frustum transform(const Mat4x4<T>& transform) const {
        return transform([&] (auto& plane) { return plane.transform(transform); });
    }

    proto_always_inline Frustum transform_inverse(const Mat4x4<T>& inverse) const {
        return transform([&] (auto& plane) { return plane.transform_inverse(inverse); });
    }

    static proto_always_inline Frustum from_mat4x4(const Mat4x4<T>& transform) {
        // Extract the frustum planes from a transform matrix
        // see: http://www8.cs.umu.se/kurser/5DV051/HT12/lab/plane_extraction.pdf
        return Frustum {
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

private:
    template <typename F>
    proto_always_inline Frustum transform(F&& f) const {
        return Frustum {
            std::array {
                f(planes[0]),
                f(planes[1]),
                f(planes[2]),
                f(planes[3]),
                f(planes[4]),
                f(planes[5]),
            }
        };
    }
};

using Frustumf = Frustum<float>;
using Frustumd = Frustum<double>;

} // namespace proto

#endif
