#ifndef PROTO_PLANE_H
#define PROTO_PLANE_H

#include "proto/bbox.h"
#include "proto/mat.h"

namespace proto {

template <typename T>
struct Plane {
    Vec4<T> values;

    Plane() = default;
    proto_always_inline Plane(const Vec4<T>& values)
        : values(values)
    {}

    proto_always_inline Vec3<T> normal() const { return Vec3<T>(values); }
    proto_always_inline T distance() const { return -values[3]; }

    enum Intersection { In, Out, Straddling };
    proto_always_inline Intersection intersect(const BBox<T>& bbox) const {
        auto n = Vec3<T>(
            values[0] > 0 ? bbox.min[0] : bbox.max[0],
            values[1] > 0 ? bbox.min[1] : bbox.max[1],
            values[2] > 0 ? bbox.min[2] : bbox.max[2]);
        if (dot(n, normal()) > distance())
            return Out;

        auto p = Vec3<T>(
            values[0] > 0 ? bbox.max[0] : bbox.min[0],
            values[1] > 0 ? bbox.max[1] : bbox.min[1],
            values[2] > 0 ? bbox.max[2] : bbox.min[2]);
        if (dot(p, normal()) < distance())
            return In;

        return Straddling;
    }

    proto_always_inline Plane transform(const Mat4x4<T>& transform) const {
        return Plane(transform.inverse().transpose() * values); 
    }

    proto_always_inline Plane transform_inverse(const Mat4x4<T>& inverse) const {
        return Plane(inverse.transpose() * values); 
    }

    static proto_always_inline Plane from_points(const Vec3<T>& a, const Vec3<T>& b, const Vec3<T>& c) {
        auto n = cross(b - a, c - a);
        return Plane(Vec4<T>(n, -dot(n, a)));
    }
};

using Planef = Plane<float>;
using Planed = Plane<double>;

} // namespace proto

#endif
