#ifndef PROTO_TRIANGLE_H
#define PROTO_TRIANGLE_H

#include <optional>

#include "proto/vec.h"
#include "proto/ray.h"
#include "proto/bbox.h"

namespace proto {

template <typename T>
struct Triangle {
    Vec3<T> v0, v1, v2;

    Triangle() = default;
    Triangle(const Vec3<T>& v0, const Vec3<T>& v1, const Vec3<T>& v2)
        : v0(v0), v1(v1), v2(v2)
    {}

    /// Returns the *unnormalized* normal of the triangle.
    Vec3<T> normal() const { return cross(v1 - v0, v2 - v0); }

    BBox<T> bbox() const { return BBox<T>(v0).extend(v1).extend(v2); }
    Vec3<T> center() const { return (v0 + v1 + v2) / T(3); }

    /// Result of the intersection between a triangle and a ray, represented
    /// as a distance along the ray, and the barycentric coordinates on the triangle.
    struct Intersection { T t, u, v; };

    std::optional<Intersection> intersect(const Ray<T>& ray) {
        auto e1 = v0 - v1;
        auto e2 = v2 - v0;
        auto n = cross(e1, e2);
        auto c = v0 - ray.org;
        auto r = cross(ray.dir, c);
        auto inv_det = T(1) / dot(n, ray.dir);

        auto u = dot(r, e2) * inv_det;
        auto v = dot(r, e1) * inv_det;
        auto w = T(1) - u - v;

        // These comparisons are designed to return false
        // when one of t, u, or v is a NaN
        if (u >= 0 && v >= 0 && w >= 0) {
            auto t = dot(n, c) * inv_det;
            if (t >= ray.tmin && t <= ray.tmax)
                return std::make_optional(Intersection { t, u, v });
        }

        return std::nullopt;
    }
};

using Trianglef = Triangle<float>;
using Triangled = Triangle<double>;

} // namespace proto

#endif
