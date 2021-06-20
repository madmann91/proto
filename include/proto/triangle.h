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
    proto_always_inline Triangle(const Vec3<T>& v0, const Vec3<T>& v1, const Vec3<T>& v2)
        : v0(v0), v1(v1), v2(v2)
    {}

    /// Returns the *unnormalized* (not necessarily unit-length) normal of the triangle.
    proto_always_inline Vec3<T> raw_normal() const { return cross(v1 - v0, v2 - v0); }
    proto_always_inline Vec3<T> normal() const { return normalize(raw_normal()); }

    proto_always_inline BBox<T> bbox() const { return BBox<T>(v0).extend(v1).extend(v2); }
    proto_always_inline Vec3<T> center() const { return (v0 + v1 + v2) / T(3); }
    proto_always_inline T area() const { return length(raw_normal()) * T(0.5); }

    /// Intersects a ray with the triangle. If an intersection is found,
    /// this function updates `ray.tmax` with the corresponding distance along the ray,
    /// and returns the barycentric coordinates on the triangle as a pair.
    proto_always_inline std::optional<std::pair<T, T>> intersect(Ray<T>& ray) const {
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
                return ray.tmax = t, std::make_optional(std::pair { u, v });
        }

        return std::nullopt;
    }

    bool operator == (const Triangle& other) const {
        return v0 == other.v0 && v1 == other.v1 && v2 == other.v2;
    }

    template <typename Hasher>
    Hasher& hash(Hasher& hasher) const {
        return v2.hash(v1.hash(v2.hash(hasher)));
    }
};

using Trianglef = Triangle<float>;
using Triangled = Triangle<double>;

} // namespace proto

#endif
