#ifndef PROTO_TRIANGLE_H
#define PROTO_TRIANGLE_H

#include <optional>

#include "proto/vec.h"
#include "proto/ray.h"
#include "proto/bbox.h"

namespace proto {

/// Triangle defined by three points.
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
    proto_always_inline T area() const { return length(raw_normal()) * T(0.5); }

    proto_always_inline BBox<T> bbox() const { return BBox<T>(v0).extend(v1).extend(v2); }
    proto_always_inline Vec3<T> center() const { return (v0 + v1 + v2) / T(3); }

    /// Intersects a ray with the triangle. If an intersection is found,
    /// this function updates `ray.tmax` with the corresponding distance along the ray,
    /// and returns the barycentric coordinates on the triangle as a pair.
    proto_always_inline std::optional<std::pair<T, T>> intersect(Ray<T>& ray) const;

    bool operator == (const Triangle& other) const {
        return v0 == other.v0 && v1 == other.v1 && v2 == other.v2;
    }

    template <typename Hasher>
    Hasher& hash(Hasher& hasher) const {
        return v2.hash(v1.hash(v0.hash(hasher)));
    }
};

/// Triangle that uses precomputed data (edges and raw normal) to speed up its ray intersection routine.
template <typename T>
struct PrecomputedTriangle {
    Vec3<T> v0, e1, e2, n;

    PrecomputedTriangle() = default;
    proto_always_inline PrecomputedTriangle(const Vec3<T>& v0, const Vec3<T>& v1, const Vec3<T>& v2)
        : v0(v0), e1(v0 - v1), e2(v2 - v0), n(cross(e1, e2))
    {}
    proto_always_inline PrecomputedTriangle(const Triangle<T>& triangle)
        : PrecomputedTriangle(triangle.v0, triangle.v1, triangle.v2)
    {}

    proto_always_inline Vec3<T> raw_normal() const { return -n; }
    proto_always_inline Vec3<T> normal() const { return normalize(raw_normal()); }
    proto_always_inline T area() const { return length(n) * T(0.5); }

    proto_always_inline BBox<T> bbox() const { return static_cast<Triangle<T>>(*this).bbox(); }
    proto_always_inline Vec3<T> center() const { return static_cast<Triangle<T>>(*this).center(); }

    /// Converts this precomputed triangle to a regular triangle.
    operator Triangle<T> () const { return Triangle<T>(v0, v0 - e1, v0 + e2); }

    /// Fast ray-triangle intersection routine using precomputed data.
    /// See `Triangle<T>::intersect()` for more details.
    proto_always_inline std::optional<std::pair<T, T>> intersect(Ray<T>& ray) const {
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

    bool operator == (const PrecomputedTriangle& other) const {
        return v0 == other.v0 && e1 == other.e1 && e2 == other.e2;
    }

    template <typename Hasher>
    Hasher& hash(Hasher& hasher) const {
        return e2.hash(e1.hash(v0.hash(hasher)));
    }
};

template <typename T>
std::optional<std::pair<T, T>> Triangle<T>::intersect(Ray<T>& ray) const {
    return PrecomputedTriangle<T>(*this).intersect(ray);
}

using Trianglef = Triangle<float>;
using Triangled = Triangle<double>;
using PrecomputedTrianglef = PrecomputedTriangle<float>;
using PrecomputedTriangled = PrecomputedTriangle<double>;

} // namespace proto

#endif
