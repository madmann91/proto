#ifndef PROTO_SPHERE_H
#define PROTO_SPHERE_H

#include <cstddef>
#include <cmath>

#include "proto/vec.h"
#include "proto/ray.h"
#include "proto/bbox.h"

namespace proto {

template <typename T>
struct Sphere  {
    Vec3<T> center;
    T radius;

    Sphere() = default;
    proto_always_inline Sphere(const Vec3<T>& center, T radius)
        : center(center), radius(radius)
    {}

    proto_always_inline BBox<T> bbox() const {
        return BBox<T>(
            center - Vec3<T>(radius),
            center + Vec3<T>(radius));
    }

    /// Returns the result of intersecting the given ray with the sphere,
    /// as sorted pair `(t0, t1)` of ray distances (if an intersection exists).
    /// This function does not check if the result is within the ray's `[tmin, tmax]` range.
    proto_always_inline std::optional<std::pair<T, T>> intersect_unchecked(const Ray<T>& ray) const {
        auto oc = ray.org - center;
        auto a = dot(ray.dir, ray.dir);
        auto b = 2 * dot(ray.dir, oc);
        auto c = dot(oc, oc) - radius * radius;

        auto delta = b * b - 4 * a * c;
        if (delta >= 0) {
            auto inv = -T(0.5) / a;
            auto sqrt_delta = std::sqrt(delta);
            auto t0 = (b + sqrt_delta) * inv;
            auto t1 = (b - sqrt_delta) * inv;
            return std::make_optional(std::pair { t0, t1 });
        }

        return std::nullopt;
    }

    /// Intersects a ray with the sphere. If an intersection is found,
    /// this function updates `ray.tmax` with the corresponding distance along the ray.
    proto_always_inline bool intersect(Ray<T>& ray) const {
        if (auto range = intersect_unchecked(ray)) {
            auto [t0, t1] = *range;
            if (t0 >= ray.tmin && t0 <= ray.tmax)
                return ray.tmax = t0, true;
            if (t1 >= ray.tmin && t1 <= ray.tmax)
                return ray.tmax = t1, true;
        }
        return false;
    }
};

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;

} // namespace proto

#endif
