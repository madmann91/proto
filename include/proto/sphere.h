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
    Sphere(const Vec3<T>& center, T radius)
        : center(center), radius(radius)
    {}

    BBox<T> bbox() const {
        return BBox<T>(
            center - Vec3<T>(radius),
            center + Vec3<T>(radius));
    }

    /// Result of intersecting a ray with a sphere.
    /// Contains either 0, 1, or two distances,
    /// depending on whether and where the ray intersects the sphere.
    /// The distances are stored by increasing distance.
    struct Intersection {
        unsigned t_count = 0;
        float t[2];

        operator bool() const { return t_count > 0; }
    };

    Intersection intersect(const Ray<T>& ray) const {
        auto oc = ray.org - center;
        auto a = dot(ray.dir, ray.dir);
        auto b = 2 * dot(ray.dir, oc);
        auto c = dot(oc, oc) - radius * radius;

        auto delta = b * b - 4 * a * c;
        if (delta >= 0) {
            auto inv = -T(0.5) / a;
            auto sqrt_delta = std::sqrt(delta);
            auto t0 = (b - sqrt_delta) * inv;
            auto t1 = (b + sqrt_delta) * inv;

            Intersection intr;
            if (t0 >= ray.tmin && t0 <= ray.tmax)
                intr[intr.t_count++] = t0;
            if (t1 >= ray.tmin && t1 <= ray.tmax)
                intr[intr.t_count++] = t1;
            return intr;
        }

        return Intersection();
    }
};

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;

} // namespace proto

#endif
