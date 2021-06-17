#ifndef PROTO_RAY_H
#define PROTO_RAY_H

#include <limits>
#include <array>
#include <cmath>

#include "proto/vec.h"

namespace proto {

/// Ray segment defined by an origin and a direction,
/// and two floating point values denoting the range of the segment.
template <typename T>
struct Ray {
    Vec3<T> org, dir;
    T tmin, tmax;

    Ray() = default;
    Ray(const Vec3<T>& org, const Vec3<T>& dir, T tmin = 0, T tmax = std::numeric_limits<T>::max())
        : org(org), dir(dir), tmin(tmin), tmax(tmax)
    {}

    Vec3<T> point_at(T t) const { return org + t * dir; }

    using Octant = std::array<unsigned, 3>;
    Octant octant() const {
        return Octant {
            std::signbit(dir[0]),
            std::signbit(dir[1]),
            std::signbit(dir[2])
        };
    }

    bool operator == (const Ray& other) const {
        return
            org == other.org &&
            dir == other.dir &&
            tmin == other.tmin &&
            tmax == other.tmax;
    }

    template <typename Hasher>
    Hasher& hash(Hasher& hasher) const {
        return dir.hash(org.hash(hasher)).combine(tmin).combine(tmax);
    }

    /// Returns the ray that connects the two given points.
    /// Can optionally offset the origin of the ray a little, in order to avoid self-intersections.
    static Ray between_points(const Vec3<T>& from, const Vec3<T>& to, T offset = T(0)) {
        return Ray(from, to - from, offset, T(1) - offset);
    }
};

using Rayf = Ray<float>;
using Rayd = Ray<double>;

} // namespace proto

#endif
