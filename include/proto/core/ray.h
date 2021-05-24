#ifndef PROTO_CORE_RAY_H
#define PROTO_CORE_RAY_H

#include <limits>
#include <array>
#include <cmath>

#include "proto/core/vec.h"

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
};

using Rayf = Ray<float>;
using Rayd = Ray<double>;

} // namespace proto

#endif
