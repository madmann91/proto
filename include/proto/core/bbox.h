#ifndef PROTO_CORE_BBOX_H
#define PROTO_CORE_BBOX_H

#include <limits>

#include "proto/core/vec.h"
#include "proto/core/mat.h"

namespace proto {

/// Bounding-box type, defined by its extrema points.
template <typename T>
struct BBox {
    Vec3<T> min;
    Vec3<T> max;

    BBox() = default;

    BBox(const Vec3<T>& min, const Vec3<T>& max)
        : min(min), max(max)
    {}

    BBox(const Vec3<T>& p)
        : BBox(p, p)
    {}

    BBox& extend(const BBox& other) {
        min = Vec3<T>::min(min, other.min);
        max = Vec3<T>::max(max, other.max);
        return *this;
    }

    BBox& extend(const Vec3<T>& p) {
        return extend(BBox(p, p));
    }

    Vec3<T> diagonal() const { return max - min; }
    Vec3<T> center() const { return (max + min) * T(0.5); }

    BBox transform(const Mat4x4<T>& transform) const {
        auto transf_center = transform * Vec4<T>(center(), T(1));
        auto transf_extents = abs(transform) * Vec4<T>(diagonal() * T(0.5), T(0));
        return BBox(
            Vec3<T>(transf_center - transf_extents),
            Vec3<T>(transf_center + transf_extents));
    }

    T half_area() const {
        auto d = diagonal();
        return (d[0] + d[1]) * d[2] + d[0] * d[1];
    }

    unsigned largest_axis() const {
        auto d = diagonal();
        unsigned axis = 0;
        if (d[axis] < d[1]) axis = 1;
        if (d[axis] < d[2]) axis = 2;
        return axis;
    }

    static BBox empty() {
        return BBox(
            Vec3<T>(+std::numeric_limits<T>::max()),
            Vec3<T>(-std::numeric_limits<T>::max()));
    }

    static BBox full() {
        return BBox(
            Vec3<T>(-std::numeric_limits<T>::max()),
            Vec3<T>(+std::numeric_limits<T>::max()));
    }
};

using BBoxf = BBox<float>;
using BBoxd = BBox<double>;

} // namespace proto

#endif
