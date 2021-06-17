#ifndef PROTO_BBOX_H
#define PROTO_BBOX_H

#include <limits>

#include "proto/utils.h"
#include "proto/vec.h"
#include "proto/mat.h"

namespace proto {

/// Bounding-box type, defined by its extrema points.
template <typename T>
struct BBox {
    Vec3<T> min;
    Vec3<T> max;

    BBox() = default;

    proto_always_inline BBox(const Vec3<T>& min, const Vec3<T>& max)
        : min(min), max(max)
    {}

    proto_always_inline BBox(const Vec3<T>& p)
        : BBox(p, p)
    {}

    proto_always_inline BBox& extend(const BBox& other) {
        min = proto::min(min, other.min);
        max = proto::max(max, other.max);
        return *this;
    }

    proto_always_inline BBox& extend(const Vec3<T>& p) {
        return extend(BBox(p, p));
    }

    proto_always_inline Vec3<T> diagonal() const { return max - min; }
    proto_always_inline Vec3<T> center() const { return (max + min) * T(0.5); }

    proto_always_inline BBox transform(const Mat4x4<T>& transform) const {
        auto transf_center = transform * Vec4<T>(center(), T(1));
        auto transf_extents = abs(transform) * Vec4<T>(diagonal() * T(0.5), T(0));
        return BBox(
            Vec3<T>(transf_center - transf_extents),
            Vec3<T>(transf_center + transf_extents));
    }

    proto_always_inline T half_area() const {
        auto d = diagonal();
        return (d[0] + d[1]) * d[2] + d[0] * d[1];
    }

    proto_always_inline unsigned largest_axis() const {
        auto d = diagonal();
        unsigned axis = 0;
        if (d[axis] < d[1]) axis = 1;
        if (d[axis] < d[2]) axis = 2;
        return axis;
    }

    proto_always_inline bool is_contained_in(const BBox& other) const {
        return
            min[0] >= other.min[0] && max[0] <= other.max[0] &&
            min[1] >= other.min[1] && max[1] <= other.max[1] &&
            min[2] >= other.min[2] && max[2] <= other.max[2];
    }

    bool operator == (const BBox& other) const {
        return min == other.min && max == other.max;
    }

    template <typename Hasher>
    Hasher& hash(Hasher& hasher) const {
        return min.hash(max.hash(hasher));
    }

    static proto_always_inline BBox empty() {
        return BBox(
            Vec3<T>(+std::numeric_limits<T>::max()),
            Vec3<T>(-std::numeric_limits<T>::max()));
    }

    static proto_always_inline BBox full() {
        return BBox(
            Vec3<T>(-std::numeric_limits<T>::max()),
            Vec3<T>(+std::numeric_limits<T>::max()));
    }
};

using BBoxf = BBox<float>;
using BBoxd = BBox<double>;

} // namespace proto

#endif
