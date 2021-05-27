#ifndef PROTO_PLANE_SET_H
#define PROTO_PLANE_SET_H

#include <array>

#include "proto/plane.h"
#include "proto/mat.h"

namespace proto {

/// Set of planes that can be used for culling.
template <typename T, size_t N>
struct PlaneSet {
    std::array<Plane<T>, N> planes;

    proto_always_inline typename Plane<T>::Intersection intersect(const BBox<T>& bbox) const {
        auto intr = Plane<T>::In;
        for (auto& plane : planes) {
            switch (plane.intersect(bbox)) {
                case Plane<T>::Out: return Plane<T>::Out;
                case Plane<T>::Straddling:
                    intr = Plane<T>::Straddling;
                    [[fallthrough]];
                case Plane<T>::In:
                    break;
            }
        }
        return intr;
    }

    proto_always_inline PlaneSet transform(const Mat4x4<T>& transform) const {
        return transform([&] (auto& plane) { return plane.transform(transform); });
    }

    proto_always_inline PlaneSet transform_inverse(const Mat4x4<T>& inverse) const {
        return transform([&] (auto& plane) { return plane.transform_inverse(inverse); });
    }

private:
    template <typename F>
    proto_always_inline PlaneSet transform(F&& f) const {
        return PlaneSet {
            std::array {
                f(planes[0]),
                f(planes[1]),
                f(planes[2]),
                f(planes[3]),
                f(planes[4]),
                f(planes[5]),
            }
        };
    }
};

template <size_t N> using PlaneSetf = PlaneSet<float, N>;
template <size_t N> using PlaneSetd = PlaneSet<double, N>;

} // namespace proto

#endif
