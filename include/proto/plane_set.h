#ifndef PROTO_PLANE_SET_H
#define PROTO_PLANE_SET_H

#include <array>
#include <algorithm>

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
        std::array<Plane<T>, N> transformed_planes;
        std::transform(planes.begin(), planes.end(), transformed_planes.begin(), f);
        return PlaneSet { transformed_planes };
    }
};

template <size_t N> using PlaneSetf = PlaneSet<float, N>;
template <size_t N> using PlaneSetd = PlaneSet<double, N>;

} // namespace proto

#endif
