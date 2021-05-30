#ifndef TARGET_NAMESPACE
#error "'TARGET_NAMESPACE' not defined"
#endif

#include <proto/vec.h>
#include <proto/ray.h>
#include <proto/triangle.h>
#include <proto/sphere.h>

namespace TARGET_NAMESPACE {

template <typename T> using Ray      = proto::Ray<T>;
template <typename T> using Vec3     = proto::Vec3<T>;
template <typename T> using BBox     = proto::BBox<T>;
template <typename T> using Triangle = proto::Triangle<T>;
template <typename T> using Sphere   = proto::Sphere<T>;

using proto::normalize;
using proto::cross;
using proto::dot;
using proto::length;
using proto::min;
using proto::max;
using proto::abs;

} // namespace TARGET_NAMESPACE
