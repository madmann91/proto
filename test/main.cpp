#include <proto/utils.h>
#include <proto/hash.h>
#include <proto/vec.h>
#include <proto/mat.h>
#include <proto/bbox.h>
#include <proto/plane.h>
#include <proto/frustum.h>
#include <proto/ray.h>
#include <proto/sphere.h>
#include <proto/triangle.h>

template <typename T>
void instantiate() {
    using namespace proto;
    [[maybe_unused]] Triangle<T> triangle;
    [[maybe_unused]] Sphere<T> sphere;
    [[maybe_unused]] BBox<T> bbox;
    [[maybe_unused]] Frustum<T> frustum;
    [[maybe_unused]] Plane<T> plane;
    [[maybe_unused]] Ray<T> ray;
}

int main() {
    instantiate<float>();
    instantiate<double>();
    return 0;
}
