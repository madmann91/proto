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
#include <proto/bvh.h>

template <typename T>
void instantiate() {
    using namespace proto;
    Triangle<T> triangle;
    Sphere<T> sphere;
    Bvh<T> bvh;
    BBox<T> bbox;
    Frustum<T> frustum;
    Plane<T> plane;
    Ray<T> ray;
}

int main() {
    instantiate<float>();
    instantiate<double>();
    return 0;
}
