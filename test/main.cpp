#include <random>
#include <cstdint>
#include <iostream>
#include <exception>

#include <proto/utils.h>
#include <proto/hash.h>
#include <proto/vec.h>
#include <proto/mat.h>
#include <proto/bbox.h>
#include <proto/plane.h>
#include <proto/frustum.h>
#include <proto/plane_set.h>
#include <proto/ray.h>
#include <proto/sphere.h>
#include <proto/triangle.h>

using namespace proto;

template <typename T>
struct State {
    std::uniform_real_distribution<T> dist;
    std::mt19937 gen;

    State(uint64_t seed = 0)
        : dist(T(0), T(1)), gen(seed)
    {}

    T random_real() { return dist(gen); }
    Vec2<T> random_vec2() { return Vec2<T>(random_real(), random_real()); }
    Vec3<T> random_vec3() { return Vec3<T>(random_real(), random_real(), random_real()); }
    Vec4<T> random_vec4() { return Vec4<T>(random_real(), random_real(), random_real(), random_real()); }
    Plane<T> random_plane() { return Plane<T>(random_vec4()); }
};

#define check_that(exp) \
    do { if (!(exp)) throw std::runtime_error("check '" #exp "' failed"); } while(false)

template <typename T>
void check_hash_and_eq(const T& t) {
    fnv::Hasher h1, h2;
    check_that(t == t);
    check_that(t.hash(h1) == t.hash(h2));
}

template <typename T>
void test_ray(State<T>& state) {
    check_hash_and_eq(Ray<T>(
        state.random_vec3(),
        state.random_vec3(),
        state.random_real(),
        state.random_real()));
}

template <typename T>
void test_bbox(State<T>& state) {
    check_hash_and_eq(BBox<T>(state.random_vec3(), state.random_vec3()));
}

template <typename T>
void test_frustum(State<T>& state) {
    check_hash_and_eq(Frustum<T>{
        {
            state.random_plane(),
            state.random_plane(),
            state.random_plane(),
            state.random_plane(),
            state.random_plane(),
            state.random_plane(),
        }
    });
}

template <typename T>
void test_sphere(State<T>& state) {
    check_hash_and_eq(Sphere<T>(state.random_vec3(), state.random_real()));
}

template <typename T>
void test_triangle(State<T>& state) {
    check_hash_and_eq(Triangle<T>(state.random_vec3(), state.random_vec3(), state.random_vec3()));
}

template <typename T>
bool test_all() {
    State<T> state;
    try {
        test_ray(state);
        test_bbox(state);
        test_frustum(state);
        test_sphere(state);
        test_triangle(state);
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}

int main() {
    return
        test_all<float>() &&
        test_all<double>() ? 0 : 1;
}
