#ifndef PROTO_BVH_H
#define PROTO_BVH_H

#include <climits>
#include <limits>
#include <memory>
#include <cassert>
#include <type_traits>

#include "proto/bbox.h"
#include "proto/vec.h"
#include "proto/utils.h"
#include "proto/ray.h"
#include "proto/frustum.h"

namespace proto {

/// BVH with a top-down, binned SAH construction algorithm.
template <typename T>
class Bvh {
public:
    using Index = std::make_unsigned_t<typename SizedInteger<sizeof(T) * CHAR_BIT>::Type>;
    struct Node {
        T bounds[6];
        Index first_child_or_primitive;

        /// Packed representation containing both the primitive count (for leaves)
        /// and the split axis (for internal nodes).
        struct Flags {
            Flags() = default;

            bool is_leaf() const { return value_ & Index{0x80000000}; }

            size_t primitive_count() const {
                assert(!is_leaf());
                return value_;
            }

            unsigned split_axis() const {
                assert(is_leaf());
                return ~value_;
            }

            static Flags for_node(size_t primitive_count) { return Flags(primitive_count); }
            static Flags for_leaf(unsigned split_axis) { return Flags(~Index(split_axis)); }

        private:
            Flags(Index value)
                : value_(value)
            {}

            Index value_;
        } flags;

        Node() = default;

        template <typename F>
        void forall_primitives(F&& f) const {
            for (size_t
                 i = first_child_or_primitive,
                 n = first_child_or_primitive + flags.primitive_count();
                 i < n; ++i) f(i);
        }

        bool is_leaf() const { return flags.is_leaf() == 0; }
        size_t primitive_count() const { return flags.primitive_count(); }
        unsigned split_axis() const { return flags.split_axis(); }

        struct BBoxProxy {
            Node& node;

            BBoxProxy(Node& node)
                : node(node)
            {}

            BBoxProxy& operator = (const BBox<T>& other) {
                node.bounds[0] = other.min[0];
                node.bounds[2] = other.min[1];
                node.bounds[4] = other.min[2];
                node.bounds[1] = other.max[0];
                node.bounds[3] = other.max[1];
                node.bounds[5] = other.max[2];
                return *this;
            }

            operator BBox<T> () const { return to_bbox(); }

            BBox<T> to_bbox() const {
                return BBox<T>(
                    Vec3<T>(node.bounds[0], node.bounds[2], node.bounds[4]),
                    Vec3<T>(node.bounds[1], node.bounds[3], node.bounds[5])
                );
            }

            BBoxProxy& extend(const BBox<T>& bbox) { return *this = to_bbox().extend(bbox); }
        };

        BBox<T> bbox() const { return bbox_proxy().to_bbox(); }
        BBoxProxy bbox_proxy() { return BBoxProxy(*this); }
        const BBoxProxy bbox_proxy() const { return BBoxProxy(const_cast<Node&>(*this)); }

        std::pair<T, T> intersect(
            const Ray<T>& ray,
            const Vec3<T>& inv_dir,
            const Vec3<T>& scaled_org,
            const typename Ray<T>::Octant& octant)
        {
            // This a fast ray-node intersection and might produce some false misses, when the ray
            // is very small or when it is very close to the surface.
            auto tmin_x = fast_mul_add(bounds[0 + octant[0]], inv_dir[0], scaled_org[0]);
            auto tmin_y = fast_mul_add(bounds[2 + octant[1]], inv_dir[1], scaled_org[1]);
            auto tmin_z = fast_mul_add(bounds[4 + octant[2]], inv_dir[2], scaled_org[2]);
            auto tmax_x = fast_mul_add(bounds[0 + 1 - octant[0]], inv_dir[0], scaled_org[0]);
            auto tmax_y = fast_mul_add(bounds[2 + 1 - octant[1]], inv_dir[1], scaled_org[1]);
            auto tmax_z = fast_mul_add(bounds[4 + 1 - octant[2]], inv_dir[2], scaled_org[2]);
            auto tmin = robust_max(tmin_x, robust_max(tmin_y, robust_max(tmin_z, ray.tmin)));
            auto tmax = robust_min(tmax_x, robust_min(tmax_y, robust_min(tmax_z, ray.tmax)));
            return std::pair { tmin, tmax };
        }
    };

    struct BuildConfig {
        size_t min_primitives_per_leaf = 1;
        size_t max_primitives_per_leaf = 4;
        T traversal_cost = 1;

        BuildConfig() = default;
    };

    static_assert(sizeof(Node) == sizeof(T) * 8);

    Bvh() = default;

    /// Builds a BVH from a set of bounding boxes and centers, passed as functions.
    template <
        typename BBoxes, typename Centers,
        std::enable_if_t<std::is_invocable_v<BBoxes, size_t>, int> = 0,
        std::enable_if_t<std::is_invocable_v<Centers, size_t>, int> = 0>
    Bvh(BBoxes bboxes, Centers centers, size_t primitive_count, const BuildConfig& config = {}) {
        auto bboxes_ptr  = std::make_unique<BBox<T>[]>(primitive_count);
        auto centers_ptr = std::make_unique<Vec3<T>[]>(primitive_count);
        for (size_t i = 0; i < primitive_count; ++i) {
            bboxes_ptr[i]  = bboxes(i);
            centers_ptr[i] = centers(i);
        }
        build(bboxes_ptr.get(), centers_ptr.get(), primitive_count, config);
    }

    /// Builds a BVH from a set of bounding boxes and centers, passed as pointers.
    Bvh(const BBox<T>* bboxes, const Vec3<T>* centers, size_t primitive_count, const BuildConfig& config = {}) {
        build(bboxes, centers, primitive_count, config);
    }

    Bvh(Bvh&& other) {
        *this = std::move(other);
    }

    Bvh& operator = (Bvh&& other) {
        nodes_ = std::move(other.nodes_);
        primitive_indices_ = std::move(other.primitive_indices_);
        node_count_ = std::move(other.node_count_);
        other.node_count_ = 0;
        return *this;
    }

    const Node* nodes() const { return nodes_.get(); }
    size_t node_count() const { return node_count_; }

    const size_t* primitive_indices() const { return primitive_indices_.get(); }

    /// Intersects a ray with the BVH,
    /// using the given function to intersect the BVH leaves.
    /// If `Any` is true, does not attempt to order nodes during traversal.
    /// This exits if `f` returns true.
    template <bool Any, typename F>
    void intersect(Ray<T>&, F f) const;

    /// Intersects a frustum with the BVH,
    /// using the given function to intersect the BVH leaves.
    /// This exits if `f` returns true.
    template <typename F>
    void intersect(Frustum<T>&, F f) const;

private:
    static constexpr size_t bin_count = 16;
    static constexpr size_t stack_capacity = 64;

    template <size_t Capacity = stack_capacity>
    struct Stack {
        size_t elems[Capacity];
        size_t size = 0;

        Stack() = default;

        bool is_empty() const { return size == 0; }

        void push(size_t elem) {
            assert(size < Capacity);
            elems[++size] = elem;
        }

        size_t pop() {
            assert(size > 0);
            return elems[--size];
        }
    };

    struct Split {
        int axis = -1;
        size_t bin_index;
        T cost = std::numeric_limits<T>::max();

        Split() = default;
        Split(int axis, size_t bin_index, T cost)
            : axis(axis), bin_index(bin_index), cost(cost)
        {}
        static Split best_of(Split, Split);
    };

    Split find_best_split(int, const Node&, const BBox<T>*, const Vec3<T>*) const;
    void build(const BBox<T>*, const Vec3<T>*, size_t, const BuildConfig&);

    std::unique_ptr<Node[]> nodes_;
    std::unique_ptr<size_t[]> primitive_indices_;
    size_t node_count_ = 0;
};

using Bvhf = Bvh<float>;
using Bvhd = Bvh<double>;

template <typename T>
template <bool Any, typename F>
void Bvh<T>::intersect(Ray<T>& ray, F f) const {
    assert(node_count() > 0);
    Stack<> stack;
    size_t top = 0;

    auto inv_dir = ray.inv_dir();
    auto octant = ray.octant();
    auto scaled_org = -ray.org * inv_dir;
    while (true) {
        auto& node = nodes_[top];
        auto [tmin, tmax] = node.intersect(ray, inv_dir, scaled_org, octant);
        if (tmin <= tmax) {
            if (node.is_leaf()) {
                if (f(ray, node)) break;
            } else {
                size_t order = Any ? 0 : octant[node.split_axis];
                top = node.first_child_or_primitive + order;
                stack.push(node.first_child_or_primitive + 1 - order);
                continue;
            }
        }
        if (stack.is_empty())
            break;
        top = stack.pop();
    }
}

template <typename T>
template <typename F>
void Bvh<T>::intersect(Frustum<T>& frustum, F f) const {
    assert(node_count() > 0);
    Stack<> stack;
    size_t top = 0;

    size_t skip = 0;
    while (true) {
        auto& node = nodes_[top];
        // Skip the frustum test if the subtree is contained in the frustum
        switch (skip != 0 ? Plane<T>::Straddling : frustum.intersect(node.bbox())) {
            case Plane<T>::In:
                skip = stack.size;
                [[fallthrough]];
            case Plane<T>::Straddling:
                if (node.is_leaf()) {
                    if (f(frustum, node)) break;
                } else {
                    top = node.first_child_or_primitive;
                    stack.push(node.first_child_or_primitive + 1);
                    continue;
                }
                break;
           default:
                break;
        }
        if (stack.is_empty())
            break;
        if (stack.size == skip) skip = 0;
        top = stack.pop();
    }
}

} // namespace proto

#endif
