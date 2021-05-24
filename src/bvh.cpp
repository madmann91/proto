#include <algorithm>
#include <numeric>
#include <stack>

#include "proto/bvh.h"

namespace proto {

template <typename T>
struct Bin {
    BBox<T> bbox = BBox<T>::empty();
    size_t primitive_count = 0;

    Bin() = default;

    Bin& merge(const Bin& other) {
        bbox.extend(other.bbox);
        primitive_count += other.primitive_count;
        return *this;
    }

    T cost() const {
        return primitive_count * bbox.half_area();
    }
};

template <typename T>
typename Bvh<T>::Split Bvh<T>::Split::best_of(Split left, Split right) {
    return left.axis >= 0 && left.cost < right.cost ? left : right;
}

template <typename T>
static inline T bin_scale(T min, T max, size_t bin_count) {
    return (T(bin_count) / (max - min));
}

template <typename T>
static inline size_t bin_index(T pos, T min, T bin_scale, size_t bin_count) {
    return std::min(bin_count - 1, size_t(std::max(ptrdiff_t{0}, ptrdiff_t((pos - min) * bin_scale))));
}

template <typename T>
typename Bvh<T>::Split Bvh<T>::find_best_split(int axis, const Node& node, const BBox<T>* bboxes, const Vec3<T>* centers) const {
    std::array<Bin<T>, bin_count> bins;
    auto scale = bin_scale(node.bbox().min[axis], node.bbox().max[axis], bin_count);

    // Place primitives in bins
    node.forall_primitives([&] (size_t i) {
        auto primitive_index = primitive_indices_[i];
        auto& bin = bins[bin_index(centers[primitive_index][axis], node.bbox().min[axis], scale, bin_count)];
        bin.bbox.extend(bboxes[primitive_index]);
        bin.primitive_count++;
    });

    // Sweep bins from the right to the left
    std::array<T, bin_count> right_cost;
    Bin<T> left_accum, right_accum;
    for (size_t i = bin_count; i > 0; --i) {
        auto& bin = bins[i - 1];
        right_accum.merge(bin);
        right_cost[i - 1] = right_accum.cost();
    }

    Split split;
    for (size_t i = 0; i < bin_count - 1; ++i) {
        auto& bin = bins[i];
        left_accum.merge(bin);
        auto cost = left_accum.cost() + right_cost[i + 1];
        if (cost < split.cost)
            split = Split(axis, i + 1, cost);
    }

    return split;
}

template <typename T>
void Bvh<T>::build(const BBox<T>* bboxes, const Vec3<T>* centers, size_t primitive_count, const BuildConfig& config) {
    assert(config.min_primitives_per_leaf > 0);

    size_t max_node_count = primitive_count * 2 - 1;
    nodes_ = std::make_unique<Node[]>(max_node_count);
    primitive_indices_ = std::make_unique<size_t[]>(primitive_count);

    std::iota(primitive_indices_.get(), primitive_indices_.get() + primitive_count, 0);

    nodes_[0].first_child_or_primitive = 0;
    nodes_[0].flags = Node::Flags::for_node(primitive_count);
    node_count_ = 1;

    std::stack<size_t> stack;
    stack.push(0);
    while (!stack.empty()) {
        auto& node = nodes_[stack.top()];
        stack.pop();

        node.bbox_proxy() = BBox<T>::empty();
        node.forall_primitives([&] (size_t i) { node.bbox_proxy().extend(bboxes[primitive_indices_[i]]); });

        if (node.primitive_count() <= config.min_primitives_per_leaf)
            continue;

        Split split;
        for (int axis = 0; axis < 3; ++axis)
            split = Split::best_of(split, find_best_split(axis, node, bboxes, centers));

        auto non_split_cost = node.bbox().half_area() * (node.primitive_count() - config.traversal_cost);
        size_t split_pos = 0;
        if (split.axis == -1 || split.cost >= non_split_cost) {
            if (node.primitive_count() > config.max_primitives_per_leaf) {
                // Fallback strategy: median split on largest axis
                auto axis = node.bbox().largest_axis();
                std::sort(
                    primitive_indices_.get() + node.first_child_or_primitive,
                    primitive_indices_.get() + node.first_child_or_primitive + node.primitive_count(),
                    [&] (size_t i, size_t j) { return centers[i][axis] < centers[j][axis]; });
                split_pos = node.first_child_or_primitive + node.primitive_count() / 2;
            } else continue;
        } else {
            auto scale = bin_scale(node.bbox().min[split.axis], node.bbox().max[split.axis], bin_count);
            split_pos = std::partition(
                primitive_indices_.get() + node.first_child_or_primitive,
                primitive_indices_.get() + node.first_child_or_primitive + node.primitive_count(),
                [&] (size_t i) {
                    return bin_index(centers[i][split.axis], node.bbox().min[split.axis], scale, bin_count) < split.bin_index;
                }) - primitive_indices_.get();
        }

        assert(split_pos != 0);

        size_t left_index  = node_count_ + 0;
        size_t right_index = node_count_ + 1;
        auto& left  = nodes_[left_index];
        auto& right = nodes_[right_index];
        left .first_child_or_primitive = node.first_child_or_primitive;
        right.first_child_or_primitive = split_pos;
        left .flags = Node::Flags::for_leaf(split_pos - left.first_child_or_primitive);
        right.flags = Node::Flags::for_leaf(node.primitive_count() - split_pos);

        node.first_child_or_primitive = node_count_;
        node.flags = Node::Flags::for_node(split.axis);
        node_count_ += 2;

        // Process the largest child first
        if (left.primitive_count() > right.primitive_count())
            std::swap(left_index, right_index);
        stack.push(left_index);
        stack.push(right_index);
    }
}

template class Bvh<float>;
template class Bvh<double>;

} // namespace proto
