#ifndef PROTO_HASH_H
#define PROTO_HASH_H

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <string_view>

namespace proto::fnv {

/// Incremental hashing utility using the Fowler-Voll-No hash function.
struct Hasher {
    static constexpr uint32_t offset = 0x811c9dc5;
    static constexpr uint32_t prime  = 0x01000193;

    uint32_t hash = offset;

    Hasher() = default;

    Hasher& combine(const std::string_view& s) { return combine(s.data(), s.size()); }

    template <typename T, std::enable_if_t<std::is_standard_layout_v<T> && std::is_trivial_v<T>, int> = 0>
    Hasher& combine(const T& t) { return combine(&t, sizeof(T)); }

    template <typename T>
    Hasher& combine(const T* t, size_t size) {
        auto bytes = reinterpret_cast<const char*>(t);
        for (size_t i = 0; i < size; ++i)
            hash = (hash ^ bytes[i]) * prime;
        return *this;
    }

    operator uint32_t () const { return hash; }
};

} // namespace proto::fnv

#endif
