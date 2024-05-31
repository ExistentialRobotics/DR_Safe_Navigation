#pragma once

#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>

#include <cstdint>
#include <vector>

namespace erl::geometry {

    /**
     * QuadtreeKey is a simple class that represents a key for a quadtree node. It is a 2D vector of uint16_t.
     * Each element counts the number of cells from the origin as discrete address of a voxel.
     */
    class QuadtreeKey {
    public:
        using KeyType = uint16_t;

    private:
        KeyType m_k_[2] = {0, 0};

    public:
        // Hash function for OctreeKey when used with absl containers.
        template<typename H>
        [[maybe_unused]] friend H
        AbslHashValue(H h, const QuadtreeKey& key) {
            return H::combine(std::move(h), key.m_k_[0], key.m_k_[1]);
        }

        // Hash function for OctreeKey when used with std hash containers.
        struct [[maybe_unused]] KeyHash {
            [[nodiscard]] std::size_t
            operator()(const QuadtreeKey& key) const {
                return static_cast<std::size_t>(key.m_k_[0]) << 16 | static_cast<std::size_t>(key.m_k_[1]);
            }
        };

        QuadtreeKey() = default;

        QuadtreeKey(const KeyType a, const KeyType b)
            : m_k_{a, b} {}

        QuadtreeKey(const QuadtreeKey& other)
            : m_k_{other.m_k_[0], other.m_k_[1]} {}

        QuadtreeKey&
        operator=(const QuadtreeKey& other) {
            if (this == &other) { return *this; }
            m_k_[0] = other.m_k_[0];
            m_k_[1] = other.m_k_[1];
            return *this;
        }

        QuadtreeKey(QuadtreeKey&& other) noexcept
            : m_k_{std::exchange(other.m_k_[0], 0), std::exchange(other.m_k_[1], 0)} {}

        QuadtreeKey&
        operator=(QuadtreeKey&& other) noexcept {
            if (this == &other) { return *this; }
            m_k_[0] = std::exchange(other.m_k_[0], 0);
            m_k_[1] = std::exchange(other.m_k_[1], 0);
            return *this;
        }

        [[nodiscard]] bool
        operator==(const QuadtreeKey& other) const {
            return m_k_[0] == other.m_k_[0] && m_k_[1] == other.m_k_[1];
        }

        [[nodiscard]] bool
        operator!=(const QuadtreeKey& other) const {
            return m_k_[0] != other.m_k_[0] || m_k_[1] != other.m_k_[1];
        }

        KeyType&
        operator[](const uint32_t i) {
            return m_k_[i];
        }

        [[nodiscard]] const KeyType&
        operator[](const uint32_t i) const {
            return m_k_[i];
        }

        [[nodiscard]] bool
        operator<(const QuadtreeKey& other) const {
            return m_k_[0] < other.m_k_[0] || (m_k_[0] == other.m_k_[0] && m_k_[1] < other.m_k_[1]);
        }

        [[nodiscard]] bool
        operator<=(const QuadtreeKey& other) const {
            return m_k_[0] < other.m_k_[0] || (m_k_[0] == other.m_k_[0] && m_k_[1] <= other.m_k_[1]);
        }

        [[nodiscard]] bool
        operator>(const QuadtreeKey& other) const {
            return m_k_[0] > other.m_k_[0] || (m_k_[0] == other.m_k_[0] && m_k_[1] > other.m_k_[1]);
        }

        [[nodiscard]] bool
        operator>=(const QuadtreeKey& other) const {
            return m_k_[0] > other.m_k_[0] || (m_k_[0] == other.m_k_[0] && m_k_[1] >= other.m_k_[1]);
        }

        [[nodiscard]] explicit
        operator std::string() const {
            return std::to_string(m_k_[0]) + "," + std::to_string(m_k_[1]);
        }

        /**
         * Compute the key of a child node from the key of its parent node and the index of the child node.
         * @param pos index of child node (0..3)
         * @param center_offset_key
         * @param parent_key
         * @param child_key
         */
        static void
        ComputeChildKey(const uint32_t pos, const KeyType center_offset_key, const QuadtreeKey& parent_key, QuadtreeKey& child_key) {
            child_key.m_k_[0] = parent_key.m_k_[0] + (pos & 1 ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
            child_key.m_k_[1] = parent_key.m_k_[1] + (pos & 2 ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
        }

        /**
         * Compute child index (0..3) from key at given level.
         * @param key
         * @param level level=0 means the leaf level
         * @return
         */
        static int
        ComputeChildIndex(const QuadtreeKey& key, const uint32_t level) {
            int pos = 0;
            const KeyType mask = 1 << level;
            if (key.m_k_[0] & mask) { pos += 1; }
            if (key.m_k_[1] & mask) { pos += 2; }
            return pos;
        }

        static bool
        KeyInAabb(const QuadtreeKey& key, const KeyType center_offset_key, const QuadtreeKey& aabb_min_key, const QuadtreeKey& aabb_max_key) {
            return aabb_min_key[0] <= key[0] + center_offset_key &&  //
                   aabb_min_key[1] <= key[1] + center_offset_key &&  //
                   aabb_max_key[0] >= key[0] - center_offset_key &&  //
                   aabb_max_key[1] >= key[1] - center_offset_key;
        }
    };

    /**
     * Data structure to efficiently compute the nodes to update from a scan insertion using a hash set.
     */
    using QuadtreeKeySet = absl::flat_hash_set<QuadtreeKey>;
    using QuadtreeKeyVectorMap = absl::flat_hash_map<QuadtreeKey, std::vector<long>>;
    using QuadtreeKeyVector = std::vector<QuadtreeKey>;

    /**
     * Data structure to efficiently track changed nodes.
     */
    using QuadtreeKeyBoolMap = absl::flat_hash_map<QuadtreeKey, bool>;
    using QuadtreeKeyRay = std::vector<QuadtreeKey>;
}  // namespace erl::geometry
