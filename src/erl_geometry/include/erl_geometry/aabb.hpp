#pragma once

#include "erl_common/yaml.hpp"

#include <utility>

namespace erl::geometry {

    struct AabbBase {};

    template<typename ScalarType, int Dim>
    struct Aabb : public Eigen::AlignedBox<ScalarType, Dim>, public AabbBase {

        using Scalar = ScalarType;
        using Point = Eigen::Vector<Scalar, Dim>;

        Point center = {};
        Point half_sizes = {};

        Aabb() = default;

        Aabb(Point center, Scalar half_size)
            : Eigen::AlignedBox<Scalar, Dim>(center.array() - half_size, center.array() + half_size),
              center(std::move(center)),
              half_sizes(Point::Constant(half_size)) {}

        Aabb(const Point &min, const Point &max)
            : Eigen::AlignedBox<Scalar, Dim>(min, max),
              center((min + max) / 2),
              half_sizes((max - min) / 2) {}

        bool
        operator==(const Aabb &rhs) const {
            return center == rhs.center && half_sizes == rhs.half_sizes;
        }

        bool
        operator!=(const Aabb &rhs) const {
            return !(*this == rhs);
        }
    };

    using Aabb2D = Aabb<double, 2>;
    using Aabb3D = Aabb<double, 3>;
}  // namespace erl::geometry

namespace YAML {
    template<typename AABB>
    struct ConvertAabb {
        static_assert(std::is_base_of_v<erl::geometry::AabbBase, AABB>, "AABB must be derived from AabbBase");

        static Node
        encode(const AABB &rhs) {
            Node node;
            node["center"] = rhs.center;
            node["half_sizes"] = rhs.half_sizes;
            return node;
        }

        static bool
        decode(const Node &node, AABB &rhs) {
            if (!node.IsMap()) { return false; }
            rhs.center = node["center"].as<typename AABB::Point>();
            rhs.half_sizes = node["half_sizes"].as<typename AABB::Point>();
            return true;
        }
    };

    template<>
    struct convert<erl::geometry::Aabb2D> : public ConvertAabb<erl::geometry::Aabb2D> {};

    template<>
    struct convert<erl::geometry::Aabb3D> : public ConvertAabb<erl::geometry::Aabb3D> {};
}  // namespace YAML
