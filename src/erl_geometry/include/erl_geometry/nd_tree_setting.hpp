#pragma once

#include "erl_common/yaml.hpp"

#include <cstdint>
#include <typeinfo>

namespace erl::geometry {

    /**
     * NDTreeSetting is a base class for all n-d tree settings.
     */
    class NdTreeSetting : public common::Yamlable<NdTreeSetting> {
    public:
        double resolution = 0.1;
        uint32_t tree_depth = 16;

        virtual bool
        operator==(const NdTreeSetting& rhs) const {
            if (typeid(*this) != typeid(rhs)) { return false; }
            return resolution == rhs.resolution && tree_depth == rhs.tree_depth;
        }

        bool
        operator!=(const NdTreeSetting& rhs) const {
            return !(*this == rhs);
        }
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::NdTreeSetting> {
    static Node
    encode(const erl::geometry::NdTreeSetting& rhs) {
        Node node;
        node["resolution"] = rhs.resolution;
        node["tree_depth"] = rhs.tree_depth;
        return node;
    }

    static bool
    decode(const Node& node, erl::geometry::NdTreeSetting& rhs) {
        if (!node.IsMap()) { return false; }
        rhs.resolution = node["resolution"].as<double>();
        rhs.tree_depth = node["tree_depth"].as<uint32_t>();
        return true;
    }
};  // namespace YAML
