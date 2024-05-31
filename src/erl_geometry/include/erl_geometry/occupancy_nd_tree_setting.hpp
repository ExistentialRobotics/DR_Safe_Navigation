#pragma once

#include "logodd.hpp"
#include "nd_tree_setting.hpp"

namespace erl::geometry {
    /**
     * OccupancyNdTreeSetting is a base class for all occupancy n-d tree settings.
     */
    struct OccupancyNdTreeSetting : public common::OverrideYamlable<NdTreeSetting, OccupancyNdTreeSetting> {
        float log_odd_min = -2;           // minimum log-odd value, default: -2 in log odd = 0.12 in probability
        float log_odd_max = 3.5;          // maximum log-odd value, default: 3.5 in log odd = 0.97 in probability
        float log_odd_hit = 0.85;         // log-odd value to add when a cell is hit by a ray, default: 0.85 in log odd = 0.7 in probability
        float log_odd_miss = -0.4;        // log-odd value to add when a cell is gone through by a ray, default: -0.4 in log odd = 0.4 in probability
        float log_odd_occ_threshold = 0;  // threshold that is used to decide whether a cell is occupied or not, default: 0 in log odd = 0.5 in probability

        void
        SetProbabilityHit(const double p) {
            log_odd_hit = logodd::LogOdd(p);
            ERL_WARN_COND(log_odd_hit <= 0, "ProbabilityHit should be > 0, but is {}", log_odd_hit);
        }

        [[nodiscard]] double
        GetProbabilityHit() const {
            return logodd::Probability(log_odd_hit);
        }

        void
        SetProbabilityMiss(const double p) {
            log_odd_miss = logodd::LogOdd(p);
            ERL_WARN_COND(log_odd_miss >= 0, "ProbabilityMiss should be < 0, but is {}", log_odd_miss);
        }

        [[nodiscard]] double
        GetProbabilityMiss() const {
            return logodd::Probability(log_odd_miss);
        }

        void
        SetProbabilityOccupiedThreshold(const double p) {
            log_odd_occ_threshold = logodd::LogOdd(p);
        }

        [[nodiscard]] double
        GetProbabilityOccupiedThreshold() const {
            return logodd::Probability(log_odd_occ_threshold);
        }

        bool
        operator==(const NdTreeSetting& rhs) const override {
            if (NdTreeSetting::operator==(rhs)) {
                const auto that = reinterpret_cast<const OccupancyNdTreeSetting&>(rhs);
                return log_odd_min == that.log_odd_min && log_odd_max == that.log_odd_max &&    //
                       log_odd_hit == that.log_odd_hit && log_odd_miss == that.log_odd_miss &&  //
                       log_odd_occ_threshold == that.log_odd_occ_threshold;
            }
            return false;
        }
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::OccupancyNdTreeSetting> {
    static Node
    encode(const erl::geometry::OccupancyNdTreeSetting& rhs) {
        Node node = convert<erl::geometry::NdTreeSetting>::encode(rhs);
        node["log_odd_min"] = rhs.log_odd_min;
        node["log_odd_max"] = rhs.log_odd_max;
        node["log_odd_hit"] = rhs.log_odd_hit;
        node["log_odd_miss"] = rhs.log_odd_miss;
        node["log_odd_occ_threshold"] = rhs.log_odd_occ_threshold;
        return node;
    }

    static bool
    decode(const Node& node, erl::geometry::OccupancyNdTreeSetting& rhs) {
        if (!node.IsMap()) { return false; }
        if (!convert<erl::geometry::NdTreeSetting>::decode(node, rhs)) { return false; }
        rhs.log_odd_min = node["log_odd_min"].as<float>();
        rhs.log_odd_max = node["log_odd_max"].as<float>();
        rhs.log_odd_hit = node["log_odd_hit"].as<float>();
        rhs.log_odd_miss = node["log_odd_miss"].as<float>();
        rhs.log_odd_occ_threshold = node["log_odd_occ_threshold"].as<float>();
        return true;
    }
};  // namespace YAML
