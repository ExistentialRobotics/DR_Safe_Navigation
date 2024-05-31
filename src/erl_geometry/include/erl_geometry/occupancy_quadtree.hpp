#pragma once

#include "occupancy_quadtree_base.hpp"
#include "occupancy_quadtree_drawer.hpp"
#include "occupancy_quadtree_node.hpp"

namespace erl::geometry {

    class OccupancyQuadtree : public OccupancyQuadtreeBase<OccupancyQuadtreeNode, OccupancyQuadtreeBaseSetting> {
    public:
        using Setting = OccupancyQuadtreeBaseSetting;
        using Drawer = OccupancyQuadtreeDrawer<OccupancyQuadtree>;

        explicit OccupancyQuadtree(const std::shared_ptr<OccupancyQuadtreeBaseSetting> &setting)
            : OccupancyQuadtreeBase(setting) {}

        OccupancyQuadtree()
            : OccupancyQuadtree(std::make_shared<Setting>()) {}

        explicit OccupancyQuadtree(const std::string &filename)
            : OccupancyQuadtree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read OccupancyQuadtree from file: {}", filename);
        }

        OccupancyQuadtree(const OccupancyQuadtree &) = delete;  // no copy constructor

    protected:
        [[nodiscard]] std::shared_ptr<AbstractQuadtree>
        Create() const override {
            return std::make_shared<OccupancyQuadtree>();
        }
    };

    ERL_REGISTER_QUADTREE(OccupancyQuadtree);
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::OccupancyQuadtree::Drawer::Setting>
    : public ConvertOccupancyQuadtreeDrawerSetting<erl::geometry::OccupancyQuadtree::Drawer::Setting> {};  // namespace YAML
