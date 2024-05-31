#pragma once

#include "surface_mapping_quadtree_node.hpp"

#include "erl_geometry/occupancy_quadtree_base.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

namespace erl::sdf_mapping {

    class SurfaceMappingQuadtree : public geometry::OccupancyQuadtreeBase<SurfaceMappingQuadtreeNode, geometry::OccupancyQuadtreeBaseSetting> {

    public:
        using Setting = geometry::OccupancyQuadtreeBaseSetting;
        using Drawer = geometry::OccupancyQuadtreeDrawer<SurfaceMappingQuadtree>;

        explicit SurfaceMappingQuadtree(const std::shared_ptr<Setting> &setting)
            : OccupancyQuadtreeBase(setting) {}

        SurfaceMappingQuadtree()
            : SurfaceMappingQuadtree(std::make_shared<Setting>()) {}

        explicit SurfaceMappingQuadtree(const std::string &filename)
            : SurfaceMappingQuadtree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read SurfaceMappingQuadtree from file: {}", filename);
        }

        SurfaceMappingQuadtree(const SurfaceMappingQuadtree &) = delete;  // no copy constructor

    protected:
        [[nodiscard]] std::shared_ptr<AbstractQuadtree>
        Create() const override {
            return std::make_shared<SurfaceMappingQuadtree>();
        }
    };

    ERL_REGISTER_QUADTREE(SurfaceMappingQuadtree);

}  // namespace erl::sdf_mapping

template<>
struct YAML::convert<erl::sdf_mapping::SurfaceMappingQuadtree::Drawer::Setting>
    : public ConvertOccupancyQuadtreeDrawerSetting<erl::sdf_mapping::SurfaceMappingQuadtree::Drawer::Setting> {};  // namespace YAML
