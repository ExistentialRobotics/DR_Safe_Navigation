#pragma once

#include "surface_mapping_quadtree.hpp"

namespace erl::sdf_mapping {

    class AbstractSurfaceMapping2D {

    public:
        virtual ~AbstractSurfaceMapping2D() = default;

        virtual geometry::QuadtreeKeySet
        GetChangedClusters() = 0;

        [[nodiscard]] virtual unsigned int
        GetClusterLevel() const = 0;

        virtual std::shared_ptr<SurfaceMappingQuadtree>
        GetQuadtree() = 0;

        [[nodiscard]] virtual double
        GetSensorNoise() const = 0;

        virtual bool
        Update(
            const Eigen::Ref<const Eigen::VectorXd> &angles,
            const Eigen::Ref<const Eigen::VectorXd> &distances,
            const Eigen::Ref<const Eigen::Matrix23d> &pose) = 0;
    };

}  // namespace erl::sdf_mapping
