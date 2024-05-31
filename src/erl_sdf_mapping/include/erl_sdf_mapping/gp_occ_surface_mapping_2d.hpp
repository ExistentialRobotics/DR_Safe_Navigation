#pragma once

#include "abstract_surface_mapping_2d.hpp"
#include "surface_mapping_quadtree.hpp"

#include "erl_common/yaml.hpp"
#include "erl_gaussian_process/lidar_gp_1d.hpp"

#include <memory>

namespace erl::sdf_mapping {

    class GpOccSurfaceMapping2D : public AbstractSurfaceMapping2D {
    public:
        struct Setting : public common::Yamlable<Setting> {

            struct ComputeVariance : public Yamlable<ComputeVariance> {
                double zero_gradient_position_var = 1.;  // position variance to use when the estimated gradient is almost zero.
                double zero_gradient_gradient_var = 1.;  // gradient variance to use when the estimated gradient is almost zero.
                double min_distance_var = 1.;            // minimum distance variance.
                double max_distance_var = 100.;          // maximum distance variance.
                double position_var_alpha = 0.01;        // scaling number of position variance.
                double min_gradient_var = 0.01;          // minimum gradient variance.
                double max_gradient_var = 1.;            // maximum gradient variance.
            };

            struct UpdateMapPoints : public Yamlable<UpdateMapPoints> {
                double min_observable_occ = -0.1;     // points of OCC smaller than this value is considered unobservable, i.e. inside the object.
                double max_surface_abs_occ = 0.02;    // maximum absolute value of surface points' OCC, which should be zero ideally.
                double max_valid_gradient_var = 0.5;  // maximum valid gradient variance, above this threshold, it won't be used for the Bayes Update.
                int max_adjust_tries = 10;
                double max_bayes_position_var = 1.;   // if the position variance by Bayes Update is above this threshold, it will be discarded.
                double max_bayes_gradient_var = 0.6;  // if the gradient variance by Bayes Update is above this threshold, it will be discarded.
                double min_position_var = 0.001;      // minimum position variance.
                double min_gradient_var = 0.001;      // minimum gradient variance.
            };

            std::shared_ptr<gaussian_process::LidarGaussianProcess1D::Setting> gp_theta = std::make_shared<gaussian_process::LidarGaussianProcess1D::Setting>();
            std::shared_ptr<ComputeVariance> compute_variance = std::make_shared<ComputeVariance>();   // parameters used by ComputeVariance.
            std::shared_ptr<UpdateMapPoints> update_map_points = std::make_shared<UpdateMapPoints>();  // parameters used by UpdateMapPoints.
            std::shared_ptr<SurfaceMappingQuadtree::Setting> quadtree = std::make_shared<SurfaceMappingQuadtree::Setting>();  // parameters used by quadtree.
            unsigned int cluster_level = 2;  // 2^2 times of the quadtree resolution.
            double perturb_delta = 0.01;
            double zero_gradient_threshold = 1.e-15;  // gradient below this threshold is considered zero.
            bool update_occupancy = true;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = std::make_shared<Setting>();
        std::shared_ptr<gaussian_process::LidarGaussianProcess1D> m_gp_theta_ = nullptr;  // the GP of regression between angle and mapped distance
        std::shared_ptr<SurfaceMappingQuadtree> m_quadtree_ = nullptr;
        Eigen::Matrix24d m_xy_perturb_ = {};
        geometry::QuadtreeKeySet m_changed_keys_ = {};

    public:
        GpOccSurfaceMapping2D()
            : m_gp_theta_(gaussian_process::LidarGaussianProcess1D::Create(m_setting_->gp_theta)) {}

        explicit GpOccSurfaceMapping2D(const std::shared_ptr<Setting> &setting)
            : m_setting_(setting),
              m_gp_theta_(gaussian_process::LidarGaussianProcess1D::Create(m_setting_->gp_theta)) {}

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        geometry::QuadtreeKeySet
        GetChangedClusters() override {
            return m_changed_keys_;
        }

        [[nodiscard]] unsigned int
        GetClusterLevel() const override {
            return m_setting_->cluster_level;
        }

        std::shared_ptr<SurfaceMappingQuadtree>
        GetQuadtree() override {
            return m_quadtree_;
        }

        [[nodiscard]] double
        GetSensorNoise() const override {
            return m_setting_->gp_theta->sensor_range_var;
        }

        bool
        Update(
            const Eigen::Ref<const Eigen::VectorXd> &angles,
            const Eigen::Ref<const Eigen::VectorXd> &distances,
            const Eigen::Ref<const Eigen::Matrix23d> &pose) override;

        void
        UpdateMapPoints();

        void
        UpdateOccupancy(
            const Eigen::Ref<const Eigen::VectorXd> &angles,
            const Eigen::Ref<const Eigen::VectorXd> &distances,
            const Eigen::Ref<const Eigen::Matrix23d> &pose);

        void
        AddNewMeasurement();

    protected:
        void
        RecordChangedKey(const geometry::QuadtreeKey &key) {
            ERL_DEBUG_ASSERT(m_quadtree_ != nullptr, "Quadtree is not initialized.");
            ERL_DEBUG_ASSERT(m_setting_ != nullptr, "Setting is not initialized.");
            m_changed_keys_.insert(m_quadtree_->AdjustKeyToDepth(key, m_quadtree_->GetTreeDepth() - m_setting_->cluster_level));
        }

        bool
        ComputeGradient1(const Eigen::Ref<const Eigen::Vector2d> &xy_local, Eigen::Ref<Eigen::Vector2d> gradient, double &occ_mean, double &distance_var);

        bool
        ComputeGradient2(const Eigen::Ref<const Eigen::Vector2d> &xy_local, Eigen::Ref<Eigen::Vector2d> gradient, double &occ_mean);

        void
        ComputeVariance(
            const Eigen::Ref<const Eigen::Vector2d> &xy_local,
            const Eigen::Ref<const Eigen::Vector2d> &grad_local,
            const double &distance,
            const double &distance_var,
            const double &occ_mean_abs,
            const double &occ_abs,
            bool new_point,
            double &var_position,
            double &var_gradient) const;

        [[nodiscard]] bool
        IsValidRangeEstimation(const double range, const double range_variance) const {
            return range >= m_setting_->gp_theta->train_buffer->valid_range_min && range <= m_setting_->gp_theta->train_buffer->valid_range_max &&
                   range_variance < m_setting_->gp_theta->max_valid_range_var;
        }
    };
}  // namespace erl::sdf_mapping

template<>
struct YAML::convert<erl::sdf_mapping::GpOccSurfaceMapping2D::Setting::ComputeVariance> {

    static Node
    encode(const erl::sdf_mapping::GpOccSurfaceMapping2D::Setting::ComputeVariance &rhs) {
        Node node;
        node["zero_gradient_position_var"] = rhs.zero_gradient_position_var;
        node["zero_gradient_gradient_var"] = rhs.zero_gradient_gradient_var;
        node["min_distance_var"] = rhs.min_distance_var;
        node["max_distance_var"] = rhs.max_distance_var;
        node["position_var_alpha"] = rhs.position_var_alpha;
        node["min_gradient_var"] = rhs.min_gradient_var;
        node["max_gradient_var"] = rhs.max_gradient_var;
        return node;
    }

    static bool
    decode(const Node &node, erl::sdf_mapping::GpOccSurfaceMapping2D::Setting::ComputeVariance &rhs) {
        if (!node.IsMap()) { return false; }
        rhs.zero_gradient_position_var = node["zero_gradient_position_var"].as<double>();
        rhs.zero_gradient_gradient_var = node["zero_gradient_gradient_var"].as<double>();
        rhs.min_distance_var = node["min_distance_var"].as<double>();
        rhs.max_distance_var = node["max_distance_var"].as<double>();
        rhs.position_var_alpha = node["position_var_alpha"].as<double>();
        rhs.min_gradient_var = node["min_gradient_var"].as<double>();
        rhs.max_gradient_var = node["max_gradient_var"].as<double>();
        return true;
    }
};

template<>
struct YAML::convert<erl::sdf_mapping::GpOccSurfaceMapping2D::Setting::UpdateMapPoints> {

    static Node
    encode(const erl::sdf_mapping::GpOccSurfaceMapping2D::Setting::UpdateMapPoints &rhs) {
        Node node;
        node["min_observable_occ"] = rhs.min_observable_occ;
        node["max_surface_abs_occ"] = rhs.max_surface_abs_occ;
        node["max_valid_gradient_var"] = rhs.max_valid_gradient_var;
        node["max_adjust_tries"] = rhs.max_adjust_tries;
        node["max_bayes_position_var"] = rhs.max_bayes_position_var;
        node["max_bayes_gradient_var"] = rhs.max_bayes_gradient_var;
        node["min_position_var"] = rhs.min_position_var;
        node["min_gradient_var"] = rhs.min_gradient_var;
        return node;
    }

    static bool
    decode(const Node &node, erl::sdf_mapping::GpOccSurfaceMapping2D::Setting::UpdateMapPoints &rhs) {
        if (!node.IsMap()) { return false; }
        rhs.min_observable_occ = node["min_observable_occ"].as<double>();
        rhs.max_surface_abs_occ = node["max_surface_abs_occ"].as<double>();
        rhs.max_valid_gradient_var = node["max_valid_gradient_var"].as<double>();
        rhs.max_adjust_tries = node["max_adjust_tries"].as<int>();
        rhs.max_bayes_position_var = node["max_bayes_position_var"].as<double>();
        rhs.max_bayes_gradient_var = node["max_bayes_gradient_var"].as<double>();
        rhs.min_position_var = node["min_position_var"].as<double>();
        rhs.min_gradient_var = node["min_gradient_var"].as<double>();
        return true;
    }
};

template<>
struct YAML::convert<erl::sdf_mapping::GpOccSurfaceMapping2D::Setting> {
    static Node
    encode(const erl::sdf_mapping::GpOccSurfaceMapping2D::Setting &setting) {
        Node node;
        node["gp_theta"] = setting.gp_theta;
        node["compute_variance"] = setting.compute_variance;
        node["update_map_points"] = setting.update_map_points;
        node["quadtree"] = setting.quadtree;
        node["cluster_level"] = setting.cluster_level;
        node["perturb_delta"] = setting.perturb_delta;
        node["zero_gradient_threshold"] = setting.zero_gradient_threshold;
        node["update_occupancy"] = setting.update_occupancy;
        return node;
    }

    static bool
    decode(const Node &node, erl::sdf_mapping::GpOccSurfaceMapping2D::Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.gp_theta = node["gp_theta"].as<decltype(setting.gp_theta)>();
        setting.compute_variance = node["compute_variance"].as<decltype(setting.compute_variance)>();
        setting.update_map_points = node["update_map_points"].as<decltype(setting.update_map_points)>();
        setting.quadtree = node["quadtree"].as<decltype(setting.quadtree)>();
        setting.cluster_level = node["cluster_level"].as<unsigned int>();
        setting.perturb_delta = node["perturb_delta"].as<double>();
        setting.zero_gradient_threshold = node["zero_gradient_threshold"].as<double>();
        setting.update_occupancy = node["update_occupancy"].as<bool>();
        return true;
    }
};
