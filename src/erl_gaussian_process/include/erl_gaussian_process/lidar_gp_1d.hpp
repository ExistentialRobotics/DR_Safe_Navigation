#pragma once

#include "mapping.hpp"
#include "vanilla_gp.hpp"

#include "erl_common/eigen.hpp"
#include "erl_common/yaml.hpp"

#include <memory>

namespace erl::gaussian_process {

    class LidarGaussianProcess1D {

    public:
        struct TrainBuffer {
            struct Setting : public common::Yamlable<Setting> {
                double valid_range_min = 0.2;
                double valid_range_max = 30.0;
                double valid_angle_min = -135. / 180. * M_PI;
                double valid_angle_max = 135. / 180. * M_PI;
                std::shared_ptr<Mapping::Setting> mapping = []() {
                    auto mapping_setting = std::make_shared<Mapping::Setting>();
                    mapping_setting->type = Mapping::Type::kInverseSqrt;
                    mapping_setting->scale = 1.0;
                    return mapping_setting;
                }();
            };

            std::shared_ptr<Setting> setting;
            std::shared_ptr<Mapping> mapping;

            // data
            Eigen::VectorXd vec_angles_original;
            Eigen::VectorXd vec_ranges_original;
            Eigen::VectorXb vec_mask_hit;
            Eigen::VectorXd vec_angles;
            Eigen::VectorXd vec_ranges;
            Eigen::VectorXd vec_mapped_distances;
            Eigen::Matrix2Xd mat_direction_local;
            Eigen::Matrix2Xd mat_xy_local;
            Eigen::Matrix2Xd mat_direction_global;
            Eigen::Matrix2Xd mat_xy_global;
            double max_distance = 0.;
            Eigen::Vector2d position;
            Eigen::Matrix2d rotation;

            TrainBuffer()
                : TrainBuffer(std::make_shared<Setting>()) {}

            explicit TrainBuffer(std::shared_ptr<Setting> setting)
                : setting(std::move(setting)) {}

            [[nodiscard]] ssize_t
            Size() const {
                return vec_angles.size();
            }

            /**
             * Store new training data to this TrainBuffer.
             * @param vec_new_angles new angle data in radian, assumed in ascending order.
             * @param vec_new_distances new distance data.
             * @param mat_new_pose 2x3 matrix of 2D rotation and translation.
             * @return true if the data is stored successfully, otherwise false.
             */
            bool
            Store(
                const Eigen::Ref<const Eigen::VectorXd> &vec_new_angles,
                const Eigen::Ref<const Eigen::VectorXd> &vec_new_distances,
                const Eigen::Ref<const Eigen::Matrix23d> &mat_new_pose);

            [[nodiscard]] Eigen::Vector2d
            GlobalToLocalSo2(const Eigen::Ref<const Eigen::Vector2d> &vec_global) const {
                return {rotation(0, 0) * vec_global.x() + rotation(1, 0) * vec_global.y(), rotation(0, 1) * vec_global.x() + rotation(1, 1) * vec_global.y()};
            }

            [[nodiscard]] Eigen::Vector2d
            LocalToGlobalSo2(const Eigen::Ref<const Eigen::Vector2d> &vec_local) const {
                return {rotation(0, 0) * vec_local.x() + rotation(0, 1) * vec_local.y(), rotation(1, 0) * vec_local.x() + rotation(1, 1) * vec_local.y()};
            }

            [[nodiscard]] Eigen::Vector2d
            GlobalToLocalSe2(const Eigen::Ref<const Eigen::Vector2d> &vec_global) const {
                return {
                    rotation(0, 0) * (vec_global.x() - position.x()) + rotation(1, 0) * (vec_global.y() - position.y()),
                    rotation(0, 1) * (vec_global.x() - position.x()) + rotation(1, 1) * (vec_global.y() - position.y())};
            }

            [[nodiscard]] Eigen::Vector2d
            LocalToGlobalSe2(const Eigen::Ref<const Eigen::Vector2d> &vec_local) const {
                return {
                    rotation(0, 0) * vec_local.x() + rotation(0, 1) * vec_local.y() + position.x(),
                    rotation(1, 0) * vec_local.x() + rotation(1, 1) * vec_local.y() + position.y()};
            }
        };

        struct Setting : public common::Yamlable<Setting> {

            int group_size = 26;   // number of points in each group, including the overlap ones.
            int overlap_size = 6;  // number of points in the overlap region.
            // points closed to margin will not be used for test because it is difficult to estimate gradient for them.
            double boundary_margin = 0.0175;
            double init_variance = 1e6;  // large value to initialize variance result in case of computation failure.
            double sensor_range_var = 0.01;
            // if the distance variance is greater than this threshold, this prediction is invalid and should be discarded.
            double max_valid_range_var = 0.1;
            double occ_test_temperature = 30;  // OCC Test is a tanh function, this controls the slope around 0.
            std::shared_ptr<TrainBuffer::Setting> train_buffer = std::make_shared<TrainBuffer::Setting>();
            std::shared_ptr<VanillaGaussianProcess::Setting> gp = std::make_shared<VanillaGaussianProcess::Setting>();  // parameters of local GP regression
        };

    protected:
        bool m_trained_ = false;
        std::shared_ptr<Setting> m_setting_;

        // m_trained_ GPs and their partitions
        std::vector<std::shared_ptr<VanillaGaussianProcess>> m_gps_;
        std::vector<double> m_partitions_;

        // stored training data
        TrainBuffer m_train_buffer_;

    public:
        static std::shared_ptr<LidarGaussianProcess1D>
        Create(std::shared_ptr<Setting> setting);

        [[nodiscard]] bool
        IsTrained() const {
            return m_trained_;
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] std::vector<std::shared_ptr<VanillaGaussianProcess>>
        GetGps() const {
            return m_gps_;
        }

        [[nodiscard]] std::vector<double>
        GetPartitions() const {
            return m_partitions_;
        }

        [[nodiscard]] TrainBuffer &
        GetTrainBuffer() {
            return m_train_buffer_;
        }

        [[nodiscard]] Eigen::Vector2d
        GlobalToLocalSo2(const Eigen::Ref<const Eigen::Vector2d> &vec_global) const {
            return m_train_buffer_.GlobalToLocalSo2(vec_global);
        }

        [[nodiscard]] Eigen::Vector2d
        LocalToGlobalSo2(const Eigen::Ref<const Eigen::Vector2d> &vec_local) const {
            return m_train_buffer_.LocalToGlobalSo2(vec_local);
        }

        [[nodiscard]] Eigen::Vector2d
        GlobalToLocalSe2(const Eigen::Ref<const Eigen::Vector2d> &vec_global) const {
            return m_train_buffer_.GlobalToLocalSe2(vec_global);
        }

        [[nodiscard]] Eigen::Vector2d
        LocalToGlobalSe2(const Eigen::Ref<const Eigen::Vector2d> &vec_local) const {
            return m_train_buffer_.LocalToGlobalSe2(vec_local);
        }

        void
        Reset();

        void
        Train(
            const Eigen::Ref<const Eigen::VectorXd> &angles,
            const Eigen::Ref<const Eigen::VectorXd> &distances,
            const Eigen::Ref<const Eigen::Matrix23d> &pose);

        void
        Test(const Eigen::Ref<const Eigen::VectorXd> &angles, Eigen::Ref<Eigen::VectorXd> fs, Eigen::Ref<Eigen::VectorXd> vars, bool un_map = true) const;

        bool
        ComputeOcc(
            const Eigen::Ref<const Eigen::Scalard> &angle,
            double r,
            Eigen::Ref<Eigen::Scalard> range_pred,
            Eigen::Ref<Eigen::Scalard> range_pred_var,
            double &occ) const;  // return false if failed to compute occ

    private:
        explicit LidarGaussianProcess1D(std::shared_ptr<Setting> setting);
    };
}  // namespace erl::gaussian_process

template<>
struct YAML::convert<erl::gaussian_process::LidarGaussianProcess1D::TrainBuffer::Setting> {
    static Node
    encode(const erl::gaussian_process::LidarGaussianProcess1D::TrainBuffer::Setting &setting) {
        Node node;
        node["valid_range_min"] = setting.valid_range_min;
        node["valid_range_max"] = setting.valid_range_max;
        node["valid_angle_min"] = setting.valid_angle_min;
        node["valid_angle_max"] = setting.valid_angle_max;
        node["mapping"] = *setting.mapping;
        return node;
    }

    static bool
    decode(const Node &node, erl::gaussian_process::LidarGaussianProcess1D::TrainBuffer::Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.valid_range_min = node["valid_range_min"].as<double>();
        setting.valid_range_max = node["valid_range_max"].as<double>();
        setting.valid_angle_min = node["valid_angle_min"].as<double>();
        setting.valid_angle_max = node["valid_angle_max"].as<double>();
        *setting.mapping = node["mapping"].as<erl::gaussian_process::Mapping::Setting>();
        return true;
    }
};

template<>
struct YAML::convert<erl::gaussian_process::LidarGaussianProcess1D::Setting> {
    static Node
    encode(const erl::gaussian_process::LidarGaussianProcess1D::Setting &setting) {
        Node node;
        node["group_size"] = setting.group_size;
        node["overlap_size"] = setting.overlap_size;
        node["boundary_margin"] = setting.boundary_margin;
        node["init_variance"] = setting.init_variance;
        node["sensor_range_var"] = setting.sensor_range_var;
        node["max_valid_range_var"] = setting.max_valid_range_var;
        node["occ_test_temperature"] = setting.occ_test_temperature;
        node["train_buffer"] = *setting.train_buffer;
        node["gp"] = *setting.gp;
        return node;
    }

    static bool
    decode(const Node &node, erl::gaussian_process::LidarGaussianProcess1D::Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.group_size = node["group_size"].as<int>();
        setting.overlap_size = node["overlap_size"].as<int>();
        setting.boundary_margin = node["boundary_margin"].as<double>();
        setting.init_variance = node["init_variance"].as<double>();
        setting.sensor_range_var = node["sensor_range_var"].as<double>();
        setting.max_valid_range_var = node["max_valid_range_var"].as<double>();
        setting.occ_test_temperature = node["occ_test_temperature"].as<double>();
        *setting.train_buffer = node["train_buffer"].as<erl::gaussian_process::LidarGaussianProcess1D::TrainBuffer::Setting>();
        *setting.gp = node["gp"].as<erl::gaussian_process::VanillaGaussianProcess::Setting>();
        return true;
    }
};
