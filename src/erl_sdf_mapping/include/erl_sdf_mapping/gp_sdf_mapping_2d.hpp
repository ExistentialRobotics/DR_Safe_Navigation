#pragma once

#include "abstract_surface_mapping_2d.hpp"
#include "log_sdf_gp.hpp"

#include "erl_common/yaml.hpp"

#include <memory>
#include <queue>

namespace erl::sdf_mapping {

    class GpSdfMapping2D {

    public:
        struct Setting : public common::Yamlable<Setting> {
            struct TestQuery : public Yamlable<TestQuery> {
                double max_test_valid_distance_var = 0.4;  // maximum distance variance of prediction.
                double search_area_half_size = 4.8;
                bool use_nearest_only = false;    // if true, only the nearest point will be used for prediction.
                bool compute_covariance = false;  // if true, compute covariance of prediction.
                bool recompute_variance = true;   // if true, compute variance using different method.
                double softmax_temperature = 10.;
            };

            uint32_t num_threads = 64;
            double update_hz = 20;                // frequency that Update() is called.
            double gp_sdf_area_scale = 4;         // ratio between GP area and Quadtree cluster area
            double offset_distance = 0.0;         // offset distance for surface points
            double max_valid_gradient_var = 0.1;  // maximum gradient variance qualified for training.
            double invalid_position_var = 2.;     // position variance of points whose gradient is labeled invalid, i.e. > max_valid_gradient_var.
            bool train_gp_immediately = false;
            std::shared_ptr<LogSdfGaussianProcess::Setting> gp_sdf = std::make_shared<LogSdfGaussianProcess::Setting>();
            std::shared_ptr<TestQuery> test_query = std::make_shared<TestQuery>();  // parameters used by Test.
            bool log_timing = false;
        };

        struct Gp {
            bool active = false;
            std::atomic_bool locked_for_test = false;
            long num_train_samples = 0;
            Eigen::Vector2d position;
            double half_size = 0;
            std::shared_ptr<LogSdfGaussianProcess> gp = {};

            void
            Train() const {
                gp->Train(num_train_samples);
            }
        };

        using QuadtreeKeyGpMap = std::unordered_map<geometry::QuadtreeKey, std::shared_ptr<Gp>, geometry::QuadtreeKey::KeyHash>;

    private:
        std::shared_ptr<Setting> m_setting_ = std::make_shared<Setting>();
        std::mutex m_mutex_;
        std::shared_ptr<AbstractSurfaceMapping2D> m_surface_mapping_ = nullptr;                 // for getting surface points, racing condition.
        std::vector<geometry::QuadtreeKey> m_clusters_to_update_ = {};                          // stores clusters that are to be updated by UpdateGpThread.
        QuadtreeKeyGpMap m_gp_map_ = {};                                                        // for getting GP from Quadtree key, racing condition.
        std::vector<std::vector<std::pair<double, std::shared_ptr<Gp>>>> m_query_to_gps_ = {};  // for testing, racing condition
        std::vector<std::array<std::shared_ptr<Gp>, 2>> m_query_used_gps_ = {};                 // for testing, racing condition
        std::list<std::shared_ptr<Gp>> m_new_gps_ = {};                                         // caching new GPs to be moved into m_gps_to_train_
        std::vector<std::shared_ptr<Gp>> m_gps_to_train_ = {};                                  // for training SDF GPs, racing condition in Update() and Test()
        double m_train_gp_time_ = 10;                                                           // us
        std::mutex m_log_mutex_;                                                                // for logging
        double m_travel_distance_ = 0;                                                          // for logging
        std::optional<Eigen::Vector2d> m_last_position_ = std::nullopt;                         // for logging
        std::ofstream m_train_log_file_;
        std::ofstream m_test_log_file_;

        // for testing
        struct TestBuffer {
            std::unique_ptr<Eigen::Ref<const Eigen::Matrix2Xd>> positions = nullptr;
            std::unique_ptr<Eigen::Ref<Eigen::VectorXd>> distances = nullptr;
            std::unique_ptr<Eigen::Ref<Eigen::Matrix2Xd>> gradients = nullptr;
            std::unique_ptr<Eigen::Ref<Eigen::Matrix3Xd>> variances = nullptr;
            std::unique_ptr<Eigen::Ref<Eigen::Matrix3Xd>> covariances = nullptr;

            [[nodiscard]] std::size_t
            Size() const {
                if (positions == nullptr) return 0;
                return positions->cols();
            }

            bool
            ConnectBuffers(
                const Eigen::Ref<const Eigen::Matrix2Xd>& positions_in,
                Eigen::VectorXd& distances_out,
                Eigen::Matrix2Xd& gradients_out,
                Eigen::Matrix3Xd& variances_out,
                Eigen::Matrix3Xd& covariances_out,
                const bool compute_covariance) {
                positions = nullptr;
                distances = nullptr;
                gradients = nullptr;
                variances = nullptr;
                covariances = nullptr;
                const long n = positions_in.cols();
                if (n == 0) return false;
                distances_out.resize(n);
                gradients_out.resize(2, n);
                variances_out.resize(3, n);
                if (compute_covariance) { covariances_out.resize(3, n); }
                this->positions = std::make_unique<Eigen::Ref<const Eigen::Matrix2Xd>>(positions_in);
                this->distances = std::make_unique<Eigen::Ref<Eigen::VectorXd>>(distances_out);
                this->gradients = std::make_unique<Eigen::Ref<Eigen::Matrix2Xd>>(gradients_out);
                this->variances = std::make_unique<Eigen::Ref<Eigen::Matrix3Xd>>(variances_out);
                this->covariances = std::make_unique<Eigen::Ref<Eigen::Matrix3Xd>>(covariances_out);
                return true;
            }

            void
            DisconnectBuffers() {
                positions = nullptr;
                distances = nullptr;
                gradients = nullptr;
                variances = nullptr;
                covariances = nullptr;
            }
        };

        TestBuffer m_test_buffer_ = {};

    public:
        explicit GpSdfMapping2D(std::shared_ptr<AbstractSurfaceMapping2D> surface_mapping, std::shared_ptr<Setting> setting = nullptr)
            : m_setting_(std::move(setting)),
              m_surface_mapping_(std::move(surface_mapping)) {
            if (m_setting_ == nullptr) { m_setting_ = std::make_shared<Setting>(); }
            ERL_ASSERTM(m_surface_mapping_ != nullptr, "surface_mapping is nullptr.");

            // get log dir from env
            if (m_setting_->log_timing) {
                char* log_dir_env = std::getenv("LOG_DIR");
                const std::filesystem::path log_dir = log_dir_env == nullptr ? std::filesystem::current_path() : std::filesystem::path(log_dir_env);
                const std::filesystem::path train_log_file_name = log_dir / "gp_sdf_mapping_2d_train.csv";
                const std::filesystem::path test_log_file_name = log_dir / "gp_sdf_mapping_2d_test.csv";
                if (std::filesystem::exists(train_log_file_name)) { std::filesystem::remove(train_log_file_name); }
                if (std::filesystem::exists(test_log_file_name)) { std::filesystem::remove(test_log_file_name); }
                m_train_log_file_.open(train_log_file_name);
                m_test_log_file_.open(test_log_file_name);
                ERL_WARN_COND(!m_train_log_file_.is_open(), ("Failed to open " + train_log_file_name.string()).c_str());
                ERL_WARN_COND(!m_test_log_file_.is_open(), ("Failed to open " + test_log_file_name.string()).c_str());
                m_train_log_file_ << "travel_distance,surf_mapping_time(us),gp_data_update_time(us),gp_delay_cnt,"
                                  << "gp_train_time(us),total_gp_update_time(ms),total_update_time(ms)" << std::endl
                                  << std::flush;
                m_test_log_file_ << "travel_distance,gp_search_time(us),gp_train_time(us),gp_test_time(us),total_test_time(ms)" << std::endl << std::flush;
            }
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] std::shared_ptr<AbstractSurfaceMapping2D>
        GetSurfaceMapping() const {
            return m_surface_mapping_;
        }

        bool
        Update(
            const Eigen::Ref<const Eigen::VectorXd>& angles,
            const Eigen::Ref<const Eigen::VectorXd>& distances,
            const Eigen::Ref<const Eigen::Matrix23d>& pose);

        bool
        Test(
            const Eigen::Ref<const Eigen::Matrix2Xd>& positions_in,
            Eigen::VectorXd& distances_out,
            Eigen::Matrix2Xd& gradients_out,
            Eigen::Matrix3Xd& variances_out,
            Eigen::Matrix3Xd& covariances_out);

        const std::vector<std::array<std::shared_ptr<Gp>, 2>>&
        GetUsedGps() const {
            return m_query_used_gps_;
        }

    private:
        void
        UpdateGps(double time_budget);

        void
        UpdateGpThread(uint32_t thread_idx, std::size_t start_idx, std::size_t end_idx);

        void
        TrainGps();

        void
        TrainGpThread(uint32_t thread_idx, std::size_t start_idx, std::size_t end_idx) const;

        void
        SearchGpThread(uint32_t thread_idx, std::size_t start_idx, std::size_t end_idx);

        void
        TestGpThread(uint32_t thread_idx, std::size_t start_idx, std::size_t end_idx);
    };
}  // namespace erl::sdf_mapping

template<>
struct YAML::convert<erl::sdf_mapping::GpSdfMapping2D::Setting::TestQuery> {

    static Node
    encode(const erl::sdf_mapping::GpSdfMapping2D::Setting::TestQuery& rhs) {
        Node node;
        node["max_test_valid_distance_var"] = rhs.max_test_valid_distance_var;
        node["search_area_half_size"] = rhs.search_area_half_size;
        node["use_nearest_only"] = rhs.use_nearest_only;
        node["compute_covariance"] = rhs.compute_covariance;
        node["recompute_variance"] = rhs.recompute_variance;
        node["softmax_temperature"] = rhs.softmax_temperature;
        return node;
    }

    static bool
    decode(const Node& node, erl::sdf_mapping::GpSdfMapping2D::Setting::TestQuery& rhs) {
        if (!node.IsMap()) { return false; }
        rhs.max_test_valid_distance_var = node["max_test_valid_distance_var"].as<double>();
        rhs.search_area_half_size = node["search_area_half_size"].as<double>();
        rhs.use_nearest_only = node["use_nearest_only"].as<bool>();
        rhs.compute_covariance = node["compute_covariance"].as<bool>();
        rhs.recompute_variance = node["recompute_variance"].as<bool>();
        rhs.softmax_temperature = node["softmax_temperature"].as<double>();
        return true;
    }
};

template<>
struct YAML::convert<erl::sdf_mapping::GpSdfMapping2D::Setting> {
    static Node
    encode(const erl::sdf_mapping::GpSdfMapping2D::Setting& setting) {
        Node node;
        node["num_threads"] = setting.num_threads;
        node["update_hz"] = setting.update_hz;
        node["gp_sdf_area_scale"] = setting.gp_sdf_area_scale;
        node["offset_distance"] = setting.offset_distance;
        node["max_valid_gradient_var"] = setting.max_valid_gradient_var;
        node["invalid_position_var"] = setting.invalid_position_var;
        node["train_gp_immediately"] = setting.train_gp_immediately;
        node["gp_sdf"] = setting.gp_sdf;
        node["test_query"] = setting.test_query;
        node["log_timing"] = setting.log_timing;
        return node;
    }

    static bool
    decode(const Node& node, erl::sdf_mapping::GpSdfMapping2D::Setting& setting) {
        if (!node.IsMap()) { return false; }
        setting.num_threads = node["num_threads"].as<uint32_t>();
        setting.update_hz = node["update_hz"].as<double>();
        setting.gp_sdf_area_scale = node["gp_sdf_area_scale"].as<double>();
        setting.offset_distance = node["offset_distance"].as<double>();
        setting.max_valid_gradient_var = node["max_valid_gradient_var"].as<double>();
        setting.invalid_position_var = node["invalid_position_var"].as<double>();
        setting.train_gp_immediately = node["train_gp_immediately"].as<bool>();
        setting.gp_sdf = node["gp_sdf"].as<decltype(setting.gp_sdf)>();
        setting.test_query = node["test_query"].as<decltype(setting.test_query)>();
        setting.log_timing = node["log_timing"].as<bool>();
        return true;
    }
};
