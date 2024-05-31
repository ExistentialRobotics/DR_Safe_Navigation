#pragma once

#include "erl_common/eigen.hpp"
#include "erl_common/exception.hpp"
#include "erl_common/logging.hpp"
#include "erl_common/yaml.hpp"

#include <memory>

namespace erl::covariance {

    // ref1: https://peterroelants.github.io/posts/gaussian-process-kernels/
    // ref2: https://www.cs.toronto.edu/~duvenaud/cookbook/

    class Covariance {
    protected:
        inline static std::map<std::string, std::shared_ptr<Covariance>> s_class_id_mapping_ = {};

    public:
        // structure for holding the parameters
        struct Setting : public common::Yamlable<Setting> {
            long x_dim = 2;           // dimension of input space
            double alpha = 1.;        // overall covariance magnitude
            double scale = 1.;        // m_scale_ length
            double scale_mix = 1.;    // used by RationalQuadratic, decreasing this value allows more local variations, inf --> Gaussian kernel
            Eigen::VectorXd weights;  // used by some custom kernels
        };

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;

    public:
        //-- factory pattern
        /**
         * returns actual class name as string for identification
         * @return The type of the tree.
         */
        [[nodiscard]] virtual std::string
        GetCovarianceType() const = 0;

        /**
         * Implemented by derived classes to create a new tree of the same type.
         * @return A new tree of the same type.
         */
        [[nodiscard]] virtual std::shared_ptr<Covariance>
        Create() const = 0;

        /**
         * Create a new covariance of the given type.
         * @param covariance_type
         * @return
         */
        static std::shared_ptr<Covariance>
        CreateCovariance(const std::string &covariance_type);

        static void
        RegisterCovarianceType(const std::shared_ptr<Covariance> &covariance);

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] static std::pair<long, long>
        GetMinimumKtrainSize(const long num_samples, const long num_samples_with_gradient, const long num_gradient_dimensions) {
            long n = num_samples + num_samples_with_gradient * num_gradient_dimensions;
            return {n, n};
        }

        [[nodiscard]] static std::pair<long, long>
        GetMinimumKtestSize(
            const long num_train_samples,
            const long num_train_samples_with_gradient,
            const long num_gradient_dimensions,
            const long num_test_queries) {
            return {num_train_samples + num_train_samples_with_gradient * num_gradient_dimensions, num_test_queries * (1 + num_gradient_dimensions)};
        }

        [[nodiscard]] virtual std::pair<long, long>
        ComputeKtrain(Eigen::Ref<Eigen::MatrixXd> k_mat, const Eigen::Ref<const Eigen::MatrixXd> &mat_x) const = 0;

        [[nodiscard]] virtual std::pair<long, long>
        ComputeKtrain(Eigen::Ref<Eigen::MatrixXd> k_mat, const Eigen::Ref<const Eigen::MatrixXd> &mat_x, const Eigen::Ref<const Eigen::VectorXd> &vec_var_y)
            const = 0;

        [[nodiscard]] virtual std::pair<long, long>
        ComputeKtest(Eigen::Ref<Eigen::MatrixXd> k_mat, const Eigen::Ref<const Eigen::MatrixXd> &mat_x1, const Eigen::Ref<const Eigen::MatrixXd> &mat_x2)
            const = 0;

        [[nodiscard]] virtual std::pair<long, long>
        ComputeKtrainWithGradient(
            Eigen::Ref<Eigen::MatrixXd> k_mat,
            const Eigen::Ref<const Eigen::MatrixXd> &mat_x,
            const Eigen::Ref<const Eigen::VectorXb> &vec_grad_flags) const = 0;

        [[nodiscard]] virtual std::pair<long, long>
        ComputeKtrainWithGradient(
            Eigen::Ref<Eigen::MatrixXd> k_mat,
            const Eigen::Ref<const Eigen::MatrixXd> &mat_x,
            const Eigen::Ref<const Eigen::VectorXb> &vec_grad_flags,
            const Eigen::Ref<const Eigen::VectorXd> &vec_var_x,
            const Eigen::Ref<const Eigen::VectorXd> &vec_var_y,
            const Eigen::Ref<const Eigen::VectorXd> &vec_var_grad) const = 0;

        /**
         * @brief compute kernel matrix between train samples and test queries with gradient.
         * @param k_mat
         * @param mat_x1
         * @param vec_grad1_flags
         * @param mat_x2
         * @return
         */
        [[nodiscard]] virtual std::pair<long, long>
        ComputeKtestWithGradient(
            Eigen::Ref<Eigen::MatrixXd> k_mat,
            const Eigen::Ref<const Eigen::MatrixXd> &mat_x1,
            const Eigen::Ref<const Eigen::VectorXb> &vec_grad1_flags,
            const Eigen::Ref<const Eigen::MatrixXd> &mat_x2) const = 0;

        virtual ~Covariance() = default;

    protected:
        explicit Covariance(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {}
    };

#define ERL_REGISTER_COVARIANCE(covariance_type)                      \
    inline volatile bool kRegistered##covariance_type = []() {        \
        auto setting = std::make_shared<covariance_type::Setting>();  \
        auto covariance = std::make_shared<covariance_type>(setting); \
        Covariance::RegisterCovarianceType(covariance);               \
        ERL_DEBUG(#covariance_type " is registered.");                \
        return true;                                                  \
    }()
}  // namespace erl::covariance

template<>
struct YAML::convert<erl::covariance::Covariance::Setting> {
    static Node
    encode(const erl::covariance::Covariance::Setting &setting) {
        Node node(NodeType::Map);
        node["x_dim"] = setting.x_dim;
        node["alpha"] = setting.alpha;
        node["scale"] = setting.scale;
        node["scale_mix"] = setting.scale_mix;
        node["weights"] = setting.weights;
        return node;
    }

    static bool
    decode(const Node &node, erl::covariance::Covariance::Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.x_dim = node["x_dim"].as<int>();
        setting.alpha = node["alpha"].as<double>();
        setting.scale = node["scale"].as<double>();
        setting.scale_mix = node["scale_mix"].as<double>();
        setting.weights = node["weights"].as<Eigen::VectorXd>();
        return true;
    }
};
