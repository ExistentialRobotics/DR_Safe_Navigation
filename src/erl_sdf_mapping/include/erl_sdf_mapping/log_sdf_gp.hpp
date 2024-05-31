#pragma once

#include "erl_gaussian_process/noisy_input_gp.hpp"

#include <utility>

namespace erl::sdf_mapping {

    class LogSdfGaussianProcess : public gaussian_process::NoisyInputGaussianProcess {

    public:
        struct Setting : public common::OverrideYamlable<NoisyInputGaussianProcess::Setting, Setting> {
            double log_lambda = 40.0;
            double edf_threshold = 0.1;
        };

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<covariance::Covariance> m_kernel_ = nullptr;
        Eigen::MatrixXd m_mat_log_k_train_ = {};
        Eigen::MatrixXd m_mat_log_l_ = {};
        Eigen::VectorXd m_vec_log_alpha_ = {};

    public:
        LogSdfGaussianProcess()
            : LogSdfGaussianProcess(std::make_shared<Setting>()) {}

        explicit LogSdfGaussianProcess(std::shared_ptr<Setting> setting)
            : NoisyInputGaussianProcess(setting),
              m_setting_(std::move(setting)) {
            if (!m_trained_) { AllocateMemory2(m_setting_->max_num_samples, m_setting_->kernel->x_dim); }
        }

        void
        Reset(const long max_num_samples, const long x_dim) override {
            NoisyInputGaussianProcess::Reset(max_num_samples, x_dim);
            if (m_setting_->max_num_samples <= 0 || m_setting_->kernel->x_dim <= 0) { AllocateMemory2(max_num_samples, x_dim); }
            m_kernel_ = covariance::Covariance::CreateCovariance(m_setting_->kernel_type);
            const auto kernel_setting = m_kernel_->GetSetting();
            *kernel_setting = *m_setting_->kernel;
            kernel_setting->scale = std::sqrt(3.) / m_setting_->log_lambda;
        }

        void
        Train(long num_train_samples) override;

        void
        Test(
            const Eigen::Ref<const Eigen::MatrixXd> &mat_x_test,
            Eigen::Ref<Eigen::MatrixXd> mat_f_out,
            Eigen::Ref<Eigen::MatrixXd> mat_var_out,
            Eigen::Ref<Eigen::MatrixXd> mat_cov_out) const override;

    protected:
        bool
        AllocateMemory2(const long max_num_samples, const long x_dim) {
            const auto [rows, cols] = covariance::Covariance::GetMinimumKtrainSize(max_num_samples, 0, x_dim);
            if (m_mat_log_k_train_.rows() < rows || m_mat_log_k_train_.cols() < cols) { m_mat_log_k_train_.resize(rows, cols); }
            if (m_mat_log_l_.rows() < rows || m_mat_log_l_.cols() < cols) { m_mat_log_l_.resize(rows, cols); }
            if (m_vec_log_alpha_.size() < max_num_samples) { m_vec_log_alpha_.resize(max_num_samples); }
            return true;
        }
    };
}  // namespace erl::sdf_mapping

template<>
struct YAML::convert<erl::sdf_mapping::LogSdfGaussianProcess::Setting> {
    static Node
    encode(const erl::sdf_mapping::LogSdfGaussianProcess::Setting &setting) {
        Node node;
        node["kernel"] = setting.kernel;
        node["log_lambda"] = setting.log_lambda;
        node["edf_threshold"] = setting.edf_threshold;
        return node;
    }

    static bool
    decode(const Node &node, erl::sdf_mapping::LogSdfGaussianProcess::Setting &setting) {
        if (!node.IsMap()) { return false; }
        convert<erl::gaussian_process::NoisyInputGaussianProcess::Setting>::decode(node, setting);
        setting.log_lambda = node["log_lambda"].as<double>();
        setting.edf_threshold = node["edf_threshold"].as<double>();
        return true;
    }
};  // namespace YAML
