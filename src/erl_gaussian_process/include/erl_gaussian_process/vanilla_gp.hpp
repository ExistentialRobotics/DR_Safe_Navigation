#pragma once

#include "erl_covariance/covariance.hpp"

#include <memory>

namespace erl::gaussian_process {

    /**
     * VanillaGaussianProcess implements the standard Gaussian Process
     */
    class VanillaGaussianProcess {

    public:
        // structure for holding the parameters
        struct Setting : public common::Yamlable<Setting> {
            std::string kernel_type = "OrnsteinUhlenbeck2D";
            std::shared_ptr<covariance::Covariance::Setting> kernel = []() -> std::shared_ptr<covariance::Covariance::Setting> {
                auto setting = std::make_shared<covariance::Covariance::Setting>();
                setting->x_dim = 2;
                setting->alpha = 1.;
                setting->scale = 0.5;
                setting->scale_mix = 1.;
                return setting;
            }();
            long max_num_samples = 256;
            bool auto_normalize = false;
        };

    protected:
        long m_x_dim_ = 0;                                            // dimension of x
        long m_num_train_samples_ = 0;                                // number of training samples
        double m_mean_ = 0.;                                          // mean of the training output samples
        double m_std_ = 0.;                                           // standard deviation of the training output samples
        bool m_trained_ = true;                                       // true if the GP is trained
        std::shared_ptr<Setting> m_setting_ = nullptr;                // setting
        std::shared_ptr<covariance::Covariance> m_kernel_ = nullptr;  // kernel
        Eigen::MatrixXd m_mat_k_train_ = {};                          // Ktrain, avoid reallocation
        Eigen::MatrixXd m_mat_x_train_ = {};                          // x1, ..., xn
        Eigen::MatrixXd m_mat_l_ = {};                                // lower triangular matrix of the Cholesky decomposition of Ktrain
        Eigen::VectorXd m_vec_alpha_ = {};                            // h(x1)..h(xn), dh(x1)/dx1_1 .. dh(xn)/dxn_1 .. dh(x1)/dx1_dim .. dh(xn)/dxn_dim
        Eigen::VectorXd m_vec_var_h_ = {};                            // variance of y1 ... yn

    public:
        VanillaGaussianProcess()
            : m_setting_(std::make_shared<Setting>()) {}

        explicit VanillaGaussianProcess(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting should not be nullptr.");
            ERL_ASSERTM(m_setting_->kernel != nullptr, "setting->kernel should not be nullptr.");
            m_trained_ = !(m_setting_->max_num_samples > 0 && m_setting_->kernel->x_dim > 0);  // if memory is allocated, the model is ready to be trained
            if (!m_trained_) { ERL_ASSERTM(AllocateMemory(m_setting_->max_num_samples, m_setting_->kernel->x_dim), "Failed to allocate memory."); }
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] bool
        IsTrained() const {
            return m_trained_;
        }

        /**
         * @brief reset the model: update flags, kernel, and allocate memory if necessary, etc.
         * @param max_num_samples maximum number of training samples
         * @param x_dim dimension of training input samples
         */
        void
        Reset(long max_num_samples, long x_dim);

        [[nodiscard]] long
        GetNumTrainSamples() const {
            return m_num_train_samples_;
        }

        [[nodiscard]] Eigen::MatrixXd &
        GetTrainInputSamplesBuffer() {
            return m_mat_x_train_;
        }

        [[nodiscard]] Eigen::VectorXd &
        GetTrainOutputSamplesBuffer() {
            return m_vec_alpha_;
        }

        [[nodiscard]] Eigen::VectorXd &
        GetTrainOutputSamplesVarianceBuffer() {
            return m_vec_var_h_;
        }

        [[nodiscard]] Eigen::MatrixXd
        GetKtrain() const {
            return m_mat_k_train_;
        }

        [[nodiscard]] Eigen::MatrixXd
        GetCholeskyDecomposition() const {
            return m_mat_l_;
        }

        void
        Train(long num_train_samples);

        void
        Test(const Eigen::Ref<const Eigen::MatrixXd> &mat_x_test, Eigen::Ref<Eigen::VectorXd> vec_f_out, Eigen::Ref<Eigen::VectorXd> vec_var_out) const;

    protected:
        bool
        AllocateMemory(const long max_num_samples, const long x_dim) {
            if (m_setting_->max_num_samples > 0 && max_num_samples > m_setting_->max_num_samples) { return false; }
            if (m_setting_->kernel->x_dim > 0 && x_dim != m_setting_->kernel->x_dim) { return false; }
            const auto [rows, cols] = covariance::Covariance::GetMinimumKtrainSize(max_num_samples, 0, 0);
            if (m_mat_k_train_.rows() < rows || m_mat_k_train_.cols() < cols) { m_mat_k_train_.resize(rows, cols); }
            if (m_mat_x_train_.rows() < x_dim || m_mat_x_train_.cols() < max_num_samples) { m_mat_x_train_.resize(x_dim, max_num_samples); }
            if (m_mat_l_.rows() < rows || m_mat_l_.cols() < cols) { m_mat_l_.resize(rows, cols); }
            if (m_vec_alpha_.size() < max_num_samples) { m_vec_alpha_.resize(max_num_samples); }
            if (m_vec_var_h_.size() < max_num_samples) { m_vec_var_h_.resize(max_num_samples); }
            return true;
        }
    };
}  // namespace erl::gaussian_process

template<>
struct YAML::convert<erl::gaussian_process::VanillaGaussianProcess::Setting> {
    static Node
    encode(const erl::gaussian_process::VanillaGaussianProcess::Setting &setting) {
        Node node;
        node["kernel_type"] = setting.kernel_type;
        node["kernel"] = *setting.kernel;
        node["auto_normalize"] = setting.auto_normalize;
        return node;
    }

    static bool
    decode(const Node &node, erl::gaussian_process::VanillaGaussianProcess::Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.kernel_type = node["kernel_type"].as<std::string>();
        setting.kernel = node["kernel"].as<std::shared_ptr<erl::covariance::Covariance::Setting>>();
        setting.auto_normalize = node["auto_normalize"].as<bool>();
        return true;
    }
};  // namespace YAML
