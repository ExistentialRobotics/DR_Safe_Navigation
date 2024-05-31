#pragma once

#include "vanilla_gp.hpp"

#include "erl_covariance/covariance.hpp"

#include <utility>

namespace erl::gaussian_process {

    class NoisyInputGaussianProcess {

    public:
        struct Setting : public common::Yamlable<Setting> {
            std::string kernel_type = "Matern32_2D";
            std::shared_ptr<covariance::Covariance::Setting> kernel = []() -> std::shared_ptr<covariance::Covariance::Setting> {
                auto setting = std::make_shared<covariance::Covariance::Setting>();
                setting->x_dim = 2;
                setting->alpha = 1.;
                setting->scale = 1.2;
                return setting;
            }();
            long max_num_samples = -1;  // maximum number of training samples, -1 means no limit
        };

    protected:
        long m_x_dim_ = 0;                                            // dimension of x
        long m_num_train_samples_ = 0;                                // number of training samples
        long m_num_train_samples_with_grad_ = 0;                      // number of training samples with gradient
        bool m_trained_ = false;                                      // true if the GP is trained
        double m_three_over_scale_square_ = 0.;                       // for computing normal variance
        std::shared_ptr<Setting> m_setting_ = nullptr;                // setting
        std::shared_ptr<covariance::Covariance> m_kernel_ = nullptr;  // kernel
        Eigen::MatrixXd m_mat_x_train_ = {};                          // x1, ..., xn
        Eigen::VectorXd m_vec_y_train_ = {};                          // h(x1), ..., h(xn)
        Eigen::MatrixXd m_mat_grad_train_ = {};                       // dh(x_j)/dx_ij for index (i, j)
        Eigen::MatrixXd m_mat_k_train_ = {};                          // Ktrain, avoid reallocation
        Eigen::MatrixXd m_mat_l_ = {};                                // lower triangular matrix of the Cholesky decomposition of Ktrain
        Eigen::VectorXb m_vec_grad_flag_ = {};                        // true if the corresponding training sample has gradient
        Eigen::VectorXd m_vec_alpha_ = {};                            // h(x1)..h(xn), dh(x1)/dx1_1 .. dh(xn)/dxn_1 .. dh(x1)/dx1_dim .. dh(xn)/dxn_dim
        Eigen::VectorXd m_vec_var_x_ = {};                            // variance of x1 ... xn
        Eigen::VectorXd m_vec_var_h_ = {};                            // variance of h(x1)..h(xn)
        Eigen::VectorXd m_vec_var_grad_ = {};                         // variance of dh(x1)/dx1_1 .. dh(xn)/dxn_1 .. dh(x1)/dx1_dim .. dh(xn)/dxn_dim

    public:
        NoisyInputGaussianProcess()
            : m_setting_(std::make_shared<Setting>()) {}

        explicit NoisyInputGaussianProcess(std::shared_ptr<Setting> setting)
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

        [[maybe_unused]] [[nodiscard]] bool
        IsTrained() const {
            return m_trained_;
        }

        virtual void
        Reset(const long max_num_samples, const long x_dim) {
            ERL_ASSERTM(max_num_samples > 0, "max_num_samples should be > 0.");
            ERL_ASSERTM(x_dim > 0, "x_dim should be > 0.");
            if (m_setting_->max_num_samples > 0 && m_setting_->kernel->x_dim > 0) {  // memory already allocated
                ERL_ASSERTM(m_setting_->max_num_samples >= max_num_samples, "max_num_samples should be <= {}.", m_setting_->max_num_samples);
            } else {
                if (m_setting_->kernel->x_dim > 0) { ERL_ASSERTM(m_setting_->kernel->x_dim == x_dim, "x_dim should be {}.", m_setting_->kernel->x_dim); }
                ERL_ASSERTM(AllocateMemory(max_num_samples, x_dim), "Failed to allocate memory.");
            }
            m_trained_ = false;
            m_kernel_ = covariance::Covariance::CreateCovariance(m_setting_->kernel_type);
            m_kernel_->GetSetting()->x_dim = x_dim;
            m_three_over_scale_square_ = 3. * m_setting_->kernel->alpha / (m_setting_->kernel->scale * m_setting_->kernel->scale);
            m_num_train_samples_ = 0;
            m_num_train_samples_with_grad_ = 0;
            m_x_dim_ = x_dim;
        }

        [[nodiscard]] long
        GetNumTrainSamples() const {
            return m_num_train_samples_;
        }

        [[nodiscard]] long
        GetNumTrainSamplesWithGrad() const {
            return m_num_train_samples_with_grad_;
        }

        [[nodiscard]] Eigen::MatrixXd &
        GetTrainInputSamplesBuffer() {
            return m_mat_x_train_;
        }

        [[nodiscard]] Eigen::VectorXd &
        GetTrainOutputSamplesBuffer() {
            return m_vec_y_train_;
        }

        [[nodiscard]] Eigen::MatrixXd &
        GetTrainOutputGradientSamplesBuffer() {
            return m_mat_grad_train_;
        }

        [[nodiscard]] Eigen::VectorXb &
        GetTrainGradientFlagsBuffer() {
            return m_vec_grad_flag_;
        }

        [[nodiscard]] Eigen::VectorXd &
        GetTrainInputSamplesVarianceBuffer() {
            return m_vec_var_x_;
        }

        [[nodiscard]] Eigen::VectorXd &
        GetTrainOutputValueSamplesVarianceBuffer() {
            return m_vec_var_h_;
        }

        [[nodiscard]] Eigen::VectorXd &
        GetTrainOutputGradientSamplesVarianceBuffer() {
            return m_vec_var_grad_;
        }

        [[nodiscard]] Eigen::MatrixXd
        GetKtrain() {
            return m_mat_k_train_;
        }

        [[nodiscard]] Eigen::VectorXd
        GetAlpha() {
            return m_vec_alpha_;
        }

        [[nodiscard]] Eigen::MatrixXd
        GetCholeskyDecomposition() {
            return m_mat_l_;
        }

        virtual void
        Train(long num_train_samples);

        virtual void
        Test(
            const Eigen::Ref<const Eigen::MatrixXd> &mat_x_test,
            Eigen::Ref<Eigen::MatrixXd> mat_f_out,
            Eigen::Ref<Eigen::MatrixXd> mat_var_out,
            Eigen::Ref<Eigen::MatrixXd> mat_cov_out) const;

        virtual ~NoisyInputGaussianProcess() = default;

    protected:
        bool
        AllocateMemory(const long max_num_samples, const long x_dim) {
            if (max_num_samples <= 0 || x_dim <= 0) { return false; }  // invalid input
            if (m_setting_->max_num_samples > 0 && max_num_samples > m_setting_->max_num_samples) { return false; }
            if (m_setting_->kernel->x_dim > 0 && x_dim != m_setting_->kernel->x_dim) { return false; }
            const auto [rows, cols] = covariance::Covariance::GetMinimumKtrainSize(max_num_samples, max_num_samples, x_dim);
            if (m_mat_k_train_.rows() < rows || m_mat_k_train_.cols() < cols) { m_mat_k_train_.resize(rows, cols); }
            if (m_mat_x_train_.rows() < x_dim || m_mat_x_train_.cols() < max_num_samples) { m_mat_x_train_.resize(x_dim, max_num_samples); }
            if (m_vec_y_train_.size() < max_num_samples) { m_vec_y_train_.resize(max_num_samples); }
            if (m_mat_grad_train_.rows() < x_dim || m_mat_grad_train_.cols() < max_num_samples) { m_mat_grad_train_.resize(x_dim, max_num_samples); }
            if (m_mat_l_.rows() < rows || m_mat_l_.cols() < cols) { m_mat_l_.resize(rows, cols); }
            if (m_vec_alpha_.size() < max_num_samples * (x_dim + 1)) { m_vec_alpha_.resize(max_num_samples * (x_dim + 1)); }
            if (m_vec_grad_flag_.size() < max_num_samples) { m_vec_grad_flag_.resize(max_num_samples); }
            if (m_vec_var_x_.size() < max_num_samples) { m_vec_var_x_.resize(max_num_samples); }
            if (m_vec_var_h_.size() < max_num_samples) { m_vec_var_h_.resize(max_num_samples); }
            if (m_vec_var_grad_.size() < max_num_samples) { m_vec_var_grad_.resize(max_num_samples); }
            return true;
        }

        void
        InitializeVectorAlpha() {
            ERL_DEBUG_ASSERT(
                m_vec_alpha_.size() >= m_num_train_samples_ * (m_x_dim_ + 1),
                "m_vec_alpha_ should have size >= {}.",
                m_num_train_samples_ * (m_x_dim_ + 1));

            m_num_train_samples_with_grad_ = 0;
            for (long i = 0; i < m_num_train_samples_; ++i) {
                m_vec_alpha_[i] = m_vec_y_train_[i];  // h(x_i)
                if (m_vec_grad_flag_[i]) { ++m_num_train_samples_with_grad_; }
            }
            for (long i = 0, j = m_num_train_samples_; i < m_num_train_samples_; ++i) {
                if (!m_vec_grad_flag_[i]) { continue; }
                for (long k = 0, l = j++; k < m_x_dim_; ++k, l += m_num_train_samples_with_grad_) { m_vec_alpha_[l] = m_mat_grad_train_(k, i); }
            }
        }
    };
}  // namespace erl::gaussian_process

template<>
struct YAML::convert<erl::gaussian_process::NoisyInputGaussianProcess::Setting> {
    static Node
    encode(const erl::gaussian_process::NoisyInputGaussianProcess::Setting &setting) {
        Node node;
        node["kernel"] = *setting.kernel;
        node["max_num_samples"] = setting.max_num_samples;
        return node;
    }

    static bool
    decode(const Node &node, erl::gaussian_process::NoisyInputGaussianProcess::Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.kernel = node["kernel"].as<std::shared_ptr<erl::covariance::Covariance::Setting>>();
        setting.max_num_samples = node["max_num_samples"].as<long>();
        return true;
    }
};  // namespace YAML
