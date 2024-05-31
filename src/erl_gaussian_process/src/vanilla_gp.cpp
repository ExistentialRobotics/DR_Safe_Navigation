#include "erl_gaussian_process/vanilla_gp.hpp"

namespace erl::gaussian_process {
    void
    VanillaGaussianProcess::Reset(const long max_num_samples, const long x_dim) {
        ERL_ASSERTM(x_dim > 0, "x_dim should be > 0.");
        if (const long &max_num_samples_setting = m_setting_->max_num_samples, &x_dim_setting = m_setting_->kernel->x_dim;
            max_num_samples_setting > 0 && x_dim_setting > 0) {  // memory already allocated
            ERL_ASSERTM(max_num_samples_setting >= max_num_samples, "max_num_samples should be <= {}.", max_num_samples_setting);
        } else {
            ERL_ASSERTM(x_dim_setting <= 0 || x_dim_setting == x_dim, "x_dim should be {}.", x_dim_setting);
            ERL_ASSERTM(AllocateMemory(max_num_samples, x_dim), "Failed to allocate memory.");
        }
        m_trained_ = false;
        m_kernel_ = covariance::Covariance::CreateCovariance(m_setting_->kernel_type);
        m_kernel_->GetSetting()->x_dim = x_dim;
        m_num_train_samples_ = 0;
        m_x_dim_ = x_dim;
    }

    void
    VanillaGaussianProcess::Train(const long num_train_samples) {

        if (m_trained_) {
            ERL_WARN("The model has been trained. Please reset the model before training.");
            return;
        }

        m_num_train_samples_ = num_train_samples;
        if (m_num_train_samples_ <= 0) {
            ERL_WARN("num_train_samples = {}, it should be > 0.", m_num_train_samples_);
            return;
        }

        // Compute kernel matrix
        const auto mat_x_train = m_mat_x_train_.topLeftCorner(m_x_dim_, m_num_train_samples_);
        const auto vec_var_h = m_vec_var_h_.head(m_num_train_samples_);
        const auto [rows, cols] = m_kernel_->ComputeKtrain(m_mat_k_train_, mat_x_train, vec_var_h);
        const auto mat_ktrain = m_mat_k_train_.topLeftCorner(rows, cols);
        auto &&mat_l = m_mat_l_.topLeftCorner(rows, cols);
        auto vec_alpha = m_vec_alpha_.head(m_num_train_samples_);
        if (m_setting_->auto_normalize) {
            m_mean_ = vec_alpha.mean();
            m_std_ = std::max(1.e-6, std::sqrt(vec_alpha.cwiseAbs2().mean() - m_mean_ * m_mean_));  // biased std
            vec_alpha = (vec_alpha.array() - m_mean_) / m_std_;
        }
        mat_l = mat_ktrain.llt().matrixL();  // A = ktrain(mat_x_train, mat_x_train) + sigma * I = mat_l @ mat_l.T
        mat_l.triangularView<Eigen::Lower>().solveInPlace(vec_alpha);
        mat_l.transpose().triangularView<Eigen::Upper>().solveInPlace(vec_alpha);  // A.m_inv_() @ vec_alpha

        m_trained_ = true;
    }

    void
    VanillaGaussianProcess::Test(
        const Eigen::Ref<const Eigen::MatrixXd> &mat_x_test,
        Eigen::Ref<Eigen::VectorXd> vec_f_out,
        Eigen::Ref<Eigen::VectorXd> vec_var_out) const {

        if (!m_trained_ || m_num_train_samples_ <= 0) { return; }

        long n = mat_x_test.cols();
        if (n == 0) { return; }
        ERL_ASSERTM(mat_x_test.rows() == m_x_dim_, "mat_x_test.rows() = {}, it should be {}.", mat_x_test.rows(), m_x_dim_);
        ERL_ASSERTM(vec_f_out.size() >= n, "vec_f_out size = {}, it should be >= {}.", vec_f_out.size(), n);
        const auto [ktest_rows, ktest_cols] = covariance::Covariance::GetMinimumKtestSize(m_num_train_samples_, 0, 0, n);
        Eigen::MatrixXd ktest(ktest_rows, ktest_cols);
        const auto [output_rows, output_cols] = m_kernel_->ComputeKtest(ktest, m_mat_x_train_.topLeftCorner(m_x_dim_, m_num_train_samples_), mat_x_test);
        ERL_DEBUG_ASSERT(
            (output_rows == ktest_rows && output_cols == ktest_cols),
            "output_size = ({}, {}), it should be ({}, {}).",
            output_rows,
            output_cols,
            ktest_rows,
            ktest_cols);

        // xt is one column of mat_x_test
        // expectation of vec_f_out = ktest(xt, X) @ (ktest(X, X) + sigma * I).m_inv_() @ y
        const auto vec_alpha = m_vec_alpha_.head(output_rows);
        if (m_setting_->auto_normalize) {
            for (long i = 0; i < output_rows; ++i) { vec_f_out[i] = ktest.col(i).dot(vec_alpha) * m_std_ + m_mean_; }
        } else {
            for (long i = 0; i < output_cols; ++i) { vec_f_out[i] = ktest.col(i).dot(vec_alpha); }
        }
        if (vec_var_out.size() == 0) { return; }  // only compute mean

        // variance of vec_f_out = ktest(xt, xt) - ktest(xt, X) @ (ktest(X, X) + sigma * I).m_inv_() @ ktest(X, xt)
        //                       = ktest(xt, xt) - ktest(xt, X) @ (m_l_ @ m_l_.T).m_inv_() @ ktest(X, xt)
        ERL_ASSERTM(vec_var_out.size() >= n, "vec_var_out size = {}, it should be >= {}.", vec_var_out.size(), n);
        m_mat_l_.topLeftCorner(output_rows, output_rows).triangularView<Eigen::Lower>().solveInPlace(ktest);
        for (long i = 0; i < ktest_cols; ++i) { vec_var_out[i] = m_setting_->kernel->alpha - ktest.col(i).squaredNorm(); }
    }
}  // namespace erl::gaussian_process
