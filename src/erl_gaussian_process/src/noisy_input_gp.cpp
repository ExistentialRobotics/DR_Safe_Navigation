#include "erl_gaussian_process/noisy_input_gp.hpp"

namespace erl::gaussian_process {

    void
    NoisyInputGaussianProcess::Train(const long num_train_samples) {

        if (m_trained_) {
            ERL_WARN("The model has been trained. Please reset the model before training.");
            return;
        }

        m_num_train_samples_ = num_train_samples;
        if (m_num_train_samples_ <= 0) {
            ERL_WARN("num_train_samples = {}, it should be > 0.", m_num_train_samples_);
            return;
        }

        InitializeVectorAlpha();  // initialize m_vec_alpha_

        // Compute kernel matrix
        const auto [rows, cols] = m_kernel_->ComputeKtrainWithGradient(
            m_mat_k_train_,                                                // buffer of mat_ktrain
            m_mat_x_train_.topLeftCorner(m_x_dim_, m_num_train_samples_),  // mat_x_train
            m_vec_grad_flag_.head(m_num_train_samples_),                   // vec_grad_flag
            m_vec_var_x_.head(m_num_train_samples_),                       // vec_var_x
            m_vec_var_h_.head(m_num_train_samples_),                       // vec_var_h
            m_vec_var_grad_.head(m_num_train_samples_));                   // vec_var_grad
        const auto mat_ktrain = m_mat_k_train_.topLeftCorner(rows, cols);  // square matrix
        auto &&mat_l = m_mat_l_.topLeftCorner(rows, cols);                 // square matrix, lower triangular
        const auto vec_alpha = m_vec_alpha_.head(rows);                    // h and gradient of h
        mat_l = mat_ktrain.llt().matrixL();
        mat_l.triangularView<Eigen::Lower>().solveInPlace(vec_alpha);
        mat_l.transpose().triangularView<Eigen::Upper>().solveInPlace(vec_alpha);
        m_trained_ = true;
    }

    void
    NoisyInputGaussianProcess::Test(
        const Eigen::Ref<const Eigen::MatrixXd> &mat_x_test,
        Eigen::Ref<Eigen::MatrixXd> mat_f_out,
        Eigen::Ref<Eigen::MatrixXd> mat_var_out,
        Eigen::Ref<Eigen::MatrixXd> mat_cov_out) const {

        if (!m_trained_) {
            ERL_WARN("The model has not been trained.");
            return;
        }

        long dim = mat_x_test.rows();
        long n = mat_x_test.cols();
        if (n == 0) { return; }

        // compute mean and gradient of the test queries
        ERL_ASSERTM(mat_f_out.rows() >= dim + 1, "mat_f_out.rows() = {}, it should be >= Dim + 1 = {}.", mat_f_out.rows(), dim + 1);
        ERL_ASSERTM(mat_f_out.cols() >= n, "mat_f_out.cols() = {}, not enough for {} test queries.", mat_f_out.cols(), n);

        const auto [ktest_rows, ktest_cols] = covariance::Covariance::GetMinimumKtestSize(m_num_train_samples_, m_num_train_samples_with_grad_, dim, n);
        Eigen::MatrixXd ktest(ktest_rows, ktest_cols);  // (dim of train samples, dim of test queries)
        auto mat_x_train = m_mat_x_train_.topLeftCorner(m_x_dim_, m_num_train_samples_);
        auto vec_grad_flag = m_vec_grad_flag_.head(m_num_train_samples_);
        const auto [output_rows, output_cols] = m_kernel_->ComputeKtestWithGradient(ktest, mat_x_train, vec_grad_flag, mat_x_test);
        ERL_DEBUG_ASSERT(
            output_rows == ktest_rows && output_cols == ktest_cols,
            "output_size = ({}, {}), it should be ({}, {}).",
            output_rows,
            output_cols,
            ktest_rows,
            ktest_cols);

        // compute value prediction
        /// ktest.T * m_vec_alpha_ = [h(x1),...,h(xn),dh(x1)/dx_1,...,dh(xn)/dx_1,...,dh(x1)/dx_dim,...,dh(xn)/dx_dim]
        auto vec_alpha = m_vec_alpha_.head(output_rows);
        for (long i = 0; i < n; ++i) {
            mat_f_out(0, i) = ktest.col(i).dot(vec_alpha);                                                            // h(x)
            for (long j = 1, jj = i + n; j <= dim; ++j, jj += n) { mat_f_out(j, i) = ktest.col(jj).dot(vec_alpha); }  // dh(x)/dx_j
        }
        if (mat_var_out.size() == 0) { return; }  // only compute mean

        // compute (co)variance of the test queries
        m_mat_l_.topLeftCorner(output_rows, output_rows).triangularView<Eigen::Lower>().solveInPlace(ktest);
        ERL_ASSERTM(mat_var_out.rows() >= dim + 1, "mat_var_out.rows() = {}, it should be >= {} for variance.", mat_var_out.rows(), dim + 1);
        ERL_ASSERTM(mat_var_out.cols() >= n, "mat_var_out.cols() = {}, not enough for {} test queries.", mat_var_out.cols(), n);
        if (mat_cov_out.size() == 0) {  // compute variance only
            // column-wise square sum of ktest = var([h(x1),...,h(xn),dh(x1)/dx_1,...,dh(xn)/dx_1,...,dh(x1)/dx_dim,...,dh(xn)/dx_dim])
            for (long i = 0; i < n; ++i) {
                mat_var_out(0, i) = m_setting_->kernel->alpha - ktest.col(i).squaredNorm();  // variance of h(x)
                for (long j = 1, jj = i + n; j <= dim; ++j, jj += n) {                       // variance of dh(x)/dx_j
                    mat_var_out(j, i) = m_three_over_scale_square_ - ktest.col(jj).squaredNorm();
                }
            }
        } else {  // compute covariance
            long min_n_rows = (dim + 1) * dim / 2;
            ERL_ASSERTM(mat_cov_out.rows() >= min_n_rows, "mat_cov_out.rows() = {}, it should be >= {} for covariance.", mat_cov_out.rows(), min_n_rows);
            ERL_ASSERTM(mat_cov_out.cols() >= n, "mat_cov_out.cols() = {}, not enough for {} test queries.", mat_cov_out.cols(), n);
            // each column of mat_cov_out is the lower triangular part of the covariance matrix of the corresponding test query
            for (long i = 0; i < n; ++i) {
                mat_var_out(0, i) = m_setting_->kernel->alpha - ktest.col(i).squaredNorm();  // var(h(x))
                long index = 0;
                for (long j = 1, jj = i + n; j <= dim; ++j, jj += n) {
                    const auto &col_jj = ktest.col(jj);
                    mat_cov_out(index++, i) = -col_jj.dot(ktest.col(i));                                                         // cov(dh(x)/dx_j, h(x))
                    for (long k = 1, kk = i + n; k < j; ++k, kk += n) { mat_cov_out(index++, i) = -col_jj.dot(ktest.col(kk)); }  // cov(dh(x)/dx_j, dh(x)/dx_k)
                    mat_var_out(j, i) = m_three_over_scale_square_ - col_jj.squaredNorm();                                       // var(dh(x)/dx_j)
                }
            }
        }
    }
}  // namespace erl::gaussian_process
