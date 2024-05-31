#include "erl_sdf_mapping/log_sdf_gp.hpp"

namespace erl::sdf_mapping {

    void
    LogSdfGaussianProcess::Train(const long num_train_samples) {

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
        const auto [ktrain_rows, ktrain_cols] = NoisyInputGaussianProcess::m_kernel_->ComputeKtrainWithGradient(  // gpis
            m_mat_k_train_,                                                                                       // buffer of mat_ktrain
            m_mat_x_train_.topLeftCorner(m_x_dim_, m_num_train_samples_),                                         // mat_x_train
            m_vec_grad_flag_.head(m_num_train_samples_),                                                          // vec_grad_flag
            m_vec_var_x_.head(m_num_train_samples_),                                                              // vec_var_x
            m_vec_var_h_.head(m_num_train_samples_),                                                              // vec_var_h
            m_vec_var_grad_.head(m_num_train_samples_));                                                          // vec_var_grad
        const auto mat_ktrain = m_mat_k_train_.topLeftCorner(ktrain_rows, ktrain_cols);                           // square matrix
        const auto [log_ktrain_rows, log_ktrain_cols] = m_kernel_->ComputeKtrain(                                 // log-gpis
            m_mat_log_k_train_,                                                                                   // buffer of mat_log_ktrain
            m_mat_x_train_.topLeftCorner(m_x_dim_, m_num_train_samples_),                                         // mat_x_train
            m_vec_var_h_.head(m_num_train_samples_));                                                             // vec_var_h
        const auto mat_log_ktrain = m_mat_log_k_train_.topLeftCorner(log_ktrain_rows, log_ktrain_cols);           // square matrix

        // Compute log-sdf mapping
        const auto vec_alpha = m_vec_alpha_.head(ktrain_rows);        // h and gradient of h
        auto vec_log_alpha = m_vec_log_alpha_.head(log_ktrain_rows);  // log mapping of h
        // vec_log_alpha[i] = std::exp(-m_setting_->log_lambda * vec_alpha[i]), but vec_alpha[i] may be nonzero
        for (long i = 0; i < m_num_train_samples_; ++i) { vec_log_alpha[i] = 1.0; }

        // Compute cholesky decomposition and alpha
        auto &&mat_l = m_mat_l_.topLeftCorner(ktrain_rows, ktrain_cols);                   // square matrix, lower triangular
        mat_l = mat_ktrain.llt().matrixL();                                                // gpis, Ktrain = L * L^T
        mat_l.triangularView<Eigen::Lower>().solveInPlace(vec_alpha);                      // Ktrain^-1 = L^-T * L^-1
        mat_l.transpose().triangularView<Eigen::Upper>().solveInPlace(vec_alpha);          // alpha = Ktrain^-1 * [h, dh/dx_1, ..., dh/dx_dim]
        auto &&mat_log_l = m_mat_log_l_.topLeftCorner(log_ktrain_rows, log_ktrain_cols);   // square matrix, lower triangular
        mat_log_l = mat_log_ktrain.llt().matrixL();                                        // log-gpis, logKtrain = logL * logL^T
        mat_log_l.triangularView<Eigen::Lower>().solveInPlace(vec_log_alpha);              // logKtrain^-1 = logL^-T * logL^-1
        mat_log_l.transpose().triangularView<Eigen::Upper>().solveInPlace(vec_log_alpha);  // log_alpha = logKtrain^-1 * log(-lambda * h)

        m_trained_ = true;
    }

    void
    LogSdfGaussianProcess::Test(
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
        ERL_ASSERTM(mat_f_out.cols() >= n, "mat_f_out.cols() = {}, it should be >= n = {}.", mat_f_out.cols(), n);
        double f_threshold = std::exp(-m_setting_->log_lambda * std::abs(m_setting_->edf_threshold));

        const auto [ktest_min_rows, ktest_min_cols] = covariance::Covariance::GetMinimumKtestSize(m_num_train_samples_, m_num_train_samples_with_grad_, dim, n);
        Eigen::MatrixXd ktest(ktest_min_rows, ktest_min_cols);  // (dim of train samples, dim of test queries)
        auto mat_x_train = m_mat_x_train_.topLeftCorner(m_x_dim_, m_num_train_samples_);
        auto vec_grad_flag1 = m_vec_grad_flag_.head(m_num_train_samples_);
        const auto [ktest_rows, ktest_cols] = NoisyInputGaussianProcess::m_kernel_->ComputeKtestWithGradient(ktest, mat_x_train, vec_grad_flag1, mat_x_test);
        ERL_DEBUG_ASSERT(
            (ktest_rows == ktest_min_rows) && (ktest_cols == ktest_min_cols),
            "output_size = ({}, {}), it should be ({}, {}).",
            ktest_rows,
            ktest_cols,
            ktest_min_rows,
            ktest_min_cols);

        const auto [log_ktest_min_rows, log_ktest_min_cols] = covariance::Covariance::GetMinimumKtestSize(m_num_train_samples_, 0, dim, n);
        Eigen::MatrixXd log_ktest(log_ktest_min_rows, log_ktest_min_cols);  // (dim of train samples, dim of test queries)
        Eigen::VectorXb vec_grad_flag2 = Eigen::VectorXb::Constant(m_num_train_samples_, false);
        const auto [log_ktest_rows, log_ktest_cols] = m_kernel_->ComputeKtestWithGradient(log_ktest, mat_x_train, vec_grad_flag2, mat_x_test);
        ERL_DEBUG_ASSERT(
            (log_ktest_rows == log_ktest_min_rows) && (log_ktest_cols == log_ktest_min_cols),
            "output_size = ({}, {}), it should be ({}, {}).",
            log_ktest_rows,
            log_ktest_cols,
            log_ktest_min_rows,
            log_ktest_min_cols);

        // output sdf and gradient
        auto vec_alpha = m_vec_alpha_.head(ktest_rows);
        auto vec_log_alpha = m_vec_log_alpha_.head(log_ktest_rows);
        for (long i = 0; i < n; ++i) {
            double f_gpis = ktest.col(i).dot(vec_alpha);
            double sign = f_gpis >= 0. ? 1. : -1.;  // sdf sign
            double f_log_gpis = log_ktest.col(i).dot(vec_log_alpha);

            mat_f_out(0, i) = std::log(std::abs(f_log_gpis)) * sign / -m_setting_->log_lambda;  // sdf magnitude
            double norm = 0;                                                                    // gradient norm
            if (f_log_gpis > f_threshold) {                                                     // close to the surface: use gpis
                for (long j = 1, jj = i + n; j <= dim; ++j, jj += n) {                          // gradient
                    double &grad = mat_f_out(j, i);
                    grad = ktest.col(jj).dot(vec_alpha);
                    norm += grad * grad;
                }
            } else {                                                       // use log-gpis
                double d = -sign / (m_setting_->log_lambda * f_log_gpis);  // d = -ln(f)/lambda, grad_d = -1/(lambda*f)*grad_f
                for (long j = 1, jj = i + n; j <= dim; ++j, jj += n) {     // gradient
                    double &grad = mat_f_out(j, i);
                    grad = log_ktest.col(jj).dot(vec_log_alpha) * d;
                    norm += grad * grad;
                }
            }
            norm = std::sqrt(norm);
            if (norm > 1.e-15) {                                              // avoid zero division
                for (long j = 1; j <= dim; ++j) { mat_f_out(j, i) /= norm; }  // gradient norm is always 1. https://en.wikipedia.org/wiki/Eikonal_equation
            }
        }
        if (mat_var_out.size() == 0 && mat_cov_out.size() == 0) { return; }

        // compute (co)variance of the test queries: use gpis, log-gpis has numerical issue!!!
        // solve Lx = ktest -> x = m_mat_l_.m_inv_() * ktest
        m_mat_l_.topLeftCorner(ktest_rows, ktest_rows).triangularView<Eigen::Lower>().solveInPlace(ktest);
        bool compute_var = mat_var_out.size() > 0;
        if (compute_var) {
            ERL_ASSERTM(mat_var_out.rows() >= dim + 1, "mat_var_out.rows() = {}, it should be >= Dim + 1 = {} for variance.", mat_var_out.rows(), dim + 1);
            ERL_ASSERTM(mat_var_out.cols() >= n, "mat_var_out.cols() = {}, not enough for {} test queries.", mat_var_out.cols(), n);
        }
        if (mat_cov_out.size() == 0 && compute_var) {  // compute variance
            // column-wise square sum of ktest = var([h(x1),...,h(xn),dh(x1)/dx_1,...,dh(xn)/dx_1,...,dh(x1)/dx_dim,...,dh(xn)/dx_dim])
            for (long i = 0; i < n; ++i) {
                mat_var_out(0, i) = m_setting_->kernel->alpha - ktest.col(i).squaredNorm();  // variance of h(x)
                for (long j = 1, jj = i + n; j <= dim; ++j, jj += n) {                       // variance of dh(x)/dx_j
                    mat_var_out(j, i) = m_three_over_scale_square_ - ktest.col(jj).squaredNorm();
                }
            }
        } else {
            long min_n_rows = (dim + 1) * dim / 2;
            ERL_ASSERTM(mat_cov_out.rows() >= min_n_rows, "mat_cov_out.rows() = {}, it should be >= {} for covariance.", mat_cov_out.rows(), min_n_rows);
            ERL_ASSERTM(mat_cov_out.cols() >= n, "mat_cov_out.cols() = {}, not enough for {} test queries.", mat_cov_out.cols(), n);
            // each column of mat_cov_out is the lower triangular part of the covariance matrix of the corresponding test query
            for (long i = 0; i < n; ++i) {
                if (compute_var) { mat_var_out(0, i) = m_setting_->kernel->alpha - ktest.col(i).squaredNorm(); }  // var(h(x))
                long index = 0;
                for (long j = 1, jj = i + n; j <= dim; ++j, jj += n) {
                    const auto &col_jj = ktest.col(jj);
                    mat_cov_out(index++, i) = -col_jj.dot(ktest.col(i));                                                         // cov(dh(x)/dx_j, h(x))
                    for (long k = 1, kk = i + n; k < j; ++k, kk += n) { mat_cov_out(index++, i) = -col_jj.dot(ktest.col(kk)); }  // cov(dh(x)/dx_j, dh(x)/dx_k)
                    if (compute_var) { mat_var_out(j, i) = m_three_over_scale_square_ - col_jj.squaredNorm(); }                  // var(dh(x)/dx_j)
                }
            }
        }
    }
}  // namespace erl::sdf_mapping
