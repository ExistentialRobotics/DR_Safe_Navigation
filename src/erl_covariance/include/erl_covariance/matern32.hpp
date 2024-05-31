#pragma once

#include "covariance.hpp"

namespace erl::covariance {

    static double
    InlineMatern32(const double alpha, const double a1, const double a2, const double r) {
        return (alpha + a1 * r) * std::exp(-a2 * r);
    }

    // cov(f1, df2/dx2) = d k(x1, x2) / dx2
    // note: dx = x1 - x2
    static double
    InlineMatern32X1BetweenGradx2(const double a, const double b, const double dx, const double r) {
        return b * dx * std::exp(-a * r);
    }

    // d^2 k(x1, x2) / dx_1 dx_2
    static double
    InlineMatern32Gradx1BetweenGradx2(const double a, const double b, const double delta, const double dx_1, const double dx_2, const double r) {
        if (std::abs(dx_1 * dx_2) < 1.e-6 && std::abs(r) < 1.e-6) { return b; }
        return b * (delta - a * dx_1 * dx_2 / r) * std::exp(-a * r);
    }

    template<long Dim>
    class Matern32 : public Covariance {

    public:
        [[nodiscard]] std::shared_ptr<Covariance>
        Create() const override {
            return std::make_shared<Matern32>(std::make_shared<Setting>());
        }

        explicit Matern32(std::shared_ptr<Setting> setting)
            : Covariance(std::move(setting)) {
            if (Dim != Eigen::Dynamic) { m_setting_->x_dim = Dim; }  // set x_dim
        }

        [[nodiscard]] std::string
        GetCovarianceType() const override {
            if (Dim == Eigen::Dynamic) { return "Matern32_Xd"; }
            return "Matern32_" + std::to_string(Dim) + "D";
        }

        [[nodiscard]] std::pair<long, long>
        ComputeKtrain(Eigen::Ref<Eigen::MatrixXd> k_mat, const Eigen::Ref<const Eigen::MatrixXd> &mat_x) const final {
            long n = mat_x.cols();
            ERL_DEBUG_ASSERT(k_mat.rows() >= n, "k_mat.rows() = {}, it should be >= {}.", k_mat.rows(), n);
            ERL_DEBUG_ASSERT(k_mat.cols() >= n, "k_mat.cols() = {}, it should be >= {}.", k_mat.cols(), n);
            long dim;
            if constexpr (Dim == Eigen::Dynamic) {
                dim = mat_x.rows();
            } else {
                dim = Dim;
            }
            const double a2 = std::sqrt(3.) / m_setting_->scale;
            const double a1 = a2 * m_setting_->alpha;
            for (long i = 0; i < n; ++i) {
                for (long j = i; j < n; ++j) {
                    if (i == j) {
                        k_mat(i, i) = m_setting_->alpha;
                    } else {
                        double r = 0.0;
                        for (long k = 0; k < dim; ++k) {
                            const double dx = mat_x(k, i) - mat_x(k, j);
                            r += dx * dx;
                        }
                        r = std::sqrt(r);  // (mat_x.col(i) - mat_x.col(j)).norm();
                        k_mat(i, j) = InlineMatern32(m_setting_->alpha, a1, a2, r);
                        k_mat(j, i) = k_mat(i, j);
                    }
                }
            }
            return {n, n};
        }

        [[nodiscard]] std::pair<long, long>
        ComputeKtrain(Eigen::Ref<Eigen::MatrixXd> k_mat, const Eigen::Ref<const Eigen::MatrixXd> &mat_x, const Eigen::Ref<const Eigen::VectorXd> &vec_var_y)
            const final {
            long n = mat_x.cols();
            ERL_DEBUG_ASSERT(k_mat.rows() >= n, "k_mat.rows() = {}, it should be >= {}.", k_mat.rows(), n);
            ERL_DEBUG_ASSERT(k_mat.cols() >= n, "k_mat.cols() = {}, it should be >= {}.", k_mat.cols(), n);
            long dim;
            if constexpr (Dim == Eigen::Dynamic) {
                dim = mat_x.rows();
            } else {
                dim = Dim;
            }
            const double a2 = std::sqrt(3.) / m_setting_->scale;
            const double a1 = a2 * m_setting_->alpha;
            for (long i = 0; i < n; ++i) {
                for (long j = i; j < n; ++j) {
                    if (i == j) {
                        k_mat(i, i) = m_setting_->alpha + vec_var_y[i];
                    } else {
                        double r = 0.0;
                        for (long k = 0; k < dim; ++k) {
                            const double dx = mat_x(k, i) - mat_x(k, j);
                            r += dx * dx;
                        }
                        r = std::sqrt(r);  // (mat_x.col(i) - mat_x.col(j)).norm();
                        k_mat(i, j) = InlineMatern32(m_setting_->alpha, a1, a2, r);
                        k_mat(j, i) = k_mat(i, j);
                    }
                }
            }
            return {n, n};
        }

        [[nodiscard]] std::pair<long, long>
        ComputeKtest(Eigen::Ref<Eigen::MatrixXd> k_mat, const Eigen::Ref<const Eigen::MatrixXd> &mat_x1, const Eigen::Ref<const Eigen::MatrixXd> &mat_x2)
            const final {
            ERL_DEBUG_ASSERT(mat_x1.rows() == mat_x2.rows(), "Sample vectors stored in x1 and x_2 should have the same dimension.");
            long n = mat_x1.cols();
            long m = mat_x2.cols();
            ERL_DEBUG_ASSERT(k_mat.rows() >= n, "k_mat.rows() = {}, it should be >= {}.", k_mat.rows(), n);
            ERL_DEBUG_ASSERT(k_mat.cols() >= m, "k_mat.cols() = {}, it should be >= {}.", k_mat.cols(), m);
            long dim;
            if constexpr (Dim == Eigen::Dynamic) {
                dim = mat_x1.rows();
            } else {
                dim = Dim;
            }
            const double a2 = std::sqrt(3.) / m_setting_->scale;
            const double a1 = a2 * m_setting_->alpha;
            for (long i = 0; i < n; ++i) {
                for (long j = 0; j < m; ++j) {
                    double r = 0;
                    for (long k = 0; k < dim; ++k) {
                        const double dx = mat_x1(k, i) - mat_x2(k, j);
                        r += dx * dx;
                    }
                    r = std::sqrt(r);  // (mat_x1.col(i) - mat_x2.col(j)).norm();
                    k_mat(i, j) = InlineMatern32(m_setting_->alpha, a1, a2, r);
                }
            }
            return {n, m};
        }

        [[nodiscard]] std::pair<long, long>
        ComputeKtrainWithGradient(
            Eigen::Ref<Eigen::MatrixXd> k_mat,
            const Eigen::Ref<const Eigen::MatrixXd> &mat_x,
            const Eigen::Ref<const Eigen::VectorXb> &vec_grad_flags) const final {

            long dim;
            if constexpr (Dim == Eigen::Dynamic) {
                dim = mat_x.rows();
            } else {
                dim = Dim;
            }

            ERL_DEBUG_ASSERT(mat_x.rows() == dim, "Each column of mat_x should be {}-D vector.", dim);
            const long n = mat_x.cols();
            std::vector<long> grad_indices;
            grad_indices.reserve(vec_grad_flags.size());
            long n_grad = 0;
            for (const bool &flag: vec_grad_flags) {
                if (flag) {
                    grad_indices.push_back(n + n_grad++);
                } else {
                    grad_indices.push_back(-1);
                }
            }
            long n_rows = n + n_grad * dim;
            long n_cols = n_rows;
            ERL_DEBUG_ASSERT(k_mat.rows() >= n_rows, "k_mat.rows() = {}, it should be >= {}.", k_mat.rows(), n_rows);
            ERL_DEBUG_ASSERT(k_mat.cols() >= n_cols, "k_mat.cols() = {}, it should be >= {}.", k_mat.cols(), n_cols);

            const double a2 = std::sqrt(3.) / m_setting_->scale;
            const double a1 = a2 * m_setting_->alpha;
            const double b = a2 * a2 * m_setting_->alpha;
            for (long i = 0; i < n; ++i) {
                k_mat(i, i) = m_setting_->alpha;  // cov(f_i, f_i)
                if (vec_grad_flags[i]) {
                    for (long k = 0, ki = grad_indices[i]; k < dim; ++k, ki += n_grad) {
                        k_mat(ki, ki) = b;  // cov(df_i/dx_k, df_i/dx_k)
                        k_mat(i, ki) = 0.;  // cov(f_i, df_i/dx_k)
                        k_mat(ki, i) = 0.;  // cov(df_i/dx_k, f_i)
                        for (long l = k + 1, li = ki + n_grad; l < dim; ++l, li += n_grad) {
                            k_mat(ki, li) = 0.;  // cov(df_i/dx_k, df_i/dx_l)
                            k_mat(li, ki) = 0.;  // cov(df_i/dx_l, df_i/dx_k)
                        }
                    }
                }

                for (long j = i + 1; j < n; ++j) {
                    double r = 0;
                    for (long k = 0; k < dim; ++k) {
                        const double dx = mat_x(k, i) - mat_x(k, j);
                        r += dx * dx;
                    }
                    r = std::sqrt(r);                                            // (mat_x.col(i) - mat_x.col(j)).norm();
                    k_mat(i, j) = InlineMatern32(m_setting_->alpha, a1, a2, r);  // cov(f_i, f_j)
                    k_mat(j, i) = k_mat(i, j);                                   // cov(f_j, f_i)

                    if (vec_grad_flags[i]) {
                        // cov(f_j, df_i) = cov(df_i, f_j)
                        for (long k = 0, ki = grad_indices[i]; k < dim; ++k, ki += n_grad) {
                            k_mat(j, ki) = InlineMatern32X1BetweenGradx2(a2, b, mat_x(k, j) - mat_x(k, i), r);  // cov(f_j, df_i/dx_k)
                            k_mat(ki, j) = k_mat(j, ki);                                                        // cov(df_i/dx_k, f_j)
                        }

                        if (vec_grad_flags[j]) {
                            for (long k = 0, ki = grad_indices[i], kj = grad_indices[j]; k < dim; ++k, ki += n_grad, kj += n_grad) {
                                k_mat(i, kj) = -k_mat(j, ki);  // cov(f_i, df_j) = -cov(df_i, f_j)
                                k_mat(kj, i) = k_mat(i, kj);   // cov(df_j, f_i) = -cov(f_j, df_i) = cov(f_i, df_j)
                            }

                            // cov(df_i, df_j) = cov(df_j, df_i)
                            for (long k = 0, ki = grad_indices[i], kj = grad_indices[j]; k < dim; ++k, ki += n_grad, kj += n_grad) {
                                // between Dim-k and Dim-k
                                const double dxk = mat_x(k, i) - mat_x(k, j);
                                k_mat(ki, kj) = InlineMatern32Gradx1BetweenGradx2(a2, b, 1., dxk, dxk, r);  // cov(df_i/dx_k, df_j/dx_k)
                                k_mat(kj, ki) = k_mat(ki, kj);                                              // cov(df_j/dx_k, df_i/dx_k)
                                for (long l = k + 1, li = ki + n_grad, lj = kj + n_grad; l < dim; ++l, li += n_grad, lj += n_grad) {
                                    // between Dim-k and Dim-l
                                    const double dxl = mat_x(l, i) - mat_x(l, j);
                                    k_mat(ki, lj) = InlineMatern32Gradx1BetweenGradx2(a2, b, 0., dxk, dxl, r);  // cov(df_i/dx_k, df_j/dx_l)
                                    k_mat(li, kj) = k_mat(ki, lj);                                              // cov(df_i/dx_l, df_j/dx_k)
                                    k_mat(lj, ki) = k_mat(ki, lj);                                              // cov(df_j/dx_l, df_i/dx_k)
                                    k_mat(kj, li) = k_mat(lj, ki);                                              // cov(df_j/dx_k, df_i/dx_l)
                                }
                            }
                        }
                    } else if (vec_grad_flags[j]) {
                        // cov(f_i, df_j) = cov(df_j, f_i)
                        for (long k = 0, kj = grad_indices[j]; k < Dim; ++k, kj += n_grad) {
                            k_mat(i, kj) = InlineMatern32X1BetweenGradx2(a2, b, mat_x(k, i) - mat_x(k, j), r);  // cov(f_i, df_j)
                            k_mat(kj, i) = k_mat(i, kj);
                        }
                    }
                }  // for (long j = i + 1; j < n; ++j)
            }      // for (long i = 0; i < n; ++i)
            return {n_rows, n_cols};
        }

        [[nodiscard]] std::pair<long, long>
        ComputeKtrainWithGradient(
            Eigen::Ref<Eigen::MatrixXd> k_mat,
            const Eigen::Ref<const Eigen::MatrixXd> &mat_x,
            const Eigen::Ref<const Eigen::VectorXb> &vec_grad_flags,
            const Eigen::Ref<const Eigen::VectorXd> &vec_var_x,
            const Eigen::Ref<const Eigen::VectorXd> &vec_var_y,
            const Eigen::Ref<const Eigen::VectorXd> &vec_var_grad) const final {

            long dim;
            if constexpr (Dim == Eigen::Dynamic) {
                dim = mat_x.rows();
            } else {
                dim = Dim;
            }

            ERL_DEBUG_ASSERT(mat_x.rows() == dim, "Each column of mat_x should be {}-D vector.", dim);
            const long n = mat_x.cols();
            std::vector<long> grad_indices;
            grad_indices.reserve(vec_grad_flags.size());
            long n_grad = 0;
            for (const bool &flag: vec_grad_flags) {
                if (flag) {
                    grad_indices.push_back(n + n_grad++);
                } else {
                    grad_indices.push_back(-1);
                }
            }
            long n_rows = n + n_grad * dim;
            long n_cols = n_rows;
            ERL_DEBUG_ASSERT(k_mat.rows() >= n_rows, "k_mat.rows() = {}, it should be >= {}.", k_mat.rows(), n_rows);
            ERL_DEBUG_ASSERT(k_mat.cols() >= n_cols, "k_mat.cols() = {}, it should be >= {}.", k_mat.cols(), n_cols);

            const double a2 = std::sqrt(3.) / m_setting_->scale;
            const double a1 = a2 * m_setting_->alpha;
            const double b = a2 * a2 * m_setting_->alpha;
            for (long i = 0; i < n; ++i) {
                k_mat(i, i) = m_setting_->alpha + vec_var_x[i] + vec_var_y[i];  // cov(f_i, f_i)
                if (vec_grad_flags[i]) {
                    for (long k = 0, ki = grad_indices[i]; k < dim; ++k, ki += n_grad) {
                        k_mat(ki, ki) = b + vec_var_grad[i];  // cov(df_i/dx_k, df_i/dx_k)
                        k_mat(i, ki) = 0.;                    // cov(f_i, df_i/dx_k)
                        k_mat(ki, i) = 0.;                    // cov(df_i/dx_k, f_i)
                        for (long l = k + 1, li = ki + n_grad; l < dim; ++l, li += n_grad) {
                            k_mat(ki, li) = 0.;  // cov(df_i/dx_k, df_i/dx_l)
                            k_mat(li, ki) = 0.;  // cov(df_i/dx_l, df_i/dx_k)
                        }
                    }
                }

                for (long j = i + 1; j < n; ++j) {
                    double r = 0;
                    for (long k = 0; k < dim; ++k) {
                        const double dx = mat_x(k, i) - mat_x(k, j);
                        r += dx * dx;
                    }
                    r = std::sqrt(r);                                            // (mat_x.col(i) - mat_x.col(j)).norm();
                    k_mat(i, j) = InlineMatern32(m_setting_->alpha, a1, a2, r);  // cov(f_i, f_j)
                    k_mat(j, i) = k_mat(i, j);                                   // cov(f_j, f_i)

                    if (vec_grad_flags[i]) {
                        // cov(f_j, df_i) = cov(df_i, f_j)
                        for (long k = 0, ki = grad_indices[i]; k < dim; ++k, ki += n_grad) {
                            k_mat(j, ki) = InlineMatern32X1BetweenGradx2(a2, b, mat_x(k, j) - mat_x(k, i), r);  // cov(f_j, df_i)
                            k_mat(ki, j) = k_mat(j, ki);                                                        // cov(df_i, f_j)
                        }

                        if (vec_grad_flags[j]) {
                            for (long k = 0, ki = grad_indices[i], kj = grad_indices[j]; k < dim; ++k, ki += n_grad, kj += n_grad) {
                                k_mat(i, kj) = -k_mat(j, ki);  // cov(f_i, df_j) = -cov(df_i, f_j)
                                k_mat(kj, i) = k_mat(i, kj);   // cov(df_j, f_i) = -cov(f_j, df_i) = cov(f_i, df_j)
                            }

                            // cov(df_i, df_j) = cov(df_j, df_i)
                            for (long k = 0, ki = grad_indices[i], kj = grad_indices[j]; k < dim; ++k, ki += n_grad, kj += n_grad) {
                                // between Dim-k and Dim-k
                                const double dxk = mat_x(k, i) - mat_x(k, j);
                                k_mat(ki, kj) = InlineMatern32Gradx1BetweenGradx2(a2, b, 1., dxk, dxk, r);  // cov(df_i, df_j)
                                k_mat(kj, ki) = k_mat(ki, kj);                                              // cov(df_j, df_i)
                                for (long l = k + 1, li = ki + n_grad, lj = kj + n_grad; l < dim; ++l, li += n_grad, lj += n_grad) {
                                    // between Dim-k and Dim-l
                                    const double dxl = mat_x(l, i) - mat_x(l, j);
                                    k_mat(ki, lj) = InlineMatern32Gradx1BetweenGradx2(a2, b, 0., dxk, dxl, r);
                                    k_mat(li, kj) = k_mat(ki, lj);
                                    k_mat(lj, ki) = k_mat(ki, lj);  // cov(df_j, df_i)
                                    k_mat(kj, li) = k_mat(lj, ki);
                                }
                            }
                        }
                    } else if (vec_grad_flags[j]) {
                        // cov(f_i, df_j) = cov(df_j, f_i)
                        for (long k = 0, kj = grad_indices[j]; k < dim; ++k, kj += n_grad) {
                            k_mat(i, kj) = InlineMatern32X1BetweenGradx2(a2, b, mat_x(k, i) - mat_x(k, j), r);  // cov(f_i, df_j)
                            k_mat(kj, i) = k_mat(i, kj);
                        }
                    }
                }  // for (long j = i + 1; j < n; ++j)
            }      // for (long i = 0; i < n; ++i)
            return {n_rows, n_cols};
        }

        [[nodiscard]] std::pair<long, long>
        ComputeKtestWithGradient(
            Eigen::Ref<Eigen::MatrixXd> k_mat,
            const Eigen::Ref<const Eigen::MatrixXd> &mat_x1,
            const Eigen::Ref<const Eigen::VectorXb> &vec_grad1_flags,
            const Eigen::Ref<const Eigen::MatrixXd> &mat_x2) const final {

            long dim;
            if constexpr (Dim == Eigen::Dynamic) {
                dim = mat_x1.rows();
            } else {
                dim = Dim;
            }

            ERL_DEBUG_ASSERT(mat_x1.rows() == dim, "Each column of mat_x1 should be {}-D vector.", dim);
            ERL_DEBUG_ASSERT(mat_x2.rows() == dim, "Each column of mat_x2 should be {}-D vector.", dim);
            const long n = mat_x1.cols();
            const long m = mat_x2.cols();
            std::vector<long> grad_indices;
            grad_indices.reserve(vec_grad1_flags.size());
            long n_grad = 0;
            for (const bool &flag: vec_grad1_flags) {
                if (flag) {
                    grad_indices.push_back(n + n_grad++);
                } else {
                    grad_indices.push_back(-1);
                }
            }
            long n_rows = n + n_grad * dim;
            long n_cols = m * (dim + 1);
            ERL_DEBUG_ASSERT(k_mat.rows() >= n_rows, "k_mat.rows() = {}, it should be >= {}.", k_mat.rows(), n_rows);
            ERL_DEBUG_ASSERT(k_mat.cols() >= n_cols, "k_mat.cols() = {}, it should be >= {}.", k_mat.cols(), n_cols);

            const double a2 = std::sqrt(3.) / m_setting_->scale;
            const double a1 = a2 * m_setting_->alpha;
            const double b = a2 * a2 * m_setting_->alpha;
            for (long i = 0; i < n; ++i) {
                for (long j = 0; j < m; ++j) {
                    double r = 0;
                    for (long k = 0; k < dim; ++k) {
                        const double dx = mat_x1(k, i) - mat_x2(k, j);
                        r += dx * dx;
                    }
                    r = std::sqrt(r);                                            // (mat_x1.col(i) - mat_x2.col(j)).norm();
                    k_mat(i, j) = InlineMatern32(m_setting_->alpha, a1, a2, r);  // cov(f1_i, f2_j)
                    for (long k = 0, kj = j + m; k < dim; ++k, kj += m) {        // cov(f1_i, df2_j/dx_k)
                        k_mat(i, kj) = InlineMatern32X1BetweenGradx2(a2, b, mat_x1(k, i) - mat_x2(k, j), r);
                    }

                    if (vec_grad1_flags[i]) {
                        for (long k = 0, ki = grad_indices[i], kj = j + m; k < dim; ++k, ki += n_grad, kj += m) {
                            k_mat(ki, j) = -k_mat(i, kj);  // cov(df1_i/dx_k, f2_j) = -cov(f1_i, df2_j/dx_k)

                            // between Dim-k and Dim-k
                            const double dxk = mat_x1(k, i) - mat_x2(k, j);
                            k_mat(ki, kj) = InlineMatern32Gradx1BetweenGradx2(a2, b, 1., dxk, dxk, r);  // cov(df1_i/dx_k, df2_j/dx_k)

                            for (long l = k + 1, li = ki + n_grad, lj = kj + m; l < dim; ++l, li += n_grad, lj += m) {
                                // between Dim-k and Dim-l
                                const double dxl = mat_x1(l, i) - mat_x2(l, j);
                                k_mat(ki, lj) = InlineMatern32Gradx1BetweenGradx2(a2, b, 0., dxk, dxl, r);  // cov(df1_i/dx_k, df2_j/dx_l)
                                k_mat(li, kj) = k_mat(ki, lj);                                              // cov(df1_i/dx_l, df2_j/dx_k)
                            }
                        }
                    }
                }
            }
            return {n_rows, n_cols};
        }
    };

    using Matern32_1D = Matern32<1>;
    using Matern32_2D = Matern32<2>;
    using Matern32_3D = Matern32<3>;
    using Matern32_Xd = Matern32<Eigen::Dynamic>;

    ERL_REGISTER_COVARIANCE(Matern32_1D);
    ERL_REGISTER_COVARIANCE(Matern32_2D);
    ERL_REGISTER_COVARIANCE(Matern32_3D);
    ERL_REGISTER_COVARIANCE(Matern32_Xd);

}  // namespace erl::covariance
