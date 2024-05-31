#pragma once

#include "covariance.hpp"

namespace erl::covariance {

    template<long Dim>
    class OrnsteinUhlenbeck : public Covariance {
        // ref1: https://en.wikipedia.org/wiki/Ornstein%E2%80%93Uhlenbeck_process
        // ref2: https://www.cs.cmu.edu/~epxing/Class/10708-15/notes/10708_scribe_lecture21.pdf

    public:
        [[nodiscard]] std::shared_ptr<Covariance>
        Create() const override {
            return std::make_shared<OrnsteinUhlenbeck>(std::make_shared<Setting>());
        }

        explicit OrnsteinUhlenbeck(std::shared_ptr<Setting> setting)
            : Covariance(std::move(setting)) {
            if (Dim != Eigen::Dynamic) { m_setting_->x_dim = Dim; }  // set x_dim
        }

        [[nodiscard]] std::string
        GetCovarianceType() const override {
            if (Dim == Eigen::Dynamic) { return "OrnsteinUhlenbeckXd"; }
            return "OrnsteinUhlenbeck" + std::to_string(Dim) + "D";
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
            const double a = -1. / m_setting_->scale;
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
                        k_mat(i, j) = m_setting_->alpha * std::exp(a * r);
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
            ERL_DEBUG_ASSERT(n == vec_var_y.size(), "#elements of vec_sigma_y does not equal to #columns of mat_x.");
            long dim;
            if constexpr (Dim == Eigen::Dynamic) {
                dim = mat_x.rows();
            } else {
                dim = Dim;
            }
            const double a = -1. / m_setting_->scale;
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
                        k_mat(i, j) = m_setting_->alpha * std::exp(a * r);
                        k_mat(j, i) = k_mat(i, j);
                    }
                }
            }
            return {n, n};
        }

        [[nodiscard]] std::pair<long, long>
        ComputeKtest(Eigen::Ref<Eigen::MatrixXd> k_mat, const Eigen::Ref<const Eigen::MatrixXd> &mat_x1, const Eigen::Ref<const Eigen::MatrixXd> &mat_x2)
            const final {
            ERL_DEBUG_ASSERT(mat_x1.rows() == mat_x2.rows(), "Sample vectors stored in x_1 and x_2 should have the same dimension.");

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
            const double a = -1. / m_setting_->scale;
            for (long i = 0; i < n; ++i) {
                for (long j = 0; j < m; ++j) {
                    double r = 0.0;
                    for (long k = 0; k < dim; ++k) {
                        const double dx = mat_x1(k, i) - mat_x2(k, j);
                        r += dx * dx;
                    }
                    r = std::sqrt(r);  // (mat_x1.col(i) - mat_x2.col(j)).norm();
                    k_mat(i, j) = m_setting_->alpha * std::exp(a * r);
                }
            }
            return {n, m};
        }

        [[nodiscard]] std::pair<long, long>
        ComputeKtrainWithGradient(
            Eigen::Ref<Eigen::MatrixXd>,                // k_mat
            const Eigen::Ref<const Eigen::MatrixXd> &,  // mat_x
            const Eigen::Ref<const Eigen::VectorXb> &   // vec_grad_flags
        ) const final {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }

        [[nodiscard]] std::pair<long, long>
        ComputeKtrainWithGradient(
            Eigen::Ref<Eigen::MatrixXd>,                // k_mat
            const Eigen::Ref<const Eigen::MatrixXd> &,  // mat_x
            const Eigen::Ref<const Eigen::VectorXb> &,  // vec_grad_flags
            const Eigen::Ref<const Eigen::VectorXd> &,  // vec_var_x
            const Eigen::Ref<const Eigen::VectorXd> &,  // vec_var_y
            const Eigen::Ref<const Eigen::VectorXd> &   // vec_var_grad
        ) const final {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }

        [[nodiscard]] std::pair<long, long>
        ComputeKtestWithGradient(
            Eigen::Ref<Eigen::MatrixXd>,                // k_mat
            const Eigen::Ref<const Eigen::MatrixXd> &,  // mat_x1
            const Eigen::Ref<const Eigen::VectorXb> &,  // vec_grad1_flags
            const Eigen::Ref<const Eigen::MatrixXd> &   // mat_x2
        ) const final {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }
    };

    using OrnsteinUhlenbeck1D = OrnsteinUhlenbeck<1>;
    using OrnsteinUhlenbeck2D = OrnsteinUhlenbeck<2>;
    using OrnsteinUhlenbeck3D = OrnsteinUhlenbeck<3>;
    using OrnsteinUhlenbeckXd = OrnsteinUhlenbeck<Eigen::Dynamic>;

    ERL_REGISTER_COVARIANCE(OrnsteinUhlenbeck1D);
    ERL_REGISTER_COVARIANCE(OrnsteinUhlenbeck2D);
    ERL_REGISTER_COVARIANCE(OrnsteinUhlenbeck3D);
    ERL_REGISTER_COVARIANCE(OrnsteinUhlenbeckXd);
}  // namespace erl::covariance
