


#include <erl_utilities/trajectories/polynomial.h>
#include <erl_utilities/trajectories/piecewise_polynomial.h>
#include <erl_utilities/trajectories/random_piecewise_polynomial.h>

#include <Eigen/Core>
#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>


namespace {

    template<typename CoefficientType>
    void testValueOutsideOfRange() {
        typedef trajectories::PiecewisePolynomial<CoefficientType> PiecewisePolynomialType;

        std::default_random_engine generator;
        std::vector<double> segment_times = trajectories::PiecewiseTrajectory<double>::RandomSegmentTimes(6, generator);
        PiecewisePolynomialType
                piecewise = trajectories::MakeRandomPiecewisePolynomial<CoefficientType>(3, 4, 5, segment_times);

        std::cout << "Start value" << std::endl;
        std::cout << piecewise.value(piecewise.start_time()) << std::endl;
        std::cout << "Start-1.0 value" << std::endl;
        std::cout << piecewise.value(piecewise.start_time() - 1.0) << std::endl;
        std::cout << "End value" << std::endl;
        std::cout << piecewise.value(piecewise.end_time()) << std::endl;
        std::cout << "End+1.0 value" << std::endl;
        std::cout << piecewise.value(piecewise.end_time() + 1.0) << std::endl;

        /*
        std::cout << CompareMatrices(piecewise.value(piecewise.start_time()),
                                    piecewise.value(piecewise.start_time() - 1.0),
                                    1e-10, drake::MatrixCompareType::absolute) << std::endl;

        std::cout << CompareMatrices(piecewise.value(piecewise.end_time()),
                                    piecewise.value(piecewise.end_time() + 1.0),
                                    1e-10, drake::MatrixCompareType::absolute) << std::endl;
        */
        EXPECT_TRUE(piecewise.value(piecewise.start_time()).isApprox(piecewise.value(piecewise.start_time() - 1.0)));
        EXPECT_TRUE(piecewise.value(piecewise.end_time()).isApprox(piecewise.value(piecewise.end_time() + 1.0)));
    }

    TEST(TestPolynomial, testValueOutsideOfRange) {
        //testIntegralAndDerivative<double>();
        //testBasicFunctionality<double>();
        testValueOutsideOfRange<double>();
    }


    TEST(TestPolynomial, TestPeriodicSpline) {
        Eigen::VectorXd breaks(5);
        breaks << 0, 1, 2, 3, 4;

        // Spline in 3d.
        Eigen::MatrixXd knots(3, 5);
        knots << 1, 1, 1,
                2, 2, 2,
                0, 3, 3,
                -2, 2, 2,
                1, 1, 1;
        const bool periodic_endpoint = true;

        trajectories::PiecewisePolynomial<double> periodic_spline =
                trajectories::PiecewisePolynomial<double>::Cubic(breaks, knots, periodic_endpoint);

        std::unique_ptr<trajectories::Trajectory<double>> spline_dt =
                periodic_spline.MakeDerivative(1);
        std::unique_ptr<trajectories::Trajectory<double>> spline_ddt =
                periodic_spline.MakeDerivative(2);

        Eigen::VectorXd begin_dt = spline_dt->value(breaks(0));
        Eigen::VectorXd end_dt = spline_dt->value(breaks(breaks.size() - 1));

        Eigen::VectorXd begin_ddt = spline_ddt->value(breaks(0));
        Eigen::VectorXd end_ddt = spline_ddt->value(breaks(breaks.size() - 1));

        Eigen::VectorXd dt_diff = end_dt - begin_dt;
        Eigen::VectorXd ddt_diff = end_ddt - begin_ddt;
        //std::cout << dt_diff.template lpNorm<Eigen::Infinity>() << std::endl;
        //std::cout << ddt_diff.template lpNorm<Eigen::Infinity>() << std::endl;
        EXPECT_TRUE(dt_diff.template lpNorm<Eigen::Infinity>() < 1e-14);
        EXPECT_TRUE(ddt_diff.template lpNorm<Eigen::Infinity>() < 1e-14);
    }


    TEST(TestPolynomial, TestFunctionDerivative) {

        auto t = trajectories::Polynomial<double>("t", 1);
        trajectories::Polynomial<double> poly = 3 * t * t - 2 * t  + 1;
        EXPECT_THAT(poly.GetDegree(), 2);
        trajectories::Polynomial<double> poly_der = poly.Derivative(1);
        EXPECT_THAT(poly_der.GetDegree(), 1);

        trajectories::PiecewisePolynomial<double> pw_poly = trajectories::PiecewisePolynomial<double>({poly}, {0, 1});
        trajectories::PiecewisePolynomial<double> pw_poly_der = pw_poly.derivative(1);
    }


    TEST(TestPolynomial, TestMakePolynomial) {
        int num_coefficients = 7; // 6th order
        int rows = 3; // 3rd dimension
        int cols = 1;

        std::vector<double> segment_times{0, 1, 2, 3, 4};
        trajectories::PiecewisePolynomial<double>
                pp = trajectories::MakeRandomPiecewisePolynomial<double>(rows, cols, num_coefficients, segment_times);

        std::cout << pp.value(0.0) << std::endl;
        std::cout << pp.getSegmentPolynomialDegree(0) << std::endl;
        std::cout << pp.getSegmentPolynomialDegree(1) << std::endl;
        std::cout << pp.value(pp.start_time()) << std::endl;
        std::cout << pp.value(pp.end_time()) << std::endl;

        Eigen::Matrix<double, Eigen::Dynamic, 1> coeff(num_coefficients);
        coeff << 0, 3, 3, 0, 0, 0, 0;
        trajectories::Polynomial<double> p1(coeff);

        Eigen::Matrix<trajectories::Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic> pmat(rows, cols);
        pmat(0, 0) = p1;
    }


    TEST(TestPolynomial, TestMakePolynomialCoefficients) {
        int num_segments = 5; // 5 segments
        const int num_coefficients = 7; // 6th order
        const int dimension = 3; // 3 Output dimensions (X, Y, Z)
        std::vector<double> segment_times{0, 1, 2, 3, 4, 5};
        trajectories::PiecewisePolynomial<double> pp = trajectories::PiecewisePolynomial<double>();


        // Compute the polynomial coefficients
        typedef Eigen::Matrix<trajectories::Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic> PolynomialVector;
        typedef std::vector<PolynomialVector> vectorOfPolynomialVectors;
        vectorOfPolynomialVectors polynomials(num_segments, PolynomialVector(num_coefficients, dimension));
        Eigen::Matrix<double, num_coefficients, 1> coeff;

        for (int k = 0; k < num_segments; ++k) {
            for (int d = 0; d < num_coefficients; ++d) {
                for (int n = 0; n < dimension; ++n)
                    coeff[n] = 0;
                coeff[dimension] = 0;
                polynomials[k](d) = trajectories::Polynomial<double>(coeff);
            }
        }
        auto traj = trajectories::PiecewisePolynomial<double>(polynomials, segment_times);
    }

}

int main(int argc, char **argv) {
    try {
        ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    } catch (std::exception &e) {
        std::cerr << "Unhandled Exception: " << e.what() << std::endl;
    }
    return 1;
}



