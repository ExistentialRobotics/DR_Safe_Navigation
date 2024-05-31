//
// Created by brent on 1/27/19.
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <erl_utilities/trajectories/piecewise_polynomial.h>
#include <erl_utilities/se3_pose.h>
#include <erl_utilities/erl_geom_utils.h>


namespace py = pybind11;

PYBIND11_MODULE(pyErlUtils, m) {
  typedef trajectories::Polynomial<double> PolynomialType;
  typedef Eigen::Matrix<PolynomialType, Eigen::Dynamic, Eigen::Dynamic> PolynomialMatrix;

  // Trajectories
  py::class_<trajectories::Trajectory<double>>(m, "Trajectory");

  py::class_<trajectories::PiecewiseTrajectory<double>, trajectories::Trajectory<double>>(m, "PiecewiseTrajectory")
      .def("value", &trajectories::Trajectory<double>::value, py::arg("t"))
      .def("start_time", (double (trajectories::PiecewiseTrajectory<double>::*)(int) const)
          &trajectories::PiecewiseTrajectory<double>::start_time)
      .def("end_time", (double (trajectories::PiecewiseTrajectory<double>::*)(int) const)
          &trajectories::PiecewiseTrajectory<double>::end_time)
      .def("start_time", (double (trajectories::PiecewiseTrajectory<double>::*)(void) const)
          &trajectories::PiecewiseTrajectory<double>::start_time)
      .def("end_time", (double (trajectories::PiecewiseTrajectory<double>::*)(void) const)
          &trajectories::PiecewiseTrajectory<double>::end_time);

  // Bind Polynomials
  py::class_<trajectories::Polynomial<double>>(m, "Polynomial")
      .def(py::init<>()) // Zero Initialization
      .def(py::init<const double>()) // Scalar Initialization
      .def(py::init<const std::string, const unsigned int>()) // Variable Initialization "t"
      .def(py::init<const double, const unsigned int>()) // Single Monomial
      .def(py::self + py::self) // Addition
      .def(py::self - py::self) // Subtraction
      .def(py::self * py::self) // Multiplication
      .def(float() * py::self) // Scalar Multiplication
      .def(py::self / float()) // Scalar Division
      .def(float() + py::self) // Scalar Addition
      .def(py::self + float()) // Scalar Addition
      .def(py::self - float()) // Scalar Subtraction
      .def("__pow__", [](const trajectories::Polynomial<double> &a, int b) { // Exponents
        return trajectories::pow(a, b);
      }, py::is_operator())
      .def("EvaluateUnivariate", &trajectories::Polynomial<double>::EvaluateUnivariate<double>);;

  // Piecewise Polynomials
  py::class_<trajectories::PiecewisePolynomial<double>,
             trajectories::PiecewiseTrajectory<double>>(m, "PiecewisePolynomial")
      .def(py::init<>())
      .def(py::init<const std::vector<trajectories::Polynomial<double>> &, const std::vector<double> &>())
      .def(py::init<const Eigen::Ref<const Eigen::MatrixXd> &>())
      .def("scalarValue", &trajectories::PiecewisePolynomial<double>::scalarValue)
      .def("derivative", &trajectories::PiecewisePolynomial<double>::derivative)
      .def("value", &trajectories::PiecewisePolynomial<double>::value);

  // SE3Pose Binding
  py::class_<erl::SE3Pose>(m, "SE3Pose")
      .def(py::init<const Eigen::Vector3d &, const Eigen::Vector4d &>())
      .def(py::init<const Eigen::Vector3d &, const Eigen::Matrix3d &>())
      .def(py::init<const Eigen::Vector3d &>())
      .def(py::init<const Eigen::Vector6d &>())
      .def(py::init<const Eigen::Matrix3d &>())
      .def(py::init<const Eigen::Matrix4d &>())
      .def("getYaw", &erl::SE3Pose::getYaw)
      .def("getPitch", &erl::SE3Pose::getPitch)
      .def("getRoll", &erl::SE3Pose::getRoll)
      .def("getSE2", &erl::SE3Pose::getSE2)
      .def("getVectorForm", &erl::SE3Pose::getVectorForm)
      .def_readonly("position", &erl::SE3Pose::position)
      .def_readonly("orientation", &erl::SE3Pose::orientation);



  // Angle Conversions
  m.def("quat2angle", &erl::quat2angle<double>, "Converts a quaternion to Euler angles in ZYX orientation.");
  m.def("angle2quat", &erl::angle2quat<double>, "Converts euler angles in ZYX orientation to a quaternion.");

}
