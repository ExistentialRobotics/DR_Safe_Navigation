
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <erl_env/environments/environment_2d.h>
#include <erl_env/environments/environment_3d.h>
#include <erl_env/environments/environment_se2.h>
#include <erl_env/environments/environment_se3.h>
#include <erl_env/environments/environment_fas.h>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <erl_map/grid_map.h>

namespace py = pybind11;

template<class state>
struct Successor {
  state wp;
  int action_idx;
  std::vector<double> coord;
  std::vector<std::vector<double>> trajectory;
};

template<class ENV, class state>
std::vector<Successor<state>> GetSuccessors(const ENV &env, const std::vector<double> &curr) {
  // Convert metric to state.
  state curr_state = env.toState(curr);
  // Compute Successors.
  std::vector<state> succ_list;
  std::vector<int> action_idx;
  std::vector<double> costs;
  env.getSuccessors(curr_state, succ_list, costs, action_idx);
  // Compute the Output
  std::vector<Successor<state>> successors(action_idx.size());
  for (unsigned i = 0; i < action_idx.size(); i++) {
    // Add coordinate.
    successors[i].coord = env.toMetric(succ_list[i]);
    // Add trajectoryForward Action / Trajectory
    for (const state &point : env.forwardAction(curr_state, action_idx[i])) {
      successors[i].trajectory.push_back(env.toMetric(point));
    }
    // For polynomials
    successors[i].wp = succ_list[i];
    successors[i].action_idx = action_idx[i];
  }
  return successors;
}
/** FAS Convenience Macros **/
#define FASBinding(handle, dim, order, name) py::class_<erl::EnvironmentFAS<dim, order, int8_t>>(handle, name) \
                    .def(py::init<const erl::GridMap<int8_t> &, \
                        const std::vector<double> &, const std::vector<double> &, double, double>(), \
                    py::arg("map"), \
                    py::arg("dimmin"), \
                    py::arg("dimmax"), \
                    py::arg("tau") = 1.0, \
                    py::arg("rho") = 2.0) \
                    .def("getSuccessors", &GetSuccessors<erl::EnvironmentFAS<dim, order, int8_t>, erl::Waypoint<dim, order>>) \
                    .def("toMetric", &erl::EnvironmentFAS<dim, order, int8_t>::toMetric) \
                    .def("toTrajectorySingle", &erl::EnvironmentFAS<dim, order, int8_t>::toTrajectorySingle) \
                    .def("toTrajectory", &erl::EnvironmentFAS<dim, order, int8_t>::toTrajectory);

#define WaypointBinding(handle, dim, order, name) py::class_<erl::Waypoint<dim, order>>(handle, name) \
                        .def(py::init()) \
                        .def_readwrite("coord", &erl::Waypoint<dim,order>::coord);

/** Successor Convenience Macro **/
#define SuccessorBinding(handle, name, type) py::class_<Successor<type>>(handle, name) \
                    .def_readwrite("wp", &Successor<type>::wp) \
                    .def_readwrite("action_idx", &Successor<type>::action_idx) \
                    .def_readwrite("coord", &Successor<type>::coord) \
                    .def_readwrite("trajectory", &Successor<type>::trajectory);

PYBIND11_MODULE(pyErlEnv, m) {
  // 2D Environment
  py::class_<erl::Environment2D<int8_t>>(m, "Environment2D")
      .def(py::init<const erl::GridMap<int8_t> &>())
      .def(py::init<const std::vector<double> &, const std::vector<double> &, const std::vector<double> &>(),
           py::arg("mapmin"), py::arg("mapmax"), py::arg("mapres"))
      .def("getSuccessors", &GetSuccessors<erl::Environment2D<int8_t>, std::vector<int>>)
      .def("toMetric", &erl::Environment2D<int8_t>::toMetric)
      .def("toTrajectory", &erl::Environment2D<int8_t>::toTrajectory);

  // 3D Environment
  py::class_<erl::Environment3D<int8_t>>(m, "Environment3D")
      .def(py::init<const erl::GridMap<int8_t> &>())
      .def("getSuccessors", &GetSuccessors<erl::Environment3D<int8_t>, std::vector<int>>)
      .def("toMetric", &erl::Environment3D<int8_t>::toMetric);

  // SE2 Environment
  py::class_<erl::EnvironmentSE2<int8_t>>(m, "EnvironmentSE2")
      .def(py::init<const std::shared_ptr<erl::GridMap<int8_t>> &,
                    const std::vector<double> &,
                    const std::vector<double> &,
                    const std::vector<double> &,
                    unsigned>(),
           py::arg("map"), py::arg("vlist"), py::arg("wlist"),
           py::arg("tlist"), py::arg("heading_discretization") = 25)
//      .def(py::init<const erl::GridMap<int8_t> &,
//                    const std::string &, unsigned>(), py::arg("map"), py::arg("mprim_yaml"),
//           py::arg("heading_discretization") = 25)
      .def("getSuccessors", &GetSuccessors<erl::EnvironmentSE2<int8_t>, std::vector<int>>)
      .def("toMetric", &erl::EnvironmentSE2<int8_t>::toMetric);


  // SE3 Environment
  py::class_<erl::EnvironmentSE3<int8_t>>(m, "EnvironmentSE3")
      .def(py::init<const std::shared_ptr<erl::GridMap<int8_t>> &,
                    const std::vector<Eigen::Vector6d> &,
                    const std::vector<double> &>(),
           py::arg("map"), py::arg("primitives"), py::arg("tlist"))
      .def("getSuccessors", &GetSuccessors<erl::EnvironmentSE3<int8_t>, erl::SE3Pose>)
      .def("toMetric", &erl::EnvironmentSE3<int8_t>::toMetric);

  // Bind FAS systems.
  FASBinding(m, 2, 3, "EnvironmentFAS2D3O");
  FASBinding(m, 2, 4, "EnvironmentFAS2D4O");

  // Bind Waypoints
  WaypointBinding(m, 2, 3, "Waypoint2D3O");
  WaypointBinding(m, 2, 4, "Waypoint2D4O");

  // Successor Bindings
  SuccessorBinding(m, "SuccessorFAS2D3O", erl::Waypoint2D3O);
  SuccessorBinding(m, "SuccessorFAS2D4O", erl::Waypoint2D4O);
  SuccessorBinding(m, "SuccessorSE3", erl::SE3Pose);
  SuccessorBinding(m, "SuccessorND", std::vector<double>);
  SuccessorBinding(m, "SuccessorNDInt", std::vector<int>);

}
