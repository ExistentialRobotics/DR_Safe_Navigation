
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <erl_astar/environments/planning_2d.h>
#include <erl_astar/environments/planning_3d.h>
#include <erl_astar/environments/planning_se2.h>
#include <erl_astar/environments/planning_fas.h>
#include <erl_astar/astar_nx.h>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>


namespace py = pybind11;

/**********************************************************************
// Convenience Macro's for Templated Types
**********************************************************************/
//AStarOutput
#define AStarOutput(handle, type, name) py::class_<erl::AStarOutput<type>>(handle, name) \
      .def_readonly("pcost", &erl::AStarOutput<type>::pcost) \
      .def_readonly("path", &erl::AStarOutput<type>::path) \
      .def_readonly("opened_list", &erl::AStarOutput<type>::opened_list) \
      .def_readonly("closed_list", &erl::AStarOutput<type>::closed_list) \
      .def_readonly("action_idx", &erl::AStarOutput<type>::action_idx);
//.def_readonly("metric_path", &erl::AStarOutput<type>::metric_path) \

// AStarWrappers for Metric environments 2D, 3D, SE2, FAS
#define AStarWrapper(handle, type, env_planning, name) handle.def(name, [](std::vector<double> start, \
const env_planning &env, double eps = 1, bool log = false) { \
  erl::ARAStar<type> AA; \
  type start_cell = env.toState(start); \
  erl::AStarOutput<type> output = AA.Astar(start_cell, env, eps, log); \
  return output; \
}, py::arg("start"), py::arg("env"), py::arg("epsilon") = 1, py::arg("log") = false);

//for (const auto &pt : output.path) \
//    output.metric_path.push_back(env.toMetric(pt)); \
  
#define FASPlanningEnv(handle, dim, order, name) py::class_<erl::PlanningEnvironmentFAS<dim, order, int8_t>,  \
        erl::EnvironmentFAS<dim,order,int8_t>>(m, name) \
        .def(py::init<const erl::EnvironmentFAS<dim, order, int8_t> &, \
        const std::vector<double> &, const std::vector<double> &>()) \
        .def("getHeuristic", &erl::PlanningEnvironmentFAS<dim,order, int8_t>::getHeuristic) \
        .def("getLQMTHeuristic", &erl::PlanningEnvironmentFAS<dim,order, int8_t>::getLQMTHeuristic);


/**********************************************************************
// Bindings
 **********************************************************************/
PYBIND11_MODULE(pyAStar, m) {
  py::module::import("pyErlEnv"); // Register the Environment package.

  //AStarOutputs
  AStarOutput(m, std::vector<int>, "AStarOutput"); // State output for 2D, 3D, SE(2)
  AStarOutput(m, erl::Waypoint2D3O, "AStarOutputFAS2D3O"); // State Output for FAS system.
  AStarOutput(m, erl::Waypoint2D4O, "AStarOutputFAS2D4O"); // State Output for FAS system.

  // PlanningEnvironments
  py::class_<erl::Planning2D<int8_t>>(m, "Planning2D")
      .def(py::init<const erl::Environment2D<int8_t> &, const std::vector<double> &>());

  py::class_<erl::Planning3D<int8_t>>(m, "Planning3D")
      .def(py::init<const erl::Environment3D<int8_t> &, const std::vector<double> &>());

  py::class_<erl::PlanningSE2<int8_t>>(m, "PlanningSE2")
      .def(py::init<const erl::EnvironmentSE2<int8_t> &, const std::vector<double> &, const std::vector<double> &>());

  FASPlanningEnv(m, 2, 4, "PlanningFAS2D4O");
  FASPlanningEnv(m, 2, 3, "PlanningFAS2D3O");

// Convenience Typedefs
  typedef erl::PlanningEnvironmentFAS<2,4, int8_t> PlanningEnvironmentFAS2D4O;
  typedef erl::PlanningEnvironmentFAS<2,3, int8_t> PlanningEnvironmentFAS2D3O;
  //AStarWrappers
  AStarWrapper(m, std::vector<int>, erl::Planning2D<int8_t>, "AStar2D");
  AStarWrapper(m, std::vector<int>, erl::Planning3D<int8_t>, "AStar3D");
  AStarWrapper(m, std::vector<int>, erl::PlanningSE2<int8_t>, "AStarSE2");
  AStarWrapper(m, erl::Waypoint2D3O, PlanningEnvironmentFAS2D3O , "AStarFAS2D3O");
  AStarWrapper(m, erl::Waypoint2D4O, PlanningEnvironmentFAS2D4O, "AStarFAS2D4O");
}
