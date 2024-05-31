#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "erl_conversions/erl_msg_utils.h"
#include "erl_conversions/message_serialization.h"

namespace py = pybind11;

PYBIND11_MODULE(pyErlConversions, m) {
  /**
   * Conversions from ROS. Write one of these to read in a message as a C++ object bound in Python.
   */
  m.def("gridmapFromROS",
        [](const std::string &buffer) { return erl::fromROS(from_python<erl_msgs::GridMap>(buffer), true); });
  // m.def("beliefFromROS", [](const std::string &buffer) {
  //   return erl::fromROS(from_python<erl_msgs::MultiTargetBelief>(buffer));
  // });

  /**
   * Conversions to ROS. Write one of these to publish a C++ object using the Python messaging API.
   */
  m.def("gridmapToROS", [](const erl::GridMap<int8_t> &gridmap) { return py::bytes(to_python(erl::toROS(gridmap))); });
  m.def("primTrajToROS", [](const trajectories::PiecewisePolynomial<double> &erl_traj, double z) {
    return py::bytes(to_python(erl::toROS(erl_traj, z)));
  });
}
