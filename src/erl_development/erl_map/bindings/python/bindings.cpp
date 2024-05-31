

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <erl_map/grid_map.h>
#include <erl_map/grid_map_utils.h>

namespace py = pybind11;

PYBIND11_MODULE(pyErlMap, m) {

  py::module::import("pyErlUtils"); // Register the Utilities as a dependency.

  // Bind the Map
  py::class_<erl::GridMap<int8_t>, std::shared_ptr<erl::GridMap<int8_t>>>(m, "GridMap")
      .def(py::init<>(), "Default Constructor")
      .def(py::init<const std::vector<double> &, // Load from min, max, res
                    const std::vector<double> &,
                    const std::vector<double> &,
                    bool>(), py::arg("min"), py::arg("max"), py::arg("res"),
                    py::arg("rowmajor") = false)
      .def("min", &erl::GridMap<int8_t>::min)
      .def("max", &erl::GridMap<int8_t>::max)
      .def("res", &erl::GridMap<int8_t>::res)
      .def("map", &erl::GridMap<int8_t>::map)
      .def("size", &erl::GridMap<int8_t>::size)
      .def("rowmajor", &erl::GridMap<int8_t>::rowmajor)
          //.def("totalSize", &nx::map_nd::totalSize)
      .def("getLinearIndex", &erl::GridMap<int8_t>::subv2ind)
      .def("get2DSlice", &erl::GridMap<int8_t>::get2DSlice)
      .def("setMap", &erl::GridMap<int8_t>::setMap)
      .def("load", &erl::GridMap<int8_t>::load)
      .def("save", &erl::GridMap<int8_t>::save)
      .def("meters2cells",
              (std::vector<int> (erl::GridMap<int8_t>::*)(const std::vector<double> &) const)
                &erl::GridMap<int8_t>::meters2cells)
      .def("load_legacy", &erl::GridMap<int8_t>::loadLegacy);


  // Helper Functions
  m.def("inflateMap2D", (std::vector<int8_t> (*) (const erl::GridMap<int8_t> &, double)) &erl::inflateMap2D, "Inflates a Map by some factor.");
  m.def("bresenham2D", &erl::bresenham2D, "2-D Bresenham");
  
}

