//
// Created by brent on 12/16/18.
//

#ifndef ERL_ASTAR_ENV_METRIC_H
#define ERL_ASTAR_ENV_METRIC_H

#include <vector>                           // std::vector
#include <erl_map/grid_map.h>
#include <erl_env/environments/environment_base.h>
#include <iostream>
#include <exception>

namespace erl {

/**
 * @brief Exception for maps with incorrect map sizing.
 */
struct BadCostMapSize : public std::exception {
  const char *what() const throw() {
    return "Error. Check Cost Map Size.";
  }
};

/**
 * @brief The EnvironmentMetric class is a virtual interface for search-based planning on a metric space. Thus, the
 * EnvironmentMetric includes a set of Map parameters and a costmap, and all states may be converted to and from an internal
 * state representation to their metric form.
 * @tparam state The type of the state.
 */
template<class state, class U>
class EnvironmentMetric : virtual public EnvironmentBase<state> {
 public:
  std::shared_ptr<erl::GridMap<U>> map; // Map parameters

  /**
   * @brief Constructs an Environment from a map and costmap.
   * @param map The map of the environment.
   */
  EnvironmentMetric(const std::shared_ptr<erl::GridMap<U>> &map_)
      : map(map_) { checkCMap(); }

  /**
   * @brief Constructs a metric environment directly from the map parameters.
   * @param mapmin The minimum boundaries of the metric space.
   * @param mapmax The maximum boundaries of the metric space.
   * @param mapres The resolution of the metric space.
   */
  EnvironmentMetric(const std::vector<double> &mapmin,
                    const std::vector<double> &mapmax,
                    const std::vector<double> &mapres)
      : map(new erl::GridMap<U>(mapmin, mapmax, mapres)) {
    map->initZeroMap();
  }

  /**
   * @brief Computes the state representation of the metric state.
   * @param metric The metric state.
   * @return The state representation.
   */
  virtual state toState(const std::vector<double> &metric) const = 0;

  /**
   * Returns the metric representation of the state form.
   * @param cell The state.
   * @return The metric state.
   */
  virtual std::vector<double> toMetric(const state &) const = 0;

  /**
   * Returns the single index representation of the state according to the metric map.
   * @param s The state to compute the index coordinate of.
   * @return The index of the corresponding state.
   */
  virtual size_t toIndex(const state & s) const {
    return map->subv2ind(map->meters2cells(toMetric(s)));
  }

  /**
   * Returns a distance metric between the two states s1 and s2 according to the given environment. Base
   * implementation is the L2 Distance.
   * @param s1 The first state.
   * @param s2 The second state.
   * @return The resulting metric.
   */
  virtual double stateMetric(const state &s1, const state &s2) const {
    double result {0.0};
    auto s1_vec = toMetric(s1);
    auto s2_vec = toMetric(s2);
    for (unsigned k = 0; k < s1_vec.size(); k++)
    {
      result += std::pow(s1_vec[k] - s2_vec[k], 2);
    }
    return std::sqrt(result);
  }

 private:
  /**
   * Checks for an empty Cost-map. If the map is empty, assigns a valid sizes map of zeros.
   */
  void checkCMap() {
    if (map->map().empty()) {
      map->initZeroMap();
//      = std::make_shared<std::vector<U>>(std::vector<U>(map->totalSize(), 0));
    } else if (map->map().size() != map->totalSize()) {
      std::cerr << "Cmap size: " << map->map().size() << " map size " << map->totalSize() << std::endl;
      throw erl::BadCostMapSize();
    }
  }
}; // End class

} // End namespace erl
#endif //ERL_ASTAR_ENV_METRIC_H

