//
// Created by brent on 12/16/18.
//

#ifndef ERL_ASTAR_ENV_PLANNING_SE2_H
#define ERL_ASTAR_ENV_PLANNING_SE2_H

#include <erl_astar/planning_interface.h>
#include <erl_env/environments/environment_se2.h>

namespace erl {
/**
 * @brief The PlanningSE2 implements the PlanningInterface specification for an SE(2) environment.
 */
 template <class U>
class PlanningSE2 : public PlanningInterface<std::vector<int>>, public EnvironmentSE2<U> {
 public:
  using EnvironmentMetric<std::vector<int>, U>::map;

  std::vector<int> goal_coord; // Desired Goal state.
  std::vector<int> goal_tol; // Goal Tolerance.

  /**
   * @brief Constructs a SE2 Planning Environment from a standard 2D environment and a goal.
   * @param env The base SE2 environment.
   * @param goal The desired goal.
   */
  PlanningSE2(const EnvironmentSE2<U> &env, const std::vector<double> &goal,
                         const std::vector<double> &gtol) : EnvironmentSE2<U>(env),
                                                            goal_coord(map->meters2cells(goal)) {
    goal_tol.reserve(gtol.size());
    // X, Y, Heading tolerance.
    goal_tol[0] = gtol[0] / map->res()[0];
    goal_tol[1] = gtol[1] / map->res()[1];
    goal_tol[2] = gtol[2] / env.heading_res; // Heading resolution is not part of the standard N-D map.
  }
  /**
   * @brief Computes the successors of the state curr.
   * @param curr The current state to compute successors of.
   * @param succ The vector of successor states.
   * @param succ_cost The vector of costs of each successor.
   * @param action_idx The vector of indices of the actions leading to the successor.
   */
  void getSuccessors(const std::vector<int> &curr,
                std::vector<std::vector<int> > &succ,
                std::vector<double> &succ_cost,
                std::vector<int> &action_idx) const override {
    EnvironmentSE2<U>::getSuccessors(curr, succ, succ_cost, action_idx);
  }


  /**
   * @brief Checks whether the state at state_coord has reached the goal or not.
   * @param state_coord The state to be checked.
   * @param goal_coord The desired goal state.
   * @return True if the state has reached the goal.
   */
  inline bool isGoal(const std::vector<int> &state_coord) const override {
    for (unsigned it = 0; it < state_coord.size(); ++it)
      if (std::abs(state_coord[it] - goal_coord[it]) > goal_tol[it])
        return false;
    return true;
  }

  /**
   * @brief Computes the heuristic from the state at state_coord to the goal coordinate.
   * @param state_coord The current state to compute the heuristic from.
   * @param goal_coord The desired goal state.
   * @return The heuristic value.
   */
  inline double getHeuristic(const std::vector<int> &state_coord) const override {
    double h = 0;
    // L2 Distance in 2D (Disregard Heading)
    for (unsigned it = 0; it < state_coord.size() - 1; ++it) {
      double diff = erl::cells2meters(state_coord[it], map->min()[it], map->res()[it]) -
          erl::cells2meters(goal_coord[it], map->min()[it], map->res()[it]);
      diff = std::max((std::abs(diff) - goal_tol[it] * map->res()[it]), 0.0);
      h += diff * diff;
    }
    return std::sqrt(h);
  }

  /**
   * @brief Computes a linear index from the state, i.e. a single integer hash value.
   * @param state_coord The state to be hashed.
   * @return The hashed state.
   */
  size_t stateToIndex(const std::vector<int> &state_coord) const override {
    size_t val = map->subv2ind_colmajor({state_coord[0], state_coord[1]});
    val += state_coord[2] * map->size()[0] * map->size()[1];
    return val;
  }
  

  /**
   * @brief Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
  
  std::vector<std::vector<int>> forwardAction(const std::vector<int>  &curr, int action_id) const override {
    return EnvironmentSE2::forwardAction(curr, action_id);
  }
   */  
}; // End class
} // End namespace
#endif //ERL_ASTAR_ENV_PLANNING_SE2_H
