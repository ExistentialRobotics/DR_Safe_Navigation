//
// Created by brent on 12/16/18.
//

#ifndef ERL_ASTAR_ENV_PLANNING_3D_H
#define ERL_ASTAR_ENV_PLANNING_3D_H

#include <erl_astar/planning_interface.h>
#include <erl_env/environments/environment_3d.h>

namespace erl {

/**
 * @brief The Planning3D implements the PlanningInterface specification for a 3-D grid environment.
 */
template <class U>
class Planning3D : public PlanningInterface<std::vector<int>>, public Environment3D<U> {
 public:
  using Environment3D<U>::map;
  std::vector<int> goal_coord; // Desired Goal state.

  /**
   * @brief Constructs a 3D Planning Environment from a standard 2D environment and a goal.
   * @param env The base 3D environment.
   * @param goal The desired goal.
   */
  Planning3D(const Environment3D<U> &env, const std::vector<double> &goal) : Environment3D<U>(env),
                                                                                     goal_coord(map->meters2cells(goal)) {}

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
    Environment3D<U>::getSuccessors(curr, succ, succ_cost, action_idx);
  }

  /**
   * @brief Computes an L1 heuristic estimate from the state to the goal.
   * @param state_coord The query state.
   * @param goal_coord The goal state.
   * @return The heuristic value.
   */
  double getHeuristic(const std::vector<int> &state_coord) const override {
    double h = 0;
    for (unsigned it = 0; it < state_coord.size(); ++it) {
      double diff = erl::cells2meters(state_coord[it], map->min()[it], map->res()[it]) -
          erl::cells2meters(goal_coord[it], map->min()[it], map->res()[it]);
      h += diff * diff;
    }
    return std::sqrt(h);
  }

  /**
   * @brief Indicates if a state s is the goal state.
   * @param s Indicates if the state s is a goal state.
   * @param goal The goal state to be queried against.
   * @return True if the state is at the goal.
   */
  bool isGoal(const std::vector<int> &state_coord) const override {
    if (state_coord.size() != goal_coord.size())
      return false;

    for (unsigned it = 0; it < state_coord.size(); ++it)
      if (state_coord[it] != goal_coord[it])
        return false;
    return true;
  }

  /**
   * @brief Converts the state to a linear index.
   * @param s The state to be converted.
   * @return The linear index.
   */
  size_t stateToIndex(const std::vector<int> &state) const override {
    return map->subv2ind_colmajor(state);
  }


  /**
   * @brief Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   
  std::vector<std::vector<int>> forwardAction(const std::vector<int>  &curr, int action_id) const override {
    return Environment3D::forwardAction(curr, action_id);
  }
  */
  
}; // End class
} // End namespace
#endif //ERL_ASTAR_ENV_PLANNING_3D_H
