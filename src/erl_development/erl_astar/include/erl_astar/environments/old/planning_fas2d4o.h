//
// Created by brent on 12/16/18.
//

#ifndef ERL_ASTAR_ENV_PLANNING_FAS2D4O_H
#define ERL_ASTAR_ENV_PLANNING_FAS2D4O_H

#include <erl_env/environments/old/environment_fas2d4o.h>
#include <erl_astar/planning_interface.h>

namespace erl {
/**
 * @brief The PlanningFAS2D4O class implements the PlanningInterface specification for a Fully Actuated System.
 */
class PlanningFAS2D4O : public PlanningInterface<Waypoint2D4O>, public EnvironmentFAS2D4O {
 protected:
  Waypoint2D4O goal_coord;
  // Goal Tolerance
  std::vector<double> gtol_; // tolerance for reaching the goal
 public:
/**
 * Constructs the FAS Planning Environment purely from the YAML file.
 * @param input_yaml The input string describing the Environment.
 * @param goal The goal state being searched for.
 */
  PlanningFAS2D4O(const EnvironmentFAS2D4O &env, const std::vector<double> &goal) : EnvironmentFAS2D4O(env) {
    // Setup Goal
    //std::vector<double> goal_8d = {goal[0], goal[1], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    goal_coord = Waypoint2D4O({goal[0], goal[1], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    // Setup Goal Tolerance
    gtol_ = std::vector<double>(8, 2.0);
  }
  
  /**
   * Computes the successors of the state curr.
   * @param curr The current state to compute successors of.
   * @param succ The vector of successor states.
   * @param succ_cost The vector of costs of each successor.
   * @param action_idx The vector of indices of the actions leading to the successor.
   */
  void getSuccessors(const  erl::Waypoint2D4O &curr,
                std::vector<erl::Waypoint2D4O > &succ,
                std::vector<double> &succ_cost,
                std::vector<int> &action_idx) const override {
    EnvironmentFAS2D4O::getSuccessors(curr, succ, succ_cost, action_idx);
  }


  /**
   * Computes the LQMT search heuristic: see https://github.com/sikang/motion_primitive_library/blob/master/include/mpl_planner/common/env_base.h
   * @param w The current waypoint.
   * @param goal_coord The goal coordinate.
   * @return The heuristic value.
   */
  double getHeuristic(const Waypoint2D4O &w) const override {
    std::vector<double> goal_ = {goal_coord.coord[0], goal_coord.coord[1]}; // Extract goal from Waypoint
    return std::max(std::max((goal_[0] - w.coord[0]) / MAP.min()[2], (goal_[0] - w.coord[0]) / MAP.max()[2]),
                    std::max((goal_[1] - w.coord[1]) / MAP.min()[3], (goal_[1] - w.coord[1]) / MAP.max()[3])) * rho_;
  }

  /**
   * Checks if the current waypoint has reached the goal.
   * @param w The current waypoint.
   * @param goal_coord The desired goal coordinate.
   * @return True if the waypoint has reached the goal.
   */
  bool isGoal(const Waypoint2D4O &w) const override {

    std::vector<double> goal_ = {goal_coord.coord[0], goal_coord.coord[1]}; // Extract goal from Waypoint
    for (size_t it = 0; it < w.coord.size(); ++it)
      if (std::abs(w.coord[it] - goal_[it]) > gtol_[it])
        return false;
    return true;
  }

  /**
   * Computes an index from the state.
   * @param w The current waypoint.
   * @return The index.
   */
  std::size_t stateToIndex(const Waypoint2D4O &w) const override {
    std::vector<int> datac(w.coord.size());
    for (unsigned k = 0; k < w.coord.size(); ++k)
      datac[k] = erl::meters2cells(w.coord[k], MAP.min()[k], MAP.res()[k]);
    return MAP.subv2ind_rowmajor(datac);
  }

  /**
   * Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   
  std::vector<erl::Waypoint2D4O> forwardAction(const erl::Waypoint2D4O  &curr, int action_id) const override {
    return EnvironmentFAS2D4O::forwardAction(curr, action_id);
  }
  */
  
}; // End class
} // End namespace
#endif //ERL_ASTAR_ENV_PLANNING_FAS_H
