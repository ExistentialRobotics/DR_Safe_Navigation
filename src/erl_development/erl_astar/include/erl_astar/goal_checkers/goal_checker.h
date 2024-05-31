//
// Created by brent on 12/16/18.
//

#ifndef ERL_ASTAR_GOAL_CHECKER_H
#define ERL_ASTAR_GOAL_CHECKER_H

namespace erl {

/**
 * Virtual Goal Checker class defines a general interface for common types of heuristic functions.
 * @tparam state The type of state to take as input.
 */
template <class state>
class GoalChecker {
 public:

  /**
   * Computes whether the state is at the goal.
   * @return True if the state has reached the goal.
   */
  virtual bool isGoal(const state &)=0;

};// End class
} // End namespace
#endif //ERL_ASTAR_GOAL_CHECKER_H
