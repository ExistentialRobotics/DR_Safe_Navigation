#ifndef ERL_ASTAR_PLANNING_INTERFACE_H
#define ERL_ASTAR_PLANNING_INTERFACE_H

#include <vector>

namespace erl
{

/**
 * @brief The PlanningInterface provides the minimum functionality to execute the A* search algorithm. It allows for obtaining
 * successors, computing heuristics, checking a goal state, and
 * hashing a state to an index.
 * @tparam state The state of the environment.
 */
template <class state>
class PlanningInterface
{
public:

  /**
  * @brief Computes the successors of the state curr.
  * @param curr The current state to compute successors of.
  * @param succ The vector of successor states.
  * @param succ_cost The vector of costs of each successor.
  * @param action_idx The vector of indices of the actions leading to the successor.
  */
  virtual void getSuccessors( const state& curr,
                         std::vector<state>& succ,
                         std::vector<double>& succ_cost,
                         std::vector<int>& action_idx ) const {
    succ.push_back(curr);
    succ_cost.push_back(0);
    action_idx.push_back(0);
  }

  /**
   * This function is not needed in A* and should be part of the environment
   *
   * @brief Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   
  virtual std::vector<state> forwardAction( const state& curr, int action_id) const {
    std::vector<state> next_micro;
    next_micro.push_back( curr );
    return next_micro;
  }
  */
  
  /**
   * @brief Computes a heuristic estimate from the state to the goal.
   * @param s The query state.
   * @return The heuristic value.
   */
  virtual double getHeuristic(const state &) const {
    return 0;
  }

  /**
   * @brief Indicates if a state s is the goal state.
   * @param s The state to check for goal conditions.
   * @return True if the state is at the goal.
   */
  virtual bool isGoal(const state &) const {
    return true;
  }

  /**
   * @brief Converts the state to a linear index.
   * @param s The state to be converted.
   * @return The linear index.
   */
  virtual size_t stateToIndex(const state &) const {
    return 0;
  }
};
}

#endif //ERL_ASTAR_PLANNING_INTERFACE_H
