#ifndef __ENV_INT_H_
#define __ENV_INT_H_

#include <vector>                           // std::vector
//#include <erl_map/grid_map_utils.h>
#include <iostream>
namespace erl {


 /**
  * @brief The EnvironmentBase is a virtual interface for search-based planning on a graph, featuring a generic
  * state with behavior defined by the implementation of the interface. Example implementations include n-D
  * grid states, and SE(2) motion primitives.
  * @tparam state The type of the state.
  */
template<class state>
class EnvironmentBase {
 public:
  /**
   * @brief Computes the successors of the state curr.
   * @param curr The current state to compute successors of.
   * @param succ The vector of successor states.
   * @param succ_cost The vector of costs of each successor.
   * @param action_idx The vector of indices of the actions leading to the successor.
   */
  virtual void getSuccessors(const state &curr,
                             std::vector<state> &succ,
                             std::vector<double> &succ_cost,
                             std::vector<int> &action_idx) const = 0;
    
  /**
   * @brief Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   */
  virtual std::vector<state> forwardAction(const state &curr, int action_id) const = 0;


}; // End class

} // End namespace

#endif
