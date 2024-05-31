//
// Created by brent on 12/16/18.
//

#ifndef ERL_ASTAR_HEURISTIC_H
#define ERL_ASTAR_HEURISTIC_H
namespace erl {

 /**
  * Virtual Heuristic class defines a general interface for common types of heuristic functions.
  * @tparam state The type of state to take as input.
  */
template <class state>
class Heuristic {
 public:
  /**
   * Computes the heuristic value of a state.
   * @return The type of heuristic.
   */
  virtual double getHeuristic(const state &)=0;

};// End class
} // End namespace
#endif //ERL_ASTAR_HEURISTIC_H
