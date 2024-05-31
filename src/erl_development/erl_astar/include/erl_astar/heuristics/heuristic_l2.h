//
// Created by brent on 12/16/18.
//

#ifndef ERL_ASTAR_HEURISTIC_L2_H
#define ERL_ASTAR_HEURISTIC_L2_H
#include <erl_astar/heuristics/heuristic.h>

namespace erl {
 class L2Heuristic : public Heuristic<std::vector<double>> {
  public:
   std::vector<double> goal;

   /**
   * Constructs the L2 Heuristic with the desired goal.
   * @param goal The goal to be reached.
   */
   L2Heuristic(const std::vector<double> &goal) : goal(goal) {}

   /**
    * Computes the value of the heuristic.
    * @param state The current state.
    * @return The value of the heuristic.
    */
   double getHeuristic(const std::vector<double> &state) {
     double h = 0;
     for (unsigned it = 0; it < state.size(); ++it) {
       double diff = (state[it] - goal[it]);
       h += diff * diff;
     }
     return std::sqrt(h);
   }
 };
}

#endif //ERL_ASTAR_HEURISTIC_L2_H
