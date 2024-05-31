//
// Created by brent on 12/16/18.
//

#ifndef ERL_ASTAR_GOAL_DISTANCE_H
#define ERL_ASTAR_GOAL_DISTANCE_H

#include <erl_astar/goal_checkers/goal_checker.h>

namespace erl {
class GoalDistance : public GoalChecker<std::vector<double>> {
public:
std::vector<double> goal;
std::vector<double> gtol;

/**
* Constructs the GoalDistance checker the desired goal and tolerance.
* @param goal The goal to be reached.
* @param gtol The desired tolerance for the goal to be reached.
*/
GoalDistance(const std::vector<double> &goal,
            const std::vector<double> &gtol) : goal(goal), gtol(gtol) {}
/**
 * Constructs the GoalDistance checker with default zero tolerance for reaching the goal.
 * @param goal The goal to be reached.
 */
GoalDistance(const std::vector<double> &goal) : GoalDistance(goal, std::vector<double>(goal.size(), 0.0)) {}
/**
 * Computes whether the state is at the goal.
 * @return True if the state has reached the goal.
 */
bool isGoal(const std::vector<double> &state) {
  for (unsigned it = 0; it < state.size(); ++it)
    if (std::abs(state[it] - state[it]) > gtol[it])
      return false;
  return true;
}
};
}

#endif //ERL_ASTAR_GOAL_DISTANCE_H
