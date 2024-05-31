
#ifndef __ENV_SIMPLE_2D_H_
#define __ENV_SIMPLE_2D_H_
#include <cstdlib>
#include <cmath>
#include <array>

#include <erl_astar/planning_interface.h>

namespace erl
{
/**
 * @brief The Planning2DSimple environment implements the PlanningInterface for a 2D Cell Environment. The simple
 * environment does not use metric information, thus it operates only on discrete coordinates.
 */
class Planning2DSimple : public PlanningInterface<std::array<int,2>>
{
protected:
  static constexpr double sqrtTwo_ = 1.414213562373095145474621858739; //c++ BS: std::sqrt(2);
  std::array<int,2> goal_coord_;    // discrete coordinates of the goal node
  int xDim_, yDim_;
  //bool *cmap_;
  const uint16_t *cmap_;                 // cost map
  
public:

  /**
   * @brief Constructs the Environment2DSimple.
   * @param goal_coord The desired goal coordinate.
   * @param xDim The number of cells in the X dimension.
   * @param yDim The number of cells in the Y dimension.
   * @param cmap_ptr Pointer to the costmap.
   */
  Planning2DSimple( const std::array<int,2>& goal_coord, int xDim, int yDim,
                    const uint16_t *cmap_ptr )
    : goal_coord_(goal_coord), xDim_(xDim), yDim_(yDim), cmap_(cmap_ptr)
  {}

  /**
  * @brief Computes the successors of the state curr.
  * @param curr The current state to compute successors of.
  * @param succ The vector of successor states.
  * @param succ_cost The vector of costs of each successor.
  * @param action_idx The vector of indices of the actions leading to the successor.
  */
  void getSuccessors( const std::array<int,2>& curr,
                 std::vector<std::array<int,2>>& succ,
                 std::vector<double>& succ_cost,
                 std::vector<int>& action_idx ) const override
  {
    succ.reserve(8);
    succ_cost.reserve(8); action_idx.reserve(8);
    for (int xShift=-1; xShift <= 1; ++xShift){
      int xNeighbor = curr[0] + xShift;
      if (xNeighbor < 0) continue; // skip outside of map
      if (xNeighbor >= xDim_) continue;
      for (int yShift=-1; yShift <= 1; yShift++){
        if (xShift==0 && yShift==0) continue; // skip current node
        int yNeighbor = curr[1] + yShift;
        if (yNeighbor < 0) continue;
        if (yNeighbor >= yDim_) continue;
        
        int indNeighbor = xNeighbor + xDim_*yNeighbor;
        // skip collisions
        if( cmap_[indNeighbor] > 0 ) continue;
        //calc the cost of the successor (neighbor)
        double costMult; // get the cost multiplier
        switch (std::abs(xShift) + std::abs(yShift)){
            case 1: costMult = 1; break;
            case 2: costMult = sqrtTwo_; break;
        }
        succ.push_back({xNeighbor,yNeighbor});
        succ_cost.push_back(costMult);
        action_idx.push_back( (xShift+1) + (yShift+1)*3); // trenary to decimal
      }
    }
  }

  /**
   * @brief Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   */
  std::vector<std::array<int,2>> forwardAction( const std::array<int,2>& curr, int action_id ) const
  {
    std::vector<std::array<int,2>> next_micro;
    int xShift = (action_id % 3)-1;
    int yShift = (action_id / 3)-1;
    next_micro.push_back({curr[0]+xShift,curr[1]+yShift});
    return next_micro;
  }

  /**
   * @brief Computes a heuristic estimate from the state to the goal.
   * @param s The query state.
   * @return The heuristic value.
   */
  double getHeuristic(const std::array<int,2>& state_coord) const override
  {
    double h = 0;
    for( unsigned it = 0; it < state_coord.size(); ++it )
    {
      double diff = (state_coord[it] - goal_coord_[it]);
      h += diff*diff;
    }
    return std::sqrt(h);
  }

  /**
   * @brief Indicates if a state s is the goal state.
   * @param s The state to check for goal conditions.
   * @return True if the state is at the goal.
   */
  bool isGoal(const std::array<int,2>& state_coord) const override
  {
    if( state_coord.size() != goal_coord_.size() )
      return false;

    for( unsigned it = 0; it < state_coord.size(); ++it )
      if( state_coord[it] != goal_coord_[it] )
        return false;
    return true;
  }
  /**
 * @brief Converts the state to a linear index.
 * @param s The state to be converted.
 * @return The linear index.
 */
  size_t stateToIndex(const std::array<int,2>& state_coord) const override
  {
    return state_coord[0] + xDim_*state_coord[1];
  }
};

}

#endif
