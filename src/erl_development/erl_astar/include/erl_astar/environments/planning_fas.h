#ifndef ERL_ASTAR_ENV_PLANNING_FAS_H
#define ERL_ASTAR_ENV_PLANNING_FAS_H

#include <erl_astar/planning_interface.h>
#include <erl_env/environments/environment_fas.h>
#include <erl_utilities/erl_poly_utils.h>

namespace erl
{

template <std::size_t D, std::size_t N, class U>
class PlanningEnvironmentFAS : public PlanningInterface<Waypoint<D,N>>, public EnvironmentFAS<D,N,U> {
  using EnvironmentFAS<D,N,U>::map_DD_ptr;
  using EnvironmentFAS<D,N,U>::rho_;

 protected:
  std::vector<double> goal_;
  std::vector<double> gtol_;

public:
  /**
   * Constructs the FAS Planning Environment purely from the YAML file.
   * @param input_yaml The input string describing the Environment.
   * @param goal The goal state being searched for.
   * @param gtol The tolerance for achieving the goal
   */
  PlanningEnvironmentFAS( const EnvironmentFAS<D,N, U> &env,
                          const std::vector<double>& goal,
                          const std::vector<double>& gtol )
    : EnvironmentFAS<D,N,U>(env), goal_(goal), gtol_(gtol) {}

  /**
   * Computes the successors of the state curr.
   * @param curr The current state to compute successors of.
   * @param succ The vector of successor states.
   * @param succ_cost The vector of costs of each successor.
   * @param action_idx The vector of indices of the actions leading to the successor.
   */  
  void getSuccessors( const Waypoint<D,N>& curr, 
                      std::vector<Waypoint<D,N>>& succ,
                      std::vector<double>& succ_cost,
                      std::vector<int>& action_idx ) const override
  { EnvironmentFAS<D,N, U>::getSuccessors(curr, succ, succ_cost, action_idx); }
  



  /**
   * Computes the LQMT search heuristic: see https://github.com/sikang/motion_primitive_library/blob/master/include/mpl_planner/common/env_base.h
   * @param w The current waypoint.
   * @param goal_coord The goal coordinate.
   * @return The heuristic value.
   */
  double getLQMTHeuristic(const Waypoint<D,N>& w) const
  {
    double t_bar = 0.0;
    for( unsigned d = 0; d < D; ++d)
      t_bar = std::max(t_bar, std::max( (goal_[d]-w.coord[d])/map_DD_ptr->min()[d+D],
                                        (goal_[d]-w.coord[d])/map_DD_ptr->max()[d+D] ));
    return t_bar * rho_;
  }

  /**
   * Computes the LQMT search heuristic: see https://github.com/sikang/motion_primitive_library/blob/master/include/mpl_planner/common/env_base.h
   * @param w The current waypoint.
   * @param goal_coord The goal coordinate.
   * @return The heuristic value.
   */
  double getHeuristic(const Waypoint<D,N>& w) const override
  {
    double t_bar = 0.0;
    for( unsigned d = 0; d < D; ++d)
      t_bar = std::max(t_bar, std::max( (goal_[d]-w.coord[d])/map_DD_ptr->min()[d+D],
                                        (goal_[d]-w.coord[d])/map_DD_ptr->max()[d+D]));                                   
    switch(N)
    {
      case 1:
        return getHeuristic1O(w, t_bar);
      case 2:
        return getHeuristic2O(w, t_bar);
      case 3:
        return getHeuristic3O(w, t_bar);
      default:
        return t_bar * rho_;
    }
  }
  
  
  /**
   * Checks if the current waypoint has reached the goal.
   * @param w The current waypoint.
   * @param goal_coord The desired goal coordinate.
   * @return True if the waypoint has reached the goal.
   */   
  bool isGoal(const Waypoint<D,N>& w) const override
  {
    for( std::size_t it = 0; it < w.coord.size(); ++it )
      if( std::abs( w.coord[it] - goal_[it] ) > gtol_[it] )
        return false;
    return true;
  }

  /**
   * Computes an index from the state.
   * @param w The current waypoint.
   * @return The index.
   */
  std::size_t stateToIndex(const Waypoint<D,N>& w) const override
  {
    std::vector<int> datac(w.coord.size());
    for( unsigned k = 0; k < w.coord.size(); ++k )
      datac[k] = erl::meters2cells( w.coord[k], map_DD_ptr->min()[k],
                                               map_DD_ptr->res()[k] );
    return map_DD_ptr->subv2ind_rowmajor(datac);
  }

  /**
   * Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   
  std::vector<erl::Waypoint<D,N>> forwardAction(const erl::Waypoint<D,N> &curr, int action_id) const override
  { return forwardAction(curr, action_id); }
  */

private:

  double getHeuristic1O(const Waypoint<D,N>& w, double t_bar) const
  {
    if( t_bar < 1e-5) return 0.0;
    double norm_dp = 0.0;
    for( unsigned d = 0; d < D; ++d )
      norm_dp += (goal_[d]-w.coord[d])*(goal_[d]-w.coord[d]);
    norm_dp = std::sqrt(norm_dp);
    if( norm_dp > t_bar*std::sqrt(rho_) )
      return 2*norm_dp*std::sqrt(rho_);
    else
      return t_bar*rho_ + norm_dp*norm_dp/t_bar;
  }

  double getHeuristic2O(const Waypoint<D,N>& w, double t_bar) const
  {
    if( t_bar < 1e-5) return 0.0;
    
    double a = rho_;
    double b = 0;
    
    double c = 0.0;
    for( unsigned k = D; k < 2*D; ++k )
      c += w.coord[k]*w.coord[k] + w.coord[k]*goal_[k] + goal_[k]*goal_[k];
    c = -4*c;
    
    double d = 0.0;
    for( unsigned k = 0; k < D; ++k )
      d += (goal_[k]-w.coord[k])*(goal_[k+D]+w.coord[k+D]);
    d = 24*d;
    
    double e = 0.0;
    for( unsigned k = 0; k < D; ++k )
      e += (goal_[k]-w.coord[k])*(goal_[k]-w.coord[k]);
    e = -36*e;
    
    std::vector<double> ts = erl::quartic(a, b, c, d, e);
    ts.push_back(t_bar);
    double min_cost = std::numeric_limits<double>::max();
    for(auto t: ts)
    {
      if(t < t_bar)
        continue;
      double cost = a*t-c/t-d/2/t-e/3/t/t/t;
      if(cost < min_cost)
        min_cost = cost;
    }
    return min_cost;
  }
     
  double getHeuristic3O(const Waypoint<D,N>& w, double t_bar) const
  {
    if( t_bar < 1e-5) return 0.0;
    
    double a = rho_;
    double b = 0;
    
    double c = 0.0;
    for( unsigned k = 2*D; k < 3*D; ++k )
      c += -9*(w.coord[k]*w.coord[k] + goal_[k]*goal_[k]) + 6*w.coord[k]*goal_[k];
    
    double d = 0.0;
    for( unsigned k = D; k < 2*D; ++k )
      d += -144*w.coord[k+D]*w.coord[k] - 96*w.coord[k+D]*goal_[k] + 96*goal_[k+D]*w.coord[k] + 144*goal_[k+D]*goal_[k];
    
    double e = 0.0;
    for( unsigned k = 0; k < D; ++k )
      e += 360*(goal_[k]-w.coord[k])*(w.coord[k+2*D]-goal_[k+2*D]) -576*(w.coord[k+D]*w.coord[k+D] + goal_[k+D]*goal_[k+D])
            -1008*w.coord[k+D]*goal_[k+D];
    
    double f = 0.0;
    for( unsigned k = 0; k < D; ++k )
      f += (goal_[k]-w.coord[k])*(goal_[k+D]+w.coord[k+D]);
    f = 2880*f;
    
    double g = 0.0;
    for( unsigned k = 0; k < D; ++k )
      g += (goal_[k]-w.coord[k])*(goal_[k]-w.coord[k]);
    g = -3600*g;
    
    std::vector<double> ts = erl::solve(a, b, c, d, e, f, g);
    ts.push_back(t_bar);
    double min_cost = std::numeric_limits<double>::max();
    for(auto t: ts)
    {
      if(t < t_bar)
        continue;
      double cost = a*t-c/t-d/2/t/t-e/3/t/t/t-f/4/t/t/t/t-g/5/t/t/t/t/t;
      if(cost < min_cost)
        min_cost = cost;
    }
    return min_cost;    
  }
  
}; // End Class


} // End namespace
#endif

