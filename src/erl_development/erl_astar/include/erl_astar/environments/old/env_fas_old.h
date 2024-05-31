
#ifndef __ENV_FAS_H_
#define __ENV_FAS_H_

#include <erl_env/environments/environment_base.h>
#include <erl_utilities/erl_map_utils.h>

namespace nx
{


class env_fas : public env_int<std::vector<int>>
{
protected:
  std::vector<double> start_; // metric coordinates of the start node
  std::vector<double> dmin_;  // limits on each dimension
  std::vector<double> dmax_;  // limits on each dimension
  std::vector<double> goal_; // metric coordinates of the goal node
  std::vector<double> gtol_; // tolerance for reaching the goal
  uint16_t *cmap;            // cost map
  const std::vector<std::vector<int>> uset = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
  const double tau = 0.5;
  const double tau2 = 0.5*0.5/2.0;
  const double tau3 = 0.5*0.5*0.5/6.0;
  const double tau4 = 0.5*0.5*0.5*0.5/24.0;
  std::vector<int> sz_;
  
  // State: Eigen::Vector<float>(dimension*order)
  // See heuristic here: https://github.com/sikang/motion_primitive_library/blob/master/include/mpl_planner/common/env_base.h
public:
  std::unique_ptr<nx::map_nd> MAP_ptr;
  
  env_fas( const std::vector<double>& start,
           const std::vector<double>& dimmin,
           const std::vector<double>& dimmax,
           const std::vector<double>& goal,
           const std::vector<double>& gtol,
           std::unique_ptr<nx::map_nd> MAP_ptr_in,
           uint16_t *cmap_ptr )
    : start_(start), dmin_(dimmin), dmax_(dimmax), goal_(goal), gtol_(gtol),
      cmap(cmap_ptr), sz_(start.size()+1), MAP_ptr( std::move(MAP_ptr_in) )
  {
    sz_[0] = std::ceil( (dimmax[0] - dimmin[0])/tau4 );
    sz_[1] = std::ceil( (dimmax[1] - dimmin[1])/tau4 );
    sz_[2] = std::ceil( (dimmax[2] - dimmin[2])/tau3 );
    sz_[3] = std::ceil( (dimmax[3] - dimmin[3])/tau3 );
    sz_[4] = std::ceil( (dimmax[4] - dimmin[4])/tau2 );
    sz_[5] = std::ceil( (dimmax[5] - dimmin[5])/tau2 );
    sz_[6] = std::ceil( (dimmax[6] - dimmin[6])/tau );
    sz_[7] = std::ceil( (dimmax[7] - dimmin[7])/tau );
    sz_[8] = 1000;
  }

  bool withinBounds( const std::vector<double>& x ) const
  {
    for( int k = 0; k < x.size(); ++k )
      if( x[k] <  dmin_[k] || x[k] > dmax_[k] )
        return false;
    return true; 
  }
  
  inline std::vector<double> state2metric(const std::vector<int>& state_coord) const
  {
    std::vector<double> x(start_.size());
    int dim = 2; // assuming 4th order
    int k = state_coord[4*dim];
    int k2 = k*k;
    int k3 = k2*k;
    // position
    for(int k = 0; k < dim; ++k)
      x[k] = tau4*state_coord[k] + start_[k] + tau*k*start_[k+dim] + tau2*k2*start_[k+2*dim] + tau3*k3*start_[k+3*dim];
    // velocity
    for(int k = dim; k < 2*dim; ++k)
      x[k] = tau3*state_coord[k] + start_[k] + tau*k*start_[k+dim] + tau2*k2*start_[k+2*dim];
    // acceleration
    for(int k = 2*dim; k < 3*dim; ++k)
      x[k] = tau2*state_coord[k] + start_[k] + tau*k*start_[k+dim];
    // jerk
    for(int k = 3*dim; k < 4*dim; ++k)
      x[k] = tau*state_coord[k] + start_[k];
    return x;
  }

  inline std::vector<int> move( const std::vector<int>& state_coord, int action_id ) const
  {
    std::vector<int> ns(state_coord.size());
    int dim = 2; // assuming 4th order
    // position
    for(int k = 0; k < dim; ++k)
      ns[k] = state_coord[k] + 4*state_coord[k+dim] + 6*state_coord[k+2*dim] + 4*state_coord[k+3*dim] + uset[action_id][k];
    // velocity
    for(int k = dim; k < 2*dim; ++k)
      ns[k] = state_coord[k] + 3*state_coord[k+dim] + 3*state_coord[k+2*dim] + uset[action_id][k-dim];
    // acceleration
    for(int k = 2*dim; k < 3*dim; ++k)
      ns[k] = state_coord[k] + 2*state_coord[k+dim] + uset[action_id][k-2*dim];
    // jerk
    for(int k = 3*dim; k < 4*dim; ++k)
      ns[k] = state_coord[k] +  uset[action_id][k-3*dim];
    // time step
    ns[4*dim] = state_coord[4*dim] + 1;
    return ns;
  }
      
  inline bool is_goal(const std::vector<int>& state_coord) const
  {
    std::vector<double> x = state2metric(state_coord);
    for( unsigned it = 0; it < x.size(); ++it )
      if( std::abs( x[it] - goal_[it] ) > gtol_[it] )
        return false;
    return true;
  }
  

  inline void get_succ( const std::vector<int>& state_coord, 
                        std::vector<std::vector<int>>& succ,
                        std::vector<int>& succ_idx,
                        std::vector<double>& succ_cost,
                        std::vector<int>& action_idx ) const
  {
    for( int k = 0; k < uset.size(); ++k )
    {
      std::vector<int> ns = move(state_coord,k);
      if( withinBounds(state2metric(ns)) )
      {
        succ.push_back(ns);
        succ_idx.push_back(state_to_idx(ns));
        succ_cost.push_back(tau);
        action_idx.push_back(k);
      }
    }
  }

  
  inline double get_heur(const std::vector<int>& state_coord) const
  {
    std::vector<double> x = state2metric(state_coord);
    return std::max( std::max((goal_[0]-x[0])/dmin_[0], (goal_[0]-x[0])/dmax_[0]),
                     std::max((goal_[1]-x[1])/dmin_[1], (goal_[1]-x[1])/dmax_[1]) );
  }

  inline void forward_action( const std::vector<int>& state_coord, int action_id, std::vector<std::vector<int>>& next_micro ) const
  {
    next_micro.push_back(move(state_coord,action_id));   
  }
  
  
  inline size_t state_to_idx(const std::vector<int>& state_coord) const
  {
    return nx::subv2ind_rowmajor( state_coord.begin(), state_coord.end(), sz_.begin(), sz_.end() );
  }
  
  
  std::size_t operator()(const std::vector<int>& v) const
  {
    if(v.end() <= v.begin() || sz_.end() <= sz_.begin()) return -1;
    size_t idx = *(v.end()-1); size_t prod = 1;
    std::vector<int>::const_iterator it1 = v.end()-2;
    std::vector<int>::const_iterator it2 = sz_.end()-1;
    for( ; it1 != (v.begin() - 1) && it2 != sz_.begin(); --it1, --it2 )
    {
      prod *= (*it2);
      idx += prod * (*it1);
    }
    return idx;

    //return subv2ind_rowmajor2(v.begin(),v.end(),sz.begin(),sz.end());
  }
  /**/
};

}

#endif
