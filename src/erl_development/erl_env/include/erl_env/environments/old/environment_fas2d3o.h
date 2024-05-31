#ifndef __ENV_FAS2D3O_H_
#define __ENV_FAS2D3O_H_

#include <erl_env/environments/environment_base.h>
#include <erl_env/systems/fully_actuated_system.h>
#include <erl_map/grid_map.h>
#include <erl_utilities/erl_poly_utils.h>
#include <numeric> // std::inner_product

namespace erl
{
class env_FAS2D3O : public env_int<Waypoint2D3O>
{
protected:
  Waypoint2D3O start_;       // metric coordinates of the start node
  std::vector<double> goal_; // metric coordinates of the goal node
  std::vector<double> gtol_; // tolerance for reaching the goal  
  uint16_t *cmap;            // cost map
  const std::vector<std::array<double,2>> uset = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
  erl::GridMap<uint16_t> MAP;
  const double rho_ = 2.0; // determines the smoothness versus time optimality
  const double sqrt_rho_ = std::sqrt(rho_);
  
public:
  FAS2D3O FAS;
  std::shared_ptr<erl::GridMap<uint16_t>> MAP3D_ptr;
  
  env_FAS2D3O( const std::vector<double>& start,
               const std::vector<double>& dimmin,
               const std::vector<double>& dimmax,
               const std::vector<double>& goal,
               const std::vector<double>& gtol,
               const std::shared_ptr<erl::GridMap<uint16_t>>& MAP3D_ptr_in,
               uint16_t *cmap_ptr,
               double tau = 1.0 )
    : goal_(goal), gtol_(gtol), cmap(cmap_ptr),
      MAP(dimmin,dimmax,{MAP3D_ptr_in->res()[0],MAP3D_ptr_in->res()[1],tau*tau/2.0,tau*tau/2.0,tau,tau}),
      FAS(tau,10), MAP3D_ptr(MAP3D_ptr_in)
    {
      std::copy_n(start.begin(), start_.coord.size(), start_.coord.begin()); // convert start to a waypoint
    }
    
  bool withinBounds(const Waypoint2D3O& w) const
  {
    for( size_t k = 0; k < w.coord.size(); ++k )
      if( w.coord[k] <  MAP.min()[k] || w.coord[k] > MAP.max()[k] )
        return false;
    return true;
  } 
  
  bool is_goal(const Waypoint2D3O& w) const
  {
    for( size_t it = 0; it < w.coord.size(); ++it )
      if( std::abs( w.coord[it] - goal_[it] ) > gtol_[it] )
        return false;
    return true;
  }

  void get_succ( const Waypoint2D3O& w, 
                 std::vector<Waypoint2D3O>& succ,
                 std::vector<double>& succ_cost,
                 std::vector<int>& action_idx ) const
  {
    // Compute the cell coordinates of the initial state
    int wcx = erl::meters2cells( w.coord[0], MAP3D_ptr->min()[0], MAP3D_ptr->res()[0] );
    int wcy = erl::meters2cells( w.coord[1], MAP3D_ptr->min()[1], MAP3D_ptr->res()[1] );
    
    for( size_t k = 0; k < uset.size(); ++k )
    {
      Waypoint2D3O nw;
      double sx = w.coord[0]; int sxc = wcx;
      double sy = w.coord[1]; int syc = wcy;
      bool collision_free = true;
      for( size_t ti = 0; ti < FAS.ntau(); ++ti )
      {
        // Compute next micro state
        nw = FAS.next(w,uset[k],ti);
        
        // Check if within dynamics limits
        if( !withinBounds(nw) )
        { collision_free = false; break; }
        
        // Convert to cells
        int exc = erl::meters2cells( nw.coord[0], MAP3D_ptr->min()[0], MAP3D_ptr->res()[0] );
        int eyc = erl::meters2cells( nw.coord[1], MAP3D_ptr->min()[1], MAP3D_ptr->res()[1] );
        
        // Find cells from pw to nw using Bressenham
        double dx = nw.coord[0] - sx;
        double dy = nw.coord[1] - sy;
        int stepX, stepY;          // direction of grid traversal
        double tDeltaX, tDeltaY; // parametric step size along different dimensions
        double tMaxX, tMaxY; // used to determine the dimension of the next step along the line
        erl::bresenhamStep( dx, sx, sxc, MAP3D_ptr->min()[0], MAP3D_ptr->res()[0], stepX, tDeltaX, tMaxX );
        erl::bresenhamStep( dy, sy, syc, MAP3D_ptr->min()[1], MAP3D_ptr->res()[1], stepY, tDeltaY, tMaxY );
        while ((sxc!=exc)||(syc!=eyc))
        {
          if (tMaxX<tMaxY)
          {
            sxc += stepX;
            tMaxX += tDeltaX;
          }
          else
          {
            syc += stepY;
            tMaxY += tDeltaY;
          }
          // check for collisions
          if( cmap[sxc + syc* MAP3D_ptr->size()[0]] > 0 )
          { collision_free = false; break; }
        }
        if(!collision_free) break;
        sx = nw.coord[0]; sy = nw.coord[1];
      }

      // Add to successors if valid
      if( collision_free )
      {
        succ.push_back(nw);
        succ_cost.push_back((std::inner_product(uset[k].begin(), uset[k].end(), uset[k].begin(), 0.0) + rho_) * FAS.tau());
        action_idx.push_back(k);
      }
    }
  }
  

  // See heuristic here: https://github.com/sikang/motion_primitive_library/blob/master/include/mpl_planner/common/env_base.h
  double get_heur(const Waypoint2D3O& w) const
  {
    double t_bar = std::max( std::max((goal_[0]-w.coord[0])/MAP.min()[2], (goal_[0]-w.coord[0])/MAP.max()[2]),
                             std::max((goal_[1]-w.coord[1])/MAP.min()[3], (goal_[1]-w.coord[1])/MAP.max()[3]) );
    return t_bar * rho_;
    //return get_heur2D3O(w,t_bar);
  }

  std::vector<Waypoint2D3O> forward_action( const Waypoint2D3O& w, int action_id) const
  { return FAS.next_micro(w,uset[action_id]); }
  
  std::size_t state_to_idx(const Waypoint2D3O& w) const
  {
    std::vector<int> datac(w.coord.size());
    for( unsigned k = 0; k < w.coord.size(); ++k )
      datac[k] = erl::meters2cells( w.coord[k], MAP.min()[k], MAP.res()[k] );
    return MAP.subv2ind_rowmajor(datac);
  }
  
private:
  double get_heur2D3O(const Waypoint2D3O& w, double t_bar) const
  {
    double a = rho_;
    double b = 0;
    double c = -9*(w.coord[4]*w.coord[4]+w.coord[5]*w.coord[5])
               +6*(w.coord[4]*goal_[4]+w.coord[5]*goal_[5])
               -9*(goal_[4]*goal_[4]+goal_[5]*goal_[5]);
    double d = -144*(w.coord[4]*w.coord[2]+w.coord[5]*w.coord[3])
               -96*(w.coord[4]*goal_[2]+w.coord[5]*goal_[3])
               +96*(goal_[4]*w.coord[2]+goal_[5]*w.coord[3])
               +144*(goal_[4]*goal_[2]+goal_[5]*goal_[3]);
    double e = 360*( (goal_[0]-w.coord[0])*(w.coord[4]-goal_[4]) + (goal_[1]-w.coord[1])*(w.coord[5]-goal_[5]) )
               -576*(w.coord[2]*w.coord[2]+w.coord[3]*w.coord[3])
               -1008*(w.coord[2]*goal_[2]+w.coord[3]*goal_[3])
               -576*(goal_[2]*goal_[2]+goal_[3]*goal_[3]);
    double f = 2880*( (goal_[0]-w.coord[0])*(goal_[2]+w.coord[2]) + (goal_[1]-w.coord[1])*(goal_[3]+w.coord[3]) );
    double g = -3600*( (goal_[0]-w.coord[0])*(goal_[0]-w.coord[0]) + (goal_[1]-w.coord[1])*(goal_[1]-w.coord[1]) );

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
  
  double get_heur2D2O(const Waypoint2D2O& w, double t_bar) const
  {
    double a = rho_;
    double b = 0;
    double c = -4*( (w.coord[2]*w.coord[2]+w.coord[3]*w.coord[3])
                  + (w.coord[2]*goal_[2]+w.coord[3]*goal_[3])
                  + (goal_[2]*goal_[2]+goal_[3]*goal_[3] ));
    double d = 24*( (goal_[0]-w.coord[0])*(goal_[2]+w.coord[2]) + (goal_[1]-w.coord[1])*(goal_[3]-w.coord[3]) );
    double e = -36*( (goal_[0]-w.coord[0])*(goal_[0]-w.coord[0]) + (goal_[1]-w.coord[1])*(goal_[1]-w.coord[1]) );
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
  
  double get_heur2D1O(const Waypoint2D2O& w, double t_bar) const
  {
    double norm_dp = std::sqrt((goal_[0]-w.coord[0])*(goal_[0]-w.coord[0]) + (goal_[1]-w.coord[1])*(goal_[1]-w.coord[1]));
    if( norm_dp/sqrt_rho_ > t_bar )
      return 2*norm_dp*sqrt_rho_;
    else
      return rho*t_bar + norm_dp*norm_dp/t_bar;
  }
  //{ return (rho_ + 1)*(goal_[0]-w.coord[0])*(goal_[0]-w.coord[0]); } // TODO: check this?? 
  
};

}

#endif
