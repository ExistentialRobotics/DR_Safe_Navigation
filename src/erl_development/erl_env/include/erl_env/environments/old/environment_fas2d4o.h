
#ifndef __ENV_FAS2D4O_H_
#define __ENV_FAS2D4O_H_

#include <erl_env/environments/environment_metric.h>
#include <erl_env/systems/fully_actuated_system.h>
#include <erl_map/grid_map.h>
#include <numeric> // std::inner_product

namespace erl
{
/**
 * @brief Implements the Fully Actuated System for 2-dimensional, 4th-order systems.
 */
class EnvironmentFAS2D4O: public EnvironmentMetric<Waypoint2D4O>
{
protected:
  const std::vector<std::array<double,2>> uset = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
  erl::GridMap<uint16_t> MAP; // TODO Give this a more descriptive name
  const double rho_ = 2.0; // determines the smoothness versus time optimality
  
public:
  FAS2D4O FAS_;

  /**
   * @brief Constructs the EnvironmentFAS according to the specified YAML input and timescale.
   * @param input_yaml Input YAML describing the environment.
   * @param tau Timescale discretization.
   */
  EnvironmentFAS2D4O( const std::string &input_yaml, double tau = 0.9) :
                      EnvironmentMetric(input_yaml), FAS_(tau, 70)
  {
    // Initialize Environment
    std::vector<double> dimmin = {map->min()[0], map->min()[1],-4,-4,-4,-4,-4,-4};
    std::vector<double> dimmax = {map->max()[0], map->max()[1],4,4,4,4,4,4};
    MAP = erl::GridMap<uint16_t>(dimmin,dimmax,{map->res()[0],map->res()[1],tau*tau*tau/6.0,tau*tau*tau/6.0,tau*tau/2.0,tau*tau/2.0,tau,tau});
  }
  
  
  /**
   * @brief Computes the successors of the state curr.
   * @param curr The current state to compute successors of.
   * @param succ The vector of successor states.
   * @param succ_cost The vector of costs of each successor.
   * @param action_idx The vector of indices of the actions leading to the successor.
   */
  void getSuccessors( const Waypoint2D4O& w,
                 std::vector<Waypoint2D4O>& succ,
                 std::vector<double>& succ_cost,
                 std::vector<int>& action_idx ) const override
  {
    for( size_t k = 0; k < uset.size(); ++k )
    {
      Waypoint2D4O nw;
      bool collision_free = true;
      for( size_t ti = 0; ti < FAS_.ntau(); ++ti )
      {
        // Compute next micro state
        nw = FAS_.next(w,uset[k],ti);
        
        // Check if within dynamics limits
        if( !withinBounds(nw) ) // TODO: this can be done in closed form with the trajectory polynomial!
        { collision_free = false; break; }
        
        // Convert to cells and check if map is occupied
        std::vector<int> datac(2);
        for( size_t k = 0; k < 2; ++k )
          datac[k] = erl::meters2cells( nw.coord[k], map->min()[k], map->res()[k] );
        if( cmap->at(map->getLinearIndex(datac)) > 0)
        { collision_free = false; break; }
      }
      
      // Add to successors if valid
      if( collision_free )
      {
        succ.push_back(nw);
        succ_cost.push_back((std::inner_product(uset[k].begin(), uset[k].end(), uset[k].begin(), 0.0) + rho_) * FAS_.tau());
        action_idx.push_back(k);
      }
    }
  }

  std::vector<Waypoint2D4O> forwardAction(const Waypoint2D4O &w, int action_id) const override {
    return FAS_.next_micro(w, uset[action_id]);
  }

  /**
   * Returns the integer (cell) representation of the metric state.
   * @param metric The metric state.
   * @return The cell state.
   */
  Waypoint2D4O toState(const std::vector<double> & metric) const override
  { 
    Waypoint2D4O wp;
    std::copy_n(metric.begin(), wp.coord.size(), wp.coord.begin()); 
    return wp;
  }

  /**
   * Returns the metric representation of the state.
   * @param cell The cell state.
   * @return The metric state.
   */
  std::vector<double> toMetric(const Waypoint2D4O &wp) const override
  {
    return std::vector<double>(wp.coord.begin(), wp.coord.end());
  }

 private:
  /**
   * Checks if the curreny waypoint w is within the boundaries of the map.
   * @param w The waypoint to be checked.
   * @return True if the waypoint is within the boundaries.
   */
  bool withinBounds(const Waypoint2D4O &w) const
  {
    for( size_t k = 0; k < w.coord.size(); ++k )
      if( w.coord[k] <  MAP.min()[k] || w.coord[k] > MAP.max()[k] )
        return false;
    return true;
  }

};

}

#endif
