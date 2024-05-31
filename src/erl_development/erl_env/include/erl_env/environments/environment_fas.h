
#ifndef __ENV_FAS_H_
#define __ENV_FAS_H_

#include <erl_env/environments/environment_metric.h>
#include <erl_env/systems/fully_actuated_system.h>
#include <erl_map/grid_map.h>
#include <erl_utilities/trajectories/piecewise_polynomial.h>
#include <numeric> // std::inner_product


namespace erl
{
/**
 * @brief Implements a D-dimensional N-order Fully Actuated System
 */
template <std::size_t D, std::size_t N, class U>
class EnvironmentFAS: public EnvironmentMetric<Waypoint<D,N>, U> {
 protected:
  FullyActuatedSystem<D,N> FAS;
  std::vector<std::array<double,D>> uset;
  std::shared_ptr<erl::GridMap<U>> map_DD_ptr;
  double rho_;
  
public:
  using EnvironmentMetric<Waypoint<D,N>, U>::map; // Bring Parent variables into the namespace.

  /**
   * @brief Constructs the EnvironmentFAS with the following parameters.
   * @param map The map of the environment.
   * @param dimmin The minimal dimensions of the state space. (Velocity, acceleration).
   * @param dimmax The maximal dimensions of the state space. (Velocity, acceleration).
   * @param tau The time discretization parameter.
   * @param rho The relative weight of the heuristic.
   */
  EnvironmentFAS( const std::shared_ptr<erl::GridMap<U>> &map,
                  const std::vector<double>& dimmin,
                  const std::vector<double>& dimmax,
                  double tau = 1.0, double rho = 2.0 )
    : EnvironmentMetric<Waypoint<D,N>, U>(map), FAS(tau,10), uset(std::pow(3,D)-1), rho_(rho)
  {
    // Set the controls to [-1,-1,-1; -1, 0, -1; ...; 1 1 1]
    for( std::size_t k = 0; k < uset.size(); ++k )
    {
      int val = k < std::floor((uset.size()+1)/2) ? k : k+1; // skip the 0 control
      for( std::size_t d = 0; d < D; ++d )
      {
        uset[k][d] = val%3-1;
        val -= uset[k][d];
        val /= 3;
      }
    }

    // Set the map resolution as follows
    // [map->res(), tau^(N-1)/(N-1)!,...,tau^2/2, tau]
    std::vector<double> DDRes(D*N);
    for( std::size_t d = 0; d < D; ++d )
      DDRes[d] = map->res()[d];
    for( std::size_t d = 0; d < D; ++d )
      DDRes[D*(N-1) + d] = tau;
    for( size_t n = (N-2); n >= 1; --n )
      for( std::size_t d = 0; d < D; ++d )
        DDRes[D*n + d] = tau * DDRes[D*(n+1) + d] / (N-n);
    map_DD_ptr.reset( new erl::GridMap<U>(dimmin,dimmax,DDRes) );
  }

  /**
   * @brief Initialize the EnvironmentFAS with regular map and cost-map. Internally converted to shared pointer.
   */
  EnvironmentFAS( const erl::GridMap<U> &map,
                  const std::vector<double>& dimmin,
                  const std::vector<double>& dimmax,
                  double tau = 1.0, double rho = 2.0 )
      : EnvironmentFAS(std::make_shared<erl::GridMap<U>>(map),
          dimmin, dimmax, tau, rho) {}

  /**
   * Checks if the curreny waypoint w is within the boundaries of the map.
   * @param w The waypoint to be checked.
   * @return True if the waypoint is within the boundaries.
   */  
  bool withinBounds(const Waypoint<D,N>& w) const
  {
    for( size_t k = 0; k < w.coord.size(); ++k )
      if( w.coord[k] <  map_DD_ptr->min()[k] || w.coord[k] > map_DD_ptr->max()[k] )
        return false;
    return true;
  }
  
  void getSuccessors( const Waypoint<D,N>& curr, 
                      std::vector<Waypoint<D,N>>& succ,
                      std::vector<double>& succ_cost,
                      std::vector<int>& action_idx ) const override
  {
    // Compute the cell coordinates of the initial state
    std::array<int,D> curr_cell;
    for( std::size_t d = 0; d < D; ++d )
      curr_cell[d] = erl::meters2cells( curr.coord[d], map->min()[d], map->res()[d] );

    for( std::size_t k = 0; k < uset.size(); ++k )
    {
      Waypoint<D,N> nw;
      std::array<double,D> start_coord;
      for( std::size_t d = 0; d < D; ++d )
        start_coord[d] = curr.coord[d];
      std::array<int,D> start_cells(curr_cell);
      bool collision_free = true;
      for( std::size_t ti = 0; ti < FAS.ntau(); ++ti )
      {
        // Compute next micro state
        nw = FAS.next(curr,uset[k],ti);
        
        // Check if within dynamics limits
        if( !withinBounds(nw) )
        { collision_free = false; break; }
        
        // Convert to cells
        std::array<int,D> end_cells;
        for( std::size_t d = 0; d < D; ++d )
          end_cells[d] = erl::meters2cells( nw.coord[d], map->min()[d], map->res()[d] );

        // Find cells from pw to nw using Bressenham
        std::array<int,D> step;
        std::array<double,D> tDelta;
        std::array<double,D> tMax;
        for( std::size_t d = 0; d < D; ++d )
          erl::bresenhamStep( nw.coord[d] - start_coord[d], start_coord[d], start_cells[d],
              map->min()[d], map->res()[d], step[d], tDelta[d], tMax[d] );
        
        while( start_cells != end_cells )
        {
          auto min_idx = std::distance(tMax.begin(), std::min_element(tMax.begin(), tMax.end()));
          start_cells[min_idx] += step[min_idx];
          tMax[min_idx] += tDelta[min_idx];
          
          // Check for collisions
          if( map->map()[map->subv2ind(std::vector<int>(start_cells.begin(), start_cells.end()))] > 0 )
          { collision_free = false; break; }
        }
        if(!collision_free) break;
        for( std::size_t d = 0; d < D; ++d )
          start_coord[d] = nw.coord[d];
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

  std::vector<Waypoint<D,N>> forwardAction( const Waypoint<D,N>& w, int action_id) const override
  { return FAS.next_micro(w,uset[action_id]); }
  
  /**
   * Returns the integer (cell) representation of the metric state.
   * @param metric The metric state.
   * @return The cell state.
   */
  Waypoint<D,N> toState(const std::vector<double> & metric) const override
  { 
    Waypoint<D,N> wp;
    std::copy_n(metric.begin(), wp.coord.size(), wp.coord.begin()); 
    return wp;
  }

  /**
   * Returns the metric representation of the state.
   * @param cell The cell state.
   * @return The metric state.
   */
  std::vector<double> toMetric(const Waypoint<D,N> &wp) const override
  {
    return std::vector<double>(wp.coord.begin(), wp.coord.end());
  }
  
  /**
   * @brief Returns a PiecewisePolynomial from a single waypoint and action index (control input).
   * @param wp The waypoint.
   * @param action_idx The sequence of action indices.
   * @return The resulting PiecewisePolynomial.
   */
  trajectories::PiecewisePolynomial<double>
  toTrajectorySingle(const Waypoint<D,N> &wp, const std::list<int> &action_idx) const
  {
    std::list<Waypoint<D,N>> path;
    path.push_back(wp);
    for( auto it = action_idx.begin(); it != action_idx.end(); ++it )
      path.push_back(FAS.next(path.back(), uset[*it]));

    return toTrajectory(path, action_idx);
  }

  /**
   * @brief Generates the resulting PiecewisePolynomial from a sequence of waypoints and action indices.
   * @param path The input path.
   * @param action_idx The sequence of action indices.
   * @return The resulting PiecewisePolynomial.
   */
  trajectories::PiecewisePolynomial<double>
  toTrajectory(const std::list<Waypoint<D,N>> &path, const std::list<int> &action_idx) const
  {
    // Compute the segment times
    std::vector<double> segment_times(action_idx.size() + 1);
    segment_times[0] = 0.0;
    for( size_t k = 0; k < action_idx.size(); ++k )
      segment_times[k+1] = segment_times[k] + FAS.tau();
    
    // Compute the polynomial coefficients
    typedef Eigen::Matrix<trajectories::Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic> PolynomialVector;
    typedef std::vector<PolynomialVector> vectorOfPolynomialVectors;
    vectorOfPolynomialVectors polynomials(action_idx.size(), PolynomialVector(D, 1));
    Eigen::Matrix<double, N+1, 1> coeff;

    auto wpit = path.begin();
    auto actit = action_idx.begin();
    for( size_t k = 0; k < action_idx.size(); ++k, ++wpit, ++actit )
    {
      for( size_t d = 0; d < D; ++d )
      {
        int factorial = 1; // Compute Factorial Term for scaling the coefficients correctly.
        for( size_t n = 0; n < N; ++n ) {
          coeff[n] = (*wpit).coord[d + n * D] / factorial;
          factorial *= (n+1);
        }
        coeff[N] = uset[*actit][d] / factorial;
        polynomials[k](d) = trajectories::Polynomial<double>(coeff);
      }
    }
    return trajectories::PiecewisePolynomial<double>(polynomials, segment_times);
  }
  
};
}

#endif
