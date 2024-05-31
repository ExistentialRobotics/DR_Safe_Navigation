#include "environments/env_SE2.h"
#include "planners/astar_nx.h"
#include <iostream>
#include "mex.h"

/*
 * @INPUT:
 *  prhs[0] = num_dim x 1 = start state in continuous coordinates
 *  prhs[1] = num_dim x 1 = goal state in continuous coordinates
 *  prhs[2] = num_dim x 1 = lower bound on continuous coordinates
 *  prhs[3] = num_dim x 1 = upper bound on continuous coordinates
 *  prhs[4] = num_dim x 1 OR 1x1 = discretization resolution
 *  prhs[5] = collision checking map
 *
 * @OUTPUT:
 *  plhs[0] = num_dim x num_sta = path
 *  plhs[1] = 1 x 1 = path cost
 *  plhs[2] = num_u x 1 = traj_id
 *  plhs[3] = num_u x 1 = traj_len
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // Process inputs
  size_t num_dim = mxGetM(prhs[0]); // number of dimensions
  double *start_ptr = mxGetPr(prhs[0]);
  double *goal_ptr = mxGetPr(prhs[1]);
  double *map_min_ptr = mxGetPr(prhs[2]);
  double *map_max_ptr = mxGetPr(prhs[3]);
  double *res_ptr = mxGetPr(prhs[4]);
  uint16_t *cmap_ptr = (uint16_t *)mxGetPr(prhs[5]);
  
  // Initialize MAP
  std::unique_ptr<erl::map_nd> MAP_ptr( new erl::map_nd(num_dim,map_min_ptr,map_max_ptr,res_ptr));
  
  // Define start and goal continuous coordinates
  std::vector<double> start(start_ptr,start_ptr+num_dim);
  std::vector<double> goal(goal_ptr,goal_ptr+num_dim);

  // Goal tolerance
  std::vector<double> goal_tol(3);
  for( int k = 0; k < goal_tol.size(); ++k ) 
    goal_tol[k] = 2*MAP_ptr->res()[k];

  // Initialize Environment
  // erl::env_SE2 ENV( goal, std::move(MAP_ptr), cmap_ptr, {3,1}, {0,-1,1,-2,2});
  std::vector<double> vlist({3,3,3,3,3,1,1,1,1,1});
  std::vector<double> wlist({0,-1,1,-2,2,0,-1,1,-2,2});
  std::vector<double> tlist({1,1,1,1,1,1,1,1,1,1});
  erl::env_SE2 ENV( goal, goal_tol, std::move(MAP_ptr), cmap_ptr, vlist, wlist, tlist );


  // Initialize planner
  std::vector<int> start_coord = ENV.MAP_ptr->meters2cells( start.begin(), start.end() );
  int start_idx = ENV.state_to_idx( start_coord );
  erl::ARAStar<std::vector<int>> AA;
  
  
  // Plan path
  std::list<std::vector<int>> path;
  std::vector<int> action_idx;
  double pcost = std::numeric_limits<double>::infinity();
  pcost = AA.Astar( start_coord, start_idx, ENV, path, action_idx, 2 );    
  
  // Return path and cost
  size_t num_sta = path.size();
  plhs[0] = mxCreateDoubleMatrix( num_dim, num_sta, mxREAL );
  double *output = mxGetPr(plhs[0]);
  plhs[1] = mxCreateDoubleScalar( pcost );
  
  // TODO: No need to reverse path... But need to reverse actions!
  
  // Convert path to continuous coordinates
  std::vector<double> const & map_min = ENV.MAP_ptr->min();
  std::vector<double> const & map_res = ENV.MAP_ptr->res();
  {
  std::list<std::vector<int>>::reverse_iterator it = path.rbegin();
  for( unsigned s = 0; it != path.rend(); ++it,++s)
    for( unsigned d = 0; d < num_dim; ++d )
    {
      output[s*num_dim+d] = erl::cells2meters( (*it)[d], map_min[d], map_res[d]);
    }
  }
  
  // Assign motion primitive ids
  size_t num_u = action_idx.size();
  plhs[2] = mxCreateDoubleMatrix( num_u, 1, mxREAL );
  double *tr = mxGetPr(plhs[2]);
  plhs[3] = mxCreateDoubleMatrix( num_u, 1, mxREAL );
  double *len = mxGetPr(plhs[3]);
  
  size_t len_traj = 1; // TODO: Generalize this
  //size_t len_traj_fine = mxGetDimensions(prhs[6])[1];
  for( unsigned u = 0; u < num_u; ++u )
  {
    tr[u] = static_cast<double>(action_idx[num_u-1 - u]/len_traj)+1;  // Correct for MATLAB INDEXING
    len[u] = static_cast<double>(action_idx[num_u-1 - u]%len_traj)+1; // Correct for MATLAB INDEXING
  }
    
}

