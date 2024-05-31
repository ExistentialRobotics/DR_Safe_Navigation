#include <erl_astar/environments/planning_2d.h>
#include <erl_astar/astar_nx.h>
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
  erl::map_nd MAP_ptr(num_dim,map_min_ptr,map_max_ptr,res_ptr);
  
  // Define start and goal continuous coordinates
  std::vector<double> start(start_ptr,start_ptr+num_dim);
  std::vector<double> goal(goal_ptr,goal_ptr+num_dim);
  
  // Initialize Environment
  erl::Environment2D env(std::make_shared<erl::map_nd>(MAP_ptr), std::make_shared<std::vector<uint16_t>>(std::vector<uint16_t>(cmap_ptr, cmap_ptr+ MAP_ptr.totalSize())));
  
  // Initialize planner
  std::vector<int> start_coord = MAP_ptr.meters2cells( start.begin(), start.end() );
  erl::Planning2D ENV(env, goal);
  erl::ARAStar<std::vector<int>> AA;
  

  // Plan path
  std::list<std::vector<int>> path;
  std::vector<int> action_idx;
  auto result = AA.Astar( start_coord, ENV);
  
  // Return path and cost
  size_t num_sta = result.path.size();
  plhs[0] = mxCreateDoubleMatrix( num_dim, num_sta, mxREAL );
  double *output = mxGetPr(plhs[0]);
  plhs[1] = mxCreateDoubleScalar( result.pcost );
  
  // TODO: No need to reverse path... But need to reverse actions!
  
  // Convert path to continuous coordinates
  std::vector<double> const & map_min = MAP_ptr.min();
  std::vector<double> const & map_res = MAP_ptr.res();
  {
  std::list<std::vector<int>>::reverse_iterator it = result.path.rbegin();
  for( unsigned s = 0; it != result.path.rend(); ++it,++s)
    for( unsigned d = 0; d < num_dim; ++d )
    {
      output[s*num_dim+d] = erl::cells2meters( (*it)[d], map_min[d], map_res[d]);
    }
  }
  
  // Assign motion primitive ids
  size_t num_u = result.action_idx.size();
  plhs[2] = mxCreateDoubleMatrix( num_u, 1, mxREAL );
  double *tr = mxGetPr(plhs[2]);
  plhs[3] = mxCreateDoubleMatrix( num_u, 1, mxREAL );
  double *len = mxGetPr(plhs[3]);
  
  size_t len_traj = 1; // TODO: Generalize this
  //size_t len_traj_fine = mxGetDimensions(prhs[6])[1];
  for( unsigned u = 0; u < num_u; ++u )
  {
    tr[u] = static_cast<double>(result.action_idx[num_u-1 - u]/len_traj)+1;  // Correct for MATLAB INDEXING
    len[u] = static_cast<double>(result.action_idx[num_u-1 - u]%len_traj)+1; // Correct for MATLAB INDEXING
  }
    
}

