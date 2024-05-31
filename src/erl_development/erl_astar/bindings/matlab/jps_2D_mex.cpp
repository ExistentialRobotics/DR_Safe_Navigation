/*
 * To compile:
 *   mex CXXFLAGS='\$CXXFLAGS -std=c++11' jps_2D_mex.cpp jps_2D.cpp
 */

#include <stdlib.h>
#include <erl_astar/jps_2D.h>
#include <mex.h>


/*
 * @INPUT:
 *  prhs[0] = 2 x 1 = start state in discrete coordinates
 *  prhs[1] = 2 x 1 = goal state in discrete coordinates
 *  prhs[2] = xDim x yDim = collision map
 *  prhs[3] = 1 x 1 = planning epsilon (heuristic inflation)
 *
 * @OUTPUT:
 *  plhs[0] = num_dim x num_sta = path
 *  plhs[1] = 1 x 1 = path cost
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  double *start_ptr = mxGetPr(prhs[0]);
  double *goal_ptr = mxGetPr(prhs[1]);
  char *cmap = (char *)mxGetPr(prhs[2]);
  size_t xDim = mxGetM(prhs[2]);
  size_t yDim = mxGetN(prhs[2]);
  double eps = mxGetScalar(prhs[3]);
  
  char val_free = 0;
  std::list<std::array<int,2>> xyPath;
  double path_cost;
  
  erl::JPS_2D JA(cmap, val_free, xDim, yDim);
  path_cost = JA.plan(start_ptr[0]-1, start_ptr[1]-1,
                      goal_ptr[0]-1,  goal_ptr[1]-1, xyPath, eps);    

  // Return path and cost
  size_t num_sta = xyPath.size();
  plhs[0] = mxCreateDoubleMatrix( 2, num_sta, mxREAL );
  double *output = mxGetPr(plhs[0]);
  plhs[1] = mxCreateDoubleScalar( path_cost );
  int s = 0;
  for( std::list<std::array<int,2>>::const_iterator it = xyPath.begin();
       it != xyPath.end(); ++it, ++s )
  {
    output[s*2+0] = static_cast<double>((*it)[0])+1; // Correct for MATLAB label
    output[s*2+1] = static_cast<double>((*it)[1])+1; // Correct for MATLAB label
  }
}


