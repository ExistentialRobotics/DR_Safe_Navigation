/*
 * To compile:
 *   mex CXXFLAGS='\$CXXFLAGS -std=c++11' jps_3D_mex.cpp jps_3D.cpp
 */

#include <stdlib.h>
#include <erl_astar/jps_3D.h>
#include <mex.h>

/*
 * @INPUT:
 *  prhs[0] = 3 x 1 = start state in discrete coordinates
 *  prhs[1] = 3 x 1 = goal state in discrete coordinates
 *  prhs[2] = xDim x yDim x zDim = collision map
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
  char *cmap = (char *)mxGetData(prhs[2]);
  size_t xDim = mxGetDimensions(prhs[2])[0];
  size_t yDim = mxGetDimensions(prhs[2])[1];
  size_t zDim = mxGetDimensions(prhs[2])[2];
  double eps = mxGetScalar(prhs[3]);

  char val_free = 0;
  std::list<std::array<int,3>> xyzPath;
  double path_cost;

  erl::JPS_3D JA(cmap, val_free, xDim, yDim, zDim);
  path_cost = JA.plan( start_ptr[0]-1, start_ptr[1]-1, start_ptr[2]-1,
                       goal_ptr[0]-1,  goal_ptr[1]-1, goal_ptr[2]-1, xyzPath, eps);  
  
  // Return path and cost
  size_t num_sta = xyzPath.size();
  plhs[0] = mxCreateDoubleMatrix( 3, num_sta, mxREAL );
  double *output = mxGetPr(plhs[0]);
  plhs[1] = mxCreateDoubleScalar( path_cost );
  int s = 0;
  for( std::list<std::array<int,3>>::const_iterator it = xyzPath.begin();
       it != xyzPath.end(); ++it, ++s )
  {
    output[s*3+0] = static_cast<double>((*it)[0])+1; // Correct for MATLAB label
    output[s*3+1] = static_cast<double>((*it)[1])+1; // Correct for MATLAB label
    output[s*3+2] = static_cast<double>((*it)[2])+1; // Correct for MATLAB label
  }
}



