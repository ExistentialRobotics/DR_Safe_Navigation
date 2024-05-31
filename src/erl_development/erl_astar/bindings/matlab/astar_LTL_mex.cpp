#include <erl_astar/environments/env_LTL>
#include <erl_astar/astar_nx.h>
#include <mex.h>
#include <iostream>

std::unique_ptr<erl::FSA> init_spec1_fsa()
{
  std::unique_ptr<erl::FSA> spec1_ptr( new erl::FSA(3,0,{2},{"a","b"},{0,1,2,3}) );
  spec1_ptr->add_transition(0,0,{0});
  spec1_ptr->add_transition(0,1,{1,3});
  spec1_ptr->add_transition(0,2,{2});
  spec1_ptr->add_transition(2,2,{0,1,2,3});
  spec1_ptr->add_transition(3,3,{0,1,2,3});
  spec1_ptr->compute_levels();
  return spec1_ptr;  
}
std::unique_ptr<erl::FSA> init_spec2_fsa()
{
  std::unique_ptr<erl::FSA> spec2_ptr( new erl::FSA(5,0,{3},{"a","b","c","d"},
                                                  {0,1,2,3,4,5,6,7,8}));  
  spec2_ptr->add_transition(0,0,{0,2,4,6});
  spec2_ptr->add_transition(0,1,{1,3,5,7});
  spec2_ptr->add_transition(1,1,{1,5,0,4});
  spec2_ptr->add_transition(1,2,{2,3,6,7});
  spec2_ptr->add_transition(2,2,{0,1,2,3});
  spec2_ptr->add_transition(2,3,{4,5,6,7});
  spec2_ptr->add_transition(3,3,{0,1,2,3,4,5,6,7,8});
  spec2_ptr->add_transition(0,4,{8});
  spec2_ptr->add_transition(1,4,{8});
  spec2_ptr->add_transition(2,4,{8});
  spec2_ptr->add_transition(4,4,{0,1,2,3,4,5,6,7,8});
  spec2_ptr->compute_levels();
  return spec2_ptr;
}
std::unique_ptr<erl::FSA> init_spec3_fsa()
{
  std::unique_ptr<erl::FSA> spec3_ptr( new erl::FSA(7,0,{5},{"phi1","phi2","phi3","phi4","phi5"},
                                                          {31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0}));
  spec3_ptr->add_transition(0,0,{0,12,8,4});
  spec3_ptr->add_transition(0,1,{16});
  spec3_ptr->add_transition(0,2,{18});
  spec3_ptr->add_transition(0,3,{14,10,6,2});
  spec3_ptr->add_transition(0,4,{20,24,28});
  spec3_ptr->add_transition(0,5,{22,26,30});
  spec3_ptr->add_transition(1,1,{16,10});
  spec3_ptr->add_transition(1,2,{18,2});
  spec3_ptr->add_transition(1,4,{4,8,12,20,24,28});
  spec3_ptr->add_transition(1,5,{6,10,14,22,26,30});
  spec3_ptr->add_transition(2,2,{18,16,2,0});
  spec3_ptr->add_transition(2,5,{4,6,8,10,12,14,20,22,24,26,28,30});
  spec3_ptr->add_transition(3,2,{18,16});
  spec3_ptr->add_transition(3,3,{14,12,10,8,6,4,2,0});
  spec3_ptr->add_transition(3,5,{20,22,24,26,28,30});
  spec3_ptr->add_transition(4,4,{28,24,20,16,12,8,4,0});
  spec3_ptr->add_transition(4,5,{30,26,22,18,14,10,6,2});
  spec3_ptr->add_transition(5,5,{30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0});
  spec3_ptr->add_transition(0,6,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec3_ptr->add_transition(1,6,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});    
  spec3_ptr->add_transition(2,6,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec3_ptr->add_transition(3,6,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});  
  spec3_ptr->add_transition(4,6,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec3_ptr->add_transition(5,6,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec3_ptr->add_transition(6,6,{31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0});
  spec3_ptr->compute_levels();
  return spec3_ptr;
}
std::unique_ptr<erl::FSA> init_spec4_fsa()
{
  std::unique_ptr<erl::FSA> spec4_ptr( new erl::FSA(9,0,{7},{"phi1","phi2","phi3","phi4","phi5"}, 
            {31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0}));
  spec4_ptr->add_transition(0,0,{12,8,4,0});
  spec4_ptr->add_transition(0,1,{14,10,6,2});
  spec4_ptr->add_transition(0,2,{20,16});
  spec4_ptr->add_transition(0,3,{22,18});
  spec4_ptr->add_transition(0,4,{24});
  spec4_ptr->add_transition(0,5,{26});
  spec4_ptr->add_transition(0,6,{28});
  spec4_ptr->add_transition(0,7,{30});
  spec4_ptr->add_transition(0,8,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec4_ptr->add_transition(1,1,{14,12,10,8,6,4,2,0});
  spec4_ptr->add_transition(1,3,{22,20,18,16});
  spec4_ptr->add_transition(1,5,{26,24});
  spec4_ptr->add_transition(1,7,{30,28});
  spec4_ptr->add_transition(1,8,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec4_ptr->add_transition(2,2,{20,16,4,0});
  spec4_ptr->add_transition(2,3,{22,18,6,2});
  spec4_ptr->add_transition(2,4,{24,8});
  spec4_ptr->add_transition(2,5,{26,10});
  spec4_ptr->add_transition(2,6,{28,12});
  spec4_ptr->add_transition(2,8,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec4_ptr->add_transition(3,3,{22,20,18,16,6,4,2,0});
  spec4_ptr->add_transition(3,5,{26,24,10,8});
  spec4_ptr->add_transition(3,7,{30,28,14,12});
  spec4_ptr->add_transition(3,8,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec4_ptr->add_transition(4,4,{24,16,8,0});
  spec4_ptr->add_transition(4,5,{26,18,10,2});
  spec4_ptr->add_transition(4,6,{28,20,12,4});
  spec4_ptr->add_transition(4,7,{30,22,14,6});
  spec4_ptr->add_transition(4,8,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec4_ptr->add_transition(5,5,{26,24,18,16,10,8,2,0});
  spec4_ptr->add_transition(5,7,{30,28,22,20,14,12,6,4});
  spec4_ptr->add_transition(5,8,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec4_ptr->add_transition(6,6,{28,24,20,16,12,8,4,0});
  spec4_ptr->add_transition(6,7,{30,26,22,18,14,10,6,2});
  spec4_ptr->add_transition(6,8,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec4_ptr->add_transition(7,7,{30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0});
  spec4_ptr->add_transition(7,8,{31,29,27,25,23,21,19,17,15,13,11,9,7,5,3,1});
  spec4_ptr->add_transition(8,8,{31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0});
  spec4_ptr->compute_levels();
  return spec4_ptr;
}



/*
 * @INPUT:
 *  prhs[0] = num_dim x 1 = start state in continuous coordinates
 *  prhs[1] = num_dim x 1 = lower bound on continuous coordinates
 *  prhs[2] = num_dim x 1 = upper bound on continuous coordinates
 *  prhs[3] = num_dim x 1 OR 1x1 = discretization resolution
 *
 *  prhs[4] = label map
 *  prhs[5] = inverse label map = cell( num_possible_labels, 1 ) 
 *          = (x,y) coordinates which satisfy a given label
 *  prhs[6] = dd_mprim.traj_fine   = 3 x len_traj_fine x num_traj
 *  prhs[7] = dd_mprim.u           = 2 x len_traj x num_traj
 *  prhs[8] = dd_mprim.samp        = 1 x 1 double
 *  prhs[9] = fsa.ID               = 1 x 1 double = id of fsa spec to use
 *  prhs[10] = label_cost2go       = num_lab x fsa.num_sta
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
  double *map_min_ptr = mxGetPr(prhs[1]);
  double *map_max_ptr = mxGetPr(prhs[2]);
  double *res_ptr = mxGetPr(prhs[3]);
  
  size_t lmap_row = mxGetM(prhs[4]);
  size_t lmap_col = mxGetN(prhs[4]);
  uint16_t *lmap_ptr = (uint16_t *)mxGetPr(prhs[4]);
  int fsa_ID = static_cast<int>( mxGetScalar(prhs[9]) );
  
  // Initialize FSA
  std::unique_ptr<erl::FSA> fsa_ptr;
  switch(fsa_ID){
    case 1:
      fsa_ptr = init_spec1_fsa();
      break;
    case 2:
      fsa_ptr = init_spec2_fsa();
      break;      
    case 3:
      fsa_ptr = init_spec3_fsa();
      break;   
    case 4:
      fsa_ptr = init_spec4_fsa();
      break;
    default:
      mexErrMsgTxt("No such automaton!");
  }
  
  // Initialize Environment
  erl::ENV_LTL ENV( num_dim, map_min_ptr, map_max_ptr, res_ptr, lmap_ptr, 
                   prhs[5], prhs[6], prhs[7], mxGetScalar(prhs[8]), 
                   std::move(fsa_ptr), prhs[10] );
  
  // Get start coordinates
  std::vector<double> start(start_ptr,start_ptr+num_dim);
  std::vector<int> start_coord = ENV.MAP.meters2cells( start.begin(), start.end()-1);
  start_coord.push_back( start.back() );
  std::vector<int> sz( ENV.MAP.size() ); sz.push_back( ENV.spec->num_sta() );
  int start_idx = erl::subv2ind_colmajor( start_coord.begin(), start_coord.end(),
                                         sz.begin(), sz.end() );
                                         
  plhs[4] = mxCreateDoubleMatrix( 0, 0, mxREAL );

  /*
  // Get successors (good for testing)
  std::vector<std::vector<int>> succ_coord;
  std::vector<int> succ_idx;
  std::vector<double> succ_cost;
  std::vector<int> succ_act_idx;
  ENV.get_succ( start_coord, succ_coord, succ_idx, succ_cost, succ_act_idx );
  std::cout << "succ_coord.size()=" << succ_coord.size() << std::endl;
  */
   
  // Initialize Planner;
  std::list<std::vector<int>> path;
  std::vector<int> action_idx;
  erl::ARAStar<std::vector<int>> AA;  
  double eps = 2;
  double path_cost = AA.Astar(start_coord, start_idx, ENV, path, action_idx, eps );
  
  // Return path and cost
  size_t num_sta = path.size();
  plhs[0] = mxCreateDoubleMatrix( num_dim, num_sta, mxREAL );
  double *output = mxGetPr(plhs[0]);
  plhs[1] = mxCreateDoubleScalar( path_cost );
  
  // Convert path to continuous coordinates
  std::vector<double> const & map_min = ENV.MAP.min();
  std::vector<double> const & map_res = ENV.MAP.res();
  int s = 0;
  for( std::list<std::vector<int>>::const_iterator it = path.begin();
       it != path.end(); ++it, ++s )
    for( unsigned d = 0; d < num_dim; ++d )
    {
      if( d != (num_dim - 1) ) 
        output[s*num_dim+d] = erl::cells2meters((*it)[d], map_min[d], map_res[d]);
      else
        output[s*num_dim+d] = static_cast<double>((*it)[d])+1; // Correct for MATLAB label
    }

    
  // Assign motion primitive ids
  size_t num_u = action_idx.size();
  //mexPrintf("num_u = %d\n",num_u);
  plhs[2] = mxCreateDoubleMatrix( num_u, 1, mxREAL );
  double *tr = mxGetPr(plhs[2]);
  plhs[3] = mxCreateDoubleMatrix( num_u, 1, mxREAL );
  double *len = mxGetPr(plhs[3]);
  
  size_t len_traj = mxGetDimensions(prhs[7])[1];
  //size_t len_traj_fine = mxGetDimensions(prhs[6])[1];
  for( unsigned u = 0; u < num_u; ++u )
  {
    tr[u] = static_cast<double>(action_idx[num_u-1 - u]/len_traj)+1;  // Correct for MATLAB INDEXING
    len[u] = static_cast<double>(action_idx[num_u-1 - u]%len_traj)+1; // Correct for MATLAB INDEXING
  }

}

  /*
  std::cout << "start_coord = " << start_coord[0] << " "
                                << start_coord[1] << " "
                                << start_coord[2] << " "
                                << start_coord[3] << std::endl;

  std::vector<unsigned int> curr_q_set;
  std::vector<unsigned int> next_q_set;
  ENV.spec->next_level_q(start_coord[3],curr_q_set,next_q_set);
  std::cout << "curr_q_set = ";
  for( unsigned k = 0; k < curr_q_set.size(); ++k )
    std::cout << curr_q_set[k] << " ";
  std::cout << std::endl;
  std::cout << "next_q_set = ";
  for( unsigned k = 0; k < next_q_set.size(); ++k )
    std::cout << next_q_set[k] << " ";
  std::cout << std::endl;  
  
  // labels that take us to next_q
  std::vector<unsigned int> labs;
  ENV.spec->trans_labels(start_coord[3], next_q_set[0], labs);
  std::cout << "labs = ";
  for( unsigned k = 0; k < labs.size(); ++k )
    std::cout << labs[k] << " ";
  std::cout << std::endl;    

  // Levels
  std::cout << "levels_ = " << std::endl;
  for( unsigned k = 0; k < ENV.spec->num_levels(); ++k ){
    for( unsigned j = 0; j < ENV.spec->levels_[k].size(); ++j )
      std::cout << ENV.spec->levels_[k][j] << " ";
    std::cout << std::endl;
  }
  
  ENV.get_heur({180,180,64,0});
  */
  /*
  // Compute Heuristic map
  plhs[4] = mxCreateDoubleMatrix( lmap_row, lmap_col, mxREAL );
  double *out = mxGetPr(plhs[4]);
  for( int r = 0; r < lmap_row; ++r )
    for( int c = 0; c < lmap_col; ++c )
      out[r + c*lmap_row] = ENV.get_heur({r,c,0,0});
  */
  
  /*
  // Display g
  double *tmp = mxGetPr(prhs[10]);
  for( size_t lb = 0; lb < mxGetM(prhs[10]); ++lb )
  {
    for( unsigned int nq = 0; nq < ENV.spec->num_sta(); ++nq )
    {
      //mexPrintf("ID = %d",lb + nq*mxGetM(prhs[10]));
      mexPrintf("%f ", tmp[lb + nq*mxGetM(prhs[10])]);
    }
    mexPrintf("\n");
  }
  */
  /*
  // Compute Heuristic map
  const mwSize dims[] = { ENV.MAP.size()[0],
                          ENV.MAP.size()[1],
                          ENV.MAP.size()[2],
                          static_cast<int>(ENV.spec->num_sta()) };
  plhs[4] = mxCreateNumericArray( 4, dims, mxDOUBLE_CLASS, mxREAL );
  double *out = mxGetPr(plhs[4]);
  for( int x = 0; x < dims[0]; ++x )
    for( int y = 0; y < dims[1]; ++y )
      for( int r = 0; r < dims[2]; ++r )
        for( int q = 0; q < dims[3]; ++q )
          out[x + y*dims[0]+r*dims[0]*dims[1] + q*dims[0]*dims[1]*dims[2]] = ENV.get_heur({x,y,r,q});  
  */

  //plhs[4] = mxCreateDoubleScalar( ENV.get_heur({0,0,0,0}) );
