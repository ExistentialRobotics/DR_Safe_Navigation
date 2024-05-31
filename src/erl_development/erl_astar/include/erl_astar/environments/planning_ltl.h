#ifndef __ENV_LTL_H_
#define __ENV_LTL_H_


#include <boost/multi_array.hpp>
#include <algorithm>
#include <memory>   // std::unique_ptr
#include <erl_env/ltl/fsa.h>
#include <erl_astar/planning_interface.h>
#include "mex.h"
#include <erl_utilities/grid_map.h>
#include <erl_utilities/grid_map_utils.h>
#include <erl_utilities/erl_geom_utils.h>
#include <erl_utilities/erl_utils.h>

//#define BOOST_DISABLE_ASSERTS

namespace erl
{
class ENV_LTL : public PlanningInterface<std::vector<int>>
{
public:
  uint16_t *lmap;                    // label map
  const mxArray *label_to_xy_cell;  // inverse label map = cell array from matlab
  //std::vector< std::vector< std::vector<int> > > dd_mprim_x;
  //std::vector< std::vector< std::vector<int> > > dd_mprim_y;
  //std::vector< std::vector< std::vector<int> > > dd_mprim_q;

  erl::GridMap MAP;                // conversion from continuous to discrete
  std::unique_ptr<erl::FSA> spec; // Logic specification

  // (MAP.size())[2] x num_traj x len_traj_fine x 3
  boost::multi_array<int, 4> dd_mprim_;
  //typedef boost::multi_array<int, 4>::index index;
  boost::multi_array<double, 3> dd_mprim_u_;  // num_traj x len_traj x 2
  double dd_mprim_samp_;
  std::vector<double> dd_mprim_cost_;

  const double *label_distance_g;
  size_t num_lab_g_;

  /*
   * dd_mprim = 3 x len_traj_fine x num_traj
   *
   * dd_mprim_u = 2 x len_traj x num_traj
   *
   * There are ( len_traj x num_traj ) successors with different lengths
   */
  ENV_LTL( size_t num_dim, double *min_ptr, double *max_ptr, double *res_ptr,
           uint16_t *lmap_ptr, const mxArray *ilmap_ptr, const mxArray *dd_mprim,
           const mxArray *dd_mprim_u, double dd_mprim_samp, std::unique_ptr<erl::FSA> fsa_ptr,
           const mxArray *g_ptr = NULL )
  : MAP(num_dim, min_ptr, max_ptr, res_ptr), lmap(lmap_ptr), label_to_xy_cell(ilmap_ptr),
    dd_mprim_(boost::extents[(MAP.size())[2]][(mxGetDimensions(dd_mprim))[2]][(mxGetDimensions(dd_mprim))[1]][3]),
    dd_mprim_u_(boost::extents[(mxGetDimensions(dd_mprim_u))[2]][(mxGetDimensions(dd_mprim_u))[1]][2]),
    dd_mprim_samp_(dd_mprim_samp), spec( std::move(fsa_ptr) ),
    label_distance_g(mxGetPr(g_ptr)), num_lab_g_(mxGetM(g_ptr))
  {
    // Get sizes
    double *dd_mprim_vals = mxGetPr(dd_mprim);
    const mwSize *dims = mxGetDimensions(dd_mprim);
    size_t len_traj_fine = dims[1];
    size_t num_traj = dims[2];
    int theta_sz = (MAP.size())[2];

    // Populate dd_mprim
    double x,y,t;
    //int origin_x_cell = erl::meters2cells( 0, MAP.min()[0], MAP.res()[0] );
    //int origin_y_cell = erl::meters2cells( 0, MAP.min()[1], MAP.res()[1] );
    //mexPrintf("origin_x_cell = %d\n",origin_x_cell);
    //mexPrintf("origin_y_cell = %d\n",origin_y_cell);
    for( unsigned k = 0; k < theta_sz; ++k )
    {
      double ori = erl::cells2meters(k,MAP.min()[2],MAP.res()[2]);
      for( unsigned len = 0; len < len_traj_fine; ++len )
        for( unsigned tr = 0; tr < num_traj; ++tr )
        {
          erl::smart_plus_SE2( MAP.origin()[0], MAP.origin()[1], ori,
                              dd_mprim_vals[0 + len*3 + tr*3*len_traj_fine],
                              dd_mprim_vals[1 + len*3 + tr*3*len_traj_fine],
                              dd_mprim_vals[2 + len*3 + tr*3*len_traj_fine],
                              x,y,t );
          dd_mprim_[k][tr][len][0] = erl::meters2cells(x, MAP.min()[0], MAP.res()[0])
              - MAP.origincells()[0];
          dd_mprim_[k][tr][len][1] = erl::meters2cells(y, MAP.min()[1], MAP.res()[1])
              - MAP.origincells()[1];
          dd_mprim_[k][tr][len][2] = erl::meters2cells(t, MAP.min()[2], MAP.res()[2]);

          //mexPrintf("x = %f, y = %f\n",x,y);
          //mexPrintf("dd_mprim_vals[1 + len*3 + tr*3*len_traj_fine] = %f\n",dd_mprim_vals[1 + len*3 + tr*3*len_traj_fine]);
          //mexPrintf("dd_mprim_[%d][%d][%d][0] = %d\n",k,tr,len,dd_mprim_[k][tr][len][0]);
          //mexPrintf("dd_mprim_[%d][%d][%d][1] = %d\n",k,tr,len,dd_mprim_[k][tr][len][1]);
        }
    }

    // Copy dd_mprim_u
    double *dd_mprim_u_vals = mxGetPr(dd_mprim_u);
    size_t len_traj = mxGetDimensions(dd_mprim_u)[1];
    for( unsigned tr = 0; tr < num_traj; ++tr )
      for( unsigned len = 0; len < len_traj; ++len )
      {
        dd_mprim_u_[tr][len][0] = dd_mprim_u_vals[0 + len*2 + tr*2*len_traj];
        dd_mprim_u_[tr][len][1] = dd_mprim_u_vals[1 + len*2 + tr*2*len_traj];
      }

    // Compute mprim cost
    // TODO: Pass cost in, instead of computing here
    dd_mprim_cost_.reserve( num_traj * len_traj );
    for( unsigned tr = 0; tr < num_traj; ++tr )
      for( unsigned len = 0; len < len_traj; ++len )
      {
        double curr_cost = dd_mprim_u_[tr][len][0] * dd_mprim_samp_;
        if( len > 0 )
          dd_mprim_cost_.push_back( dd_mprim_cost_[tr*len_traj + len-1] +  curr_cost );
        else
          dd_mprim_cost_.push_back( curr_cost );
      }
  }



  bool isGoal(const std::vector<int>& state_coord) const override
  {
    //return (std::find(spec.F.begin(), spec.F.end(), state_coord[3]) != spec.F.end());
    return spec->isGoal(state_coord[3]);
  }



  double getHeuristicInconsistent(std::vector<int> const &state_coord) const
  {
    //return 0;
    //mexPrintf("Computing heuristic...\n");
    double h = std::numeric_limits<double>::infinity();
    double dist2;
    const mxArray *cellElement;

    // Find set of next automaton states
    std::vector<unsigned int> curr_q_set;
    std::vector<unsigned int> next_q_set;
    if( !spec->next_level_q(state_coord[3],curr_q_set,next_q_set) )
    return h; // state_coord[3] is a sink state

    if( next_q_set.empty() )
    return 0; // state_coord[3] is a goal state

    // determine the min cost to reaching one of those next_q
    next_q_set.insert( next_q_set.end(), curr_q_set.begin(), curr_q_set.end() );
    for( std::vector<unsigned int>::const_iterator it = next_q_set.begin();
    it != next_q_set.end(); ++it )
    {
      // labels that take us to next_q
      std::vector<unsigned int> labs;
      spec->trans_labels(state_coord[3], *it, labs);

      //mexPrintf("q = %d, next_q = %d\n",state_coord[3],*it);

      //mexPrintf("labs.size() = %d\n",labs.size());
      //mexPrintf("labs = ");
      //for( unsigned k = 0; k < labs.size(); ++k )
      //  mexPrintf("%d ",labs[k]);
      //mexPrintf("\n");

      // find all positions that take us to labs
      for( std::vector<unsigned int>::const_iterator it1 = labs.begin();
        it1 != labs.end(); ++it1 )
      {
        //mexPrintf("lab = %d\n",*it1);
        cellElement = mxGetCell(label_to_xy_cell,*it1);
        double *xy_array = mxGetPr(cellElement);
        size_t num_sta = mxGetN( cellElement );
        //mexPrintf("num_xy = %d\n",num_sta);
        for( size_t sta = 0; sta < num_sta; ++sta )
        {
          // Distance is in meters!
          dist2 = (xy_array[0+sta*2] - static_cast<double>(state_coord[0]))*MAP.res()[0]*
              (xy_array[0+sta*2] - static_cast<double>(state_coord[0]))*MAP.res()[0]+
              (xy_array[1+sta*2] - static_cast<double>(state_coord[1]))*MAP.res()[1]*
                  (xy_array[1+sta*2] - static_cast<double>(state_coord[1]))*MAP.res()[1];

          if( dist2 < h )
          h = dist2;
        }
        //mexPrintf("h = %f\n",h);
      }
    }
    return std::sqrt(h);  
  }      

  void getSuccessors( const std::vector<int>& curr_coord,
                      std::vector<std::vector<int>>& succ_coord,
                      //std::vector<int>& succ_idx,
                      std::vector<double>& succ_cost,
                      std::vector<int>& mprim_idx ) const override
  {
    succ_coord.reserve( dd_mprim_cost_.size() );
    //succ_idx.reserve( dd_mprim_cost_.size() );
    succ_cost.reserve( dd_mprim_cost_.size() );
    size_t len_traj_fine = dd_mprim_.shape()[2];
    size_t len_traj = dd_mprim_u_.shape()[1];
    size_t num_traj = dd_mprim_.shape()[1];
    size_t num_samp_vals = len_traj_fine / len_traj;
    std::vector<int> sz( MAP.size() ); sz.push_back( spec->num_sta() );

    int tmp_x, tmp_y, tmp_t, tmp_q;
    for( int tr = 0; tr < num_traj; ++tr )
    {
      tmp_q = curr_coord[3];
      for( int len = 0; len < len_traj_fine; ++len )
      {
        // Move along motion primitive
        tmp_x = curr_coord[0] + dd_mprim_[curr_coord[2]][tr][len][0];
        tmp_y = curr_coord[1] + dd_mprim_[curr_coord[2]][tr][len][1];
        tmp_t = dd_mprim_[curr_coord[2]][tr][len][2];

        // Discard motion primitives that go outside of the map
        if( tmp_x < 0 || tmp_x >= MAP.size()[0] ||
            tmp_y < 0 || tmp_y >= MAP.size()[1] )
          break;

        // Look up map label
        uint16_t label = lmap[ tmp_x + tmp_y * sz[0] ];

        // Update automaton state
        tmp_q = spec->next_q( tmp_q, static_cast<unsigned int>(label) );

        // Discard motion primitives that reach a sink automaton state
        if( spec->isSink( tmp_q ) )
          break;

        // Save successor coordinates
        if( (len % num_samp_vals) == (num_samp_vals - 1) )
        {
          succ_coord.push_back( {tmp_x,tmp_y,tmp_t,tmp_q} );
          //succ_idx.push_back( erl::subv2ind_colmajor( succ_coord.back().begin(),
          //                                           succ_coord.back().end(),
          //                                           sz.begin(), sz.end() ) );
          succ_cost.push_back(dd_mprim_cost_[tr*len_traj + len/num_samp_vals]);
          mprim_idx.push_back( tr*len_traj + len/num_samp_vals );

          //mexPrintf("tr = %d, len = %d, len_fine = %d\n", tr, len/num_samp_vals, len);
          //mexPrintf("tr_computed = %f, len_computed = %f\n", static_cast<double>(mprim_idx.back()/len_traj),
          //                                                   static_cast<double>(mprim_idx.back()%len_traj));

        }
      }
    }
  }


  double getHeuristic(const std::vector<int>& state_coord) const override
  {
    if( label_distance_g == NULL )
      return 0;

    double h = std::numeric_limits<double>::infinity();

    if( spec->isSink(state_coord[3]) )
      return h; // state_coord[3] is a sink state
    if( spec->isGoal(state_coord[3]) )
      return 0; // state_coord[3] is a goal state

    // for each successor of q
    for( unsigned int nq = 0; nq < spec->num_sta(); ++nq )
    {
      // skip self-loops
      if( state_coord[3] == nq  )
        continue;

      // labels that take us to next_q
      std::vector<unsigned int> labs;
      spec->trans_labels(state_coord[3], nq, labs);

      // find the distance to the successor labels
      for( std::vector<unsigned int>::const_iterator it1 = labs.begin();
           it1 != labs.end(); ++it1 )
      {
        // distnace from point to set
        double c = std::numeric_limits<double>::infinity();
        const mxArray *cellElement = mxGetCell(label_to_xy_cell,*it1);
        double *xy_array = mxGetPr(cellElement);
        size_t num_sta = mxGetN( cellElement );

        if( num_sta == 0 )
          continue;

        //mexPrintf("num_xy = %d\n",num_sta);
        for( size_t sta = 0; sta < num_sta; ++sta )
        {
          // Distance is in meters!
          double dist2 = (xy_array[0+sta*2] - static_cast<double>(state_coord[0]))*MAP.res()[0]*
              (xy_array[0+sta*2] - static_cast<double>(state_coord[0]))*MAP.res()[0]+
              (xy_array[1+sta*2] - static_cast<double>(state_coord[1]))*MAP.res()[1]*
                  (xy_array[1+sta*2] - static_cast<double>(state_coord[1]))*MAP.res()[1];

          if( dist2 < c )
            c = dist2;
        }
        c = std::sqrt(c);

        double tentative_h = c + label_distance_g[*it1 + nq*num_lab_g_];

        //mexPrintf("c = %f, g = %f, l=%d, nq=%d, num_sta = %d\n",
        //      c,label_distance_g[*it1 + nq*num_lab_g_],*it1,nq,num_sta);

        if( tentative_h < h )
          h = tentative_h;
      }
    }
    return h;
  }

  double labelDistance(int l1, int l2) const
  {
    if( l1 == l2)
      return 0; // Distance from label to itself is always 0

    const mxArray *cellElement = mxGetCell(label_to_xy_cell,l1);
    double *xy_arr1 = mxGetPr(cellElement);
    size_t num_xy1 = mxGetN(cellElement);

    cellElement = mxGetCell(label_to_xy_cell,l2);
    double *xy_arr2 = mxGetPr(cellElement);
    size_t num_xy2 = mxGetN(cellElement);


    if( num_xy1 == 0 || num_xy2 == 0 )
      return std::numeric_limits<double>::infinity(); // Distance to empty label is inf
    if( l1 == 0 || l2 == 0 )
      return 0; // Distance to label 0 is always 0

    double min_d = std::numeric_limits<double>::infinity();

    for( size_t st1 = 0; st1 < num_xy1; ++st1 )
      for( size_t st2 = 0; st2 < num_xy2; ++st2 )
      {
        double d = (xy_arr1[0+st1*2] -  xy_arr2[0+st2*2])*MAP.res()[0]*
            (xy_arr1[0+st1*2] -  xy_arr2[0+st2*2])*MAP.res()[0]+
            (xy_arr1[1+st1*2] -  xy_arr2[1+st2*2])*MAP.res()[1]*
                (xy_arr1[1+st1*2] -  xy_arr2[1+st2*2])*MAP.res()[1];
        if( d < min_d )
          min_d = d;
      }
    return std::sqrt(min_d);
  }

  void forwardAction( const std::vector<int>& curr, int action_id,
                       std::vector<std::vector<int>>& next_micro ) const
  {
    size_t len_traj_fine = dd_mprim_.shape()[2];
    size_t len_traj = dd_mprim_u_.shape()[1];
    size_t num_traj = dd_mprim_.shape()[1];
    size_t num_samp_vals = len_traj_fine / len_traj;

    unsigned int tr = action_id / len_traj ;  // find which trajectory it is
    unsigned int len = (action_id % len_traj) * num_samp_vals + (num_samp_vals-1);
    int tmp_x, tmp_y, tmp_t, tmp_q;
    tmp_q = curr[3];

    next_micro.reserve(len+1);
    for( unsigned l = 0; l <= len; ++l )
    {
      tmp_x = curr[0] + dd_mprim_[curr[2]][tr][l][0];
      tmp_y = curr[1] + dd_mprim_[curr[2]][tr][l][1];
      tmp_t = dd_mprim_[curr[2]][tr][l][2];
      uint16_t label = lmap[ tmp_x + tmp_y * MAP.size()[0] ];
      tmp_q = spec->next_q( tmp_q, static_cast<unsigned int>(label));
      next_micro.push_back( {tmp_x,tmp_y,tmp_t,tmp_q} );
    }
  }

};

}
#endif
