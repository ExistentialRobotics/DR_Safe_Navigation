#ifndef __FSA_H_
#define __FSA_H_

#include <string>
#include <vector>
//#include <set>
#include <unordered_map>
//#include <algorithm>
//#include <iterator>
#include <iostream>

namespace erl
{
class FSA
{
 protected:
  typedef std::unordered_map<unsigned int,std::vector<unsigned int>> T0;
  typedef std::unordered_map<unsigned int,unsigned int> T1;

  T0 trans0;  // Labels for a given transition
  T1 trans1;  // Next state for a given state and label

 public:
  std::vector<std::vector<unsigned int>> levels_;
  std::vector<std::vector<bool>> levels_b_;
  std::vector<bool> sink_states_;

  unsigned int num_sta_;
  unsigned int q0_;  // zero-based!
  std::vector<unsigned int> F_;
  std::vector<std::string> AP_;
  std::vector<unsigned int> Sigma_; // The alphabet


  FSA( unsigned int num_sta, unsigned int q0, std::vector<unsigned int> const & F,
       std::vector<std::string> const &AP, std::vector<unsigned int> const & Sigma )
      :num_sta_(num_sta), q0_(q0), F_(F), AP_(AP), Sigma_(Sigma)
  {}

  void add_transition(unsigned int q, unsigned int next_q, std::vector<unsigned int> const &labs)
  {
    unsigned int key = q + next_q*num_sta_;  // sub2ind (zero-based!)
    trans0.emplace( key, labs );

    if( next_q != q)
    {
      for( std::vector<unsigned int>::const_iterator it = labs.begin();
           it != labs.end(); ++it )
      {
        key = q + (*it)*num_sta_;
        trans1.emplace( key, next_q );
      }
    }
  }

  void trans_labels(unsigned int q, unsigned int next_q, std::vector<unsigned int> &labs) const
  {
    unsigned int key = q + next_q*num_sta_; // sub2ind (zero-based!)
    T0::const_iterator it = trans0.find(key);
    if( it != trans0.end() )
      labs = it->second;
  }

  unsigned int next_q(unsigned int q, unsigned int a) const
  {
    unsigned int key = q + a*num_sta_; // sub2ind (zero-based!)
    T1::const_iterator it = trans1.find(key);
    if( it == trans1.end() )
      return q;
    else
      return it->second;
  }

  unsigned int get_level_id(unsigned int q) const
  {
    if( sink_states_[q] )
      return -1;
    for( unsigned int k = 0; k < levels_b_.size(); ++k )
      if( levels_b_[k][q] )
        return k;
  }

  // returns false if q is a sink state; returns true and next level otherwise
  bool next_level_q( unsigned int q, std::vector<unsigned int> &curr, std::vector<unsigned int> &next ) const
  {
    int level_id = get_level_id(q);

    if( level_id == -1 )
      return false;

    // return current level states reachable from q
    for( std::vector<unsigned int>::const_iterator it = levels_[level_id].begin();
         it != levels_[level_id].end(); ++it )
      if( q!=(*it) && ( trans0.find( q + (*it)*num_sta_ ) != trans0.end() ) )
        curr.push_back( *it );

    if( level_id == 0 )
      return true;

    // return next level states reachable from q
    for( std::vector<unsigned int>::const_iterator it = levels_[level_id-1].begin();
         it != levels_[level_id-1].end(); ++it )
      if( trans0.find( q + (*it)*num_sta_ ) != trans0.end() )
        next.push_back( *it );

    return true;
  }

  unsigned int num_sta() { return num_sta_; }
  unsigned int q0() { return q0_; }
  unsigned int num_levels() { return levels_.size(); }
  bool isGoal(unsigned int q){ return levels_b_[0][q]; }
  bool isSink(unsigned int q){ return sink_states_[q]; }
  unsigned int num_ap() { return AP_.size(); }

  void compute_levels()
  {
    levels_.emplace_back( F_.begin(), F_.end() );
    levels_b_.emplace_back( num_sta_, false );
    sink_states_.resize( num_sta_, true );
    for( std::vector<unsigned int>::const_iterator it = F_.begin();
         it != F_.end(); ++it )
    {
      sink_states_[*it] = false;
      levels_b_[0][*it] = true;
    }

    for( unsigned k = 0; true; ++k )
    {
      bool done = true;
      // Find predecessors of current level states that are still sink states
      for( unsigned int idx = 0; idx < levels_[k].size(); ++idx )
        for( unsigned int prev_q = 0; prev_q < num_sta_; ++prev_q )
          // if prev_q is a sink state but also a predecessor of levels[k][idx]:
          if( sink_states_[prev_q] && (trans0.find( prev_q + levels_[k][idx]*num_sta_ ) != trans0.end()) )
          {
            //std::cout << "prev_q = " << prev_q << " "
            //          << "q = "      << levels_[k][idx] << std::endl;
            sink_states_[prev_q] = false;
            if( done )
            {
              done = false;
              levels_.emplace_back( 1, prev_q );
              levels_b_.emplace_back( num_sta_, false );
              levels_b_[k+1][ prev_q ] = true;
            }
            else
            {
              levels_[k+1].push_back( prev_q );
              levels_b_[k+1][prev_q] = true;
            }
          }
      if( done )
        break;
    }
  }

  /*
  void get_succ( unsigned int q, std::vector<unsigned int> &succ) const
  {

  }

  void get_preds(unsigned int next_q, std::set<unsigned int> &preds) const
  {
    for( T1::const_iterator it = trans1.begin();
         it != trans1.end(); ++it )
    {
      if( it->second == next_q )
        preds.insert( (it->first)%num_sta_ );  // ind2sub
    }
  }
  */
};
}

#endif
