#ifndef __ASTAR_NX_H_
#define __ASTAR_NX_H_

#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <map>
#include <unordered_map>                  // std::unordered_map
#include <array>                          // std::array
#include <list>                           // std::list
#include <chrono>                         // std::chrono::high_resolution_clock
#include <iostream>


#include <erl_astar/planning_interface.h>

//#include <erl_env/environments/env_int.h> // environment
//#include <erl_utilities/erl_utils.h> // Tic, Toc.

namespace erl {

// heap element comparison
template<class arastate>
struct ComparePair {
  bool operator()(const std::pair<double, std::shared_ptr<arastate>> &p1,
                  const std::pair<double, std::shared_ptr<arastate>> &p2) const {
    if ((p1.first >= p2.first - 0.000001) && (p1.first <= p2.first + 0.000001)) {
      // if equal compare gvals
      return (p1.second->g) < (p2.second->g);
    }
    return p1.first > p2.first;
  }
};

template<class state, class arastate, class hash>
using hashMap = std::unordered_map<state, std::shared_ptr<arastate>, hash>;

template<class arastate>
using priorityQueue = boost::heap::d_ary_heap<std::pair<double, std::shared_ptr<arastate>>,
                                              boost::heap::mutable_<true>,
                                              boost::heap::arity<2>,
                                              boost::heap::compare<ComparePair<arastate> >>;

// ARA state definition
template<class state>
struct ARAState {
  // location data
  state coord;                            // discrete coordinates of this node
  std::shared_ptr<ARAState<state>> parent; // pointer to parent node
  int parentActionId = -1;
  // pointer to heap location
  typename priorityQueue<ARAState<state>>::handle_type heapkey;

  // plan data
  double g = std::numeric_limits<double>::infinity();
  double h;

  // Which search was this node opened or closed. These are either 0 or 1 for Astar
  //unsigned int searchiterationopened = 0;
  //unsigned int searchiterationclosed = 0;

  // Iteration at which this node was opened or closed.
  unsigned int iterationopened = 0;
  unsigned int iterationclosed = 0;

  // ARA* info:
  double v = std::numeric_limits<double>::infinity();
  bool bInconsistent = false;
  //ARAState(){}

  ARAState(const state &coord)
      : coord(coord)//, parent(nullptr)
  {}
};

template<class state>
struct ARAStateSpace {
  std::list<std::shared_ptr<ARAState<state>>> il;
  priorityQueue<ARAState<state>> pq;
  hashMap<state, ARAState<state>, std::function<size_t(const state &s)>> hm;
  // hashMap<state, ARAState<state>, typename environment<state>::Hasher> hm;

  double eps;
  double eps_satisfied = std::numeric_limits<double>::infinity();
  unsigned int expands = 0; // number of expands in the current search
  unsigned int searchexpands = 0; // total number of expands over all searches; Not used for Astar
  //unsigned int searchiteration = 1; // number of searches (replanning steps); Always 1 for AStar

  bool use_il = false;
  bool reopen_nodes = false;

  // Initialize the hashMap hash function to the specific environment implementation.
  ARAStateSpace(const PlanningInterface<state> &env, const double eps = 1)
      : hm(10000, [&env](const state &s) { return env.stateToIndex(s); }), eps(eps) {}
};

/**
 * @brief The AStarOutput is the result of an A* search.
 * @tparam state The type of state generated.
 */
template<class state>
struct AStarOutput {
  std::list<state> path; // Sequence of states comprising a path
  //std::list<std::vector<double>> metric_path; // Sequence of metric states comprising the path. (Optional).
  std::list<int> action_idx; // Sequence of action indices.
  double pcost{std::numeric_limits<double>::infinity()}; // Total Plan cost.
  std::map<int, std::list<state>> opened_list; // Logging Map of time to Open States
  std::map<int, state> closed_list; // Logging Map of time to Closed States
};

/**
 * @brief The Astar class is used to solve Astar planning problems for a provided PlanningInterface.
 * @tparam state The type underlying state in the PlanningInterface.
 */
template<class state>
class ARAStar {
 public:

  /**
   * @brief Solves an Astar planning problem.
   * @param start_coord The starting coordinate.
   * @param env The PlanningInterface defining the state space.
   * @param eps Optional heuristic inflation for faster, but sub-optimal searching.
   * @param log Optional logging parameter which returns additional information about state expansions.
   * @return The resulting output of the search.
   */
  AStarOutput<state> Astar(const state &start_coord,
                           const PlanningInterface<state> &env,
                           double eps = 1,
                           bool log = false);
  /**
   * @brief Solves an ARAstar planning problem, i.e. anytime search with a specified run time.
   * @param start_coord The starting coordinate.
   * @param env The PlanningInterface defining the state space.
   * @param eps Optional heuristic inflation for faster, but sub-optimal searching.
   * @param allocated_time_secs Maximum allowed time to solve the problem.
   * @return The resulting output of the search.
   */
  AStarOutput<state> ARAstar(const state &start_coord,
                             const PlanningInterface<state> &env,
                             double eps = 1,
                             double allocated_time_secs = std::numeric_limits<double>::infinity());

 private:
  void spin(const std::shared_ptr<ARAState<state>> &currNode_pt,
            std::shared_ptr<ARAStateSpace<state>> &sss_ptr,
            const PlanningInterface<state> &env);
  AStarOutput<state> recoverPath(std::shared_ptr<ARAState<state>> &currNode_pt,
                                 const PlanningInterface<state> &env) const;
  void logStates(const std::shared_ptr<ARAStateSpace<state>> &sss_ptr, AStarOutput<state> &output) const;
  void moveInconsToOpen(std::shared_ptr<ARAStateSpace<state>> &sss_ptr);
  void reevaluateFVals(std::shared_ptr<ARAStateSpace<state>> &sss_ptr);
};
} // end of namespace erl



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// A* Implementation  ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class state>
erl::AStarOutput<state> erl::ARAStar<state>::Astar(const state &start_coord,
                                                 const PlanningInterface<state> &env,
                                                 double eps,
                                                 bool log /*= false*/) {
  // Initialize State Space
  std::shared_ptr<ARAStateSpace<state>> sss_ptr(new ARAStateSpace<state>(env, eps));

  // Initialize start node
  std::shared_ptr<ARAState<state>>
      &currNode_pt = sss_ptr->hm[start_coord]; // Using a reference to a shared_ptr is critical!!!
  currNode_pt.reset(new ARAState<state>(start_coord));
  currNode_pt->g = 0;
  currNode_pt->h = env.getHeuristic(start_coord);
  currNode_pt->iterationopened = sss_ptr->expands;

  while (true) {
    // Check if done
    if (env.isGoal(currNode_pt->coord)) {
      erl::AStarOutput<state> output = recoverPath(currNode_pt, env);
      if (log) logStates(sss_ptr, output);
      // std::cout << "Number of expands = " << sss_ptr->expands << std::endl;
      return output;
    }

    // Count the number of expands
    sss_ptr->expands++;

    // Add to closed list
    currNode_pt->v = currNode_pt->g;
    //currNode_pt->searchiterationclosed = sss_ptr->searchiteration; // Add to closed list
    currNode_pt->iterationclosed = sss_ptr->expands;

    // Update heap
    spin(currNode_pt, sss_ptr, env);

    if (sss_ptr->pq.empty()) {
      erl::AStarOutput<state> output;
      if (log) logStates(sss_ptr, output);
      return output; // infeasible problem
    }

    // Remove the element with smallest cost
    currNode_pt = sss_ptr->pq.top().second;
    sss_ptr->pq.pop();
  }
}

template<class state>
erl::AStarOutput<state> erl::ARAStar<state>::ARAstar(const state &start_coord,
                                                   const PlanningInterface<state> &env,
                                                   double eps,
                                                   double allocated_time_secs) {
  std::chrono::high_resolution_clock::time_point time_started = std::chrono::high_resolution_clock::now();
  double eps_final = 1;
  double eps_dec = 0.2;

  // Initialize State Space
  std::shared_ptr<ARAStateSpace<state>> sss_ptr(new ARAStateSpace<state>(env, eps));
  sss_ptr->use_il = true;
  std::shared_ptr<ARAState<state>> &currNode_pt = sss_ptr->hm[start_coord];

  //the main loop of ARA*
  AStarOutput<state> output;
  std::chrono::high_resolution_clock::time_point loop_time;
  while (sss_ptr->eps_satisfied > eps_final + 0.00001 &&
      std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-time_started).count() < allocated_time_secs)
  {
    loop_time = std::chrono::high_resolution_clock::now();

    //decrease eps for all subsequent iterations
    if (std::abs(sss_ptr->eps_satisfied - sss_ptr->eps) < 0.00001) {
      sss_ptr->eps = std::max(sss_ptr->eps - eps_dec, eps_final);
      reevaluateFVals(sss_ptr);   //the priorities need to be updated
      moveInconsToOpen(sss_ptr);  // starting a new search
    }

    //improve or compute path
    //std::cout << "Starting improve path" << std::endl;
    sss_ptr->expands = 0;
    while ( std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-time_started).count() < allocated_time_secs)
    {    
      // Initialize Start Node
      if (!currNode_pt) {
        currNode_pt.reset(new ARAState<state>(start_coord));
        currNode_pt->g = 0;
        currNode_pt->h = env.getHeuristic(start_coord);
        //currNode_pt->searchiterationopened = sss_ptr->searchiteration;
        currNode_pt->iterationopened = sss_ptr->expands;
      } else
        currNode_pt = sss_ptr->pq.top().second;

      // Check if done
      if (env.isGoal(currNode_pt->coord)) {
        output = recoverPath(currNode_pt, env);
        break;
      }

      // Count the number of expands
      sss_ptr->expands++;

      // Add to closed list
      if (!sss_ptr->pq.empty())
        sss_ptr->pq.pop();
      currNode_pt->v = currNode_pt->g;
      //currNode_pt->searchiterationclosed = sss_ptr->searchiteration;
      currNode_pt->iterationclosed = sss_ptr->searchexpands + sss_ptr->expands;

      // Update heap
      spin(currNode_pt, sss_ptr, env);

      if (sss_ptr->pq.empty()) // something went wrong
        return erl::AStarOutput<state>();
    }
    /*
    //print the solution cost and eps bound
    std::cout << "eps= " << sss_ptr->eps << " "
              << "expands= " << sss_ptr->expands << " "
              << "g(goal)= " << output.pcost << " "
              << "time= " << toc(loop_time) << " "
              << "time(total)= " << toc(time_started) << std::endl;
              << "time= " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-loop_time).count() << " "
              << "time(total)= " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-time_started).count() << std::endl;
    */
    sss_ptr->eps_satisfied = sss_ptr->eps;
    sss_ptr->searchexpands += sss_ptr->expands;
    //sss_ptr->searchiteration++;
  }
  return output;
}

template<class state>
void erl::ARAStar<state>::spin(const std::shared_ptr<erl::ARAState<state>> &currNode_pt,
                              std::shared_ptr<erl::ARAStateSpace<state>> &sss_ptr,
                              const PlanningInterface<state> &env) {
  // Get successors
  std::vector<state> succ_coord;
  std::vector<double> succ_cost;
  std::vector<int> succ_act_idx;
  env.getSuccessors(currNode_pt->coord, succ_coord, succ_cost, succ_act_idx);
  //std::cout << "num succ=" << succ_coord.size() << std::endl;

  // Process successors
  for (unsigned s = 0; s < succ_coord.size(); ++s) {
    // Get child
    std::shared_ptr<ARAState<state>> &child_pt = sss_ptr->hm[succ_coord[s]];
    if (!(child_pt)) {
      //std::cout << "INIT: " << succ_coord[s] << std::endl;
      child_pt.reset(new ARAState<state>(succ_coord[s]));
      child_pt->h = env.getHeuristic(child_pt->coord);   // compute heuristic
    }

    //see if we can improve the value of succstate
    //taking into account the cost of action
    double tentative_gval = currNode_pt->v + succ_cost[s];

    //std::cout << "tentative_gval= " << tentative_gval << std::endl;
    //std::cout << "child_pt->g= " << child_pt->g << std::endl;
    if (tentative_gval < child_pt->g) {
      child_pt->parent = currNode_pt;  // Assign new parent
      child_pt->parentActionId = succ_act_idx[s];
      child_pt->g = tentative_gval;    // Update gval
      double fval = child_pt->g + (sss_ptr->eps) * child_pt->h;

      // if currently in OPEN, update
      //if (child_pt->searchiterationopened > child_pt->searchiterationclosed) {
      if (child_pt->iterationopened > child_pt->iterationclosed) {
        //std::cout << "UPDATE fval(old) = " << (*child_pt->heapkey).first << std::endl;
        //std::cout << "UPDATE fval = " << fval << std::endl;
        //std::cout << "eps*h = " << (sss_ptr->eps) * child_pt->h << std::endl;
        (*child_pt->heapkey).first = fval;     // update heap element
        sss_ptr->pq.increase(child_pt->heapkey);       // update heap
      }
        // if currently in CLOSED
        //else if (child_pt->searchiterationclosed == sss_ptr->searchiteration) {
      else if (child_pt->iterationclosed > sss_ptr->searchexpands) {
        if (sss_ptr->reopen_nodes) // reopen node
        {
          //std::cout << "REOPEN" << std::endl;
          child_pt->heapkey = sss_ptr->pq.push(std::make_pair(fval, child_pt));
          //child_pt->searchiterationclosed = 0;
          child_pt->iterationclosed = 0;
        } else if (sss_ptr->use_il && !child_pt->bInconsistent) // inconsistent node
        {
          //std::cout << "INCONS" << std::endl;
          sss_ptr->il.push_back(child_pt);
          child_pt->bInconsistent = true;
        }
      } else // new node, add to heap
      {
        //if (log)
        //  openList_[expands].push_back(succ_coord[s]);

        //std::cout << "ADD: " << succ_coord[s] << std::endl;
        //std::cout << "ADD fval = " << fval << std::endl;
        child_pt->heapkey = sss_ptr->pq.push(std::make_pair(fval, child_pt));
        //child_pt->searchiterationopened = sss_ptr->searchiteration;
        child_pt->iterationopened = sss_ptr->searchexpands + sss_ptr->expands;
      }
    } //
  } // Process successors

  //if (log)   // Save Expanded Node
  //  closedList_[expands] = currNode_pt->coord;
}

/*
 * Private stuff
 */
template<class state>
erl::AStarOutput<state> erl::ARAStar<state>::recoverPath(std::shared_ptr<erl::ARAState<state>> &currNode_pt,
                                                       const PlanningInterface<state> &env) const {
  AStarOutput<state> output;
  output.pcost = currNode_pt->g;
  while (currNode_pt->parent) {
    output.path.push_front(currNode_pt->coord);  
    output.action_idx.push_front(currNode_pt->parentActionId);
    currNode_pt = currNode_pt->parent;
    //std::vector<state> next_micro = env.forwardAction(currNode_pt->coord, output.action_idx.back());
    //for (typename std::vector<state>::reverse_iterator it = next_micro.rbegin();
    //     it != next_micro.rend(); ++it)
    //{
    //  output.path.push_front(*it);
    //}
  }
  output.path.push_front(currNode_pt->coord);
  return output;
}

template<class state>
void erl::ARAStar<state>::logStates(const std::shared_ptr<erl::ARAStateSpace<state>> &sss_ptr,
                                   erl::AStarOutput<state> &output) const {
  for (auto &it: sss_ptr->hm) {
    //std::cout << it.second->coord[0] << ", " << it.second->coord[1] << std::endl;
    output.opened_list[it.second->iterationopened].push_back(it.second->coord);
    //std::cout << "time opened = " << it.second->iterationopened << std::endl;
    if (it.second->iterationclosed > 0) {
      output.closed_list[it.second->iterationclosed] = it.second->coord;
      //std::cout << "time closed = " << it.second->iterationclosed << std::endl;
    }
  }
}

template<class state>
void erl::ARAStar<state>::moveInconsToOpen(std::shared_ptr<erl::ARAStateSpace<state>> &sss_ptr) {
  while (!sss_ptr->il.empty()) {
    double fval = sss_ptr->il.front()->g + sss_ptr->eps * sss_ptr->il.front()->h;

    // insert element in OPEN
    //std::cout << "INCONS fval=" << fval << std::endl;
    sss_ptr->il.front()->heapkey = sss_ptr->pq.push(std::make_pair(fval, sss_ptr->il.front()));
    //sss_ptr->il.front()->searchiterationopened = sss_ptr->searchiteration;
    sss_ptr->il.front()->iterationopened = sss_ptr->searchexpands;
    sss_ptr->il.front()->bInconsistent = false;

    // remove element from INCONS
    sss_ptr->il.pop_front();
  }
}

template<class state>
void erl::ARAStar<state>::reevaluateFVals(std::shared_ptr<erl::ARAStateSpace<state>> &sss_ptr) {
  //recompute priorities for states in OPEN and reorder it
  priorityQueue<ARAState<state>> new_pq;
  for (typename priorityQueue<ARAState<state>>::ordered_iterator it = sss_ptr->pq.ordered_begin();
       it != sss_ptr->pq.ordered_end(); ++it) {
    double new_fval = (*it).second->g + sss_ptr->eps * (*it).second->h;
    (*it).second->heapkey = new_pq.push(std::make_pair(new_fval, (*it).second));
    //std::cout << "REEVAL fval=" << new_fval << std::endl;
  }
  sss_ptr->pq = std::move(new_pq);
}

#endif
