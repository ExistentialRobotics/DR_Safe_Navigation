#ifndef __ENV_LTL_2D_H_
#define __ENV_LTL_2D_H_

#include <memory>   // std::unique_ptr
#include <unordered_map>
#include <boost/heap/d_ary_heap.hpp>      // heap
#include <Eigen/Dense>
#include <erl_map/grid_map.h>
#include <erl_astar/planning_interface.h>
#include <erl_env/ltl/fsa.h>

namespace erl {

/**
 * @brief The PlanningLTL2D class implements PlanningInterface for a Linear Temporal Logic Environment and 2 Dimensions.
 */
class PlanningLTL2D : public PlanningInterface<std::array<int, 3>> {
//uint16_t *lmap;                    // label map
  const Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &lmap_;
  std::unordered_map<uint16_t, std::vector<std::array<int, 2> >> label_to_xy_cell;

 public:
  Eigen::MatrixXd label_distance_g_; // num_labels x spec->num_sta()
//const double *label_distance_g;
//size_t num_lab_g_;


  std::unique_ptr<erl::GridMap<uint16_t>> MAP_ptr;
  std::unique_ptr<erl::FSA> spec;                 // Logic specification

  /**
   * @brief Constructs the PlanningLTL2D Environment.
   * @param lmap
   * @param MAP_ptr_in The input Map pointer.
   * @param fsa_ptr The FSA pointer.
   */
  PlanningLTL2D(const Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &lmap,
             std::unique_ptr<erl::GridMap<uint16_t>> MAP_ptr_in,
             std::unique_ptr<erl::FSA> fsa_ptr)
      : lmap_(lmap), MAP_ptr(std::move(MAP_ptr_in)),
        spec(std::move(fsa_ptr)) {
    // initialize label_to_xy_cell
    for (int x = 0; x < MAP_ptr->size()[0]; ++x)
      for (int y = 0; y < MAP_ptr->size()[1]; ++y) {
        uint16_t label = lmap_(x, y);
        auto it = label_to_xy_cell.find(label);

        // if element does not exist, insert it!
        if (it == label_to_xy_cell.end()) {
          auto it1 = label_to_xy_cell.insert(std::make_pair(label,
                                                            std::vector<std::array<int, 2>>(1, {x, y})));
          it1.first->second.reserve(MAP_ptr->size()[0] * MAP_ptr->size()[1]);
        } else
          it->second.push_back({x, y});
      }

    for (int lab = 0; lab < 32; ++lab) {
      auto it = label_to_xy_cell.find(lab);
      if (it != label_to_xy_cell.end())
        std::cout << "ilmap(" << lab << ").size()=" << it->second.size() << std::endl;
    }
    // initialize label_distance_g_
    label_distance_g_ = std::move(label_cost2go());
  }

  /**
  * @brief Computes the successors of the state curr.
  * @param curr The current state to compute successors of.
  * @param succ The vector of successor states.
  * @param succ_cost The vector of costs of each successor.
  * @param action_idx The vector of indices of the actions leading to the successor.
  */
  void getSuccessors(const std::array<int, 3> &curr,
                     std::vector<std::array<int, 3>> &succ,
                     std::vector<double> &succ_cost,
                     std::vector<int> &action_idx) const {
    succ.reserve(8);
    succ_cost.reserve(8);
    action_idx.reserve(8);
    for (int xShift = -1; xShift <= 1; ++xShift) {
      int xNeighbor = curr[0] + xShift;
      if (xNeighbor < 0) continue; // skip outside of map
      if (xNeighbor >= MAP_ptr->size()[0]) continue;
      for (int yShift = -1; yShift <= 1; yShift++) {
        if (xShift == 0 && yShift == 0) continue; // skip current node
        int yNeighbor = curr[1] + yShift;
        if (yNeighbor < 0) continue;
        if (yNeighbor >= MAP_ptr->size()[1]) continue;

        // Look up map label
        uint16_t label = lmap_(xNeighbor, yNeighbor);

        // Update automaton state
        int qNeighbor = spec->next_q(curr[2], static_cast<unsigned int>(label));

        // Discard successors that reach a sink automaton state
        if (spec->isSink(qNeighbor)) continue;

        // skip collisions
        //int indNeighbor = xNeighbor + yNeighbor*MAP_ptr->size[0];
        //if( cmap_[indNeighbor] ) continue;

        //calc the cost of the successor (neighbor)
        //double costMult; // get the cost multiplier
        //switch (std::abs(xShift) + std::abs(yShift)){
        //    case 1: costMult = 1.0; break;
        //    case 2: costMult = std::sqrt(2); break;
        //}

        double dx = xShift * MAP_ptr->res()[0];
        double dy = yShift * MAP_ptr->res()[1];
        double costMult = std::sqrt(dx * dx + dy * dy);

        succ.push_back({xNeighbor, yNeighbor, qNeighbor});
        succ_cost.push_back(costMult);
        action_idx.push_back((xShift + 1) + (yShift + 1) * 3); // trenary to decimal
      }
    }
  }

  /**
   * @brief Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   */
  std::vector<std::array<int, 3>> forwardAction(const std::array<int, 3> &curr, int action_id) const {
    std::vector<std::array<int, 3>> next_micro;
    // decode from decimal to trenary
    int xShift = (action_id % 3) - 1;
    int yShift = (action_id / 3) - 1;
    next_micro.push_back({curr[0] + xShift, curr[1] + yShift, -1});
    // Look up map label
    uint16_t label = lmap_(next_micro.back()[0], next_micro.back()[1]);
    next_micro.back()[2] = spec->next_q(curr[2], static_cast<unsigned int>(label));
    return next_micro;
  }

  /**
   * @brief Indicates if a state s is the goal state.
   * @param s The state to check for goal conditions.
   * @return True if the state is at the goal.
   */
  inline bool isGoal(const std::array<int, 3> &state_coord) const { return spec->isGoal(state_coord[2]); }

  /**
   * @brief Computes a heuristic estimate from the state to the goal.
   * @param s The query state.
   * @return The heuristic value.
   */
  double getHeuristic(const std::array<int, 3> &state_coord) const {
    if (label_distance_g_.size() == 0)
      return 0;

    double h = std::numeric_limits<double>::infinity();

    if (spec->isSink(state_coord[2]))
      return h; // state_coord[2] is a sink state
    if (spec->isGoal(state_coord[2]))
      return 0; // state_coord[2] is a goal state

    // for each successor of q
    for (unsigned int nq = 0; nq < spec->num_sta(); ++nq) {
      // skip self-loops
      if (state_coord[2] == (int) nq) continue;

      // labels that take us to next_q
      std::vector<unsigned int> labs;
      spec->trans_labels(state_coord[2], nq, labs);

      // find the distance to the successor labels
      for (std::vector<unsigned int>::const_iterator lab_it = labs.begin();
           lab_it != labs.end(); ++lab_it) {
        // distnace from point to set
        double c = std::numeric_limits<double>::infinity();
        auto it = label_to_xy_cell.find(*lab_it);
        if (it == label_to_xy_cell.end()) continue;
        size_t num_sta = it->second.size();
        for (size_t sta = 0; sta < num_sta; ++sta) {
          double dx = static_cast<double>(it->second[sta][0]
              - state_coord[0]) * MAP_ptr->res()[0];
          double dy = static_cast<double>(it->second[sta][1]
              - state_coord[1]) * MAP_ptr->res()[1];
          double dist2 = dx * dx + dy * dy;
          if (dist2 < c) c = dist2;
        }
        c = std::sqrt(c);

        double tentative_h = c + label_distance_g_(*lab_it, nq);
        if (tentative_h < h) h = tentative_h;
      }
    }
    return h;
  }

  /**
   * @brief Converts the state to a linear index.
   * @param s The state to be converted.
   * @return The linear index.
   */
  inline size_t stateToIndex(const std::array<int, 3> &state_coord) const {
    return state_coord[0] + state_coord[1] * MAP_ptr->size()[0]
        + state_coord[2] * MAP_ptr->size()[0] * MAP_ptr->size()[1];
  }



 private:

  double label_distance(int l1, int l2) const {
    if (l1 == l2)
      return 0; // Distance from label to itself is always 0

    auto xy_arr1 = label_to_xy_cell.find(l1);
    if (xy_arr1 == label_to_xy_cell.end())
      return std::numeric_limits<double>::infinity(); // Distance to empty label is inf
    auto xy_arr2 = label_to_xy_cell.find(l2);
    if (xy_arr2 == label_to_xy_cell.end())
      return std::numeric_limits<double>::infinity(); // Distance to empty label is inf
    if (l1 == 0 || l2 == 0)
      return 0; // Distance to label 0 is always 0

    double min_d = std::numeric_limits<double>::infinity();

    size_t num_xy1 = xy_arr1->second.size();
    size_t num_xy2 = xy_arr2->second.size();
    for (size_t st1 = 0; st1 < num_xy1; ++st1)
      for (size_t st2 = 0; st2 < num_xy2; ++st2) {
        double dx = (xy_arr1->second[st1][0] - xy_arr2->second[st2][0]) * MAP_ptr->res()[0];
        double dy = (xy_arr1->second[st1][1] - xy_arr2->second[st2][1]) * MAP_ptr->res()[1];
        double d = dx * dx + dy * dy;
        if (d < min_d) min_d = d;
      }
    return std::sqrt(min_d);
  }

  typedef std::tuple<double, int, int> CostNodePair; // pair of a double cost and node number

  struct compare_cnpair {
    bool operator()(const CostNodePair &t1,
                    const CostNodePair &t2) const { return std::get<0>(t1) > std::get<0>(t2); }
  };

  Eigen::MatrixXd label_cost2go() const {
    using PriorityQueue = boost::heap::d_ary_heap<CostNodePair, boost::heap::mutable_<true>,
                                                  boost::heap::arity<2>, boost::heap::compare<compare_cnpair >>;
    typedef PriorityQueue::handle_type heapkey;

    unsigned num_lab = std::pow(2, spec->num_ap());
    // transition cost between labels
    Eigen::MatrixXd costL;
    costL.setConstant(num_lab, num_lab, std::numeric_limits<double>::infinity());
    // gvalues
    Eigen::MatrixXd g;
    g.setConstant(num_lab, spec->num_sta(), std::numeric_limits<double>::infinity());
    // closed list
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> cl;
    cl.setZero(num_lab, spec->num_sta());
    // store heapkeys
    Eigen::Matrix<heapkey, Eigen::Dynamic, Eigen::Dynamic> hk(num_lab, spec->num_sta());
    // open list
    PriorityQueue pq;
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> op;
    op.setZero(num_lab, spec->num_sta());

    // Initialize the g-values of all accepting states to 0 and add them to OPEN
    for (auto fin_sta : spec->F_) {
      g.col(fin_sta) = Eigen::VectorXd::Zero(num_lab);
      for (unsigned lab = 0; lab < num_lab; ++lab) {
        hk(lab, fin_sta) = pq.push(std::make_tuple(0.0, lab, fin_sta));
        op(lab, fin_sta) = true;
      }
    }

    while (!pq.empty()) {
      // get node with min gval
      CostNodePair currNode = pq.top();
      pq.pop();
      int lCurr = std::get<1>(currNode);
      int qCurr = std::get<2>(currNode);
      op(lCurr, qCurr) = false;
      cl(lCurr, qCurr) = true;

      // find predecessors
      for (unsigned int qPred = 0; qPred < spec->num_sta(); ++qPred) {
        if (qPred == static_cast<unsigned int>(qCurr)) continue; // skip self-loops
        if (spec->next_q(qPred, lCurr) != static_cast<unsigned int>(qCurr)) continue;

        for (unsigned int lPred = 0; lPred < num_lab; ++lPred) {
          if (cl(lPred, qPred)) continue; // skip closed

          // compute transition cost if needed
          if (std::isinf(costL(lPred, lCurr))) {
            //std::cout << "label_distance(" << lPred << ", "
            //          << lCurr << ")=" << label_distance(lPred, lCurr)
            //          << std::endl;
            costL(lPred, lCurr) = label_distance(lPred, lCurr);
            costL(lCurr, lPred) = costL(lPred, lCurr);
          }

          double tentative_g = costL(lPred, lCurr) + g(lCurr, qCurr);

          if (tentative_g < g(lPred, qPred)) {
            g(lPred, qPred) = tentative_g;
            if (op(lPred, qPred))
              pq.increase(hk(lPred, qPred), std::make_tuple(tentative_g, lPred, qPred));
            else
              hk(lPred, qPred) = pq.push(std::make_tuple(tentative_g, lPred, qPred));
          }
        }
      }
    }
    return g;
  }
};

}

#endif
