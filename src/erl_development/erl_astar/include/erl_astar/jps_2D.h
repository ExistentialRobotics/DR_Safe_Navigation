#ifndef __JPS_2D_H_
#define __JPS_2D_H_

#include <boost/heap/d_ary_heap.hpp>      // heap
#include <cmath>
#include <limits>                         // std::numeric_limits
#include <vector>
#include <list>                           // std::list
#include <array>

namespace erl
{
  template <class T>
  struct compare_astar
  {
    bool operator()(T* a1, T* a2) const
    {
      double f1 = a1->g + a1->h;
      double f2 = a2->g + a2->h;
      if( ( f1 >= f2 - 0.000001) && (f1 <= f2 +0.000001) )
        return a1->g < a2->g; // if equal compare gvals
      return f1 > f2;
    }
  };
  
  struct Astate; // forward declaration
  using PriorityQueue = boost::heap::d_ary_heap<Astate*, boost::heap::mutable_<true>, 
                                                boost::heap::arity<2>, boost::heap::compare< compare_astar<Astate> >>;
  
  struct Astate
  {
    int ID;
    int dx, dy;
    double g = std::numeric_limits<double>::infinity();
    double h;
    bool cl = false;
    int parent = -1;
    PriorityQueue::handle_type heapkey;
    Astate(int ind, int dx, int dy): ID(ind), dx(dx), dy(dy){}
  };
  
  class JPS_2D
  {
    const char *cMap_;
    const char val_free_;
    int xDim_, yDim_, xyDim_;
    PriorityQueue pq_;
    std::vector<Astate*> hashMap_;
    std::vector<bool> seen_;
    
    //static constexpr double sqrtTwo_ = std::sqrt(2);
    //static constexpr double sqrtThree_ = std::sqrt(3);

    //int* neib_[3][3][2];
    //int* fneib1_[3][3][2];
    //int* fneib2_[3][3][2];    

  public:  
    JPS_2D(const char* cMap, const char val_free, int xDim, int yDim)
      : cMap_(cMap), val_free_(val_free),
        xDim_(xDim), yDim_(yDim), xyDim_(xDim*yDim),
        hashMap_(xyDim_), seen_(xyDim_)
      {}
    double plan( int xStart, int yStart,
                 int xGoal,  int yGoal,
                 //std::list<int>& indPath,
                 std::list<std::array<int,2>>& xyPath,
                 double eps = 1 );
  private:
    void spin(Astate *currNode_pt);
    void jump(int x, int y, int dx, int dy);
    bool hasForced(int x, int y, int dx, int dy, int norm1);
    void Neib(int dx, int dy, int nx, int ny, int& tx, int& ty);

    // temp global data
    double eps_;
    int xGoal_, yGoal_;
    int xJump_, yJump_;
  };
}
#endif


