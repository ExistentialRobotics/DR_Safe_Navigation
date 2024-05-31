#ifndef __ASTAR_GRID_H_
#define __ASTAR_GRID_H_

#include <utility>      // std::pair, std::make_pair
#include <boost/heap/d_ary_heap.hpp>

namespace astar
{

// Heap stuff
struct compare_pair
{
  bool operator()(const std::pair<double,int>& p1, 
                  const std::pair<double,int>& p2) const
  { return p1.first > p2.first; }
};
typedef std::pair<double, int> CostNodePair; // pair of a double cost and node number
typedef boost::heap::d_ary_heap<CostNodePair,
                                boost::heap::mutable_<true>,
                                boost::heap::arity<2>,
                                boost::heap::compare< compare_pair >> PriorityQueue;
typedef PriorityQueue::handle_type heapkey;


constexpr double infCost = std::numeric_limits<double>::infinity();
constexpr double sqrtTwoCost = 1.414213562373095145474621858739; //c++ BS: std::sqrt(2);
constexpr double sqrtThreeCost = 1.732050807568877193176604123437; //c++ BS: std::sqrt(3);



/*
 * @INPUT:
 *  cMap a 2D array of costs stored in column major order
 */
double planOn2DGrid( const uint16_t* cMap, int xDim, int yDim, 
                   int xStart, int yStart,
                   int xGoal,  int yGoal,
                   std::list<std::array<int,2>>& xyPath,
                   double eps = 1 )
{
  // Start and Goal indices
  int indStart = xStart + xDim*yStart; // colmajor
  if( cMap[indStart] ) return infCost;   // Quit if start is in collision 
  int indGoal  = xGoal  + xDim*yGoal;
  if( cMap[indGoal] ) return infCost;   // Quit if goal is in collision 

  // Initialize all costs to infinity
  int cMapLength = xDim*yDim;
  double *g = new double[cMapLength];
  double *h = new double[cMapLength];
  int *iPrev = new int[cMapLength]; // keep path indices
  bool *cl = new bool[cMapLength]; // closed list
  heapkey *hk = new heapkey[cMapLength]; // store heapkeys
  for (int i = 0; i < cMapLength; ++i)
  { cl[i] = false; g[i] = infCost; }
  g[indStart]=0;
  h[indStart]=eps*std::sqrt( (xGoal-xStart)*(xGoal-xStart) +
                             (yGoal-yStart)*(yGoal-yStart) );
  PriorityQueue pq;
  hk[indStart] = pq.push(CostNodePair(g[indStart]+h[indStart],indStart));
  //int nNode=0;
  while ( !pq.empty() )
  {
    //nNode++;
    CostNodePair currNode = pq.top(); //element with smallest cost
    pq.pop(); //delete it from the queue          
    int indCurrNode = currNode.second;
    cl[indCurrNode] = true;
    
    //if (g[indCurrNode] > g[indGoal]) break; // done
    if( indCurrNode == indGoal ) break;

    int xCurr = indCurrNode % xDim;
    int yCurr = indCurrNode / xDim;

    //iterate over neighbor nodes
    for (int xShift=-1; xShift <= 1; ++xShift){
      int xNeighbor = xCurr + xShift;
      if (xNeighbor < 0) continue; // skip outside of map
      if (xNeighbor >= xDim) continue;
      
      for (int yShift=-1; yShift <= 1; yShift++){
        // skip current node
        if (xShift==0 && yShift==0) continue;     
        int yNeighbor = yCurr + yShift;
        if (yNeighbor < 0) continue;
        if (yNeighbor >= yDim) continue;
        
        int indNeighbor = xNeighbor + xDim*yNeighbor;
        // skip collisions and closed nodes
        if( cMap[indNeighbor] || cl[indNeighbor] ) continue;
        
        //calc the cost of the successor (neighbor)
        double stageCost = (std::abs(xShift) + std::abs(yShift) > 1) ? sqrtTwoCost : 1;
        double costNeighbor = g[indCurrNode] + stageCost;
        
        if (costNeighbor < g[indNeighbor])
        {
          //update the heuristic value
          if( !std::isinf(g[indNeighbor]) )
          {
            // node seen before
            g[indNeighbor] = costNeighbor;
            // increase == decrease with our comparator (!)
            pq.increase(hk[indNeighbor],CostNodePair(g[indNeighbor]+h[indNeighbor],indNeighbor)); 
          }else
          {
            h[indNeighbor] = eps*std::sqrt((xNeighbor-xGoal)*(xNeighbor-xGoal) + 
        			                             (yNeighbor-yGoal)*(yNeighbor-yGoal));
            g[indNeighbor] = costNeighbor;
            hk[indNeighbor] = pq.push(CostNodePair(g[indNeighbor]+h[indNeighbor],indNeighbor));
    			}
          iPrev[indNeighbor] = indCurrNode; // update parent index     
        }
      }
    } //END: iterate over neighbor nodes
  }// END: while
  
  // recover path
  int indCurrNode = indGoal;
  if( !std::isinf(g[indCurrNode]) )
  {
    int xCurr = xGoal; int yCurr = yGoal;
    while( indCurrNode != indStart )
    {
      xyPath.push_front({xCurr,yCurr});
      indCurrNode = iPrev[indCurrNode];
      xCurr = indCurrNode % xDim;
      yCurr = indCurrNode / xDim;     
    }
    xyPath.push_front({xCurr,yCurr});
  }

  double cost = g[indGoal];
  // Free memory
  delete [] g;
  delete [] h;
  delete [] iPrev;
  delete [] cl;
  delete [] hk;
  return cost;
}


/*
 * @INPUT:
 *  cMap a 3D array of costs stored in column major order
 */
double planOn3DGrid( const uint16_t* cMap, int xDim, int yDim, int zDim, 
                   int xStart, int yStart, int zStart,
                   int xGoal,  int yGoal,  int zGoal,
                   std::list<std::array<int,3>>& xyzPath,
                   double eps = 1 )
{
  // Start and Goal indices
  int xyDim = xDim*yDim;
  int indStart = xStart + xDim*yStart + xyDim*zStart; // colmajor
  if( cMap[indStart] ) return infCost;   // Quit if start is in collision 
  int indGoal  = xGoal  + xDim*yGoal  + xyDim*zGoal;
  if( cMap[indGoal] ) return infCost;   // Quit if goal is in collision 
  
  // Initialize all costs to infinity
  int cMapLength = xyDim*zDim;
  double *g = new double[cMapLength];
  double *h = new double[cMapLength];
  int *iPrev = new int[cMapLength]; // keep path indices
  bool *cl = new bool[cMapLength]; // closed list
  heapkey *hk = new heapkey[cMapLength]; // store heapkeys
  for (int i = 0; i < cMapLength; ++i)
  { cl[i] = false; g[i] = infCost; }
  g[indStart]=0;
  h[indStart]=eps*std::sqrt( (xGoal-xStart)*(xGoal-xStart) +
                             (yGoal-yStart)*(yGoal-yStart) +
                             (zGoal-zStart)*(zGoal-zStart) );
  PriorityQueue pq;
  hk[indStart] = pq.push(CostNodePair(g[indStart]+h[indStart],indStart));
  //int nNode=0;
  while ( !pq.empty() )
  {
    //nNode++;
    CostNodePair currNode = pq.top(); //element with smallest cost
    pq.pop(); //delete it from the queue          
    int indCurrNode = currNode.second;
    cl[indCurrNode] = true;
    
    //if (g[indCurrNode] > g[indGoal]) break; // done
    if( indCurrNode == indGoal ) break;

    int xCurr = (indCurrNode % xyDim) % xDim;
    int yCurr = (indCurrNode % xyDim) / xDim;
    int zCurr = indCurrNode / xyDim;

    //iterate over neighbor nodes
    for (int xShift=-1; xShift <= 1; ++xShift){
      int xNeighbor = xCurr + xShift;
      if (xNeighbor < 0) continue; // skip outside of map
      if (xNeighbor >= xDim) continue;
      
      for (int yShift=-1; yShift <= 1; yShift++){        
        int yNeighbor = yCurr + yShift;
        if (yNeighbor < 0) continue;
        if (yNeighbor >= yDim) continue;
        
        for (int zShift=-1; zShift <= 1; zShift++){
          // skip current node
          if (xShift==0 && yShift==0 && zShift==0) continue;
          
          int zNeighbor = zCurr + zShift;
          if (zNeighbor < 0) continue;
          if (zNeighbor >= zDim) continue;
          
          int indNeighbor = xNeighbor + xDim*yNeighbor + xyDim*zNeighbor;
          // skip collisions and closed nodes
          if( cMap[indNeighbor] || cl[indNeighbor] ) continue;
          
          //calc the cost of the successor (neighbor)
          double costMult = 1; // get the cost multiplier
          switch (std::abs(xShift) + std::abs(yShift)+ std::abs(zShift)){
            case 2: costMult = sqrtTwoCost; break;
            case 3: costMult = sqrtThreeCost; break;
          }
          double costNeighbor = g[indCurrNode] + costMult;
          
          
          if (costNeighbor < g[indNeighbor])
          {
            //update the heuristic value
            if( !std::isinf(g[indNeighbor]) )
            {
              // node seen before
              g[indNeighbor] = costNeighbor;
              // increase == decrease with our comparator (!)
              pq.increase(hk[indNeighbor],CostNodePair(g[indNeighbor]+h[indNeighbor],indNeighbor)); 
            }else
            {
              h[indNeighbor] = eps*std::sqrt((xNeighbor-xGoal)*(xNeighbor-xGoal) + 
          			                            (yNeighbor-yGoal)*(yNeighbor-yGoal) + 
          			                            (zNeighbor-zGoal)*(zNeighbor-zGoal));
	            g[indNeighbor] = costNeighbor;
	            hk[indNeighbor] = pq.push(CostNodePair(g[indNeighbor]+h[indNeighbor],indNeighbor));
	    			}
            iPrev[indNeighbor] = indCurrNode; // update parent index     
          }
        }
      }
    } //END: iterate over neighbor nodes
  }// END: while
  
  // recover path
  int indCurrNode = indGoal;
  if( !std::isinf(g[indCurrNode]) )
  {
    int xCurr = xGoal; int yCurr = yGoal; int zCurr = zGoal;
    while( indCurrNode != indStart )
    {
      xyzPath.push_front({xCurr,yCurr,zCurr});
      indCurrNode = iPrev[indCurrNode];
      xCurr = (indCurrNode % xyDim) % xDim;
      yCurr = (indCurrNode % xyDim) / xDim;
      zCurr = indCurrNode / xyDim;       
    }
    xyzPath.push_front({xCurr,yCurr,zCurr});
  }

  double cost = g[indGoal];
  // Free memory
  delete [] g;
  delete [] h;
  delete [] iPrev;
  delete [] cl;
  delete [] hk;
  return cost;
}

}

#endif
