/* Astar algorithm implementation
   by Aleksandr Kushleyev <akushley@seas.upenn.edu>
   University of Pennsylvania, 02/2008
   
   Based on Dr. Daniel Lee's dijkstra_nonholonomic16.cc, which he has written for Darpa Urban Challenge
*/

#include <math.h>
#include "mex.h"
#include <set>

using namespace std;

typedef pair<double, int> CostNodePair; // pair of a double cost and node number


//#define PI 3.14159265

double calcHeading(int iFrom,int jFrom,int iTo, int jTo)
{
  int dx=iTo-iFrom;
  int dy=jTo-jFrom;
  
  return atan2(dy,dx);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
  if (nrhs < 3) {
    mexErrMsgTxt("Need at least three input arguments: map, [xStart,yStart], [xGoal,yGoal]");
  }

  double *costs = mxGetPr(prhs[0]);     //cost map
  int m = mxGetM(prhs[0]);              //dimension of the map (height)(ROWS)
  int n = mxGetN(prhs[0]);              //dimension of the map (width)(COLS)
  double sqrtTwo = sqrt(2);
  /*
  for (int i=0; i < n;i++){
    for (int j=0; j < m; j++) {
      printf("%d",(int)costs[j+m*i]);
    }
    printf(" ");
  }
  */
  
  
  if (mxGetM(prhs[1])*mxGetN(prhs[1]) != 2) {
    mexErrMsgTxt("Start should be (xgoal, ygoal)");
  }
  double *prStart = mxGetPr(prhs[1]);

  if (mxGetM(prhs[2])*mxGetN(prhs[2]) != 2) {
    mexErrMsgTxt("Goal should be (xstart, ystart)");
  }
  double *prGoal = mxGetPr(prhs[2]);
  
  int iStart = (int)(prStart[0]-1);
  int jStart = (int)(prStart[1]-1);
  int iGoal = (int)(prGoal[0]-1);
  int jGoal = (int)(prGoal[1]-1);
  int indStart = m*iStart + jStart;
  int indGoal = m*iGoal + jGoal;
  
  plhs[0] =mxCreateDoubleMatrix(m, n, mxREAL);              //create output array total costs
  plhs[1] =mxCreateDoubleMatrix(m, n, mxREAL);              //create output array of previous lowest-cost neighbors (xPrev)
  plhs[2] =mxCreateDoubleMatrix(m, n, mxREAL);              //create output array of previous lowest-cost neighbors (yPrev)
  /* MY ADDITION */
  plhs[3] =mxCreateDoubleMatrix(1, 2, mxREAL);		    //create output array of new goal coordinates
  /* END */
  double *g = mxGetPr(plhs[0]);                     	    //pointer to the final total costs
  double *xPrev = mxGetPr(plhs[1]);                         //pointer to xPrev (x-offset of the previous neighbor)
  double *yPrev = mxGetPr(plhs[2]);                         //pointer to yPrev (y-offset of the previous neighbor)
  double *f = new double[m*n];				    //f=g+h; 
  double *h = new double[m*n];
  /* MY ADDITION */
  double *newGoal = mxGetPr(plhs[3]);
  int indMinL2 = indStart;				    //save the explored node with the minimum L2 from the goal
  /* END */
  
  for (int i = 0; i < m*n; i++) g[i] = INFINITY;    //initialize all costs to infinity
  g[indStart]=costs[indStart];
  h[indStart]=sqrt( pow(iGoal-iStart,2) + pow(jGoal-jStart,2) );
  f[indStart]=g[indStart] + h[indStart];
  
  set<CostNodePair> Queue;
  Queue.insert(CostNodePair(f[indStart],indStart));
  
  int nNode=0;
  

  while ( !Queue.empty()){
    nNode++;
    CostNodePair currNode = *Queue.begin();     //extract the element with smallest cost
    Queue.erase(Queue.begin());                 //delete it from the queue
        
    int indCurrNode = currNode.second;
    
    if (g[indCurrNode] > g[indGoal]) break;
    
    int iCurr = indCurrNode / m;
    int jCurr = indCurrNode % m;
    
    //iterate over neighbor nodes
    for (int iShift=-1; iShift <= 1; iShift++){
      int iNeighbor = iCurr + iShift;
      if (iNeighbor < 0) continue;
      if (iNeighbor >= n) continue; 
      
      for (int jShift=-1; jShift <= 1; jShift++){
        if (iShift==0 && jShift==0) continue;
        
        int jNeighbor = jCurr + jShift;
        if (jNeighbor < 0) continue;
        if (jNeighbor >= m) continue;
        
        int indNeighbor = m*iNeighbor + jNeighbor;

				//get the cost multiplier (diagonal transitions have sqrt(2) )
        double costMult = (abs(iShift) + abs(jShift)) > 1 ? sqrtTwo : 1; 
        
				//calc the cost of the successor (neighbor)
        double costNeighbor = g[indCurrNode] + costMult*costs[indNeighbor];
        
        if (costNeighbor < g[indNeighbor]){ 
					//the cost is lower, update it          
					if (!isinf(g[indNeighbor])){
            Queue.erase(CostNodePair(f[indNeighbor],indNeighbor));
          }

					//update the g (cost) value
          g[indNeighbor] = costNeighbor;

					//update the heuristic value
	  			h[indNeighbor] = sqrt( (iNeighbor-iGoal)*(iNeighbor-iGoal) + (jNeighbor-jGoal)*(jNeighbor-jGoal));
	  
	  
	        /* MY ADDITION */
	        // check if the node with min L2 norm and finite gVal needs to be updated			
	        if(h[indNeighbor] < h[indMinL2])
	        	indMinL2 = indNeighbor;
	        if((h[indNeighbor] == h[indMinL2])&&(g[indNeighbor] < g[indMinL2]))
	        	indMinL2 = indNeighbor;
	        /* END */

					//update the f value	  			
					f[indNeighbor] = g[indNeighbor] + h[indNeighbor];
          
		  //update the indeces of the parent of the node
		  xPrev[indNeighbor] = iCurr+1;
          yPrev[indNeighbor] = jCurr+1;
          Queue.insert(CostNodePair(f[indNeighbor],indNeighbor));
        }
      }
    }
  }
  //printf("indMinL2 = %d\n",indMinL2);
  newGoal[0] = (indMinL2/m)+1;
  newGoal[1] = (indMinL2%m)+1;
  //printf("Astar: cost from start to finish = %f\n",g[indGoal]);
  delete [] f;
  delete [] h;
}
