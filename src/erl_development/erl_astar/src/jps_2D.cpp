#include <erl_astar/jps_2D.h>
#include <iostream>

double erl::JPS_2D::plan( int xStart, int yStart,
                         int xGoal,  int yGoal,
                         //std::list<int>& indPath,
                         std::list<std::array<int,2>>& xyPath,
                         double eps )
{
  eps_ = eps;
  xGoal_ = xGoal; yGoal_ = yGoal;
  int indStart = xStart + xDim_*yStart; // colmajor
  int indGoal  = xGoal  + xDim_*yGoal;
  
  // Quit if start or goal state is in collision
  if( cMap_[indStart]!=val_free_ ||
      cMap_[indGoal] !=val_free_ )
    return std::numeric_limits<double>::infinity(); 
  
  hashMap_[indStart] = new Astate(indStart,0,0);
  hashMap_[indStart]->h = eps*std::sqrt( (xGoal-xStart)*(xGoal-xStart) +
                                         (yGoal-yStart)*(yGoal-yStart) );
  hashMap_[indStart]->g = 0;
  seen_[indStart] = true;
  
  //int nNode=0;
  Astate *currNode_pt = hashMap_[indStart];
  currNode_pt->cl = true;
  while(true)
  {
    //nNode++;
    if( currNode_pt->ID == indGoal ) break;
    spin(currNode_pt);
    
    if( pq_.empty() )
      return std::numeric_limits<double>::infinity();
    currNode_pt = pq_.top(); pq_.pop(); // get element with smallest cost
    currNode_pt->cl = true; 
  }
  
  // recover path
  int indCurrNode = indGoal;
  if( seen_[indCurrNode] )
  {
    int xCurr = xGoal; int yCurr = yGoal;

    while( indCurrNode != indStart )
    {
      //indPath.push_front(indCurrNode);
      xyPath.push_front({xCurr,yCurr});
      indCurrNode = hashMap_[indCurrNode]->parent;
      xCurr = indCurrNode % xDim_;
      yCurr = indCurrNode / xDim_;     
    }
    
    double cost = hashMap_[indGoal]->g;
    // cleanup
    for (int i = 0; i < xyDim_; ++i)
      if( seen_[i] ) delete hashMap_[i];
    //std::cout << "nNode = " << nNode << std::endl;
    return cost;
  }
  else
    return std::numeric_limits<double>::infinity();  
}


void erl::JPS_2D::spin(Astate *currNode_pt)
{
  int xCurr = currNode_pt->ID % xDim_;
  int yCurr = currNode_pt->ID / xDim_;
  int norm1 = std::abs(currNode_pt->dx) + std::abs(currNode_pt->dy);

  //iterate over neighbor nodes
  int num_neib;
  switch(norm1)
  {
    case 0: num_neib = 8; break;
    case 1: num_neib = 3; break;
    case 2: num_neib = 5; break;
  }
  /*
  int num_neib, num_fneib;
  switch(norm1)
  {
    case 0: num_neib = 8; num_fneib = 0; break;
    case 1: num_neib = 1; num_fneib = 2; break;
    case 2: num_neib = 3; num_fneib = 2; break;
  }
  */
  int nx, ny, dxNeighbor, dyNeighbor;
  for( int dev = 0; dev < num_neib; ++dev) // +num_fneib
  {
    if(norm1==0)
    {
      switch(dev){
        case 0: dxNeighbor = -1; dyNeighbor = -1; break;
        case 1: dxNeighbor = -1; dyNeighbor =  0; break;
        case 2: dxNeighbor = -1; dyNeighbor =  1; break;
        case 3: dxNeighbor =  0; dyNeighbor = -1; break;
        case 4: dxNeighbor =  0; dyNeighbor =  1; break;
        case 5: dxNeighbor =  1; dyNeighbor = -1; break;
        case 6: dxNeighbor =  1; dyNeighbor =  0; break;
        case 7: dxNeighbor =  1; dyNeighbor =  1; break;
      }
    }
    if(norm1==1)
    {
      switch(dev){
        case 0: dxNeighbor = currNode_pt->dx; dyNeighbor = currNode_pt->dy; break;
        case 1:
          Neib(currNode_pt->dx,currNode_pt->dy,0,1,dxNeighbor,dyNeighbor);
          nx = xCurr + dxNeighbor;
          ny = yCurr + dyNeighbor;
          if( nx < 0 || nx >= xDim_ || ny < 0 || ny >= yDim_ || cMap_[nx + ny*xDim_]==val_free_ )
            continue;
          Neib(currNode_pt->dx,currNode_pt->dy,1,1,dxNeighbor,dyNeighbor);
          break;
        case 2:
          Neib(currNode_pt->dx,currNode_pt->dy,0,-1,dxNeighbor,dyNeighbor);
          nx = xCurr + dxNeighbor;
          ny = yCurr + dyNeighbor;
          if( nx < 0 || nx >= xDim_ || ny < 0 || ny >= yDim_ || cMap_[nx + ny*xDim_]==val_free_ )
            continue;
          Neib(currNode_pt->dx,currNode_pt->dy,1,-1,dxNeighbor,dyNeighbor);
          break;        
      }
    }
    if(norm1==2)
    {
      switch(dev){
        case 0: Neib(currNode_pt->dx,currNode_pt->dy,1,1,dxNeighbor,dyNeighbor); break;
        case 1: Neib(currNode_pt->dx,currNode_pt->dy,1,-1,dxNeighbor,dyNeighbor); break;
        case 2: dxNeighbor = currNode_pt->dx; dyNeighbor = currNode_pt->dy; break;
        case 3:
          Neib(currNode_pt->dx,currNode_pt->dy,-1,1,dxNeighbor,dyNeighbor);
          nx = xCurr + dxNeighbor;
          ny = yCurr + dyNeighbor;
          if( nx < 0 || nx >= xDim_ || ny < 0 || ny >= yDim_ || cMap_[nx + ny*xDim_]==val_free_ )
            continue;
          Neib(currNode_pt->dx,currNode_pt->dy,0,1,dxNeighbor,dyNeighbor);
          break;
        case 4:
          Neib(currNode_pt->dx,currNode_pt->dy,-1,-1,dxNeighbor,dyNeighbor);
          nx = xCurr + dxNeighbor;
          ny = yCurr + dyNeighbor;
          if( nx < 0 || nx >= xDim_ || ny < 0 || ny >= yDim_ || cMap_[nx + ny*xDim_]==val_free_ )
            continue;
          Neib(currNode_pt->dx,currNode_pt->dy,0,-1,dxNeighbor,dyNeighbor);
          break;
      }
    }
    
    
    /*
    if( dev < num_neib )
    {
      dxNeighbor = neib_[currNode_pt->dx+1][currNode_pt->dy+1][0][dev];
      dyNeighbor = neib_[currNode_pt->dx+1][currNode_pt->dy+1][1][dev];
    }
    else
    {
      // Check if there is a forced neighbor
      nx = xCurr + fneib1_[currNode_pt->dx+1][currNode_pt->dy+1][0][dev-num_neib];
      ny = yCurr + fneib1_[currNode_pt->dx+1][currNode_pt->dy+1][1][dev-num_neib];
      if( nx < 0 || nx >= xDim_ || ny < 0 || ny >= yDim_ || !cMap_[nx + ny*xDim_] )
        continue;
      dxNeighbor = fneib2_[currNode_pt->dx+1][currNode_pt->dy+1][0][dev-num_neib];
      dyNeighbor = fneib2_[currNode_pt->dx+1][currNode_pt->dy+1][1][dev-num_neib];
    }
    */
    jump(xCurr,yCurr,dxNeighbor,dyNeighbor);
    if( xJump_ == -1 ) continue;
    
    int indNeighbor = xJump_ + xDim_*yJump_;
    // initialize if never seen before
    if( !seen_[indNeighbor] )
    {
      seen_[indNeighbor] = true; 
      hashMap_[indNeighbor] = new Astate(indNeighbor,dxNeighbor,dyNeighbor);
    }
    
    // skip closed nodes
    if(hashMap_[indNeighbor]->cl) continue;
          
    double costMult = std::sqrt((xCurr - xJump_)*(xCurr - xJump_) + 
                      			    (yCurr - yJump_)*(yCurr - yJump_));
              			
    double costNeighbor = hashMap_[currNode_pt->ID]->g + costMult;
    
    if (costNeighbor < hashMap_[indNeighbor]->g)
    {
      //update the heuristic value
      if( !std::isinf(hashMap_[indNeighbor]->g) )
      {
        // node seen before
        hashMap_[indNeighbor]->dx = dxNeighbor;
        hashMap_[indNeighbor]->dy = dyNeighbor;
        hashMap_[indNeighbor]->g = costNeighbor;
        pq_.increase(hashMap_[indNeighbor]->heapkey); // increase == decrease with our comparator (!)
      }else
      {
        hashMap_[indNeighbor]->h = eps_*std::sqrt( (xJump_-xGoal_)*(xJump_-xGoal_) + 
              			                               (yJump_-yGoal_)*(yJump_-yGoal_) );          		
    		hashMap_[indNeighbor]->g = costNeighbor;
    		hashMap_[indNeighbor]->heapkey = pq_.push(hashMap_[indNeighbor]);
			}
			hashMap_[indNeighbor]->parent = currNode_pt->ID;    
    }
  }
}



void erl::JPS_2D::jump(int x, int y, int dx, int dy)
{
  int nx = x+dx, ny = y+dy;
  if( nx<0 || nx >= xDim_ || ny<0 || ny >= yDim_ || cMap_[nx + ny*xDim_]!=val_free_ )
  {
    xJump_ = -1; yJump_ = -1;
    return;
  }  
  if( nx == xGoal_ && ny == yGoal_ )
  {
    xJump_ = nx; yJump_ = ny;
    return;
  }
  int norm1 = std::abs(dx) + std::abs(dy);
  if( hasForced(nx, ny, dx, dy, norm1) )
  {
    xJump_ = nx; yJump_ = ny;
    return;
  }
  
  if( norm1 == 2 )
  {
    int tx,ty;
    Neib(dx,dy,1,1,tx,ty);
    jump(nx,ny,tx,ty);
    if( xJump_ != -1 )
    {
      xJump_ = nx; yJump_ = ny;
      return;
    }
    Neib(dx,dy,1,-1,tx,ty);
    jump(nx,ny,tx,ty);
    if( xJump_ != -1 )
    {
      xJump_ = nx; yJump_ = ny;
      return;
    }    
  }
  jump(nx,ny,dx,dy);  
}



bool erl::JPS_2D::hasForced(int x, int y, int dx, int dy, int norm1)
{
  int nx,ny;
  switch(norm1)
  {
    case 1:
      Neib(dx,dy,0,1,nx,ny);
      nx = x + nx; ny = y + ny;
      if( nx >= 0 && nx < xDim_ && ny >= 0 && ny < yDim_ && cMap_[nx + ny*xDim_]!=val_free_ )
        return true;
      Neib(dx,dy,0,-1,nx,ny);
      nx = x + nx; ny = y + ny;
      if( nx >= 0 && nx < xDim_ && ny >= 0 && ny < yDim_ && cMap_[nx + ny*xDim_]!=val_free_ )
        return true;
      return false;
    case 2:
      Neib(dx,dy,-1,1,nx,ny);
      nx = x + nx; ny = y + ny;
      if( nx >= 0 && nx < xDim_ && ny >= 0 && ny < yDim_ && cMap_[nx + ny*xDim_]!=val_free_ )
        return true;
      Neib(dx,dy,-1,-1,nx,ny);
      nx = x + nx; ny = y + ny;
      if( nx >= 0 && nx < xDim_ && ny >= 0 && ny < yDim_ && cMap_[nx + ny*xDim_]!=val_free_ )
        return true;
      return false;      
  }
}

/*         
bool erl::JPS_2D::hasForced(int x, int y, int dx, int dy)
{
  int nx,ny;
  for( int fn = 0; fn < 2; ++fn )
  {
    nx = x + fneib1_[dx+1][dy+1][0][fn];
    ny = y + fneib1_[dx+1][dy+1][1][fn];  
    if( nx >= 0 && nx < xDim_ && ny >= 0 && ny < yDim_ && cMap_[nx + ny*xDim_] )
      return true;
  }
  return false;
}
*/

void erl::JPS_2D::Neib(int dx, int dy, int nx, int ny, int& tx, int& ty)
{
  //double norm2 = std::sqrt(dx*dx + dy*dy);
  //double aa = std::sqrt(2)*(dx*nx - dy*ny)/norm2;
  //double bb = std::sqrt(2)*(dy*nx + dx*ny)/norm2;
  //tx = std::round(std::max(std::min( aa, 1.0 ), -1.0 ));
  //ty = std::round(std::max(std::min( bb, 1.0 ), -1.0 ));

  if(nx == 1 && ny == 0)
  {
    tx = dx; ty = dy; return;
  }  
  if(nx == -1 && ny == 0)
  {
    tx = -dx; ty = -dy; return;
  }
  if(nx == -1 && ny == -1)
  {
    int norm1 = std::abs(dx) + std::abs(dy);
    tx = (dy-dx)/norm1; ty = (-dx-dy)/norm1;
    return;
  }
  if(nx == 0 && ny == -1)
  {
    tx = dy; ty = -dx; return;
  }
  if(nx == 1 && ny == -1)
  {
    int norm1 = std::abs(dx) + std::abs(dy);
    tx = (dx+dy)/norm1; ty = (dy-dx)/norm1;
    return;
  }
  if(nx == 1 && ny == 1)
  {
    int norm1 = std::abs(dx) + std::abs(dy);
    tx = (dx-dy)/norm1; ty = (dx+dy)/norm1;
    return;
  }
  if(nx == 0 && ny == 1)
  {
    tx = -dy; ty = dx; return;
  }
  if(nx == -1 && ny == 1)
  {
    int norm1 = std::abs(dx) + std::abs(dy);
    tx = (-dx-dy)/norm1; ty = (dx-dy)/norm1;
    return;
  }
}



