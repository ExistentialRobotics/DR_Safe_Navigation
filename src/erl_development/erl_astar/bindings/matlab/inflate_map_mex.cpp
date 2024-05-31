/*
 * To compile:
 *   mex CXXFLAGS='\$CXXFLAGS -std=c++11' inflate_map_mex.cpp
 */

#include "mex.h"
#include <vector>
#include <cmath>

// LOL:
// http://stackoverflow.com/questions/27029048/perfect-forwarding-variadic-template-to-standard-thread
template<typename Functor, typename... Args>
void nestedForLoop( const std::vector<int>& formin,
                    const std::vector<int>& formax,
                    Functor&& f, Args&&... args )
{
  std::vector<int> idx(formin);
  int ndim = idx.size();
  if( ndim < 1) return;
  int p = 0;
  while( p < ndim )
  {
    f(idx,std::forward<Args>(args)...); //perfect forwarding...
    
    // increment nested for loop indices
    idx[0]++;
    while(idx[p]==formax[p])
    {
      idx[p] = formin[p]; p++;
      if( p == ndim ) break;
      idx[p]++;
      if(idx[p]!=formax[p]) p=0;
    }
  }
}


void updateNS( const std::vector<int>& idx,
               std::vector<std::vector<int>>& ns,
               int rad )
{  
  double d = 0;
  for( int k = 0; k < idx.size(); ++k )
    d += idx[k]*idx[k];
  d = std::sqrt(d);
  if( 0 < d && d <= rad )
    ns.push_back( idx );
}

// Column major order as in MATLAB
int subv2ind_colmajor( const std::vector<int>& datac,
                       const std::vector<int>& size_ )
{
  int idx = datac[0]; int prod = 1;
  for( int k = 1; k < datac.size(); ++k )
  {
    prod *= size_[k-1];
    idx += datac[k] * prod;
  }
  return idx;
}

bool inMap( const std::vector<int>& idx,
            const std::vector<int>& size_ )
{
  for( int k=0; k < idx.size(); ++k )
    if( idx[k] < 0 || idx[k] >= size_[k] )
      return false;
  return true;
}

void updateOmap( const std::vector<int>& idx,
                 const std::vector<int>& size_,
                 const std::vector<std::vector<int>>& ns,
                 char val_free,
                 char val_occ,
                 const char *cmap,
                 char *imap )
{
  if( cmap[subv2ind_colmajor(idx,size_)] != val_free )
  {
    for(const auto& it : ns )
    {
      std::vector<int> nx(idx);
      for( int k = 0; k < nx.size(); ++k )
        nx[k] += it[k];
      if( inMap(nx, size_) )
      {
        imap[subv2ind_colmajor(nx,size_)] = val_occ;
      }
    }
  }
}


/*
 * @INPUT:
 *  prhs[0] = collision map = n-D int8 matrix 
 *  prhs[1] = 1 x 1 = map inflation radius (int)
 *  prhs[2] = 1 x 1 = occupied value for inflation (int8)
 *
 * @OUTPUT:
 *  plhs[0] = inflated map = n-D int8 matrix 
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // input map
  mwSize ndim = mxGetNumberOfDimensions(prhs[0]);
  const mwSize *dims = mxGetDimensions(prhs[0]);  
  char *cmap = (char *)mxGetData(prhs[0]);

  // output map
  plhs[0] = mxDuplicateArray(prhs[0]);
  char *imap = (char *)mxGetData(plhs[0]);
  
  // inflation radius
  int infrad = static_cast<int>(std::ceil(mxGetScalar(prhs[1])));
  char val_free = 0;
  char val_occ;
  if( nrhs > 2 )
    val_occ = static_cast<char>(mxGetScalar(prhs[2]));
  else
    val_occ = 1;

  // compute neighbor indices
  std::vector<std::vector<int>> ns;
  ns.reserve(std::pow((2*infrad+1),ndim));
  std::vector<int> formin( ndim, -infrad);
  std::vector<int> formax( ndim, infrad+1);
  nestedForLoop( formin, formax, updateNS, ns, infrad );

  formin = std::vector<int>(ndim,0);
  for( int k = 0; k < formax.size(); ++k )
    formax[k] = dims[k];
  nestedForLoop( formin, formax, updateOmap, formax, ns, val_free, val_occ, cmap, imap ); 
}


