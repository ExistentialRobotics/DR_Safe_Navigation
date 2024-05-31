
#ifndef __FULLYACTUATEDSYSTEM_H_
#define __FULLYACTUATEDSYSTEM_H_
#include <array>

namespace erl
{

template <size_t D, size_t N>
struct Waypoint
{
  std::array<double, N*D> coord;

  /**
   * Default Constructor.
   */
  Waypoint() {};

  /**
   * Constructs a Waypoint from an array of coordinates.
   * @param coord
   */
  Waypoint(const std::array<double, N*D> &coord) : coord(coord) {};

  /**
   * Constructs a Waypoint from a vector of coordinates.
   * @param coord
   
  Waypoint(const std::vector<double> &coord_) {
    for (unsigned i = 0; i < coord_.size(); i++)
      coord[i] = coord_[i];
  };
  */
  
  bool operator==(const Waypoint &other) const
  {
    for( unsigned it = 0; it < N*D; ++it )
      if( std::abs( coord[it] - other.coord[it] ) > 1.0e-5 )
        return false;
    return true;
  }
};
typedef Waypoint<2,1> Waypoint2D1O;
typedef Waypoint<2,2> Waypoint2D2O;
typedef Waypoint<2,3> Waypoint2D3O;
typedef Waypoint<2,4> Waypoint2D4O;
typedef Waypoint<2,5> Waypoint2D5O;
typedef Waypoint<3,1> Waypoint3D1O;
typedef Waypoint<3,2> Waypoint3D2O;
typedef Waypoint<3,3> Waypoint3D3O;
typedef Waypoint<3,4> Waypoint3D4O;
typedef Waypoint<3,5> Waypoint3D5O;

template <size_t D, size_t N>
class FullyActuatedSystem
{
  std::vector<std::array<double,N>> tauPowerMat_;
public:
  // initialize tauPowerMat = [(tau/ntau, tau^2/2/ntau^2,..., tau^n/n!/ntau^n),...,(tau, tau^2/2, tau^3/3!, ... tau^n/n!)]
  FullyActuatedSystem(double tau, size_t ntau)
    : tauPowerMat_(ntau)
  {
    for(size_t i = 0; i < ntau; ++i)
    {
      tauPowerMat_[i][0] = (i+1)*tau/ntau;
      for(size_t n = 1; n < N; ++n)
        tauPowerMat_[i][n] = tauPowerMat_[i][0] * tauPowerMat_[i][n-1] / (n+1); 
    }
  }
  
  Waypoint<D,N> next(const Waypoint<D,N>& w, const std::array<double,D>& u) const
  {
    return next(w,u,tauPowerMat_.size()-1);
  }
  
  Waypoint<D,N> next(const Waypoint<D,N>& w, const std::array<double,D>& u, size_t dtau_id) const
  {
    Waypoint<D,N> nw;
    for(size_t n = 0; n < N; ++n)
      for(size_t d = n*D; d < (n+1)*D; ++d)
      {
        nw.coord[d] = w.coord[d];
        for(size_t i = 0; i < (N-1-n); ++i)
          nw.coord[d] += tauPowerMat_[dtau_id][i] * w.coord[d+(i+1)*D];
        nw.coord[d] += tauPowerMat_[dtau_id][(N-1-n)] * u[d-n*D];
      }
    return nw;
  }
  
  std::vector<Waypoint<D,N>> next_micro(const Waypoint<D,N>& w, const std::array<double,D>& u) const
  {
    std::vector<Waypoint<D,N>> nwa(tauPowerMat_.size());
    for( size_t i = 0; i < nwa.size(); ++i )
      nwa[i] = next(w,u,i);
    return nwa;
    /*
    if(NTAU < 1) return nwa;
    nwa[0] = next(w,u,0);
    for( size_t i = 1; i < NTAU; ++i )
      nwa[i] = next(nwa[i-1],u,0);
    return nwa;
    */
  }
  
  double tau() const
  { return tauPowerMat_[tauPowerMat_.size()-1][0]; }
  size_t ntau() const
  { return tauPowerMat_.size(); }
  
  
  
  /*
  std::pair<Waypoint<D,N>,Waypoint<D,N>> next_minmax(const Waypoint<D,N>& w, const std::array<double,D>& u) const
  {
    // find derivative
    // find roots: std::vector<double> ts = solve(a, b, c, d, e, f, g);
    // find min and max root using horner method to evaluate
  }
  */
};
typedef FullyActuatedSystem<2,1> FAS2D1O;
typedef FullyActuatedSystem<2,2> FAS2D2O;
typedef FullyActuatedSystem<2,3> FAS2D3O;
typedef FullyActuatedSystem<2,4> FAS2D4O;
typedef FullyActuatedSystem<2,5> FAS2D5O;
typedef FullyActuatedSystem<3,1> FAS3D1O;
typedef FullyActuatedSystem<3,2> FAS3D2O;
typedef FullyActuatedSystem<3,3> FAS3D3O;
typedef FullyActuatedSystem<3,4> FAS3D4O;
typedef FullyActuatedSystem<3,5> FAS3D5O;
}

#endif
