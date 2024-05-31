#ifndef __DD_MOTION_MODEL_H__
#define __DD_MOTION_MODEL_H__
#include <cmath>    // std::sin
#include <array>
#include <erl_utilities/erl_geom_utils.h>

namespace erl
{
  inline double dd_motion_model_x(double v, double w, double t)
  {
    double tw_ = t*w;
    if( std::abs(tw_) < 0.0001 )
      return t*v*std::cos(tw_);
    else
      return v/w*std::sin(tw_);
  }
  inline double dd_motion_model_y(double v, double w, double t)
  {
    double tw_ = t*w;
    if( std::abs(tw_) < 0.0001 )
      return t*v*std::sin(tw_);
    else
      return v/w*(1 - std::cos(tw_));
  }
  inline double dd_motion_model_q(double v, double w, double t) { return t*w; }

  inline std::array<double,3> dd_motion_model(double v, double w, double t)
  {
    std::array<double,3> x;
    x[2] = t*w;
    if( std::abs(x[2]) < 0.0001 )
    {
      x[0] = t*v*std::cos(x[2]);
      x[1] = t*v*std::sin(x[2]);
    }else
    {
      x[0] = v/w*std::sin(x[2]);
      x[1] = v/w*(1 - std::cos(x[2]));
    }
    return x;
  }
  
  inline std::array<double,3> dd_motion_model(const std::array<double,3>& x,
                                              const std::array<double,2>& u, double t)
  {
    std::array<double,3> nx;
    double tw_ = t*u[1];
    nx[2] = erl::restrictAngle(x[2] + tw_);
    if( std::abs(tw_) < 0.0001 )
    {
      nx[0] = x[0] + t*u[0]*std::cos(nx[2]);
      nx[1] = x[1] + t*u[0]*std::sin(nx[2]);      
    }else
    {
      nx[0] = x[0] + u[0]/u[1]*( std::sin(nx[2]) - std::sin(x[2]) );
      nx[1] = x[1] + u[0]/u[1]*( std::cos(x[2])  - std::cos(nx[2]));      
    }
    return nx;
  }

  
  /*
   * Given coeff C and time T computes:
   *  Y = C(N)*T^N + C(N-1)*T^(N-1) + ... + C(1)*T + C(0)
   */
  inline double polyval_nx(const std::vector<double>& coeff, double t)
  {
    int N = coeff.size()-1; // power
    if( N < 0 )
      return 0;
    
    double y = coeff[N];
    for( int c = N-1; c > -1; --c )
      y = y*t + coeff[c];
    return y;
  }

}

#endif
