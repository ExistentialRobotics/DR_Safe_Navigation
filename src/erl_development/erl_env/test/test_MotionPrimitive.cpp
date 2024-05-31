
#include <iostream>
#include <erl_env/systems/motion_primitive.h>
#include <erl_env/systems/dd_motion_model.h>
#include <erl_utilities/erl_map_utils.h>

/*
double chilu(double a, double b)
{
  return a*b;
}
double chilu(const double& a, const std::array<double,1>& b)
{
  return a*b[0];
}



template<typename state, typename control>
void test_chilu(const state& a, const control& b,
                std::function<state (const state&, const control&)> f,
                state& res)
{
 res = std::move(f(a,b));
}


template<typename state, typename control>
void test_chilu(const state& a, const control& b,
                state (*f)(const state&, const control&),
                state& res)
{
 res = std::move(f(a,b));
}

typedef double (*funtype)(const double&, const std::array<double,1>&);
funtype f = chilu;
double res;
test_chilu<double,std::array<double,1>>(2,std::array<double,1>({3}), static_cast<double (*)(const double&, const std::array<double,1>&)>(chilu), res);
*/

template<typename state, typename control>
void test1( const state& x0,
            const control& u,
            double t,
            std::function<state (const state&, const control&, double)> f,
            state& x1 )
{
  x1 = std::move(f(x0,u,t));
}


int main( int argc, char** argv)
{
  std::vector< erl::MotionPrimitive<std::array<double,3>,std::array<double,2>> > mprim_;
  std::string fname(argv[1]);
  std::array<double,3> x0({0,0,0});
  std::array<double,2> u({1,0});

  erl::mprmsFromYAML<std::array<double,3>,std::array<double,2>>( fname, static_cast<std::array<double,3> (*)(const std::array<double,3>&, const std::array<double,2>&,double)>(erl::dd_motion_model), x0, mprim_);

  YAML::Emitter out;
  for( unsigned int k = 0; k < mprim_.size(); ++k )
    out << mprim_[k];
  
  // string to file
  std::ofstream ofs;
  ofs.open ("test.yaml", std::ofstream::out | std::ofstream::trunc);
  ofs << out.c_str();
  ofs.close();

  
  /*
  test1<std::array<double,3>,std::array<double,2>>( x0,u,1,static_cast<std::array<double,3> (*)(const std::array<double,3>&, const std::array<double,2>&,double)>(erl::dd_motion_model),x1);
  std::cout << x1[0] << " " << x1[1] << " " << x1[2] << std::endl;
  */
  
  //test1<std::array<double,3>,std::array<double,2>>( x0,u,1,erl::dd_motion_model,x1 );
  //std::cout << x1[0] << " " << x1[1] << " " << x1[2] << std::endl;
  
  //static_cast<std::function<std::array<double,3> (const std::array<double,3>&,const std::array<double,2>&,double)>>(erl::dd_motion_model)
  
  //auto x1 = erl::dd_motion_model(x0,u,1.0);
  //
  
  //erl::mprmsFromYAML( fname, erl::dd_motion_model, x0, mprim_ );
  
  /*
  erl::mprmsFromYAML( fname, static_cast<std::function<std::array<double,3> (std::array<double,3>,std::array<double,2>,double)>>(erl::dd_motion_model), x0, mprim_ );
  
  
  */
  
  /*
  mprim_[0].mp_u_ptr->push_back( {3,0} );
  mprim_[0].mp_t_ptr->push_back( 1 );
  mprim_[0].mp_c_ptr->push_back( 3 );

  mprim_[1].mp_u_ptr->push_back( {3,-1} );
  mprim_[1].mp_t_ptr->push_back( 1 );
  mprim_[1].mp_c_ptr->push_back( 3 );
  
  double dt = 0.05;
  for( unsigned k = 0; k < mprim_.size(); ++k )
  {
    double v = (*mprim_[k].mp_u_ptr)[0][0];
    double w = (*mprim_[k].mp_u_ptr)[0][1];
    double tf = (*mprim_[k].mp_t_ptr)[0];
    
    (*mprim_[k].mp_x_ptr).push_back( std::vector< std::array<double,3> >() );
    for( double t = dt; t < (tf+dt); t += dt)
    {
      (*mprim_[k].mp_x_ptr)[0].push_back( {
        erl::dd_motion_model_x(v,w,t),
        erl::dd_motion_model_y(v,w,t),
        erl::dd_motion_model_q(v,w,t) });        
    }
  }  
  
  mprim_[0].toYAML();
  mprim_[1].toYAML();
  
  YAML::Emitter out;
  out << mprim_[0];
  out << mprim_[1];

  // string to file
  std::ofstream ofs;
  ofs.open ("test.yaml", std::ofstream::out | std::ofstream::trunc);
  ofs << out.c_str();
  ofs.close();
  */
  
  
  /*
  out << YAML::Flow;
  out << YAML::BeginSeq << 2 << 3 << 5 << 7 << 11 << YAML::EndSeq;
  out << YAML::Flow;
  out << YAML::BeginSeq << 2 << 3 << 5 << 7 << 11 << YAML::EndSeq;
  out << YAML::BeginMap;
  out << YAML::Key << "cameras";
  out << YAML::Value << YAML::BeginSeq;
  out << "camera" << "camera" << YAML::EndSeq;
  
  out << YAML::Key << "imu_params";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "delay_imu_cam";
  out << YAML::Value << "0.0";
  out << YAML::Key << "max_imu_delta_t";
  out << YAML::Value << "0.01";
  out << YAML::EndMap;
  
  out << YAML::Key << "imu_initialization";
  out << YAML::Value << "Barack Obama";
  out << YAML::EndMap;
  std::cout << "Here's the output YAML:\n" << out.c_str();
  std::cout << std::endl;
  */
  /*
  YAML::Node config = YAML::LoadFile("/media/natanaso/Data/Stuff/Research/git/libnx_ws/src/nx_planning/astar_nx/data/sample.yaml");
  */
  
  
  return 0;
}
