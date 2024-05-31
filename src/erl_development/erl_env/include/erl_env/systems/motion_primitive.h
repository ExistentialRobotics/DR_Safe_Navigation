#ifndef __MOTION_PRIMITIVE_H__
#define __MOTION_PRIMITIVE_H__

#include <functional> //std::function
#include <numeric>    // std::partial_sum
#include <erl_utilities/erl_yaml_addon.h>
//#include <memory>   // std::unique_ptr
#include <iostream>

namespace erl
{
  template <class state, class control>
  struct MotionPrimitive
  {
    std::vector< control > uVec;
    std::vector< double > tVec;
    std::vector< double > cVec;
    std::vector< std::vector< state > > xVecVec; // micro states for collision checking
  };

  // emitting
  template<typename state, typename control>
  inline YAML::Emitter& operator<< (YAML::Emitter& out,
                                    const MotionPrimitive<state,control>& mp)
  {
    out << YAML::BeginMap;
    //out << YAML::Key << "length";
    //out << YAML::Value << mp.mp_u_ptr->size();
    out << YAML::Key << "controls";
    out << YAML::Value << mp.uVec;
    out << YAML::Key << "durations";
    out << YAML::Value << YAML::Flow << mp.tVec;
    out << YAML::Key << "costs";
    out << YAML::Value << YAML::Flow << mp.cVec;
    out << YAML::EndMap;
    return out;
  }

  // reading in
  template<typename state, typename control>
  void mprmsFromYAML( const std::string & fromYAML,
                      std::function<state (state,control,double)> f,
                      const state& x0,
                      std::vector< MotionPrimitive<state,control> >& mprms )
  {
    // Read motion primitives from file
    std::vector<YAML::Node> docs = YAML::LoadAllFromFile(fromYAML);    
    //std::cout << "docs.size()=" << docs.size() << std::endl;
    
    for( unsigned k = 0; k < docs.size(); ++k )
    {
      mprms.push_back( MotionPrimitive<state,control>() );
      if( !docs[k]["controls"] )
      {
        std::cerr << "[MotionPrimitive] mprim.yaml missing controls field..." << std::endl;
        return;
      }
      std::size_t mprim_len =  docs[k]["controls"].size();
      //std::cout << k << " mprim_len=" << mprim_len << std::endl;
      mprms.back().uVec.reserve( mprim_len );
      for(std::size_t u=0; u<mprim_len; ++u) 
        mprms.back().uVec.push_back( docs[k]["controls"][u].as<control>() );
      
      if( !docs[k]["durations"] )
      {
        std::cerr << "[MotionPrimitive] mprim.yaml missing durations field..." << std::endl;
        return;
      }
      mprms.back().tVec = docs[k]["durations"].as<std::vector<double>>(); // copy
      
      if( !docs[k]["costs"] )
      {
        std::cerr << "[MotionPrimitive] mprim.yaml missing costs field..." << std::endl;
        return;
      }
      mprms.back().cVec = docs[k]["costs"].as<std::vector<double>>(); // copy
      std::partial_sum(mprms.back().cVec.begin(), mprms.back().cVec.end(),
                       mprms.back().cVec.begin()); // Cost should be cumulative (!)
      
      // Initialize the micro states using the dynamics f
      mprms.back().xVecVec.resize(mprim_len);
      double dt = 0.05; const state* x0_ = &x0;
      for(std::size_t u=0; u<mprim_len; ++u)
      {
        double tf = mprms.back().tVec[u];
        for( double t = dt; t < (tf+dt); t += dt)
          mprms.back().xVecVec[u].push_back( f(*x0_, mprms.back().uVec[u], t) );
        x0_ = &(mprms.back().xVecVec[u].back());
      }
      
    }// while(parser)
  }// mprmsFromFile()
}

#endif

/*
void toYAML()
{
   YAML::Emitter out;
   out << YAML::Literal << "BEGIN MOTION PRIMITIVE";
   out << YAML::BeginMap;
   out << YAML::Key << "control length";
   out << YAML::Value << mp_u_ptr->size();
   out << YAML::EndMap;
   out << (*mp_u_ptr);
   out << YAML::BeginMap;
   out << YAML::Key << "time length";
   out << YAML::Value << mp_t_ptr->size();
   out << YAML::EndMap;
   out << (*mp_t_ptr);    
   out << YAML::BeginMap;
   out << YAML::Key << "cost length";
   out << YAML::Value << mp_c_ptr->size();
   out << YAML::EndMap;
   out << (*mp_c_ptr);
   out << YAML::BeginMap;
   out << YAML::Key << "state length";
   out << YAML::Value << mp_x_ptr->size();
   out << YAML::EndMap;
   for( unsigned k = 0; k < mp_x_ptr->size(); ++k )
   {
     out << YAML::BeginMap;
     out << YAML::Key << "segment length";
     out << YAML::Value << (*mp_x_ptr)[k].size();
     out << YAML::EndMap;
     out << (*mp_x_ptr)[k];
   }
   out << YAML::Literal << "END MOTION PRIMITIVE\n";
   
   std::cout << "Here's the output YAML:\n" << out.c_str();
   std::cout << std::endl;
}
*/
